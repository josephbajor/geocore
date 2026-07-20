//! The body checker (PK_BODY_check analog).
//!
//! [`check_body`] walks a body and reports every violated invariant as a
//! [`Fault`] — it never panics and never stops at the first fault, because
//! downstream (import diagnostics, boolean failure reports) wants the full
//! list. Traversal follows the store's deterministic orders, so the fault
//! list is deterministic too. Stale references are faults attached to the
//! *referring* entity, never errors.
//!
//! **Structural checks:** parent/child back-pointer agreement at every
//! level, entity-kind consistency (region kinds, wire/acorn content rules),
//! loop ring closure, fin pairing and opposed traversal on manifold edges,
//! edge bounds sanity, ring-edge conventions, zero-loop faces only on
//! closed surfaces, and the Euler–Poincaré identity (below).
//!
//! **Geometric checks:** vertices on their edges' curves and edge curves on
//! their adjacent faces' surfaces (within `max(entity tolerance, session
//! resolution)`), coordinates inside the ±500 m size box, and loop
//! orientation (outer counterclockwise w.r.t. the face normal).
//!
//! # The Euler identity enforced
//!
//! Per shell of a solid body, with cells that may be circles and
//! punctured disks, the Euler characteristic is
//!
//! ```text
//! χ = Σ_vertices 1  −  Σ_edges χ(edge)  +  Σ_faces χ(face)
//! χ(edge) = 1 for a vertex-bounded edge, 0 for a ring edge (a circle)
//! χ(face) = 2 − L for a face with L ≥ 1 loops (disk with L−1 holes)
//!           2 for a zero-loop sphere face, 0 for a zero-loop torus face
//! ```
//!
//! and a closed orientable boundary requires `χ = 2 − 2G`, i.e. **χ even
//! and ≤ 2**. A block gives 8 − 12 + 6 = 2; a solid cylinder gives
//! 0 − 0 + (0 + 1 + 1) = 2; sphere and torus bodies give 2 and 0.
//!
//! [`check_body`] is the compatibility Fast entry point. [`check_body_report`]
//! makes assurance explicit: Full reports preserve proven faults and, once the
//! Fast structure is clean, separately enumerate every proof obligation the
//! current implementation cannot discharge. A clean sample is therefore never
//! presented as Full validity.
//!
//! # Current Fast limits (reported as Full verification gaps)
//!
//! - Loop orientation is proven only for exact, strictly closed straight
//!   segment loops on planes. Curved, tolerance-joined, nonlinear-chart, and
//!   periodic loops remain Full verification gaps.
//! - A zero-loop face on a NURBS surface is faulted: closed NURBS
//!   surfaces land with periodic NURBS (M3).
//! - Curve-less tolerant ring edges are not supported; vertex-bounded
//!   tolerant edges are realized by their fins' lifted pcurves.
//! - Shells containing unclassifiable faces are exempt from the Euler
//!   identity (they still get all local checks).

use crate::entity::{
    Body, BodyId, BodyKind, Edge, EdgeId, EntityRef, Face, FaceId, Fin, FinId, LoopId, RegionKind,
    SeamSide, ShellId, VertexId,
};
use crate::geom::SurfaceGeom;
use crate::graph_work::GraphQueryWork;
use crate::incidence::{
    ContextualGraphQueries, ContextualPcurveError, IncidenceCertification, PcurveIssue,
    certify_edge_surface_incidence, certify_pcurve_incidence, check_pcurve_chart_contextual,
    check_pcurve_incidence_contextual, check_pcurve_metadata_contextual,
    check_pcurve_parameterization,
};
use crate::loop_proof::{
    LoopContainment, LoopSimplicity, certify_loop_containment, certify_loop_simplicity,
    certify_planar_loop_layout,
};
use crate::shell_proof::{ShellEmbedding, ShellOrientation, certify_shell_in_scope};
use crate::store::{Entity, Store};
use kcore::arena::Handle;
use kcore::error::{CapabilityId, Error, Result};
use kcore::operation::{
    BudgetPlan, ExecutionPolicy, LimitSnapshot, NumericalPolicy, OperationContext,
    OperationOutcome, OperationPolicyError, OperationScope, PolicyVersion, SessionPolicy,
    SessionPrecision,
};
use kcore::tolerance::{LINEAR_RESOLUTION, SIZE_BOX_HALF, Tolerances};
use kgeom::param::ParamRange;
use kgeom::surface_point::distance_to_surface;
use kgraph::{EvalBudgetProfile, EvalError, EvalLimits, SurfaceDerivativeOrder};

/// What is wrong, attached to the offending entity in a [`Fault`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum FaultKind {
    /// A child's parent pointer disagrees with the parent's child list,
    /// or an entity is referenced from two owners.
    BackPointerMismatch,
    /// The entity references a removed or unknown handle.
    StaleReference,
    /// The body has no regions.
    NoRegions,
    /// `regions[0]` (the infinite exterior) is not a void region.
    ExteriorNotVoid,
    /// A solid body has no solid region.
    NoSolidRegion,
    /// Entity content is inconsistent with its body's kind (faces on a
    /// wire body, a sheet body with a solid region, an acorn shell
    /// without its vertex, …).
    KindMismatch,
    /// A loop has no fins.
    EmptyLoop,
    /// A loop's fins do not form a closed ring.
    OpenLoop,
    /// A ring edge appears in a loop with more than one fin.
    RingEdgeInLongRing,
    /// A ring edge's curve is not periodic.
    RingEdgeNotPeriodic,
    /// A vertex-bearing edge has no parameter bounds.
    MissingBounds,
    /// A bounded edge is missing one or both vertices.
    MissingVertices,
    /// An edge has neither exact curve geometry nor a valid tolerant-edge
    /// representation.
    MissingCurve,
    /// Edge bounds are not finite, not increasing, outside the curve's
    /// range, or wider than its period.
    BadBounds,
    /// An edge has the wrong number of fins for its body kind.
    BadFinCount,
    /// The two fins of a manifold edge traverse it in the same direction.
    FinsNotOpposed,
    /// A face with zero loops sits on a surface that is not closed.
    ZeroLoopFaceOnOpenSurface,
    /// A loop winds the wrong way for its position (outer loops run
    /// counterclockwise w.r.t. the face normal; holes clockwise).
    WrongLoopOrientation,
    /// A whole-loop proof found a proper crossing, non-adjacent touch, or
    /// positive-length adjacent overlap.
    LoopSelfIntersection,
    /// An edge endpoint does not lie on the edge's curve at its bound
    /// parameter, within tolerance.
    VertexOffCurve,
    /// A sampled edge point does not lie on an adjacent face's surface,
    /// within tolerance.
    EdgeOffSurface,
    /// A fin's pcurve range or edge-to-pcurve parameter correspondence is
    /// invalid for the referenced 2D/3D curves.
    BadPcurveRange,
    /// A pcurve chart shifts a non-periodic direction or produces invalid
    /// surface parameters.
    BadPcurveChart,
    /// Explicit closed-use winding does not match the pcurve endpoint
    /// displacement or was attached to an open edge.
    BadPcurveClosure,
    /// A pcurve endpoint marked singular does not lie on a declared
    /// degeneracy of its supporting surface.
    BadPcurveSingularity,
    /// An explicit seam role is not on a full-period chart boundary of its
    /// supporting face.
    BadPcurveSeam,
    /// A fin that requires graph-backed surface evaluation has no pcurve representation.
    MissingPcurve,
    /// The supporting surface is proven singular at a required pcurve sample.
    SurfaceSingular,
    /// Supporting-surface metadata or evaluation failed for another typed reason.
    SurfaceEvaluationFailed,
    /// A sampled pcurve point lifted through the face surface does not
    /// coincide with the corresponding 3D edge point within tolerance.
    PcurveOffSurface,
    /// A tolerant pcurve endpoint misses its topological vertex by more
    /// than the edge/vertex tolerance.
    PcurveEndpointOffVertex,
    /// Two lifted fin pcurves of one tolerant edge disagree by more than
    /// the edge tolerance at a common logical parameter.
    PcurvesDisagree,
    /// A coordinate is not finite or lies outside the ±500 m size box.
    OutsideSizeBox,
    /// An entity tolerance is below session resolution or not finite.
    BadTolerance,
    /// A supplied face UV work box is non-finite, empty, outside a
    /// non-periodic surface range, wider than one surface period, or does
    /// not cover the whole surface for a zero-loop closed face.
    BadFaceDomain,
    /// A declared face work box does not contain an actual charted pcurve
    /// endpoint.
    FaceDomainMissesPcurveEndpoint,
    /// The Euler–Poincaré identity fails for a shell (see module docs).
    EulerViolation,
    /// A global shell proof found one or more facet normals pointing into
    /// the material.
    ShellOrientation,
}

/// One checker finding.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fault {
    /// The offending entity.
    pub entity: EntityRef,
    /// The violated invariant.
    pub kind: FaultKind,
}

/// Requested checker assurance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CheckLevel {
    /// Structural checks plus the current bounded sampling checks.
    Fast,
    /// Request proof-complete validation. Any invariant that the current
    /// checker cannot prove on a Fast-clean body is reported as a
    /// [`VerificationGap`]. Invalid bodies return faults without attempting
    /// downstream geometric proofs over inconsistent topology.
    Full,
}

/// Version-1 aggregate budget for all contextual Full-check proof stages.
///
/// Callers depend on this checker-owned aggregate rather than composing leaf
/// proof implementation profiles themselves. New Full-check stages can grow
/// this profile without changing operation call sites.
pub struct FullCheckBudgetProfile;

impl FullCheckBudgetProfile {
    /// Returns the exact defaults for the Full-check proof stages migrated so far.
    pub fn v1_defaults() -> BudgetPlan {
        let face_domain = crate::domain::FaceDomainContainmentBudgetProfile::v1_defaults();
        face_domain
            .overlaid(&crate::shell_proof::shell_proof_budget())
            .overlaid(&crate::planar_shell_proof::planar_shell_proof_budget())
    }
}

/// Version-1 aggregate budget for Fast graph queries plus optional Full proofs.
pub struct CheckBudgetProfile;

impl CheckBudgetProfile {
    /// Defaults for one requested assurance level.
    pub fn v1_defaults(level: CheckLevel) -> BudgetPlan {
        let graph = EvalBudgetProfile::v1_defaults();
        if level == CheckLevel::Full {
            graph.overlaid(&FullCheckBudgetProfile::v1_defaults())
        } else {
            graph
        }
    }
}

/// Overall checker result at the requested assurance level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CheckOutcome {
    /// No fault or missing proof remains at this level.
    Valid,
    /// At least one violated invariant was found.
    Invalid,
    /// No violation was found, but one or more required proofs are absent.
    Indeterminate,
}

/// Proof obligation not yet discharged by checker v2.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum VerificationGapKind {
    /// The supporting surface has not been proven regular over the complete
    /// trimmed face domain.
    SurfaceRegularity,
    /// Edge-to-surface incidence is not covered by a whole-interval
    /// certificate for this representation pair.
    EdgeSurfaceIncidence,
    /// Pcurve/edge/surface incidence is not covered by a whole-interval
    /// certificate for this representation tuple.
    PcurveSurfaceIncidence,
    /// The declared UV domain cannot yet be proven to contain the complete
    /// face boundary.
    FaceDomainContainment,
    /// A loop has not been proven free of self-intersection.
    LoopSelfIntersection,
    /// Relative containment of multiple loops has not been proven.
    LoopContainment,
    /// A loop's orientation has not been proven from its represented boundary.
    LoopOrientation,
    /// A shell has not been proven free of global self-intersection.
    ShellSelfIntersection,
    /// A solid shell's global outward orientation has not been proven.
    ShellOrientation,
    /// A wire body has not been proven free of global self-intersection.
    WireSelfIntersection,
}

/// Why a Full-check proof obligation remains unresolved.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum VerificationGapCause {
    /// The requested proof capability does not cover this representation yet.
    Capability(CapabilityId),
    /// A deterministic resource allowance stopped an otherwise supported proof.
    Limit(LimitSnapshot),
    /// Arithmetic resolution prevented further meaningful progress.
    NumericResolution {
        /// Numerical stage that could not progress.
        stage: kcore::operation::StageId,
    },
}

/// One missing proof attached to the smallest relevant entity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VerificationGap {
    /// Entity whose proof is incomplete.
    pub entity: EntityRef,
    /// Missing proof category.
    pub kind: VerificationGapKind,
    /// Structured reason the obligation remains unresolved, when migrated.
    ///
    /// `None` preserves legacy proof gaps whose stop or capability has not yet
    /// moved to the shared F2/F4 vocabulary; callers must not infer a cause.
    pub cause: Option<VerificationGapCause>,
}

/// Checker findings and proof gaps at one assurance level.
#[derive(Debug, Clone, PartialEq)]
pub struct CheckReport {
    /// Requested assurance level.
    pub level: CheckLevel,
    /// Proven invariant violations.
    pub faults: Vec<Fault>,
    /// Proof obligations that remain unresolved.
    pub gaps: Vec<VerificationGap>,
}

impl CheckReport {
    /// Summarize faults and proof gaps without conflating an unknown result
    /// with success or failure.
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

/// Check a body, returning every fault found (empty = clean).
///
/// Errors only if `body` itself is a stale handle; everything else —
/// including stale references *inside* the body — is reported as faults.
pub fn check_body(store: &Store, body: BodyId) -> Result<Vec<Fault>> {
    Ok(check_body_report(store, body, CheckLevel::Fast)?.faults)
}

/// Check a body at an explicit assurance level.
///
/// A clean [`CheckLevel::Fast`] report means no current structural or
/// sampled fault was found. Only a [`CheckLevel::Full`] report whose outcome
/// is [`CheckOutcome::Valid`] is proof-complete; until all checker-v2
/// obligations land, clean bodies generally return `Indeterminate` at that
/// level.
pub fn check_body_report(store: &Store, body: BodyId, level: CheckLevel) -> Result<CheckReport> {
    let session = SessionPolicy::new(
        SessionPrecision::parasolid(),
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        legacy_check_budget(level),
        PolicyVersion::V1,
    );
    let context = OperationContext::new(&session, Tolerances::default())
        .expect("validated default tolerances satisfy v1 session precision");
    check_body_report_with_context(store, body, level, &context)
        .expect("built-in v1 checker policy is valid")
        .into_result()
}

/// Check a body while retaining deterministic operation accounting.
///
/// Checker family defaults fill graph stages (and Full proof stages when
/// requested) omitted by the caller. Matching
/// session entries override those defaults, and explicit request overrides
/// have final precedence. Fast checking does not install or consume the Full
/// proof budget.
pub fn check_body_report_with_context(
    store: &Store,
    body: BodyId,
    level: CheckLevel,
    context: &OperationContext<'_>,
) -> core::result::Result<OperationOutcome<CheckReport>, OperationPolicyError> {
    let family_context = context
        .clone()
        .with_family_budget_defaults(contextual_check_budget(level));
    let context = &family_context;
    EvalLimits::from_budget_plan(&context.effective_budget())?;
    if level == CheckLevel::Full {
        validate_full_check_budget(context)?;
    }
    let mut scope = OperationScope::new(context);
    let result = check_body_report_in_scope(store, body, level, &mut scope);
    Ok(scope.finish(result))
}

/// Check a body using an existing operation scope.
///
/// This is the composition seam for constructors and higher-level modeling
/// operations once they adopt contextual checking: nested checking borrows
/// the caller's ledger and never creates or resets an operation scope.
pub fn check_body_report_in_scope(
    store: &Store,
    body: BodyId,
    level: CheckLevel,
    scope: &mut OperationScope<'_, '_>,
) -> Result<CheckReport> {
    let b = store.get(body)?;
    let mut graph = GraphQueryWork::reserve(scope, 0).map_err(Error::from)?;
    let fast = check_body_fast_report_with_graph(store, body, b, &mut graph);
    graph.merge(scope).map_err(Error::from)?;
    let report = fast?;
    if level == CheckLevel::Fast {
        return Ok(report);
    }
    complete_full_report_in_scope(store, body, b, report, scope)
}

/// Extend one already-computed Fast-clean report with Full proof evidence.
///
/// Transaction commit uses this seam so structural validation and graph work
/// are performed exactly once before proof-complete acceptance is decided.
pub(crate) fn complete_full_report_in_scope(
    store: &Store,
    body: BodyId,
    body_value: &Body,
    mut report: CheckReport,
    scope: &mut OperationScope<'_, '_>,
) -> Result<CheckReport> {
    debug_assert_eq!(report.level, CheckLevel::Fast);
    let (proof_faults, gaps) = if report.faults.is_empty() {
        collect_full_verification(store, body, body_value, scope)?
    } else {
        (Vec::new(), Vec::new())
    };
    report.level = CheckLevel::Full;
    report.faults.extend(proof_faults);
    report.gaps = gaps;
    Ok(report)
}

pub(crate) fn check_body_fast_report_with_graph(
    store: &Store,
    body: BodyId,
    b: &Body,
    graph: &mut GraphQueryWork,
) -> Result<CheckReport> {
    let mut checker = Checker {
        store,
        // Fast validity remains tied to the kernel's fixed checker tolerance;
        // an operation request may budget work but may not weaken acceptance.
        tol: Tolerances::default(),
        faults: Vec::new(),
        graph,
        policy_error: None,
    };
    checker.run(body, b);
    let policy_error = checker.policy_error.take();
    if let Some(error) = policy_error {
        return Err(error.into());
    }
    Ok(CheckReport {
        level: CheckLevel::Fast,
        faults: checker.faults,
        gaps: Vec::new(),
    })
}

fn contextual_check_budget(level: CheckLevel) -> BudgetPlan {
    CheckBudgetProfile::v1_defaults(level)
}

fn legacy_check_budget(level: CheckLevel) -> BudgetPlan {
    let graph =
        EvalBudgetProfile::for_limits(EvalLimits::default().max_dependency_depth, usize::MAX);
    if level == CheckLevel::Full {
        graph.overlaid(&FullCheckBudgetProfile::v1_defaults())
    } else {
        graph
    }
}

pub(crate) fn validate_full_check_budget(
    context: &OperationContext<'_>,
) -> core::result::Result<(), OperationPolicyError> {
    let plan = context.effective_budget();
    for required in FullCheckBudgetProfile::v1_defaults().limits() {
        plan.require_limit(required.stage, required.resource, required.mode)?;
    }
    Ok(())
}

fn collect_full_verification(
    store: &Store,
    body_id: BodyId,
    body: &Body,
    scope: &mut OperationScope<'_, '_>,
) -> Result<(Vec<Fault>, Vec<VerificationGap>)> {
    let mut faults = Vec::new();
    let mut gaps = Vec::new();
    let mut push = |entity, kind: VerificationGapKind, cause| {
        let gap = VerificationGap {
            entity,
            kind,
            cause,
        };
        if !gaps.contains(&gap) {
            gaps.push(gap);
        }
    };

    for face_id in store.faces_of_body(body_id)? {
        let face = store.get(face_id)?;
        if matches!(store.get(face.surface), Ok(SurfaceGeom::Offset(_))) {
            push(
                EntityRef::Face(face_id),
                VerificationGapKind::SurfaceRegularity,
                None,
            );
        }
        let containment =
            crate::domain::certify_face_domain_containment_in_scope(store, face_id, scope)?;
        if containment.status != crate::domain::FaceDomainContainment::Certified {
            push(
                EntityRef::Face(face_id),
                VerificationGapKind::FaceDomainContainment,
                face_domain_gap_cause(containment),
            );
        }
        let layout = certify_planar_loop_layout(store, &face.loops)?;
        for &loop_id in &face.loops {
            if layout
                .orientations
                .iter()
                .find_map(|(candidate, orientation)| {
                    (*candidate == loop_id).then_some(*orientation)
                })
                .flatten()
                .is_none()
            {
                push(
                    EntityRef::Loop(loop_id),
                    VerificationGapKind::LoopOrientation,
                    None,
                );
            }
            match certify_loop_simplicity(store, loop_id)? {
                LoopSimplicity::Certified => {}
                LoopSimplicity::SelfIntersecting => faults.push(Fault {
                    entity: EntityRef::Loop(loop_id),
                    kind: FaultKind::LoopSelfIntersection,
                }),
                LoopSimplicity::Indeterminate => push(
                    EntityRef::Loop(loop_id),
                    VerificationGapKind::LoopSelfIntersection,
                    None,
                ),
            }
            for &fin_id in &store.get(loop_id)?.fins {
                let fin = store.get(fin_id)?;
                let edge = store.get(fin.edge)?;
                let tolerance = edge
                    .tolerance
                    .map(crate::tolerance::EntityTolerance::value)
                    .unwrap_or(0.0)
                    .max(LINEAR_RESOLUTION);
                if let Some(pcurve) = fin.pcurve {
                    if certify_pcurve_incidence(store, fin.edge, face.surface, pcurve, tolerance)?
                        != IncidenceCertification::Certified
                    {
                        push(
                            EntityRef::Fin(fin_id),
                            VerificationGapKind::PcurveSurfaceIncidence,
                            None,
                        );
                    }
                } else if certify_edge_surface_incidence(store, fin.edge, face.surface, tolerance)?
                    != IncidenceCertification::Certified
                {
                    push(
                        EntityRef::Edge(fin.edge),
                        VerificationGapKind::EdgeSurfaceIncidence,
                        None,
                    );
                }
            }
        }
        if face.loops.len() > 1
            && certify_loop_containment(store, &face.loops)? != LoopContainment::Certified
        {
            push(
                EntityRef::Face(face_id),
                VerificationGapKind::LoopContainment,
                None,
            );
        }
    }

    if body.kind == BodyKind::Wire {
        push(
            EntityRef::Body(body_id),
            VerificationGapKind::WireSelfIntersection,
            None,
        );
    }
    for &region_id in &body.regions {
        let region = store.get(region_id)?;
        for &shell_id in &region.shells {
            let shell = store.get(shell_id)?;
            if !shell.faces.is_empty() {
                let certification =
                    certify_shell_in_scope(store, shell_id, body.kind, region.kind, scope)?;
                if certification.embedding != ShellEmbedding::Certified {
                    push(
                        EntityRef::Shell(shell_id),
                        VerificationGapKind::ShellSelfIntersection,
                        None,
                    );
                }
                if body.kind == BodyKind::Solid {
                    match certification.orientation {
                        ShellOrientation::Certified => {}
                        ShellOrientation::Invalid => faults.push(Fault {
                            entity: EntityRef::Shell(shell_id),
                            kind: FaultKind::ShellOrientation,
                        }),
                        ShellOrientation::Indeterminate => push(
                            EntityRef::Shell(shell_id),
                            VerificationGapKind::ShellOrientation,
                            None,
                        ),
                    }
                }
            }
        }
    }
    Ok((faults, gaps))
}

fn face_domain_gap_cause(
    evidence: crate::domain::FaceDomainContainmentEvidence,
) -> Option<VerificationGapCause> {
    evidence.limit.map(VerificationGapCause::Limit)
}

/// Number of interior samples when verifying an edge lies on its faces.
const EDGE_SAMPLES: usize = 5;

struct Checker<'a, 'graph> {
    store: &'a Store,
    tol: Tolerances,
    faults: Vec<Fault>,
    graph: &'graph mut GraphQueryWork,
    policy_error: Option<OperationPolicyError>,
}

/// One edge reachable from the body, with the faces using it.
struct EdgeUse {
    edge: EdgeId,
    faces: Vec<FaceId>,
    /// True if the edge was found only in a shell's wireframe list.
    wire_only: bool,
}

impl<'a> Checker<'a, '_> {
    fn fault(&mut self, entity: EntityRef, kind: FaultKind) {
        self.faults.push(Fault { entity, kind });
    }

    fn graph_query<T>(
        &mut self,
        tolerances: Tolerances,
        query: impl FnOnce(&mut kgraph::EvalContext<'_>) -> kgraph::EvalResult<T>,
    ) -> Option<kgraph::EvalResult<T>> {
        if self.policy_error.is_some() {
            return None;
        }
        match self
            .graph
            .query_with_tolerances(self.store, tolerances, query)
        {
            Ok(Err(error)) if error.limit().is_some() => {
                self.policy_error = error.limit().map(OperationPolicyError::LimitReached);
                None
            }
            Ok(result) => Some(result),
            Err(error) => {
                self.policy_error = Some(error);
                None
            }
        }
    }

    /// Borrow a live entity or record a stale-reference fault against the
    /// *referring* entity `at`.
    fn live<T: Entity>(&mut self, handle: Handle<T>, at: EntityRef) -> Option<&'a T> {
        match self.store.get(handle) {
            Ok(v) => Some(v),
            Err(_) => {
                self.fault(at, FaultKind::StaleReference);
                None
            }
        }
    }

    fn run(&mut self, body_id: BodyId, body: &Body) {
        // Regions and their kinds.
        if body.regions.is_empty() {
            self.fault(EntityRef::Body(body_id), FaultKind::NoRegions);
        }
        let mut has_solid_region = false;
        let mut shells: Vec<ShellId> = Vec::new();
        for (i, &rid) in body.regions.iter().enumerate() {
            let Some(region) = self.live(rid, EntityRef::Region(rid)) else {
                continue;
            };
            if region.body != body_id {
                self.fault(EntityRef::Region(rid), FaultKind::BackPointerMismatch);
            }
            if i == 0 && region.kind != RegionKind::Void {
                self.fault(EntityRef::Region(rid), FaultKind::ExteriorNotVoid);
            }
            if region.kind == RegionKind::Solid {
                has_solid_region = true;
                if body.kind == BodyKind::Sheet || body.kind == BodyKind::Wire {
                    self.fault(EntityRef::Region(rid), FaultKind::KindMismatch);
                }
            }
            for &sid in &region.shells {
                let Some(shell) = self.live(sid, EntityRef::Shell(sid)) else {
                    continue;
                };
                if shell.region != rid {
                    self.fault(EntityRef::Shell(sid), FaultKind::BackPointerMismatch);
                }
                if shells.contains(&sid) {
                    self.fault(EntityRef::Shell(sid), FaultKind::BackPointerMismatch);
                    continue;
                }
                shells.push(sid);
                self.check_shell_kind(sid, shell, body.kind);
            }
        }
        if body.kind == BodyKind::Solid && !has_solid_region {
            self.fault(EntityRef::Body(body_id), FaultKind::NoSolidRegion);
        }

        // Faces, loops, and the edge/vertex closure.
        let mut seen_faces: Vec<FaceId> = Vec::new();
        let mut seen_loops: Vec<LoopId> = Vec::new();
        let mut seen_fins: Vec<FinId> = Vec::new();
        let mut edge_uses: Vec<EdgeUse> = Vec::new();
        let mut shell_faces: Vec<(ShellId, Vec<FaceId>)> = Vec::new();
        for &sid in &shells {
            let Ok(shell) = self.store.get(sid) else {
                continue;
            };
            let mut faces_here: Vec<FaceId> = Vec::new();
            for &fid in &shell.faces {
                let Some(face) = self.live(fid, EntityRef::Face(fid)) else {
                    continue;
                };
                if seen_faces.contains(&fid) {
                    self.fault(EntityRef::Face(fid), FaultKind::BackPointerMismatch);
                    continue;
                }
                seen_faces.push(fid);
                faces_here.push(fid);
                self.check_face(
                    fid,
                    face,
                    sid,
                    &mut seen_loops,
                    &mut seen_fins,
                    &mut edge_uses,
                );
            }
            for &eid in &shell.edges {
                if self.live(eid, EntityRef::Shell(sid)).is_some()
                    && !edge_uses.iter().any(|u| u.edge == eid)
                {
                    edge_uses.push(EdgeUse {
                        edge: eid,
                        faces: Vec::new(),
                        wire_only: true,
                    });
                }
            }
            shell_faces.push((sid, faces_here));
        }

        // Edges (deduplicated), then vertices (deduplicated).
        let mut vertices: Vec<VertexId> = Vec::new();
        for use_ in &edge_uses {
            self.check_edge(use_, body.kind);
            if let Ok(edge) = self.store.get(use_.edge) {
                for v in edge.vertices.into_iter().flatten() {
                    if !vertices.contains(&v) {
                        vertices.push(v);
                    }
                }
            }
        }
        for &sid in &shells {
            if let Ok(shell) = self.store.get(sid)
                && let Some(v) = shell.vertex
                && !vertices.contains(&v)
            {
                vertices.push(v);
            }
        }
        for &vid in &vertices {
            self.check_vertex(vid);
        }

        // Loop orientation, then the Euler identity per shell.
        for &fid in &seen_faces {
            self.check_face_orientation(fid);
        }
        if body.kind == BodyKind::Solid {
            for (sid, faces) in &shell_faces {
                self.check_euler(*sid, faces);
            }
        }
    }

    /// Kind rules for one shell (see module docs for what v1 enforces).
    fn check_shell_kind(&mut self, sid: ShellId, shell: &crate::entity::Shell, kind: BodyKind) {
        let at = EntityRef::Shell(sid);
        match kind {
            BodyKind::Acorn => {
                if shell.vertex.is_none() || !shell.faces.is_empty() || !shell.edges.is_empty() {
                    self.fault(at, FaultKind::KindMismatch);
                }
            }
            BodyKind::Wire => {
                if shell.vertex.is_some() || !shell.faces.is_empty() {
                    self.fault(at, FaultKind::KindMismatch);
                }
            }
            BodyKind::Sheet => {
                if shell.vertex.is_some() {
                    self.fault(at, FaultKind::KindMismatch);
                }
            }
            BodyKind::Solid => {
                if shell.vertex.is_some() || shell.faces.is_empty() {
                    self.fault(at, FaultKind::KindMismatch);
                }
            }
        }
        if let Some(v) = shell.vertex {
            let _ = self.live(v, at);
        }
    }

    fn check_face(
        &mut self,
        fid: FaceId,
        face: &'a Face,
        sid: ShellId,
        seen_loops: &mut Vec<LoopId>,
        seen_fins: &mut Vec<FinId>,
        edge_uses: &mut Vec<EdgeUse>,
    ) {
        if face.shell != sid {
            self.fault(EntityRef::Face(fid), FaultKind::BackPointerMismatch);
        }
        let surface = self.live(face.surface, EntityRef::Face(fid));
        if let Some(tolerance) = face.tolerance
            && self.tol.entity_tolerance(tolerance.value()).is_err()
        {
            self.fault(EntityRef::Face(fid), FaultKind::BadTolerance);
        }
        if let (Some(domain), Some(_)) = (face.domain, surface) {
            if !valid_face_domain(self, face.surface, domain) {
                self.fault(EntityRef::Face(fid), FaultKind::BadFaceDomain);
            } else if !face_domain_contains_pcurve_endpoints(self, face, domain) {
                self.fault(
                    EntityRef::Face(fid),
                    FaultKind::FaceDomainMissesPcurveEndpoint,
                );
            }
        }
        if face.loops.is_empty()
            && !matches!(
                surface,
                Some(SurfaceGeom::Sphere(_)) | Some(SurfaceGeom::Torus(_)) | None
            )
        {
            // Closed NURBS surfaces are accepted once periodic NURBS land
            // (M3); until then a zero-loop NURBS face is a fault too.
            self.fault(EntityRef::Face(fid), FaultKind::ZeroLoopFaceOnOpenSurface);
        }
        if face.loops.is_empty()
            && let Some(surface @ (SurfaceGeom::Sphere(_) | SurfaceGeom::Torus(_))) = surface
            && face
                .domain
                .is_none_or(|domain| !domain_covers_natural_surface(domain, surface))
        {
            self.fault(EntityRef::Face(fid), FaultKind::BadFaceDomain);
        }
        for &lid in &face.loops {
            let Some(lp) = self.live(lid, EntityRef::Face(fid)) else {
                continue;
            };
            if lp.face != fid {
                self.fault(EntityRef::Loop(lid), FaultKind::BackPointerMismatch);
            }
            if seen_loops.contains(&lid) {
                self.fault(EntityRef::Loop(lid), FaultKind::BackPointerMismatch);
                continue;
            }
            seen_loops.push(lid);
            self.check_loop(lid, lp, fid, seen_fins, edge_uses);
        }
    }

    fn check_loop(
        &mut self,
        lid: LoopId,
        lp: &'a crate::entity::Loop,
        fid: FaceId,
        seen_fins: &mut Vec<FinId>,
        edge_uses: &mut Vec<EdgeUse>,
    ) {
        if lp.fins.is_empty() {
            self.fault(EntityRef::Loop(lid), FaultKind::EmptyLoop);
            return;
        }
        // (tail, head) per fin; None where the fin or its edge is unusable.
        let mut ends: Vec<Option<(Option<VertexId>, Option<VertexId>)>> = Vec::new();
        let mut ring_in_long_ring = false;
        for &fin_id in &lp.fins {
            let Some(fin) = self.live(fin_id, EntityRef::Loop(lid)) else {
                ends.push(None);
                continue;
            };
            if fin.parent != lid {
                self.fault(EntityRef::Fin(fin_id), FaultKind::BackPointerMismatch);
            }
            if seen_fins.contains(&fin_id) {
                self.fault(EntityRef::Fin(fin_id), FaultKind::BackPointerMismatch);
            } else {
                seen_fins.push(fin_id);
            }
            let Some(edge) = self.live(fin.edge, EntityRef::Fin(fin_id)) else {
                ends.push(None);
                continue;
            };
            if !edge.fins.contains(&fin_id) {
                self.fault(EntityRef::Fin(fin_id), FaultKind::BackPointerMismatch);
            }
            self.check_fin_pcurve(fin_id, fin, edge, fid);
            match edge_uses.iter_mut().find(|u| u.edge == fin.edge) {
                Some(use_) => {
                    use_.wire_only = false;
                    if !use_.faces.contains(&fid) {
                        use_.faces.push(fid);
                    }
                }
                None => edge_uses.push(EdgeUse {
                    edge: fin.edge,
                    faces: vec![fid],
                    wire_only: false,
                }),
            }
            let is_ring = edge.bounds.is_none() && edge.vertices == [None, None];
            if is_ring && lp.fins.len() > 1 {
                ring_in_long_ring = true;
            }
            let (tail, head) = if fin.sense.is_forward() {
                (edge.vertices[0], edge.vertices[1])
            } else {
                (edge.vertices[1], edge.vertices[0])
            };
            ends.push(Some((tail, head)));
        }
        if ring_in_long_ring {
            self.fault(EntityRef::Loop(lid), FaultKind::RingEdgeInLongRing);
            return;
        }
        // Ring closure.
        if lp.fins.len() == 1 {
            if let Some(Some((tail, head))) = ends.first() {
                let ring = tail.is_none() && head.is_none();
                let closed_through_vertex = tail.is_some() && tail == head;
                if !ring && !closed_through_vertex {
                    self.fault(EntityRef::Loop(lid), FaultKind::OpenLoop);
                }
            }
            return;
        }
        for i in 0..ends.len() {
            let next = (i + 1) % ends.len();
            let (Some((_, head)), Some((tail, _))) = (&ends[i], &ends[next]) else {
                continue;
            };
            if head.is_none() || *head != *tail {
                self.fault(EntityRef::Loop(lid), FaultKind::OpenLoop);
                return;
            }
        }
    }

    /// Validate the full incidence tuple `(edge curve, pcurve, surface)`.
    /// Missing pcurves remain accepted for exact legacy leaf topology. A
    /// curve-less tolerant edge and every procedural face use require one.
    fn check_fin_pcurve(&mut self, fin_id: FinId, fin: &Fin, edge: &Edge, fid: FaceId) {
        let Some(pcurve_use) = fin.pcurve else {
            let procedural = self
                .store
                .get(fid)
                .and_then(|face| self.store.get(face.surface))
                .is_ok_and(|surface| surface.as_leaf_surface().is_none());
            if edge.curve.is_none() || procedural {
                self.fault(EntityRef::Fin(fin_id), FaultKind::MissingPcurve);
            }
            return;
        };
        let at = EntityRef::Fin(fin_id);
        let Ok(face) = self.store.get(fid) else {
            self.fault(at, FaultKind::StaleReference);
            return;
        };

        // Structural pcurve obligations are independent of whether surface
        // evaluation can be classified. Prove or fault them before any
        // procedural regularity precheck can return Indeterminate.
        let mut queries = ContextualGraphQueries::new(self.graph, self.tol);
        let chart =
            check_pcurve_chart_contextual(self.store, face.surface, pcurve_use, &mut queries);
        if !self.record_contextual_pcurve(at, chart) {
            return;
        }
        if edge.bounds.is_some()
            && let Err(issue) = check_pcurve_parameterization(self.store, edge.bounds, pcurve_use)
        {
            self.record_pcurve_issue(at, issue);
            return;
        }
        let mut queries = ContextualGraphQueries::new(self.graph, self.tol);
        let metadata = check_pcurve_metadata_contextual(
            self.store,
            edge,
            face.surface,
            face.domain,
            pcurve_use,
            &mut queries,
        );
        if !self.record_contextual_pcurve(at, metadata) {
            return;
        }

        if self
            .store
            .get(face.surface)
            .is_ok_and(|surface| surface.as_leaf_surface().is_none())
            && !self.check_procedural_surface_samples(at, edge, face.surface, pcurve_use)
        {
            return;
        }

        let Some(curve) = edge.curve else {
            return;
        };
        let mut queries = ContextualGraphQueries::new(self.graph, self.tol);
        let result = check_pcurve_incidence_contextual(
            self.store,
            curve,
            edge.bounds,
            face.surface,
            pcurve_use,
            edge.tolerance
                .map(crate::tolerance::EntityTolerance::value)
                .unwrap_or(0.0)
                .max(self.tol.linear()),
            &mut queries,
        );
        self.record_contextual_pcurve(at, result);
    }

    fn record_contextual_pcurve(
        &mut self,
        at: EntityRef,
        result: core::result::Result<(), ContextualPcurveError>,
    ) -> bool {
        match result {
            Ok(()) => true,
            Err(ContextualPcurveError::Issue(issue)) => {
                self.record_pcurve_issue(at, issue);
                false
            }
            Err(ContextualPcurveError::Policy(error)) => {
                if self.policy_error.is_none() {
                    self.policy_error = Some(error);
                }
                false
            }
        }
    }

    fn record_pcurve_issue(&mut self, at: EntityRef, issue: PcurveIssue) {
        match issue {
            PcurveIssue::StaleReference => self.fault(at, FaultKind::StaleReference),
            PcurveIssue::BadRange => self.fault(at, FaultKind::BadPcurveRange),
            PcurveIssue::BadChart => self.fault(at, FaultKind::BadPcurveChart),
            PcurveIssue::BadClosure => self.fault(at, FaultKind::BadPcurveClosure),
            PcurveIssue::BadSingularity => {
                self.fault(at, FaultKind::BadPcurveSingularity);
            }
            PcurveIssue::BadSeam => self.fault(at, FaultKind::BadPcurveSeam),
            PcurveIssue::OffSurface => self.fault(at, FaultKind::PcurveOffSurface),
        }
    }

    fn check_procedural_surface_samples(
        &mut self,
        at: EntityRef,
        edge: &Edge,
        surface: crate::entity::SurfaceId,
        pcurve_use: crate::entity::FinPcurve,
    ) -> bool {
        let Some(periods_result) =
            self.graph_query(self.tol, |eval| eval.surface_periodicity(surface))
        else {
            return false;
        };
        let periods = match periods_result {
            Ok(periods) => periods,
            Err(error) => {
                self.fault(
                    at,
                    surface_eval_fault(&error).unwrap_or(FaultKind::SurfaceEvaluationFailed),
                );
                return false;
            }
        };
        let Ok(pcurve) = self.store.get(pcurve_use.curve()) else {
            // The immediately following legacy validation reports StaleReference.
            return true;
        };
        let range = match edge.bounds {
            Some((lo, hi)) if lo.is_finite() && hi.is_finite() && lo < hi => (lo, hi),
            None => {
                let Some(curve) = edge.curve.and_then(|curve| self.store.get(curve).ok()) else {
                    // Legacy edge/pcurve validation reports the missing or stale curve.
                    return true;
                };
                let range = curve.as_curve().param_range();
                if !range.is_finite() || range.lo >= range.hi {
                    // Legacy incidence validation reports BadRange.
                    return true;
                }
                (range.lo, range.hi)
            }
            Some(_) => {
                // `check_edge` and legacy incidence validation report BadBounds/BadRange.
                return true;
            }
        };
        for index in 0..EDGE_SAMPLES {
            let t = range.0 + (range.1 - range.0) * index as f64 / (EDGE_SAMPLES - 1) as f64;
            let Ok(uv) = pcurve_use.evaluate_uv(pcurve.as_curve(), t, periods) else {
                // The immediately following legacy validation reports BadChart/BadRange.
                return true;
            };
            let Some(result) = self.graph_query(self.tol, |eval| {
                eval.eval_surface(surface, [uv.x, uv.y], SurfaceDerivativeOrder::Position)
            }) else {
                return false;
            };
            if let Err(error) = result {
                if let Some(kind) = surface_eval_fault(&error) {
                    self.fault(at, kind);
                }
                // Ill-conditioning is not a proven violation. Full checking
                // reports the face's SurfaceRegularity obligation, while
                // graph-backed tessellation refuses to produce a partial mesh.
                return false;
            }
        }
        true
    }

    fn check_edge(&mut self, use_: &EdgeUse, kind: BodyKind) {
        let eid = use_.edge;
        let at = EntityRef::Edge(eid);
        let Ok(edge) = self.store.get(eid) else {
            // The stale reference was already recorded where it was found.
            return;
        };
        if let Some(t) = edge.tolerance
            && self.tol.entity_tolerance(t.value()).is_err()
        {
            self.fault(at, FaultKind::BadTolerance);
        }
        let tolerant_curveless = edge.curve.is_none() && edge.tolerance.is_some();
        let curve = match edge.curve {
            None if !tolerant_curveless => {
                self.fault(at, FaultKind::MissingCurve);
                None
            }
            None => None,
            Some(c) => self.live(c, at),
        };

        // Bounds / vertices / ring classification.
        let mut bounds_ok = false;
        match (edge.bounds, edge.vertices) {
            (None, [None, None]) => {
                if let Some(g) = curve
                    && g.as_curve().periodicity().is_none()
                {
                    self.fault(at, FaultKind::RingEdgeNotPeriodic);
                }
            }
            (None, _) => self.fault(at, FaultKind::MissingBounds),
            (Some((t0, t1)), verts) => {
                if !t0.is_finite() || !t1.is_finite() || t0 >= t1 {
                    self.fault(at, FaultKind::BadBounds);
                } else if let Some(g) = curve {
                    let c = g.as_curve();
                    let in_range = match c.periodicity() {
                        Some(period) => t1 - t0 <= period,
                        None => c.param_range().contains(t0) && c.param_range().contains(t1),
                    };
                    if in_range {
                        bounds_ok = true;
                    } else {
                        self.fault(at, FaultKind::BadBounds);
                    }
                } else if tolerant_curveless {
                    bounds_ok = true;
                }
                if verts[0].is_none() || verts[1].is_none() {
                    self.fault(at, FaultKind::MissingVertices);
                }
            }
        }

        // Fin back-pointers, count, and opposed traversal.
        for &fin_id in &edge.fins {
            let Some(fin) = self.live(fin_id, at) else {
                continue;
            };
            if fin.edge != eid {
                self.fault(EntityRef::Fin(fin_id), FaultKind::BackPointerMismatch);
            }
            match self.store.get(fin.parent) {
                Ok(lp) => {
                    if !lp.fins.contains(&fin_id) {
                        self.fault(EntityRef::Fin(fin_id), FaultKind::BackPointerMismatch);
                    }
                }
                Err(_) => self.fault(EntityRef::Fin(fin_id), FaultKind::StaleReference),
            }
        }
        if !use_.wire_only {
            let count_ok = match kind {
                BodyKind::Solid => edge.fins.len() == 2,
                _ => (1..=2).contains(&edge.fins.len()),
            };
            if !count_ok {
                self.fault(at, FaultKind::BadFinCount);
            }
        }
        if edge.fins.len() == 2
            && let (Ok(a), Ok(b)) = (self.store.get(edge.fins[0]), self.store.get(edge.fins[1]))
            && a.sense == b.sense
        {
            self.fault(at, FaultKind::FinsNotOpposed);
        }
        self.check_seam_pairs(edge);

        if tolerant_curveless {
            self.check_tolerant_edge_geometry(eid, edge, bounds_ok);
            return;
        }

        // Geometry: endpoints on the curve, samples on adjacent surfaces,
        // and the size box.
        let Some(g) = curve else {
            return;
        };
        let c = g.as_curve();
        let edge_tol = edge
            .tolerance
            .map(crate::tolerance::EntityTolerance::value)
            .unwrap_or(0.0)
            .max(LINEAR_RESOLUTION);
        if bounds_ok {
            let (t0, t1) = edge.bounds.expect("bounds_ok implies Some");
            for (vh, t) in [(edge.vertices[0], t0), (edge.vertices[1], t1)] {
                let Some(vid) = vh else { continue };
                let Ok(pos) = self.store.vertex_position(vid) else {
                    continue;
                };
                let vtol = self
                    .store
                    .get(vid)
                    .ok()
                    .and_then(|v| v.tolerance)
                    .map(crate::tolerance::EntityTolerance::value)
                    .unwrap_or(0.0);
                if (c.eval(t) - pos).norm() > edge_tol.max(vtol) {
                    self.fault(at, FaultKind::VertexOffCurve);
                }
            }
        }
        let window = match (edge.bounds, edge.vertices) {
            (Some((t0, t1)), _) if bounds_ok => Some((t0, t1)),
            (None, [None, None]) => {
                let r = c.param_range();
                r.is_finite().then_some((r.lo, r.hi))
            }
            _ => None,
        };
        let Some((a, b)) = window else {
            return;
        };
        let mut boxed = true;
        let mut off_faces: Vec<FaceId> = Vec::new();
        for i in 0..EDGE_SAMPLES {
            let t = a + (b - a) * (i as f64) / ((EDGE_SAMPLES - 1) as f64);
            let p = c.eval(t);
            if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite())
                || p.x.abs() > SIZE_BOX_HALF
                || p.y.abs() > SIZE_BOX_HALF
                || p.z.abs() > SIZE_BOX_HALF
            {
                if boxed {
                    self.fault(at, FaultKind::OutsideSizeBox);
                    boxed = false;
                }
                continue;
            }
            for &fid in &use_.faces {
                if off_faces.contains(&fid) {
                    continue;
                }
                let Ok(face) = self.store.get(fid) else {
                    continue;
                };
                let Ok(sg) = self.store.get(face.surface) else {
                    continue;
                };
                match sg.as_leaf_surface() {
                    Some(surface) => {
                        if let Ok(distance) = distance_to_surface(surface, p)
                            && distance.distance > edge_tol
                        {
                            self.fault(at, FaultKind::EdgeOffSurface);
                            off_faces.push(fid);
                        }
                    }
                    None => {
                        // Procedural incidence is checked from each mandatory
                        // fin pcurve in `check_fin_pcurve`; 3D inversion is not
                        // a certified fallback for graph surfaces.
                    }
                }
            }
        }
    }

    fn check_seam_pairs(&mut self, edge: &Edge) {
        for &fin_id in &edge.fins {
            let Ok(fin) = self.store.get(fin_id) else {
                continue;
            };
            let Some(seam) = fin.pcurve.and_then(|use_| use_.seam()) else {
                continue;
            };
            let Ok(loop_) = self.store.get(fin.parent) else {
                continue;
            };
            let paired = edge.fins.iter().copied().any(|other_id| {
                if other_id == fin_id {
                    return false;
                }
                let Ok(other) = self.store.get(other_id) else {
                    return false;
                };
                let Ok(other_loop) = self.store.get(other.parent) else {
                    return false;
                };
                let Some(other_seam) = other.pcurve.and_then(|use_| use_.seam()) else {
                    return false;
                };
                other_loop.face == loop_.face
                    && other_seam.direction() == seam.direction()
                    && matches!(
                        (seam.side(), other_seam.side()),
                        (SeamSide::Lower, SeamSide::Upper) | (SeamSide::Upper, SeamSide::Lower)
                    )
            });
            if !paired {
                self.fault(EntityRef::Fin(fin_id), FaultKind::BadPcurveSeam);
            }
        }
    }

    fn check_tolerant_edge_geometry(&mut self, eid: EdgeId, edge: &Edge, bounds_ok: bool) {
        if !bounds_ok {
            return;
        }
        let Some((t0, t1)) = edge.bounds else {
            return;
        };
        let edge_tol = edge
            .tolerance
            .map(crate::tolerance::EntityTolerance::value)
            .unwrap_or(0.0)
            .max(LINEAR_RESOLUTION);
        let mut uses = Vec::new();
        for &fin_id in &edge.fins {
            let Ok(fin) = self.store.get(fin_id) else {
                continue;
            };
            let Some(pcurve_use) = fin.pcurve else {
                continue;
            };
            let Ok(lp) = self.store.get(fin.parent) else {
                continue;
            };
            let Ok(face) = self.store.get(lp.face) else {
                continue;
            };
            let (Ok(pcurve), Ok(_surface)) = (
                self.store.get(pcurve_use.curve()),
                self.store.get(face.surface),
            ) else {
                continue;
            };
            let Some(periods_result) =
                self.graph_query(self.tol, |eval| eval.surface_periodicity(face.surface))
            else {
                continue;
            };
            let periods = match periods_result {
                Ok(periods) => periods,
                Err(error) => {
                    if let Some(kind) = surface_eval_fault(&error) {
                        self.fault(EntityRef::Fin(fin_id), kind);
                    }
                    continue;
                }
            };
            uses.push((fin_id, pcurve_use, pcurve.as_curve(), face.surface, periods));
        }

        let mut endpoint_faulted = Vec::new();
        let mut surface_faulted = Vec::new();
        let mut disagreement_faulted = false;
        let mut boxed = true;
        for i in 0..EDGE_SAMPLES {
            let t = t0 + (t1 - t0) * (i as f64) / ((EDGE_SAMPLES - 1) as f64);
            let mut reference = None;
            for &(fin_id, pcurve_use, pcurve, surface, periods) in &uses {
                if surface_faulted.contains(&fin_id) {
                    continue;
                }
                let Ok(uv) = pcurve_use.evaluate_uv(pcurve, t, periods) else {
                    continue;
                };
                let Some(evaluation) = self.graph_query(self.tol, |eval| {
                    eval.eval_surface(
                        surface,
                        [uv.x, uv.y],
                        kgraph::SurfaceDerivativeOrder::Position,
                    )
                }) else {
                    continue;
                };
                let value = match evaluation {
                    Ok(value) => value,
                    Err(error) => {
                        if let Some(kind) = surface_eval_fault(&error) {
                            self.fault(EntityRef::Fin(fin_id), kind);
                        }
                        surface_faulted.push(fin_id);
                        continue;
                    }
                };
                let p = value.p;
                if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite())
                    || p.x.abs() > SIZE_BOX_HALF
                    || p.y.abs() > SIZE_BOX_HALF
                    || p.z.abs() > SIZE_BOX_HALF
                {
                    if boxed {
                        self.fault(EntityRef::Edge(eid), FaultKind::OutsideSizeBox);
                        boxed = false;
                    }
                    continue;
                }
                if let Some(other) = reference
                    && p.dist(other) > edge_tol
                    && !disagreement_faulted
                {
                    self.fault(EntityRef::Edge(eid), FaultKind::PcurvesDisagree);
                    disagreement_faulted = true;
                }
                reference = Some(reference.unwrap_or(p));

                let vertex = if i == 0 {
                    edge.vertices[0]
                } else if i == EDGE_SAMPLES - 1 {
                    edge.vertices[1]
                } else {
                    None
                };
                if let Some(vertex) = vertex
                    && !endpoint_faulted.contains(&fin_id)
                    && let Ok(position) = self.store.vertex_position(vertex)
                {
                    let vertex_tol = self
                        .store
                        .get(vertex)
                        .ok()
                        .and_then(|v| v.tolerance)
                        .map(crate::tolerance::EntityTolerance::value)
                        .unwrap_or(0.0);
                    if p.dist(position) > edge_tol.max(vertex_tol) {
                        self.fault(EntityRef::Fin(fin_id), FaultKind::PcurveEndpointOffVertex);
                        endpoint_faulted.push(fin_id);
                    }
                }
            }
        }
    }

    fn check_vertex(&mut self, vid: VertexId) {
        let at = EntityRef::Vertex(vid);
        let Ok(vertex) = self.store.get(vid) else {
            return;
        };
        if let Some(t) = vertex.tolerance
            && self.tol.entity_tolerance(t.value()).is_err()
        {
            self.fault(at, FaultKind::BadTolerance);
        }
        let Some(&p) = self.live(vertex.point, at) else {
            return;
        };
        if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite())
            || p.x.abs() > SIZE_BOX_HALF
            || p.y.abs() > SIZE_BOX_HALF
            || p.z.abs() > SIZE_BOX_HALF
        {
            self.fault(at, FaultKind::OutsideSizeBox);
        }
    }

    /// Exact loop-orientation check for one face.
    ///
    /// Only strictly closed planar straight-loop layouts can emit faults.
    /// Unsupported representations remain silent at Fast and become explicit
    /// Full verification gaps.
    fn check_face_orientation(&mut self, fid: FaceId) {
        let Ok(face) = self.store.get(fid) else {
            return;
        };
        let Ok(layout) = certify_planar_loop_layout(self.store, &face.loops) else {
            return;
        };
        let Some(outer) = layout.outer else {
            return;
        };
        for (lid, orientation) in layout.orientations {
            let Some(orientation) = orientation else {
                return;
            };
            // Counterclockwise in UV = counterclockwise around the
            // *surface* normal; the face normal flips with face.sense. XT
            // does not require the outer loop to be first in the face's loop
            // chain, so the exact containment proof supplies its identity.
            let expect_positive = (lid == outer) == face.sense.is_forward();
            if (orientation == kcore::predicates::Orientation::Positive) != expect_positive {
                self.fault(EntityRef::Loop(lid), FaultKind::WrongLoopOrientation);
            }
        }
    }

    /// The Euler–Poincaré identity for one shell (see module docs).
    fn check_euler(&mut self, sid: ShellId, faces: &[FaceId]) {
        if faces.is_empty() {
            return;
        }
        let mut chi: i64 = 0;
        let mut edges: Vec<EdgeId> = Vec::new();
        let mut verts: Vec<VertexId> = Vec::new();
        for &fid in faces {
            let Ok(face) = self.store.get(fid) else {
                return;
            };
            if face.loops.is_empty() {
                match self.store.get(face.surface) {
                    Ok(SurfaceGeom::Sphere(_)) => chi += 2,
                    Ok(SurfaceGeom::Torus(_)) => chi += 0,
                    // Unclassifiable face: exempt the shell (v1 limit).
                    _ => return,
                }
                continue;
            }
            chi += 2 - face.loops.len() as i64;
            for &lid in &face.loops {
                let Ok(lp) = self.store.get(lid) else { return };
                for &fin_id in &lp.fins {
                    let Ok(fin) = self.store.get(fin_id) else {
                        return;
                    };
                    if !edges.contains(&fin.edge) {
                        edges.push(fin.edge);
                    }
                }
            }
        }
        for &eid in &edges {
            let Ok(edge) = self.store.get(eid) else {
                return;
            };
            if edge.bounds.is_some() {
                chi -= 1; // interval edge; ring edges (circles) contribute 0
            }
            for v in edge.vertices.into_iter().flatten() {
                if !verts.contains(&v) {
                    verts.push(v);
                }
            }
        }
        chi += verts.len() as i64;
        // Closed orientable boundary: χ = 2 − 2G, G ≥ 0.
        if chi > 2 || chi.rem_euclid(2) != 0 {
            self.fault(EntityRef::Shell(sid), FaultKind::EulerViolation);
        }
    }
}

fn surface_eval_fault(error: &EvalError) -> Option<FaultKind> {
    match error {
        EvalError::SingularSurface { .. } => Some(FaultKind::SurfaceSingular),
        EvalError::IllConditionedSurface { .. } => None,
        _ => Some(FaultKind::SurfaceEvaluationFailed),
    }
}

fn valid_face_domain(
    checker: &mut Checker<'_, '_>,
    surface: crate::entity::SurfaceId,
    domain: crate::entity::FaceDomain,
) -> bool {
    let ranges = [domain.u, domain.v];
    if ranges
        .iter()
        .any(|range| !range.is_finite() || !range.width().is_finite() || range.width() <= 0.0)
    {
        return false;
    }
    let Some(natural) = checker.graph_query(checker.tol, |eval| eval.surface_param_range(surface))
    else {
        return false;
    };
    let Ok(natural) = natural else {
        return false;
    };
    let Some(periodicity) =
        checker.graph_query(checker.tol, |eval| eval.surface_periodicity(surface))
    else {
        return false;
    };
    let Ok(periodicity) = periodicity else {
        return false;
    };
    ranges
        .into_iter()
        .zip(natural)
        .zip(periodicity)
        .all(|((domain, natural), period)| valid_domain_range(domain, natural, period))
}

fn face_domain_contains_pcurve_endpoints(
    checker: &mut Checker<'_, '_>,
    face: &crate::entity::Face,
    domain: crate::entity::FaceDomain,
) -> bool {
    let Some(periods) =
        checker.graph_query(checker.tol, |eval| eval.surface_periodicity(face.surface))
    else {
        return false;
    };
    let Ok(periods) = periods else {
        return false;
    };
    for &loop_id in &face.loops {
        let Ok(loop_) = checker.store.get(loop_id) else {
            continue;
        };
        for &fin_id in &loop_.fins {
            let Ok(fin) = checker.store.get(fin_id) else {
                continue;
            };
            let Some(use_) = fin.pcurve else { continue };
            let Ok(curve) = checker.store.get(use_.curve()) else {
                continue;
            };
            for q in [use_.range().lo, use_.range().hi] {
                let Ok(uv) = use_.chart().apply(curve.as_curve().eval(q), periods) else {
                    continue;
                };
                if !domain_contains_uv(domain, [uv.x, uv.y]) {
                    return false;
                }
            }
        }
    }
    true
}

fn domain_contains_uv(domain: crate::entity::FaceDomain, uv: [f64; 2]) -> bool {
    range_contains_value(domain.u, uv[0]) && range_contains_value(domain.v, uv[1])
}

fn range_contains_value(range: ParamRange, value: f64) -> bool {
    let epsilon =
        256.0 * f64::EPSILON * (1.0 + range.lo.abs().max(range.hi.abs()).max(value.abs()));
    value >= range.lo - epsilon && value <= range.hi + epsilon
}

fn valid_domain_range(domain: ParamRange, natural: ParamRange, period: Option<f64>) -> bool {
    let scale = domain
        .lo
        .abs()
        .max(domain.hi.abs())
        .max(natural.lo.abs().min(1.0e6))
        .max(natural.hi.abs().min(1.0e6))
        .max(1.0);
    let epsilon = 128.0 * f64::EPSILON * scale;
    match period {
        Some(period) => domain.width() <= period + epsilon,
        None => {
            (!natural.lo.is_finite() || domain.lo >= natural.lo - epsilon)
                && (!natural.hi.is_finite() || domain.hi <= natural.hi + epsilon)
        }
    }
}

fn domain_covers_natural_surface(domain: crate::entity::FaceDomain, surface: &SurfaceGeom) -> bool {
    let Some(surface) = surface.as_leaf_surface() else {
        return false;
    };
    let natural = surface.param_range();
    let periodicity = surface.periodicity();
    [domain.u, domain.v]
        .into_iter()
        .zip(natural)
        .zip(periodicity)
        .all(|((domain, natural), period)| {
            let scale = natural.lo.abs().max(natural.hi.abs()).max(1.0);
            let epsilon = 128.0 * f64::EPSILON * scale;
            match period {
                Some(period) => (domain.width() - period).abs() <= epsilon,
                None => {
                    (domain.lo - natural.lo).abs() <= epsilon
                        && (domain.hi - natural.hi).abs() <= epsilon
                }
            }
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::entity::{
        Body, Edge, Face, FaceDomain, Fin, FinPcurve, Loop, ParamMap1d, Region, Sense, Shell,
        Vertex,
    };
    use crate::geom::{Curve2dGeom, CurveGeom};
    use crate::make::{
        block, cone, cylinder, cylindrical_sheet, planar_sheet, solid_body_scaffold,
        sphere as make_sphere, torus as make_torus,
    };
    use kcore::operation::{AccountingMode, ResourceKind};
    use kgeom::curve::{Circle, Line};
    use kgeom::curve2d::NurbsCurve2d;
    use kgeom::frame::Frame;
    use kgeom::nurbs::NurbsCurve;
    use kgeom::param::ParamRange;
    use kgeom::surface::{Cylinder, Plane, Sphere};
    use kgeom::vec::{Point2, Point3, Vec3};
    use kgraph::OffsetSurfaceDescriptor;

    fn checker_session(default_budget: kcore::operation::BudgetPlan) -> SessionPolicy {
        SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            default_budget,
            PolicyVersion::V1,
        )
    }

    fn kinds(faults: &[Fault]) -> Vec<FaultKind> {
        faults.iter().map(|f| f.kind).collect()
    }

    fn assert_has(faults: &[Fault], kind: FaultKind) {
        assert!(
            faults.iter().any(|f| f.kind == kind),
            "expected {kind:?} in {:?}",
            kinds(faults)
        );
    }

    fn clean_block(store: &mut Store) -> BodyId {
        block(store, &Frame::world(), [2.0, 3.0, 4.0]).unwrap()
    }

    fn zero_offset_sheet(store: &mut Store) -> BodyId {
        let body = planar_sheet(
            store,
            &Frame::world(),
            &[
                Point2::new(-1.0, -1.0),
                Point2::new(1.0, -1.0),
                Point2::new(1.0, 1.0),
                Point2::new(-1.0, 1.0),
            ],
        )
        .unwrap();
        let face = store.faces_of_body(body).unwrap()[0];
        let basis = store.get(face).unwrap().surface;
        let offset = store
            .insert_surface(OffsetSurfaceDescriptor::new(basis, 0.0).into())
            .unwrap();
        store.get_mut(face).unwrap().surface = offset;
        body
    }

    fn adaptive_domain_sheet(store: &mut Store) -> (BodyId, FaceId) {
        let body = planar_sheet(
            store,
            &Frame::world(),
            &[
                Point2::new(0.0, 3.0),
                Point2::new(4.0, 3.0),
                Point2::new(4.0, 6.0),
                Point2::new(0.0, 6.0),
            ],
        )
        .unwrap();
        let face = store.faces_of_body(body).unwrap()[0];
        store.get_mut(face).unwrap().domain =
            Some(FaceDomain::from_bounds(0.0, 4.0, 0.0, 6.0).unwrap());
        let loop_id = store.get(face).unwrap().loops[0];
        let fin_id = store.get(loop_id).unwrap().fins[0];
        let edge_id = store.get(fin_id).unwrap().edge;
        let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let uv_points = vec![
            Point2::new(0.0, 3.0),
            Point2::new(4.0 / 3.0, 9.0),
            Point2::new(8.0 / 3.0, -3.0),
            Point2::new(4.0, 3.0),
        ];
        let xyz_points = uv_points
            .iter()
            .map(|point| Point3::new(point.x, point.y, 0.0))
            .collect();
        let edge_curve = store
            .insert_curve(CurveGeom::Nurbs(
                NurbsCurve::new(3, knots.clone(), xyz_points, None).unwrap(),
            ))
            .unwrap();
        let pcurve = store
            .insert_pcurve(Curve2dGeom::Nurbs(
                NurbsCurve2d::new(3, knots, uv_points, None).unwrap(),
            ))
            .unwrap();
        store.get_mut(edge_id).unwrap().curve = Some(edge_curve);
        store.get_mut(edge_id).unwrap().bounds = Some((0.0, 1.0));
        store.get_mut(fin_id).unwrap().pcurve = Some(
            FinPcurve::new(pcurve, ParamRange::new(0.0, 1.0), ParamMap1d::identity()).unwrap(),
        );
        assert!(check_body(store, body).unwrap().is_empty());
        (body, face)
    }

    /// Hand-built solid cylinder r=1, h=2: two ring edges, a two-loop side
    /// face, two single-ring-fin caps. Exercises every ring convention.
    fn cylinder_body(store: &mut Store) -> BodyId {
        let (body, shell) = solid_body_scaffold(store);
        let z = Vec3::new(0.0, 0.0, 1.0);
        let x = Vec3::new(1.0, 0.0, 0.0);
        let bot_frame = Frame::world();
        let top_frame = Frame::new(Point3::new(0.0, 0.0, 2.0), z, x).unwrap();

        let bot_curve = store
            .insert_curve(CurveGeom::Circle(Circle::new(bot_frame, 1.0).unwrap()))
            .unwrap();
        let top_curve = store
            .insert_curve(CurveGeom::Circle(Circle::new(top_frame, 1.0).unwrap()))
            .unwrap();
        let ring_edge = |store: &mut Store, curve| {
            store.add(Edge {
                curve: Some(curve),
                vertices: [None, None],
                bounds: None,
                fins: Vec::new(),
                tolerance: None,
            })
        };
        let e_bot = ring_edge(store, bot_curve);
        let e_top = ring_edge(store, top_curve);

        let side_surf = store
            .insert_surface(SurfaceGeom::Cylinder(
                Cylinder::new(Frame::world(), 1.0).unwrap(),
            ))
            .unwrap();
        let bot_surf = store
            .insert_surface(SurfaceGeom::Plane(Plane::new(
                Frame::new(Point3::new(0.0, 0.0, 0.0), -z, x).unwrap(),
            )))
            .unwrap();
        let top_surf = store
            .insert_surface(SurfaceGeom::Plane(Plane::new(
                Frame::new(Point3::new(0.0, 0.0, 2.0), z, x).unwrap(),
            )))
            .unwrap();

        let face_with_ring_loops = |store: &mut Store, surface, rings: &[(EdgeId, Sense)]| {
            let face = store.add(Face {
                shell,
                loops: Vec::new(),
                surface,
                sense: Sense::Forward,
                domain: None,
                tolerance: None,
            });
            store.get_mut(shell).unwrap().faces.push(face);
            for &(edge, sense) in rings {
                let lp = store.add(Loop {
                    face,
                    fins: Vec::new(),
                });
                store.get_mut(face).unwrap().loops.push(lp);
                let fin = store.add(Fin {
                    parent: lp,
                    edge,
                    sense,
                    pcurve: None,
                });
                store.get_mut(lp).unwrap().fins.push(fin);
                store.get_mut(edge).unwrap().fins.push(fin);
            }
            face
        };
        face_with_ring_loops(
            store,
            side_surf,
            &[(e_bot, Sense::Forward), (e_top, Sense::Reversed)],
        );
        face_with_ring_loops(store, bot_surf, &[(e_bot, Sense::Reversed)]);
        face_with_ring_loops(store, top_surf, &[(e_top, Sense::Forward)]);
        body
    }

    /// Hand-built solid sphere: one zero-loop face.
    fn sphere_body(store: &mut Store) -> BodyId {
        let (body, shell) = solid_body_scaffold(store);
        let surf = store
            .insert_surface(SurfaceGeom::Sphere(
                Sphere::new(Frame::world(), 1.0).unwrap(),
            ))
            .unwrap();
        let face = store.add(Face {
            shell,
            loops: Vec::new(),
            surface: surf,
            sense: Sense::Forward,
            domain: Some(
                crate::entity::FaceDomain::from_bounds(
                    0.0,
                    core::f64::consts::TAU,
                    -core::f64::consts::FRAC_PI_2,
                    core::f64::consts::FRAC_PI_2,
                )
                .unwrap(),
            ),
            tolerance: None,
        });
        store.get_mut(shell).unwrap().faces.push(face);
        body
    }

    /// Hand-built open sheet: one planar unit square.
    fn sheet_square(store: &mut Store) -> BodyId {
        let body = store.add(Body {
            kind: BodyKind::Sheet,
            regions: Vec::new(),
        });
        let void = store.add(Region {
            body,
            kind: RegionKind::Void,
            shells: Vec::new(),
        });
        store.get_mut(body).unwrap().regions.push(void);
        let shell = store.add(Shell {
            region: void,
            faces: Vec::new(),
            edges: Vec::new(),
            vertex: None,
        });
        store.get_mut(void).unwrap().shells.push(shell);

        let surf = store
            .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        let face = store.add(Face {
            shell,
            loops: Vec::new(),
            surface: surf,
            sense: Sense::Forward,
            domain: None,
            tolerance: None,
        });
        store.get_mut(shell).unwrap().faces.push(face);
        let lp = store.add(Loop {
            face,
            fins: Vec::new(),
        });
        store.get_mut(face).unwrap().loops.push(lp);

        let corners = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let vids: Vec<VertexId> = corners
            .iter()
            .map(|&p| {
                let point = store.add(p);
                store.add(Vertex {
                    point,
                    tolerance: None,
                })
            })
            .collect();
        for k in 0..4 {
            let a = corners[k];
            let b = corners[(k + 1) % 4];
            let curve = store
                .insert_curve(CurveGeom::Line(Line::new(a, b - a).unwrap()))
                .unwrap();
            let edge = store.add(Edge {
                curve: Some(curve),
                vertices: [Some(vids[k]), Some(vids[(k + 1) % 4])],
                bounds: Some((0.0, (b - a).norm())),
                fins: Vec::new(),
                tolerance: None,
            });
            let fin = store.add(Fin {
                parent: lp,
                edge,
                sense: Sense::Forward,
                pcurve: None,
            });
            store.get_mut(lp).unwrap().fins.push(fin);
            store.get_mut(edge).unwrap().fins.push(fin);
        }
        body
    }

    #[test]
    fn clean_bodies_have_zero_faults() {
        let mut store = Store::new();
        let body = clean_block(&mut store);
        assert_eq!(check_body(&store, body).unwrap(), Vec::new());

        let frame = Frame::new(
            Point3::new(0.3, -1.2, 2.1),
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let tilted = block(&mut store, &frame, [1.0, 2.0, 0.5]).unwrap();
        assert_eq!(check_body(&store, tilted).unwrap(), Vec::new());
    }

    #[test]
    fn explicit_check_levels_do_not_conflate_clean_with_proven() {
        let mut store = Store::new();
        let body = clean_block(&mut store);
        let fast = check_body_report(&store, body, CheckLevel::Fast).unwrap();
        assert_eq!(fast.outcome(), CheckOutcome::Valid);
        assert!(fast.faults.is_empty() && fast.gaps.is_empty());

        let full = check_body_report(&store, body, CheckLevel::Full).unwrap();
        assert_eq!(full.outcome(), CheckOutcome::Valid);
        assert!(full.faults.is_empty());
        assert!(full.gaps.is_empty());

        let face = store.faces_of_body(body).unwrap()[0];
        store.get_mut(face).unwrap().domain = None;
        let full = check_body_report(&store, body, CheckLevel::Full).unwrap();
        assert_eq!(full.outcome(), CheckOutcome::Indeterminate);
        assert!(full.gaps.iter().any(|gap| {
            gap.entity == EntityRef::Face(face)
                && gap.kind == VerificationGapKind::FaceDomainContainment
        }));

        store.get_mut(body).unwrap().regions.clear();
        let invalid = check_body_report(&store, body, CheckLevel::Full).unwrap();
        assert_eq!(invalid.outcome(), CheckOutcome::Invalid);
        assert!(
            invalid.gaps.is_empty(),
            "faults take precedence over proof gaps"
        );
    }

    #[test]
    fn contextual_fast_check_preserves_validity_and_accounts_procedural_queries() {
        let mut store = Store::new();
        let body = zero_offset_sheet(&mut store);
        let legacy = check_body_report(&store, body, CheckLevel::Fast).unwrap();
        assert_eq!(legacy.outcome(), CheckOutcome::Valid);

        let session = checker_session(BudgetPlan::empty());
        let loose = Tolerances::with_linear(1.0e-3).unwrap();
        let context = OperationContext::new(&session, loose).unwrap();
        let contextual =
            check_body_report_with_context(&store, body, CheckLevel::Fast, &context).unwrap();
        assert_eq!(contextual.result(), Ok(&legacy));
        assert_eq!(
            contextual
                .report()
                .usage()
                .iter()
                .find(|snapshot| snapshot.stage == kgraph::eval_stage::DEPENDENCY_DEPTH)
                .unwrap()
                .consumed,
            2
        );
        assert!(
            contextual
                .report()
                .usage()
                .iter()
                .find(|snapshot| snapshot.stage == kgraph::eval_stage::NODE_VISITS)
                .unwrap()
                .consumed
                > 0
        );
    }

    #[test]
    fn contextual_fast_check_reconciles_leaf_and_root_limits_after_root_resolution() {
        let mut store = Store::new();
        let body = clean_block(&mut store);
        let run = |budget| {
            let session = checker_session(budget);
            let context = OperationContext::new(&session, Tolerances::default()).unwrap();
            check_body_report_with_context(&store, body, CheckLevel::Fast, &context).unwrap()
        };

        let leaf = run(EvalBudgetProfile::for_limits(64, 0));
        let leaf_limit = LimitSnapshot {
            stage: kgraph::eval_stage::NODE_VISITS,
            resource: ResourceKind::Work,
            consumed: 1,
            allowed: 0,
        };
        assert_eq!(
            leaf.result().as_ref().unwrap_err().limit(),
            Some(leaf_limit)
        );
        assert_eq!(leaf.report().limit_events(), &[leaf_limit]);

        let root = run(EvalBudgetProfile::for_limits(64, 8).with_total_work_limit(1));
        let root_limit = LimitSnapshot {
            stage: kcore::operation::TOTAL_WORK_STAGE,
            resource: ResourceKind::Work,
            consumed: 2,
            allowed: 1,
        };
        assert_eq!(
            root.result().as_ref().unwrap_err().limit(),
            Some(root_limit)
        );
        assert_eq!(root.report().limit_events(), &[root_limit]);

        let offset = zero_offset_sheet(&mut store);
        let session = checker_session(EvalBudgetProfile::for_limits(1, 64));
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let depth =
            check_body_report_with_context(&store, offset, CheckLevel::Fast, &context).unwrap();
        let depth_limit = LimitSnapshot {
            stage: kgraph::eval_stage::DEPENDENCY_DEPTH,
            resource: ResourceKind::Depth,
            consumed: 2,
            allowed: 1,
        };
        assert_eq!(
            depth.result().as_ref().unwrap_err().limit(),
            Some(depth_limit)
        );
        assert_eq!(depth.report().limit_events(), &[depth_limit]);

        let stale = clean_block(&mut store);
        store.remove(stale).unwrap();
        let session = checker_session(EvalBudgetProfile::for_limits(64, 0));
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let stale =
            check_body_report_with_context(&store, stale, CheckLevel::Fast, &context).unwrap();
        assert!(matches!(stale.result(), Err(Error::StaleHandle)));
        assert!(stale.report().limit_events().is_empty());
    }

    #[test]
    fn full_check_profile_composes_current_leaf_and_has_an_additive_growth_seam() {
        let leaf = crate::domain::FaceDomainContainmentBudgetProfile::v1_defaults();
        let shell = crate::shell_proof::shell_proof_budget();
        let planar_shell = crate::planar_shell_proof::planar_shell_proof_budget();
        let aggregate = FullCheckBudgetProfile::v1_defaults();
        assert_eq!(aggregate, leaf.overlaid(&shell).overlaid(&planar_shell));

        const FUTURE_STAGE: kcore::operation::StageId =
            match kcore::operation::StageId::new("ktopo.check.future-proof-work") {
                Ok(stage) => stage,
                Err(_) => panic!("valid future test stage"),
            };
        let grown = BudgetPlan::new(aggregate.limits().iter().copied().chain([
            kcore::operation::LimitSpec::new(
                FUTURE_STAGE,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                10,
            ),
        ]))
        .unwrap();
        assert_eq!(grown.limits().len(), aggregate.limits().len() + 1);
        assert!(
            grown.limits().windows(2).all(|pair| {
                (pair[0].stage, pair[0].resource) < (pair[1].stage, pair[1].resource)
            })
        );
    }

    #[test]
    fn contextual_full_check_matches_legacy_and_reuses_one_scope() {
        let mut store = Store::new();
        let body = clean_block(&mut store);
        let legacy = check_body_report(&store, body, CheckLevel::Full).unwrap();

        let session = checker_session(BudgetPlan::empty());
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let contextual =
            check_body_report_with_context(&store, body, CheckLevel::Full, &context).unwrap();
        assert_eq!(contextual.result(), Ok(&legacy));
        assert_eq!(
            contextual.report().usage(),
            [
                LimitSnapshot {
                    stage: kgraph::eval_stage::DEPENDENCY_DEPTH,
                    resource: ResourceKind::Depth,
                    consumed: 1,
                    allowed: 64,
                },
                LimitSnapshot {
                    stage: kgraph::eval_stage::NODE_VISITS,
                    resource: ResourceKind::Work,
                    consumed: 306,
                    allowed: 4096,
                },
                LimitSnapshot {
                    stage: crate::domain::FACE_DOMAIN_CONTAINMENT_SEGMENTS,
                    resource: ResourceKind::Items,
                    consumed: 1,
                    allowed: 4096,
                },
                LimitSnapshot {
                    stage: crate::planar_shell_proof::PLANAR_SHELL_PAIR_WORK,
                    resource: ResourceKind::Work,
                    consumed: 0,
                    allowed: 200_000,
                },
                LimitSnapshot {
                    stage: crate::shell_proof::SHELL_FACET_PAIR_WORK,
                    resource: ResourceKind::Work,
                    consumed: 0,
                    allowed: 100_000,
                },
            ]
        );

        const CALLER_STAGE: kcore::operation::StageId =
            match kcore::operation::StageId::new("ktopo.check.caller-work") {
                Ok(stage) => stage,
                Err(_) => panic!("valid test stage"),
            };
        let composed = kcore::operation::BudgetPlan::new(
            CheckBudgetProfile::v1_defaults(CheckLevel::Full)
                .limits()
                .iter()
                .copied()
                .chain([kcore::operation::LimitSpec::new(
                    CALLER_STAGE,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    5,
                )]),
        )
        .unwrap();
        let composed_session = checker_session(composed);
        let composed_context =
            OperationContext::new(&composed_session, Tolerances::default()).unwrap();
        let mut scope = OperationScope::new(&composed_context);
        scope.ledger_mut().charge(CALLER_STAGE, 3).unwrap();
        scope
            .ledger_mut()
            .observe(
                crate::domain::FACE_DOMAIN_CONTAINMENT_SEGMENTS,
                ResourceKind::Items,
                2,
            )
            .unwrap();
        let first = check_body_report_in_scope(&store, body, CheckLevel::Full, &mut scope).unwrap();
        let second =
            check_body_report_in_scope(&store, body, CheckLevel::Full, &mut scope).unwrap();
        assert_eq!(first, legacy);
        assert_eq!(second, legacy);
        let report = scope.finish(Ok(())).report().clone();
        assert_eq!(
            report.usage(),
            [
                LimitSnapshot {
                    stage: kgraph::eval_stage::DEPENDENCY_DEPTH,
                    resource: ResourceKind::Depth,
                    consumed: 1,
                    allowed: 64,
                },
                LimitSnapshot {
                    stage: kgraph::eval_stage::NODE_VISITS,
                    resource: ResourceKind::Work,
                    consumed: 612,
                    allowed: 4096,
                },
                LimitSnapshot {
                    stage: CALLER_STAGE,
                    resource: ResourceKind::Work,
                    consumed: 3,
                    allowed: 5,
                },
                LimitSnapshot {
                    stage: crate::domain::FACE_DOMAIN_CONTAINMENT_SEGMENTS,
                    resource: ResourceKind::Items,
                    consumed: 2,
                    allowed: 4096,
                },
                LimitSnapshot {
                    stage: crate::planar_shell_proof::PLANAR_SHELL_PAIR_WORK,
                    resource: ResourceKind::Work,
                    consumed: 0,
                    allowed: 200_000,
                },
                LimitSnapshot {
                    stage: crate::shell_proof::SHELL_FACET_PAIR_WORK,
                    resource: ResourceKind::Work,
                    consumed: 0,
                    allowed: 100_000,
                },
            ]
        );
        assert!(report.limit_events().is_empty());
    }

    #[test]
    fn full_checker_limit_is_a_gap_at_n_minus_one_and_disappears_at_n() {
        let mut store = Store::new();
        let (body, face) = adaptive_domain_sheet(&mut store);
        let default_session = checker_session(FullCheckBudgetProfile::v1_defaults());
        let default_context =
            OperationContext::new(&default_session, Tolerances::default()).unwrap();
        let baseline =
            check_body_report_with_context(&store, body, CheckLevel::Full, &default_context)
                .unwrap();
        let needed = baseline
            .report()
            .usage()
            .iter()
            .find(|snapshot| snapshot.stage == crate::domain::FACE_DOMAIN_CONTAINMENT_SEGMENTS)
            .unwrap()
            .consumed;
        assert!(needed > 1 && needed < 4096);
        assert!(baseline.result().as_ref().unwrap().gaps.iter().all(|gap| {
            gap.entity != EntityRef::Face(face)
                || gap.kind != VerificationGapKind::FaceDomainContainment
        }));

        let run = |allowed| {
            let budget = kcore::operation::BudgetPlan::new([kcore::operation::LimitSpec::new(
                crate::domain::FACE_DOMAIN_CONTAINMENT_SEGMENTS,
                ResourceKind::Items,
                AccountingMode::HighWater,
                allowed,
            )])
            .unwrap();
            let session = checker_session(budget);
            let context = OperationContext::new(&session, Tolerances::default()).unwrap();
            check_body_report_with_context(&store, body, CheckLevel::Full, &context).unwrap()
        };

        let below = run(needed - 1);
        let exact = run(needed);
        let above = run(needed + 1);
        let snapshot = LimitSnapshot {
            stage: crate::domain::FACE_DOMAIN_CONTAINMENT_SEGMENTS,
            resource: ResourceKind::Items,
            consumed: needed,
            allowed: needed - 1,
        };
        let limited_gap = below
            .result()
            .as_ref()
            .unwrap()
            .gaps
            .iter()
            .find(|gap| {
                gap.entity == EntityRef::Face(face)
                    && gap.kind == VerificationGapKind::FaceDomainContainment
            })
            .unwrap();
        assert_eq!(
            limited_gap.cause,
            Some(VerificationGapCause::Limit(snapshot))
        );
        assert_eq!(below.report().limit_events(), &[snapshot]);
        for completed in [&exact, &above] {
            assert!(completed.report().limit_events().is_empty());
            assert!(completed.result().as_ref().unwrap().gaps.iter().all(|gap| {
                gap.entity != EntityRef::Face(face)
                    || gap.kind != VerificationGapKind::FaceDomainContainment
            }));
        }
        assert_eq!(exact.result(), above.result());
    }

    #[test]
    fn face_domain_limit_snapshot_is_attached_to_the_exact_gap() {
        let snapshot = LimitSnapshot {
            stage: crate::domain::FACE_DOMAIN_CONTAINMENT_SEGMENTS,
            resource: ResourceKind::Items,
            consumed: 4097,
            allowed: 4096,
        };
        let evidence = crate::domain::FaceDomainContainmentEvidence {
            status: crate::domain::FaceDomainContainment::Indeterminate,
            limit: Some(snapshot),
        };
        let mut store = Store::new();
        let body = clean_block(&mut store);
        let face = store.faces_of_body(body).unwrap()[0];
        let gap = VerificationGap {
            entity: EntityRef::Face(face),
            kind: VerificationGapKind::FaceDomainContainment,
            cause: face_domain_gap_cause(evidence),
        };

        assert_eq!(gap.cause, Some(VerificationGapCause::Limit(snapshot)));
        let report = CheckReport {
            level: CheckLevel::Full,
            faults: Vec::new(),
            gaps: vec![gap],
        };
        assert_eq!(report.outcome(), CheckOutcome::Indeterminate);
    }

    #[test]
    fn full_checker_discharges_supported_analytic_incidence() {
        let mut store = Store::new();
        let frame = Frame::new(
            Point3::new(0.3, -1.2, 2.1),
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let bodies = [
            block(&mut store, &frame, [1.0, 2.0, 0.5]).unwrap(),
            cylinder(&mut store, &frame, 1.2, 2.5).unwrap(),
            cone(&mut store, &frame, 1.3, 0.7, 2.0).unwrap(),
            cylindrical_sheet(&mut store, &frame, 0.8, 1.5).unwrap(),
            cylinder_body(&mut store),
            sheet_square(&mut store),
        ];

        for body in bodies {
            let report = check_body_report(&store, body, CheckLevel::Full).unwrap();
            assert_ne!(report.outcome(), CheckOutcome::Invalid);
            assert!(report.faults.is_empty());
            assert!(
                report.gaps.iter().all(|gap| !matches!(
                    gap.kind,
                    VerificationGapKind::EdgeSurfaceIncidence
                        | VerificationGapKind::PcurveSurfaceIncidence
                )),
                "supported analytic incidence remained unresolved: {:?}",
                report.gaps
            );
        }
    }

    #[test]
    fn unsupported_curved_loop_orientation_is_a_full_gap_not_a_fast_fault() {
        let mut store = Store::new();
        let body = cylinder(&mut store, &Frame::world(), 1.0, 2.0).unwrap();

        let fast = check_body_report(&store, body, CheckLevel::Fast).unwrap();
        assert_eq!(fast.outcome(), CheckOutcome::Valid, "{fast:?}");
        assert!(
            fast.faults
                .iter()
                .all(|fault| fault.kind != FaultKind::WrongLoopOrientation)
        );

        let full = check_body_report(&store, body, CheckLevel::Full).unwrap();
        assert_eq!(full.outcome(), CheckOutcome::Indeterminate, "{full:?}");
        assert!(full.faults.is_empty(), "{full:?}");
        assert!(
            full.gaps
                .iter()
                .any(|gap| gap.kind == VerificationGapKind::LoopOrientation),
            "{full:?}"
        );
    }

    #[test]
    fn exact_and_zero_loop_primitives_keep_full_valid_orientation_evidence() {
        let mut store = Store::new();
        let sheet = planar_sheet(
            &mut store,
            &Frame::world(),
            &[
                Point2::new(-1.0, -1.0),
                Point2::new(1.0, -1.0),
                Point2::new(1.0, 1.0),
                Point2::new(-1.0, 1.0),
            ],
        )
        .unwrap();
        for body in [clean_block(&mut store), sheet] {
            let report = check_body_report(&store, body, CheckLevel::Full).unwrap();
            assert_eq!(report.outcome(), CheckOutcome::Valid, "{report:?}");
            assert!(
                report
                    .gaps
                    .iter()
                    .all(|gap| gap.kind != VerificationGapKind::LoopOrientation),
                "{report:?}"
            );
        }
        for body in [
            make_sphere(&mut store, &Frame::world(), 1.0).unwrap(),
            make_torus(&mut store, &Frame::world(), 2.0, 0.5).unwrap(),
        ] {
            let report = check_body_report(&store, body, CheckLevel::Full).unwrap();
            assert_ne!(report.outcome(), CheckOutcome::Invalid, "{report:?}");
            assert!(report.faults.is_empty(), "{report:?}");
            assert!(
                report
                    .gaps
                    .iter()
                    .all(|gap| gap.kind != VerificationGapKind::LoopOrientation),
                "{report:?}"
            );
        }
    }

    #[test]
    fn full_checker_proves_simple_loops_and_rejects_a_bow_tie() {
        let mut store = Store::new();
        let sheet = sheet_square(&mut store);
        let simple = check_body_report(&store, sheet, CheckLevel::Full).unwrap();
        assert!(simple.faults.is_empty());
        assert!(simple.gaps.iter().all(|gap| {
            !matches!(gap.entity, EntityRef::Loop(_))
                || gap.kind != VerificationGapKind::LoopSelfIntersection
        }));

        let rings = cylinder_body(&mut store);
        let ring_report = check_body_report(&store, rings, CheckLevel::Full).unwrap();
        assert!(ring_report.faults.is_empty());
        assert!(ring_report.gaps.iter().all(|gap| {
            !matches!(gap.entity, EntityRef::Loop(_))
                || gap.kind != VerificationGapKind::LoopSelfIntersection
        }));

        let face = store.faces_of_body(sheet).unwrap()[0];
        let loop_id = store.get(face).unwrap().loops[0];
        let fins = store.get(loop_id).unwrap().fins.clone();
        let positions = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        ];
        let mut transaction = store.transaction().unwrap();
        for (index, &fin_id) in fins.iter().enumerate() {
            let vertex = transaction.store().fin_tail(fin_id).unwrap().unwrap();
            let point = transaction.store().get(vertex).unwrap().point;
            *transaction.assembly().get_mut(point).unwrap() = positions[index];
        }
        for (index, &fin_id) in fins.iter().enumerate() {
            let edge_id = transaction.store().get(fin_id).unwrap().edge;
            let curve_id = transaction.store().get(edge_id).unwrap().curve.unwrap();
            let start = positions[index];
            let end = positions[(index + 1) % positions.len()];
            transaction
                .assembly()
                .replace_curve(
                    curve_id,
                    CurveGeom::Line(Line::new(start, end - start).unwrap()),
                )
                .unwrap();
            transaction.assembly().get_mut(edge_id).unwrap().bounds = Some((0.0, start.dist(end)));
        }

        let fast = check_body_report(transaction.store(), sheet, CheckLevel::Fast).unwrap();
        assert_eq!(fast.outcome(), CheckOutcome::Valid);
        let full = check_body_report(transaction.store(), sheet, CheckLevel::Full).unwrap();
        assert_eq!(full.outcome(), CheckOutcome::Invalid);
        assert!(full.faults.iter().any(|fault| {
            fault.entity == EntityRef::Loop(loop_id)
                && fault.kind == FaultKind::LoopSelfIntersection
        }));
    }

    #[test]
    fn full_checker_rejects_an_inward_whole_sphere() {
        let mut store = Store::new();
        let body = sphere_body(&mut store);
        let face = store.faces_of_body(body).unwrap()[0];
        store.get_mut(face).unwrap().sense = Sense::Reversed;

        let fast = check_body_report(&store, body, CheckLevel::Fast).unwrap();
        assert_eq!(fast.outcome(), CheckOutcome::Valid);
        let full = check_body_report(&store, body, CheckLevel::Full).unwrap();
        assert_eq!(full.outcome(), CheckOutcome::Invalid);
        assert!(full.faults.iter().any(|fault| {
            fault.kind == FaultKind::ShellOrientation && matches!(fault.entity, EntityRef::Shell(_))
        }));
    }

    #[test]
    fn clean_cylinder_with_ring_edges_is_clean() {
        let mut store = Store::new();
        let body = cylinder_body(&mut store);
        assert_eq!(check_body(&store, body).unwrap(), Vec::new());
    }

    #[test]
    fn clean_sphere_with_zero_loop_face_is_clean() {
        let mut store = Store::new();
        let body = sphere_body(&mut store);
        assert_eq!(check_body(&store, body).unwrap(), Vec::new());
    }

    #[test]
    fn clean_sheet_square_is_clean() {
        let mut store = Store::new();
        let body = sheet_square(&mut store);
        assert_eq!(check_body(&store, body).unwrap(), Vec::new());
    }

    #[test]
    fn invalid_face_domain_and_tolerance_fault() {
        let mut store = Store::new();
        let body = sphere_body(&mut store);
        let face = store.faces_of_body(body).unwrap()[0];
        store.get_mut(face).unwrap().domain = Some(
            crate::entity::FaceDomain::from_bounds(
                0.0,
                core::f64::consts::TAU * 1.1,
                -core::f64::consts::FRAC_PI_2,
                core::f64::consts::FRAC_PI_2,
            )
            .unwrap(),
        );
        store.get_mut(face).unwrap().tolerance =
            Some(crate::tolerance::EntityTolerance::unchecked(1e-12));
        let faults = check_body(&store, body).unwrap();
        assert_has(&faults, FaultKind::BadFaceDomain);
        assert_has(&faults, FaultKind::BadTolerance);
    }

    #[test]
    fn outer_loop_orientation_does_not_depend_on_storage_order() {
        let mut store = Store::new();
        let body = sheet_square(&mut store);
        let face = store.faces_of_body(body).unwrap()[0];
        let hole = store.add(Loop {
            face,
            fins: Vec::new(),
        });
        store.get_mut(face).unwrap().loops.push(hole);

        // Clockwise inner square in the face's UV frame.
        let corners = [
            Point3::new(0.25, 0.25, 0.0),
            Point3::new(0.25, 0.75, 0.0),
            Point3::new(0.75, 0.75, 0.0),
            Point3::new(0.75, 0.25, 0.0),
        ];
        let vertices: Vec<_> = corners
            .iter()
            .map(|&position| {
                let point = store.add(position);
                store.add(Vertex {
                    point,
                    tolerance: None,
                })
            })
            .collect();
        for i in 0..corners.len() {
            let a = corners[i];
            let b = corners[(i + 1) % corners.len()];
            let curve = store
                .insert_curve(CurveGeom::Line(Line::new(a, b - a).unwrap()))
                .unwrap();
            let edge = store.add(Edge {
                curve: Some(curve),
                vertices: [Some(vertices[i]), Some(vertices[(i + 1) % vertices.len()])],
                bounds: Some((0.0, (b - a).norm())),
                fins: Vec::new(),
                tolerance: None,
            });
            let fin = store.add(Fin {
                parent: hole,
                edge,
                sense: Sense::Forward,
                pcurve: None,
            });
            store.get_mut(hole).unwrap().fins.push(fin);
            store.get_mut(edge).unwrap().fins.push(fin);
        }
        let expected = check_body_report(&store, body, CheckLevel::Fast).unwrap();
        assert_eq!(expected.outcome(), CheckOutcome::Valid, "{expected:?}");
        assert_eq!(
            check_body_report(&store, body, CheckLevel::Fast).unwrap(),
            expected
        );
        let full = check_body_report(&store, body, CheckLevel::Full).unwrap();
        assert!(full.faults.is_empty(), "{full:?}");
        assert!(full.gaps.iter().all(|gap| {
            !matches!(
                gap.kind,
                VerificationGapKind::LoopOrientation | VerificationGapKind::LoopContainment
            )
        }));

        let loops = store.get(face).unwrap().loops.clone();
        for loop_id in &loops {
            store.get_mut(*loop_id).unwrap().fins.rotate_left(1);
        }
        assert_eq!(
            check_body_report(&store, body, CheckLevel::Fast).unwrap(),
            expected
        );

        store.get_mut(face).unwrap().loops.swap(0, 1);
        assert_eq!(
            check_body_report(&store, body, CheckLevel::Fast).unwrap(),
            expected
        );

        let mut reversed_hole = store.get(hole).unwrap().fins.clone();
        reversed_hole.reverse();
        for &fin in &reversed_hole {
            let sense = store.get(fin).unwrap().sense;
            store.get_mut(fin).unwrap().sense = sense.flipped();
        }
        store.get_mut(hole).unwrap().fins = reversed_hole;
        let reversed = check_body_report(&store, body, CheckLevel::Fast).unwrap();
        assert_eq!(reversed.outcome(), CheckOutcome::Invalid, "{reversed:?}");
        let orientation_faults = reversed
            .faults
            .iter()
            .filter(|fault| fault.kind == FaultKind::WrongLoopOrientation)
            .collect::<Vec<_>>();
        assert_eq!(orientation_faults.len(), 1, "{reversed:?}");
        assert_eq!(orientation_faults[0].entity, EntityRef::Loop(hole));

        store.get_mut(face).unwrap().loops.swap(0, 1);
        let permuted = check_body_report(&store, body, CheckLevel::Fast).unwrap();
        assert_eq!(
            permuted
                .faults
                .iter()
                .filter(|fault| fault.kind == FaultKind::WrongLoopOrientation)
                .collect::<Vec<_>>(),
            orientation_faults
        );
    }

    #[test]
    fn flipped_fin_sense_opens_the_loop() {
        let mut store = Store::new();
        let body = clean_block(&mut store);
        let face = store.faces_of_body(body).unwrap()[0];
        let lp = store.get(face).unwrap().loops[0];
        let fin = store.get(lp).unwrap().fins[0];
        let s = store.get(fin).unwrap().sense;
        store.get_mut(fin).unwrap().sense = s.flipped();
        let faults = check_body(&store, body).unwrap();
        assert_has(&faults, FaultKind::OpenLoop);
        assert_has(&faults, FaultKind::FinsNotOpposed);
    }

    #[test]
    fn reversed_loop_has_wrong_orientation() {
        let mut store = Store::new();
        let body = clean_block(&mut store);
        let face = store.faces_of_body(body).unwrap()[0];
        let lp = store.get(face).unwrap().loops[0];
        let mut fins = store.get(lp).unwrap().fins.clone();
        fins.reverse();
        for &fin in &fins {
            let s = store.get(fin).unwrap().sense;
            store.get_mut(fin).unwrap().sense = s.flipped();
        }
        store.get_mut(lp).unwrap().fins = fins;
        let faults = check_body(&store, body).unwrap();
        assert_has(&faults, FaultKind::WrongLoopOrientation);
        assert_has(&faults, FaultKind::FinsNotOpposed);
    }

    #[test]
    fn moved_vertex_is_off_curve() {
        let mut store = Store::new();
        let body = clean_block(&mut store);
        let v = store.vertices_of_body(body).unwrap()[0];
        let point = store.get(v).unwrap().point;
        *store.get_mut(point).unwrap() += Vec3::new(1e-4, 0.0, 0.0);
        let faults = check_body(&store, body).unwrap();
        assert_has(&faults, FaultKind::VertexOffCurve);
    }

    #[test]
    fn oversized_coordinate_leaves_size_box() {
        let mut store = Store::new();
        let body = clean_block(&mut store);
        let v = store.vertices_of_body(body).unwrap()[0];
        let point = store.get(v).unwrap().point;
        *store.get_mut(point).unwrap() = Point3::new(600.0, 0.0, 0.0);
        let faults = check_body(&store, body).unwrap();
        assert_has(&faults, FaultKind::OutsideSizeBox);
        assert_has(&faults, FaultKind::VertexOffCurve);
    }

    #[test]
    fn region_kind_faults() {
        let mut store = Store::new();
        let body = clean_block(&mut store);
        let exterior = store.get(body).unwrap().regions[0];
        store.get_mut(exterior).unwrap().kind = RegionKind::Solid;
        assert_has(
            &check_body(&store, body).unwrap(),
            FaultKind::ExteriorNotVoid,
        );
        store.get_mut(exterior).unwrap().kind = RegionKind::Void;

        let solid = store.get(body).unwrap().regions[1];
        store.get_mut(solid).unwrap().kind = RegionKind::Void;
        assert_has(&check_body(&store, body).unwrap(), FaultKind::NoSolidRegion);
    }

    #[test]
    fn body_without_regions_faults() {
        let mut store = Store::new();
        let body = store.add(Body {
            kind: BodyKind::Solid,
            regions: Vec::new(),
        });
        let faults = check_body(&store, body).unwrap();
        assert_has(&faults, FaultKind::NoRegions);
        assert_has(&faults, FaultKind::NoSolidRegion);
    }

    #[test]
    fn fin_missing_from_loop_list_opens_ring() {
        let mut store = Store::new();
        let body = clean_block(&mut store);
        let face = store.faces_of_body(body).unwrap()[0];
        let lp = store.get(face).unwrap().loops[0];
        store.get_mut(lp).unwrap().fins.remove(1);
        let faults = check_body(&store, body).unwrap();
        assert_has(&faults, FaultKind::OpenLoop);
        assert_has(&faults, FaultKind::BackPointerMismatch);
    }

    #[test]
    fn reversed_bounds_are_bad() {
        let mut store = Store::new();
        let body = clean_block(&mut store);
        let e = store.edges_of_body(body).unwrap()[0];
        let (t0, t1) = store.get(e).unwrap().bounds.unwrap();
        store.get_mut(e).unwrap().bounds = Some((t1, t0));
        assert_has(&check_body(&store, body).unwrap(), FaultKind::BadBounds);
    }

    #[test]
    fn vertex_bearing_edge_without_bounds_faults() {
        let mut store = Store::new();
        let body = clean_block(&mut store);
        let e = store.edges_of_body(body).unwrap()[0];
        store.get_mut(e).unwrap().bounds = None;
        assert_has(&check_body(&store, body).unwrap(), FaultKind::MissingBounds);
    }

    #[test]
    fn zero_loop_face_on_plane_faults() {
        let mut store = Store::new();
        let body = sphere_body(&mut store);
        let face = store.faces_of_body(body).unwrap()[0];
        let plane = store
            .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        store.get_mut(face).unwrap().surface = plane;
        assert_has(
            &check_body(&store, body).unwrap(),
            FaultKind::ZeroLoopFaceOnOpenSurface,
        );
    }

    #[test]
    fn sub_resolution_tolerance_faults() {
        let mut store = Store::new();
        let body = clean_block(&mut store);
        let v = store.vertices_of_body(body).unwrap()[0];
        store.get_mut(v).unwrap().tolerance =
            Some(crate::tolerance::EntityTolerance::unchecked(1e-12));
        assert_has(&check_body(&store, body).unwrap(), FaultKind::BadTolerance);
    }

    #[test]
    fn removed_face_breaks_euler_and_fin_counts() {
        let mut store = Store::new();
        let body = clean_block(&mut store);
        let face = store.faces_of_body(body).unwrap()[0];
        let shell = store.get(face).unwrap().shell;
        let lp = store.get(face).unwrap().loops[0];
        let fins = store.get(lp).unwrap().fins.clone();
        for fin in fins {
            let e = store.get(fin).unwrap().edge;
            store.get_mut(e).unwrap().fins.retain(|&f| f != fin);
            store.remove(fin).unwrap();
        }
        store.remove(lp).unwrap();
        store.get_mut(shell).unwrap().faces.retain(|&f| f != face);
        store.remove(face).unwrap();
        let faults = check_body(&store, body).unwrap();
        assert_has(&faults, FaultKind::EulerViolation);
        assert_has(&faults, FaultKind::BadFinCount);
    }

    #[test]
    fn loop_with_foreign_parent_mismatches() {
        let mut store = Store::new();
        let body = clean_block(&mut store);
        let faces = store.faces_of_body(body).unwrap();
        let lp = store.get(faces[0]).unwrap().loops[0];
        store.get_mut(lp).unwrap().face = faces[1];
        assert_has(
            &check_body(&store, body).unwrap(),
            FaultKind::BackPointerMismatch,
        );
    }

    #[test]
    fn stale_surface_reference_faults() {
        let mut store = Store::new();
        let body = clean_block(&mut store);
        let face = store.faces_of_body(body).unwrap()[0];
        let surface = store.get(face).unwrap().surface;
        let mut transaction = store.transaction().unwrap();
        transaction.assembly().remove_surface(surface).unwrap();
        assert_has(
            &check_body(transaction.store(), body).unwrap(),
            FaultKind::StaleReference,
        );
    }

    #[test]
    fn wire_body_with_faces_mismatches_kind() {
        let mut store = Store::new();
        let body = sheet_square(&mut store);
        store.get_mut(body).unwrap().kind = BodyKind::Wire;
        assert_has(&check_body(&store, body).unwrap(), FaultKind::KindMismatch);
    }

    #[test]
    fn extra_fin_breaks_manifold_count() {
        let mut store = Store::new();
        let body = clean_block(&mut store);
        let e = store.edges_of_body(body).unwrap()[0];
        let some_loop = {
            let face = store.faces_of_body(body).unwrap()[0];
            store.get(face).unwrap().loops[0]
        };
        let fin = store.add(Fin {
            parent: some_loop,
            edge: e,
            sense: Sense::Forward,
            pcurve: None,
        });
        store.get_mut(e).unwrap().fins.push(fin);
        let faults = check_body(&store, body).unwrap();
        assert_has(&faults, FaultKind::BadFinCount);
        assert_has(&faults, FaultKind::BackPointerMismatch);
    }

    #[test]
    fn shifted_curve_leaves_adjacent_surfaces() {
        let mut store = Store::new();
        let body = clean_block(&mut store);
        let e = store.edges_of_body(body).unwrap()[0];
        let curve = store.get(e).unwrap().curve.unwrap();
        let CurveGeom::Line(line) = *store.get(curve).unwrap() else {
            panic!("block edges are lines");
        };
        let shifted = Line::new(line.origin() + Vec3::new(1e-4, 1e-4, 0.0), line.dir()).unwrap();
        let mut transaction = store.transaction().unwrap();
        transaction
            .assembly()
            .replace_curve(curve, CurveGeom::Line(shifted))
            .unwrap();
        let faults = check_body(transaction.store(), body).unwrap();
        assert_has(&faults, FaultKind::EdgeOffSurface);
        assert_has(&faults, FaultKind::VertexOffCurve);
    }

    #[test]
    fn ring_edge_on_non_periodic_curve_faults() {
        let mut store = Store::new();
        let body = cylinder_body(&mut store);
        let e = store.edges_of_body(body).unwrap()[0];
        let line = store
            .insert_curve(CurveGeom::Line(
                Line::new(Point3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap(),
            ))
            .unwrap();
        store.get_mut(e).unwrap().curve = Some(line);
        let faults = check_body(&store, body).unwrap();
        assert_has(&faults, FaultKind::RingEdgeNotPeriodic);
    }

    #[test]
    fn ring_edge_inside_long_ring_faults() {
        let mut store = Store::new();
        let body = cylinder_body(&mut store);
        // Splice a second fin (over the top ring edge) into the bottom
        // cap's single-fin loop.
        let faces = store.faces_of_body(body).unwrap();
        let cap = faces[1];
        let lp = store.get(cap).unwrap().loops[0];
        let edges = store.edges_of_body(body).unwrap();
        let e_top = edges[1];
        let fin = store.add(Fin {
            parent: lp,
            edge: e_top,
            sense: Sense::Forward,
            pcurve: None,
        });
        store.get_mut(lp).unwrap().fins.push(fin);
        store.get_mut(e_top).unwrap().fins.push(fin);
        let faults = check_body(&store, body).unwrap();
        assert_has(&faults, FaultKind::RingEdgeInLongRing);
    }
}
