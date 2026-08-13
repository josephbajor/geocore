//! Exact discriminant admission for nonparallel Cylinder/Cylinder supports.
//!
//! A ruling of the first canonical cylinder is substituted into a
//! division-free dual chart of the second cylinder's stored frame. The
//! resulting exact quadratic in ruling height has an exact cyclic
//! second-harmonic discriminant. A strictly negative discriminant proves a
//! complete miss. A strictly positive discriminant proves the existence of two
//! infinite-support sheets. Publication requires paired active-range residual
//! certificates for every retained procedural carrier and both pcurves. Four
//! exact axial-bound queries admit root-free whole sheets and simple
//! non-wrapping open spans with exact source-root endpoint evidence. Contact,
//! coincident and failed exact classifications remain typed
//! indeterminate. A parameterization-local projection fold may retry the
//! reverse chart, but only a strict-positive
//! reverse proof can supersede Contact; no sampled marcher may claim completion.

use kcore::error::CapabilityId;
use kcore::operation::{DiagnosticCode, DiagnosticKind, OperationScope, StageId};
use kcore::predicates::{Orientation, orient3d};
use kcore::proof::{IncompleteCause, IncompleteEvidence};
use kgeom::curve2d::Curve2d;
use kgeom::param::ParamRange;
use kgeom::surface::Cylinder;
use kgeom::vec::Point3;
use kgraph::{
    IntersectionCertificateError, PairedSkewCylinderBranchResidualCertificate,
    PersistentSkewCylinderAxialRootEventInput,
    PersistentSkewCylinderFiniteWindowIsolatedPointCertificate,
    PersistentSkewCylinderFiniteWindowMemberInput,
    PersistentSkewCylinderFiniteWindowThroughContactCertificate,
    PersistentSkewCylinderFoldedSupportCertificate, PersistentSkewCylinderFoldedSupportEndpoint,
    PersistentSkewCylinderSupportContactCertificate,
    PersistentSkewCylinderTouchingSupportCertificate,
    PersistentSkewCylinderTouchingSupportEndpoint, SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    SKEW_CYLINDER_BRANCH_CERTIFICATE_WORK, SKEW_CYLINDER_BRANCH_PCURVE_ROOT_CORRIDOR_WORK,
    SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK, SKEW_CYLINDER_ROOT_CLUSTER_MAX_EXACT_WORK,
    SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK, SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK,
    SkewCylinderExactDiscriminantTopology, SkewCylinderFiniteSheetTopology,
    SkewCylinderFiniteWindowRootEventKind, SkewCylinderFiniteWindowTopologyCertificate,
    SkewCylinderOpenSpan, SkewCylinderOpenSpanEndpointProof, SkewCylinderOpenSpanFailure,
    SkewCylinderOpenSpanTopologyInput, SkewCylinderRootInsideSide, SkewCylinderSheet,
    SkewCylinderStrictPositiveTwoSheetAdmissionCertificate,
    certify_paired_skew_cylinder_branch_residuals,
    certify_paired_skew_cylinder_branch_subrange_residuals,
    certify_persistent_skew_cylinder_finite_window_family,
    certify_persistent_skew_cylinder_folded_support,
    certify_persistent_skew_cylinder_support_contact,
    certify_persistent_skew_cylinder_touching_support,
    certify_skew_cylinder_folded_support_topology, certify_skew_cylinder_touching_support_topology,
    classify_skew_cylinder_exact_discriminant, classify_skew_cylinder_open_spans,
    plan_persistent_skew_cylinder_support_contact_boundaries, plan_skew_cylinder_root_clusters,
};

use super::cylinder_cylinder::{compare_cylinder_windows, validate_ranges};
use super::error::IntersectionError;
use super::graph_branch_certificate::{
    SkewCylinderFoldedSupportBranchCertificate, SkewCylinderOpenSpanBranchCertificate,
    SkewCylinderTouchingSupportBranchCertificate, SkewCylinderWholeContactBranchCertificate,
};
use super::graph_skew_cylinder_endpoint::{
    IntersectionBranchEndpointProof, SkewCylinderAxialBoundaryProof,
    SkewCylinderAxialRelationProof, SkewCylinderAxialRootEndpointProof,
    SkewCylinderFoldedSupportRootEndpointProof, SkewCylinderFoldedSupportSeamEndpointProof,
    SkewCylinderHalfAngleChartProof, SkewCylinderRootInsideSideProof,
    SkewCylinderTouchingSupportChartJoinEndpointProof,
    SkewCylinderTouchingSupportRootEndpointProof, SkewCylinderTouchingSupportSeamEndpointProof,
};
use super::graph_surface::{GraphSurfaceIntersectionError, GraphSurfaceIntersectionResult};
use super::result::{
    ContactKind, SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
    SurfaceSurfacePoint,
};
use super::skew_cylinder_sheet_occupancy::{
    SKEW_CYLINDER_AXIAL_BOUNDS_EXACT_WORK, collect_skew_cylinder_axial_bound_topologies,
};
use kgraph::{
    SkewCylinderAxialBoundary, SkewCylinderAxialRelation, SkewCylinderAxialRootFailure,
    SkewCylinderFoldedSupportCellLocation, SkewCylinderHalfAngleChart,
};

const TWO_SHEET_REASON: &str = "strict-positive skew Cylinder/Cylinder discriminant requires certified contained full-cycle branch carriers";
const CLIPPED_TOPOLOGY_REASON: &str = "finite axial cuts of strict-positive skew Cylinder/Cylinder sheets require certified clipped branch topology";
const CONTACT_TOPOLOGY_REASON: &str =
    "skew Cylinder/Cylinder discriminant contact roots require certified branch topology";
const NUMERIC_RESOLUTION_REASON: &str =
    "exact skew Cylinder/Cylinder classification or branch proof did not finish";
const NONPARALLEL_REASON: &str =
    "skew Cylinder/Cylinder discriminant admission requires exact nonparallel axes";
const ROOT_CORRIDOR_REASON: &str =
    "bounded skew Cylinder/Cylinder endpoints require certified physical-root pcurve corridors";

/// Stable work stage for one exact full-cycle skew-cylinder discriminant proof.
pub const SKEW_CYLINDER_DISCRIMINANT_WORK: StageId =
    match StageId::new("kops.intersect.skew-cylinder-discriminant-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid skew-cylinder discriminant stage"),
    };

/// Exact atomic work charged by one admitted skew-cylinder classification.
pub const SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK: u64 = 2 * SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK;

/// Stable work stage for one atomic pair of certified procedural branches.
pub const SKEW_CYLINDER_TWO_SHEET_WORK: StageId =
    match StageId::new("kops.intersect.skew-cylinder-two-sheet-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid skew-cylinder two-sheet stage"),
    };

/// Atomic work charged before certifying both procedural skew branches.
pub const SKEW_CYLINDER_TWO_SHEET_EXACT_WORK: u64 = 2 * SKEW_CYLINDER_BRANCH_CERTIFICATE_WORK;

/// Additional existing residual-certifier work for one whole sheet whose
/// authored finite window is closed-contact rather than strictly contained.
pub const SKEW_CYLINDER_THROUGH_CONTACT_EXACT_WORK_PER_BRANCH: u64 =
    SKEW_CYLINDER_BRANCH_CERTIFICATE_WORK;

/// Total cumulative two-sheet work for a closed finite window that touches
/// authored bounds on both whole sheets: the initial strict-window attempt
/// plus one existing residual recertification per sheet.
pub const SKEW_CYLINDER_THROUGH_CONTACT_EXACT_WORK: u64 =
    SKEW_CYLINDER_TWO_SHEET_EXACT_WORK + 2 * SKEW_CYLINDER_THROUGH_CONTACT_EXACT_WORK_PER_BRANCH;

/// Stable work stage for one atomic four-bound axial occupancy proof.
pub const SKEW_CYLINDER_AXIAL_CLIP_WORK: StageId =
    match StageId::new("kops.intersect.skew-cylinder-axial-clip-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid skew-cylinder axial-clip stage"),
    };

/// Atomic work charged before classifying all four finite axial bounds.
pub const SKEW_CYLINDER_AXIAL_CLIP_EXACT_WORK: u64 = SKEW_CYLINDER_AXIAL_BOUNDS_EXACT_WORK;

/// Stable work stage for exact equality queries between overlapping
/// finite-window root corridors.
pub const SKEW_CYLINDER_ROOT_CLUSTER_WORK: StageId =
    match StageId::new("kops.intersect.skew-cylinder-root-cluster-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid skew-cylinder root-cluster stage"),
    };

/// Maximum atomic root-cluster work for one four-bound family.
pub const SKEW_CYLINDER_ROOT_CLUSTER_MAX_WORK: u64 = SKEW_CYLINDER_ROOT_CLUSTER_MAX_EXACT_WORK;

/// Stable work stage for independently certified bounded skew-sheet spans.
pub const SKEW_CYLINDER_OPEN_SPAN_WORK: StageId =
    match StageId::new("kops.intersect.skew-cylinder-open-span-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid skew-cylinder open-span stage"),
    };

/// Atomic work charged for each retained non-wrapping open span.
pub const SKEW_CYLINDER_OPEN_SPAN_EXACT_WORK_PER_BRANCH: u64 =
    SKEW_CYLINDER_BRANCH_CERTIFICATE_WORK + 2 * SKEW_CYLINDER_BRANCH_PCURVE_ROOT_CORRIDOR_WORK;

/// Missing carrier for the two sheets proved by a strict-positive discriminant.
pub const SKEW_CYLINDER_TWO_SHEET_BRANCH_CARRIER: CapabilityId =
    match CapabilityId::new("kops.intersect.skew-cylinder-two-sheet-branch-carrier") {
        Ok(capability) => capability,
        Err(_) => panic!("valid skew-cylinder two-sheet capability"),
    };

/// Missing finite branch topology for one or more exact axial cuts.
pub const SKEW_CYLINDER_CLIPPED_BRANCH_TOPOLOGY: CapabilityId =
    match CapabilityId::new("kops.intersect.skew-cylinder-clipped-branch-topology") {
        Ok(capability) => capability,
        Err(_) => panic!("valid skew-cylinder clipped-branch capability"),
    };

/// Missing topology for zeroes of the exact cyclic discriminant.
pub const SKEW_CYLINDER_CONTACT_ROOT_TOPOLOGY: CapabilityId =
    match CapabilityId::new("kops.intersect.skew-cylinder-contact-root-topology") {
        Ok(capability) => capability,
        Err(_) => panic!("valid skew-cylinder contact-root capability"),
    };

/// Strict-positive discriminant was proved, but its branch carrier is pending.
pub const SKEW_CYLINDER_TWO_SHEET_INCOMPLETE: DiagnosticCode =
    match DiagnosticCode::new("kops.intersect.skew-cylinder-two-sheet-incomplete") {
        Ok(code) => code,
        Err(_) => panic!("valid skew-cylinder two-sheet diagnostic"),
    };

/// Exact axial roots exist, but clipped branch publication is not yet certified.
pub const SKEW_CYLINDER_CLIPPED_TOPOLOGY_INCOMPLETE: DiagnosticCode =
    match DiagnosticCode::new("kops.intersect.skew-cylinder-clipped-topology-incomplete") {
        Ok(code) => code,
        Err(_) => panic!("valid skew-cylinder clipped-topology diagnostic"),
    };

/// The discriminant has contact/root topology outside this initial admission.
pub const SKEW_CYLINDER_CONTACT_TOPOLOGY_INCOMPLETE: DiagnosticCode =
    match DiagnosticCode::new("kops.intersect.skew-cylinder-contact-topology-incomplete") {
        Ok(code) => code,
        Err(_) => panic!("valid skew-cylinder contact-topology diagnostic"),
    };

/// Exact construction or cyclic classification failed inside its safe envelope.
pub const SKEW_CYLINDER_DISCRIMINANT_NUMERIC_RESOLUTION: DiagnosticCode =
    match DiagnosticCode::new("kops.intersect.skew-cylinder-discriminant-numeric-resolution") {
        Ok(code) => code,
        Err(_) => panic!("valid skew-cylinder numeric-resolution diagnostic"),
    };

/// Non-forgeable proof that an exact nonparallel Cylinder/Cylinder pair has a
/// strictly negative ruling discriminant over the complete canonical cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SkewCylinderStrictDiscriminantMiss {
    _private: (),
}

/// Exact-root-owned zero-dimensional contact from one strict-positive skew
/// Cylinder/Cylinder finite-window family.
///
/// This is deliberately distinct from a curve branch: it has an exact point
/// carrier and exact source-root identity, but no curve or carrier range.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderIsolatedContact {
    certificate: PersistentSkewCylinderFiniteWindowIsolatedPointCertificate,
    raw_point: SurfaceSurfacePoint,
}

impl SkewCylinderIsolatedContact {
    /// Sealed kgraph proof owning the exact zero-dimensional carrier.
    pub const fn certificate(self) -> PersistentSkewCylinderFiniteWindowIsolatedPointCertificate {
        self.certificate
    }

    /// Exact analytic point carrier, independent of the symmetric raw point.
    pub fn point(self) -> Point3 {
        self.certificate.point()
    }

    /// Caller-order parameters on the two source cylinders.
    pub fn surface_parameters(self) -> [[f64; 2]; 2] {
        self.certificate.source_surface_parameters()
    }

    /// Number of exact authored-bound roots grouped into this contact.
    pub const fn root_count(self) -> usize {
        self.certificate.event_certificate().event().root_count()
    }

    /// Exact authored-bound root grouped into this contact.
    pub fn root(self, ordinal: usize) -> Option<PersistentSkewCylinderAxialRootEventInput> {
        self.certificate.root(ordinal)
    }

    /// Canonical raw point retained by the legacy solver result.
    pub const fn raw_point(self) -> SurfaceSurfacePoint {
        self.raw_point
    }

    fn mint(
        certificate: PersistentSkewCylinderFiniteWindowIsolatedPointCertificate,
    ) -> Result<Self, IntersectionCertificateError> {
        let parameters = certificate.source_surface_parameters();
        let [first, second] = certificate.source_surface_points();
        let residual = first.dist(second);
        let tolerance = certificate.family().tolerance();
        if !residual.is_finite() || residual > tolerance {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
        let raw_point = SurfaceSurfacePoint {
            point: (first + second) * 0.5,
            uv_a: parameters[0],
            uv_b: parameters[1],
            residual,
            kind: ContactKind::Transverse,
        };
        Ok(Self {
            certificate,
            raw_point,
        })
    }
}

/// Exact-root-owned contact that lies on one represented positive-length skew
/// branch.
///
/// Unlike [`SkewCylinderIsolatedContact`], this carrier contributes no raw
/// point or graph vertex. Its sealed sheet identity binds the contact to the
/// already published positive-length branch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderThroughContact {
    certificate: PersistentSkewCylinderFiniteWindowThroughContactCertificate,
}

/// Exact projective-root-owned isolated tangency of two skew cylinder
/// supports, proven inside or on an exact authored boundary of both finite
/// source windows.
#[derive(Debug, Clone, PartialEq)]
pub struct SkewCylinderSupportContact {
    certificate: PersistentSkewCylinderSupportContactCertificate,
    raw_point: SurfaceSurfacePoint,
}

/// One exact two-root folded support component represented by guarded sheet
/// members and their exact support-root and optional authored-seam joins.
#[derive(Debug, Clone, PartialEq)]
pub struct SkewCylinderFoldedSupportCurve {
    certificate: PersistentSkewCylinderFoldedSupportCertificate,
    source_reversed: bool,
}

impl SkewCylinderFoldedSupportCurve {
    /// Complete exact topology, finite-window, and paired residual proof.
    pub const fn certificate(&self) -> &PersistentSkewCylinderFoldedSupportCertificate {
        &self.certificate
    }

    /// Guarded residual members in caller source order.
    pub fn residuals(&self) -> Vec<PairedSkewCylinderBranchResidualCertificate> {
        self.certificate
            .formula_residuals()
            .iter()
            .map(|residual| {
                if self.source_reversed {
                    residual.swapped()
                } else {
                    *residual
                }
            })
            .collect()
    }

    /// The two exact support joins in increasing formula-longitude order.
    pub const fn endpoint_points(&self) -> [Point3; 2] {
        self.certificate.endpoint_points()
    }

    /// Caller-order parameters at the two exact support joins.
    pub fn source_endpoint_parameters(&self) -> [[[f64; 2]; 2]; 2] {
        self.certificate.source_endpoint_parameters()
    }
}

/// One exact repeated-root touching-support family represented by six guarded
/// sheet members and exact root, seam, and chart-transition joins.
#[derive(Debug, Clone, PartialEq)]
pub struct SkewCylinderTouchingSupportCurve {
    certificate: PersistentSkewCylinderTouchingSupportCertificate,
    source_reversed: bool,
}

impl SkewCylinderTouchingSupportCurve {
    /// Complete exact topology, finite-window, and paired residual proof.
    pub const fn certificate(&self) -> &PersistentSkewCylinderTouchingSupportCertificate {
        &self.certificate
    }

    /// Six guarded residual members in caller source order.
    pub fn residuals(&self) -> Vec<PairedSkewCylinderBranchResidualCertificate> {
        self.certificate
            .formula_residuals()
            .iter()
            .map(|residual| {
                if self.source_reversed {
                    residual.swapped()
                } else {
                    *residual
                }
            })
            .collect()
    }
}

impl SkewCylinderSupportContact {
    /// Sealed kgraph root, cell, and finite-window proof.
    pub const fn certificate(&self) -> &PersistentSkewCylinderSupportContactCertificate {
        &self.certificate
    }

    /// Deterministic representative of the exact projective point carrier.
    pub fn point(&self) -> Point3 {
        self.certificate.point()
    }

    /// Caller-order parameters on the two source cylinders.
    pub fn surface_parameters(&self) -> [[f64; 2]; 2] {
        self.certificate.source_surface_parameters()
    }

    /// Canonical raw tangent point retained by the legacy solver result.
    pub const fn raw_point(&self) -> SurfaceSurfacePoint {
        self.raw_point
    }

    fn mint(
        certificate: PersistentSkewCylinderSupportContactCertificate,
    ) -> Result<Self, IntersectionCertificateError> {
        let parameters = certificate.source_surface_parameters();
        let [first, second] = certificate.source_surface_points();
        let residual = first.dist(second);
        if !residual.is_finite() || residual > certificate.tolerance() {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
        Ok(Self {
            raw_point: SurfaceSurfacePoint {
                point: (first + second) * 0.5,
                uv_a: parameters[0],
                uv_b: parameters[1],
                residual,
                kind: ContactKind::Tangent,
            },
            certificate,
        })
    }
}

impl SkewCylinderThroughContact {
    /// Sealed kgraph proof owning the exact branch-attached event.
    pub const fn certificate(self) -> PersistentSkewCylinderFiniteWindowThroughContactCertificate {
        self.certificate
    }

    /// Exact analytic point on the represented branch.
    pub fn point(self) -> Point3 {
        self.certificate.point()
    }

    /// Caller-order parameters on the two source cylinders.
    pub fn surface_parameters(self) -> [[f64; 2]; 2] {
        self.certificate.source_surface_parameters()
    }

    /// Ordered quadratic sheet carrying this contact and its whole branch.
    pub const fn sheet(self) -> SkewCylinderSheet {
        self.certificate.sheet()
    }

    /// Number of exact authored-bound roots grouped into this contact.
    pub const fn root_count(self) -> usize {
        self.certificate.event_certificate().event().root_count()
    }

    /// Exact authored-bound root grouped into this contact.
    pub fn root(self, ordinal: usize) -> Option<PersistentSkewCylinderAxialRootEventInput> {
        self.certificate.root(ordinal)
    }

    fn mint(
        certificate: PersistentSkewCylinderFiniteWindowThroughContactCertificate,
    ) -> Result<Self, IntersectionCertificateError> {
        let [first, second] = certificate.source_surface_points();
        let residual = first.dist(second);
        let tolerance = certificate.family().tolerance();
        if !residual.is_finite() || residual > tolerance {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
        Ok(Self { certificate })
    }
}

/// Complete graph inputs produced by the exact skew-cylinder admission.
pub(super) struct CertifiedSkewCylinderIntersections {
    pub(super) raw: SurfaceSurfaceIntersections,
    pub(super) strict_miss: Option<SkewCylinderStrictDiscriminantMiss>,
    pub(super) branches: Option<Vec<CertifiedSkewCylinderBranch>>,
    pub(super) isolated_contacts: Vec<SkewCylinderIsolatedContact>,
    pub(super) through_contacts: Vec<SkewCylinderThroughContact>,
    pub(super) support_contacts: Vec<SkewCylinderSupportContact>,
    pub(super) folded_support_curves: Vec<SkewCylinderFoldedSupportCurve>,
    pub(super) touching_support_curves: Vec<SkewCylinderTouchingSupportCurve>,
}

/// Proof and exact endpoint evidence aligned with one canonicalized raw branch.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct CertifiedSkewCylinderBranch {
    pub(super) proof: CertifiedSkewCylinderBranchProof,
    pub(super) endpoint_proofs: [Option<IntersectionBranchEndpointProof>; 2],
}

/// Sealed whole-sheet or bounded-span proof retained through graph promotion.
#[derive(Debug, Clone, PartialEq)]
pub(super) enum CertifiedSkewCylinderBranchProof {
    TwoSheet(Box<PairedSkewCylinderBranchResidualCertificate>),
    WholeContact(Box<SkewCylinderWholeContactBranchCertificate>),
    OpenSpan(Box<SkewCylinderOpenSpanBranchCertificate>),
    FoldedSupport(Box<SkewCylinderFoldedSupportBranchCertificate>),
    TouchingSupport(Box<SkewCylinderTouchingSupportBranchCertificate>),
}

impl CertifiedSkewCylinderBranchProof {
    pub(super) fn residual(&self) -> PairedSkewCylinderBranchResidualCertificate {
        match self {
            Self::TwoSheet(certificate) => **certificate,
            Self::WholeContact(certificate) => certificate.residual_certificate(),
            Self::OpenSpan(certificate) => certificate.residual_certificate(),
            Self::FoldedSupport(certificate) => certificate.residual_certificate(),
            Self::TouchingSupport(certificate) => certificate.residual_certificate(),
        }
    }
}

impl SkewCylinderStrictDiscriminantMiss {
    const fn certified() -> Self {
        Self { _private: () }
    }
}

/// Classify one validated exact-nonparallel pair from a canonical source order.
pub(super) fn intersect_certified_skew_cylinders(
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
    tolerance: f64,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<CertifiedSkewCylinderIntersections> {
    validate_ranges(ranges[0], ranges[1])
        .map_err(IntersectionError::from)
        .map_err(GraphSurfaceIntersectionError::Intersection)?;
    let (cylinders, ranges, reversed) = canonical_pair(cylinders, ranges);
    if !axes_are_exactly_nonparallel(cylinders) {
        return Err(GraphSurfaceIntersectionError::BranchCertificate(
            kgraph::IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: NONPARALLEL_REASON,
            },
        ));
    }

    // The two deterministic parameterization attempts form one atomic proof
    // unit. A failed charge records the attempted N/N-1 crossing without
    // partially consuming this stage.
    scope.ledger_mut().charge(
        SKEW_CYLINDER_DISCRIMINANT_WORK,
        SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK,
    )?;

    let first_admission = classify_one_parameterization(cylinders);
    let (admission, parameterization_reversed) = match first_admission {
        admission @ (DiscriminantAdmission::StrictPositive(_)
        | DiscriminantAdmission::StrictNegative) => (admission, false),
        DiscriminantAdmission::NumericResolution => (
            classify_one_parameterization([cylinders[1], cylinders[0]]),
            true,
        ),
        DiscriminantAdmission::Contact(contact) => {
            let reversed = classify_one_parameterization([cylinders[1], cylinders[0]]);
            // A projection fold may look like Contact in one ruling chart
            // while the reverse chart proves two regular sheets. Conversely,
            // a contradictory reverse miss cannot supersede retained contact.
            match reversed {
                DiscriminantAdmission::StrictPositive(_) => (reversed, true),
                DiscriminantAdmission::Contact(reversed_contact)
                    if prefers_double_touching_chart_roots(&reversed_contact, &contact) =>
                {
                    (DiscriminantAdmission::Contact(reversed_contact), true)
                }
                DiscriminantAdmission::Contact(reversed_contact)
                    if cylinders[0].radius() > cylinders[1].radius() =>
                {
                    // Folded/contact publication uses the smaller-radius
                    // cylinder as its ruling carrier whenever both exact
                    // parameterizations retain Contact. Radius order is
                    // invariant under operand swap and rigid motion, unlike
                    // the storage-order tie-break used by the general
                    // canonical dispatcher.
                    (DiscriminantAdmission::Contact(reversed_contact), true)
                }
                DiscriminantAdmission::StrictNegative
                | DiscriminantAdmission::Contact(_)
                | DiscriminantAdmission::NumericResolution => {
                    (DiscriminantAdmission::Contact(contact), false)
                }
            }
        }
    };

    match admission {
        DiscriminantAdmission::StrictNegative => Ok(CertifiedSkewCylinderIntersections {
            raw: SurfaceSurfaceIntersections::complete_empty(),
            strict_miss: Some(SkewCylinderStrictDiscriminantMiss::certified()),
            branches: None,
            isolated_contacts: Vec::new(),
            through_contacts: Vec::new(),
            support_contacts: Vec::new(),
            folded_support_curves: Vec::new(),
            touching_support_curves: Vec::new(),
        }),
        DiscriminantAdmission::StrictPositive(strict_positive) => {
            let (proof_cylinders, proof_ranges) = if parameterization_reversed {
                ([cylinders[1], cylinders[0]], [ranges[1], ranges[0]])
            } else {
                (cylinders, ranges)
            };
            intersect_strict_positive_two_sheet(
                strict_positive,
                proof_cylinders,
                proof_ranges,
                reversed ^ parameterization_reversed,
                tolerance,
                scope,
            )
        }
        DiscriminantAdmission::Contact(contact) => {
            let (proof_ranges, source_reversed) = if parameterization_reversed {
                ([ranges[1], ranges[0]], reversed ^ true)
            } else {
                (ranges, reversed)
            };
            intersect_isolated_support_contact(
                *contact,
                proof_ranges,
                source_reversed,
                tolerance,
                scope,
            )
        }
        DiscriminantAdmission::NumericResolution => Ok(CertifiedSkewCylinderIntersections {
            raw: numeric_resolution(scope, SKEW_CYLINDER_DISCRIMINANT_WORK),
            strict_miss: None,
            branches: None,
            isolated_contacts: Vec::new(),
            through_contacts: Vec::new(),
            support_contacts: Vec::new(),
            folded_support_curves: Vec::new(),
            touching_support_curves: Vec::new(),
        }),
    }
}

fn prefers_double_touching_chart_roots(
    candidate: &kgraph::SkewCylinderDiscriminantContactTopologyCertificate,
    current: &kgraph::SkewCylinderDiscriminantContactTopologyCertificate,
) -> bool {
    is_double_touching_chart_root_layout(candidate)
        && !is_double_touching_chart_root_layout(current)
}

fn is_double_touching_chart_root_layout(
    topology: &kgraph::SkewCylinderDiscriminantContactTopologyCertificate,
) -> bool {
    let Ok(topology) = certify_skew_cylinder_touching_support_topology(topology.clone()) else {
        return false;
    };
    let [first, second] = topology.roots() else {
        return false;
    };
    let first = first.angular_bracket();
    let second = second.angular_bracket();
    first.lo.to_bits() == core::f64::consts::FRAC_PI_2.to_bits()
        && first.hi.to_bits() == core::f64::consts::FRAC_PI_2.to_bits()
        && second.lo.to_bits() == (3.0 * core::f64::consts::FRAC_PI_2).to_bits()
        && second.hi.to_bits() == (3.0 * core::f64::consts::FRAC_PI_2).to_bits()
}

fn intersect_isolated_support_contact(
    contact: kgraph::SkewCylinderDiscriminantContactTopologyCertificate,
    formula_ranges: [[ParamRange; 2]; 2],
    source_reversed: bool,
    tolerance: f64,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<CertifiedSkewCylinderIntersections> {
    let formula_to_source = if source_reversed { [1, 0] } else { [0, 1] };
    let boundary_plan = match plan_persistent_skew_cylinder_support_contact_boundaries(
        &contact,
        formula_ranges,
        formula_to_source,
        tolerance,
    ) {
        Ok(plan) => plan,
        Err(_) => {
            return intersect_folded_support_contact(
                contact,
                formula_ranges,
                formula_to_source,
                source_reversed,
                tolerance,
                scope,
            );
        }
    };
    if boundary_plan.work() > 0 {
        scope
            .ledger_mut()
            .charge(SKEW_CYLINDER_ROOT_CLUSTER_WORK, boundary_plan.work())?;
    }
    let certificate = match certify_persistent_skew_cylinder_support_contact(
        contact.clone(),
        formula_ranges,
        formula_to_source,
        tolerance,
        boundary_plan.work(),
    ) {
        Ok(certificate) => certificate,
        Err(_) => {
            return intersect_folded_support_contact(
                contact,
                formula_ranges,
                formula_to_source,
                source_reversed,
                tolerance,
                scope,
            );
        }
    };
    let support = SkewCylinderSupportContact::mint(certificate)
        .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;
    publish_skew_topology(Vec::new(), Vec::new(), Vec::new(), vec![support])
}

fn intersect_folded_support_contact(
    contact: kgraph::SkewCylinderDiscriminantContactTopologyCertificate,
    formula_ranges: [[ParamRange; 2]; 2],
    formula_to_source: [usize; 2],
    source_reversed: bool,
    tolerance: f64,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<CertifiedSkewCylinderIntersections> {
    let topology = match certify_skew_cylinder_folded_support_topology(contact.clone()) {
        Ok(topology) => topology,
        Err(_) => {
            return intersect_touching_support_contact(
                contact,
                formula_ranges,
                formula_to_source,
                source_reversed,
                tolerance,
                scope,
            );
        }
    };
    let folded_work = match topology.positive_cell() {
        SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots => {
            SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK
        }
        SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam => {
            SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK
        }
    };
    scope
        .ledger_mut()
        .charge(SKEW_CYLINDER_OPEN_SPAN_WORK, folded_work)?;
    let certificate = match certify_persistent_skew_cylinder_folded_support(
        topology,
        formula_ranges,
        formula_to_source,
        tolerance,
        folded_work,
    ) {
        Ok(certificate) => certificate,
        Err(_) => {
            return Ok(CertifiedSkewCylinderIntersections {
                raw: contact_topology_incomplete(scope),
                strict_miss: None,
                branches: None,
                isolated_contacts: Vec::new(),
                through_contacts: Vec::new(),
                support_contacts: Vec::new(),
                folded_support_curves: Vec::new(),
                touching_support_curves: Vec::new(),
            });
        }
    };
    let folded = SkewCylinderFoldedSupportCurve {
        certificate: certificate.clone(),
        source_reversed,
    };
    let roots = certificate.topology().roots();
    let residuals = folded.residuals();
    let branches = residuals
        .into_iter()
        .zip(certificate.formula_branch_endpoints())
        .map(|(residual, branch_endpoints)| {
            let range = residual.carrier_range();
            let endpoint_proofs = branch_endpoints.map(|endpoint| {
                let inside_parameter = if endpoint == branch_endpoints[0] {
                    range.lo
                } else {
                    range.hi
                };
                Some(match endpoint {
                    PersistentSkewCylinderFoldedSupportEndpoint::Root(root_ordinal) => {
                        let root = roots[root_ordinal].bracket();
                        IntersectionBranchEndpointProof::SkewCylinderFoldedSupportRoot(
                            SkewCylinderFoldedSupportRootEndpointProof {
                                root_ordinal,
                                half_angle_chart: match root.chart {
                                    SkewCylinderHalfAngleChart::Tangent => {
                                        SkewCylinderHalfAngleChartProof::Tangent
                                    }
                                    SkewCylinderHalfAngleChart::Cotangent => {
                                        SkewCylinderHalfAngleChartProof::Cotangent
                                    }
                                },
                                half_angle_bracket: [root.lo, root.hi],
                                inside_parameter,
                                point: certificate.endpoint_point(endpoint),
                                surface_parameters: certificate.source_parameters(endpoint),
                            },
                        )
                    }
                    PersistentSkewCylinderFoldedSupportEndpoint::Seam(sheet) => {
                        IntersectionBranchEndpointProof::SkewCylinderFoldedSupportSeam(
                            SkewCylinderFoldedSupportSeamEndpointProof {
                                sheet,
                                inside_parameter,
                                point: certificate.endpoint_point(endpoint),
                                surface_parameters: certificate.source_parameters(endpoint),
                            },
                        )
                    }
                })
            });
            CertifiedSkewCylinderBranch {
                proof: CertifiedSkewCylinderBranchProof::FoldedSupport(Box::new(
                    SkewCylinderFoldedSupportBranchCertificate::mint(residual, certificate.clone())
                        .expect(
                            "folded source-order residual is retained by its shared certificate",
                        ),
                )),
                endpoint_proofs,
            }
        })
        .collect();
    publish_skew_topology_with_folded(branches, Vec::new(), Vec::new(), Vec::new(), vec![folded])
}

fn intersect_touching_support_contact(
    contact: kgraph::SkewCylinderDiscriminantContactTopologyCertificate,
    formula_ranges: [[ParamRange; 2]; 2],
    formula_to_source: [usize; 2],
    source_reversed: bool,
    tolerance: f64,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<CertifiedSkewCylinderIntersections> {
    let topology = match certify_skew_cylinder_touching_support_topology(contact) {
        Ok(topology) => topology,
        Err(_) => return Ok(contact_topology_result_incomplete(scope)),
    };
    scope.ledger_mut().charge(
        SKEW_CYLINDER_OPEN_SPAN_WORK,
        SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK,
    )?;
    let certificate = match certify_persistent_skew_cylinder_touching_support(
        topology,
        formula_ranges,
        formula_to_source,
        tolerance,
        SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK,
    ) {
        Ok(certificate) => certificate,
        Err(_) => return Ok(contact_topology_result_incomplete(scope)),
    };
    let touching = SkewCylinderTouchingSupportCurve {
        certificate: certificate.clone(),
        source_reversed,
    };
    let roots = certificate.topology().roots();
    let branches = touching
        .residuals()
        .into_iter()
        .zip(certificate.formula_branch_endpoints())
        .map(|(residual, branch_endpoints)| {
            let range = residual.carrier_range();
            let endpoint_proofs = branch_endpoints.map(|endpoint| {
                let inside_parameter = if endpoint == branch_endpoints[0] {
                    range.lo
                } else {
                    range.hi
                };
                Some(match endpoint {
                    PersistentSkewCylinderTouchingSupportEndpoint::Root {
                        root: root_identity,
                        continuation,
                    } => {
                        let root = roots[usize::from(root_identity.ordinal())].bracket();
                        IntersectionBranchEndpointProof::SkewCylinderTouchingSupportRoot(
                            SkewCylinderTouchingSupportRootEndpointProof {
                                root: root_identity,
                                continuation,
                                half_angle_chart: match root.chart {
                                    SkewCylinderHalfAngleChart::Tangent => {
                                        SkewCylinderHalfAngleChartProof::Tangent
                                    }
                                    SkewCylinderHalfAngleChart::Cotangent => {
                                        SkewCylinderHalfAngleChartProof::Cotangent
                                    }
                                },
                                half_angle_bracket: [root.lo, root.hi],
                                inside_parameter,
                                point: certificate.endpoint_point(endpoint),
                                surface_parameters: certificate.source_parameters(endpoint),
                            },
                        )
                    }
                    PersistentSkewCylinderTouchingSupportEndpoint::Seam(sheet) => {
                        IntersectionBranchEndpointProof::SkewCylinderTouchingSupportSeam(
                            SkewCylinderTouchingSupportSeamEndpointProof {
                                sheet,
                                inside_parameter,
                                point: certificate.endpoint_point(endpoint),
                                surface_parameters: certificate.source_parameters(endpoint),
                            },
                        )
                    }
                    PersistentSkewCylinderTouchingSupportEndpoint::ChartJoin { sheet, join } => {
                        IntersectionBranchEndpointProof::SkewCylinderTouchingSupportChartJoin(
                            SkewCylinderTouchingSupportChartJoinEndpointProof {
                                sheet,
                                join,
                                longitude: certificate
                                    .chart_join_longitude_for(join)
                                    .expect("endpoint retains one certificate-owned chart join"),
                                inside_parameter,
                                point: certificate.endpoint_point(endpoint),
                                surface_parameters: certificate.source_parameters(endpoint),
                            },
                        )
                    }
                })
            });
            CertifiedSkewCylinderBranch {
                proof: CertifiedSkewCylinderBranchProof::TouchingSupport(Box::new(
                    SkewCylinderTouchingSupportBranchCertificate::mint(
                        residual,
                        certificate.clone(),
                    )
                    .expect("touching source-order residual is retained by its certificate"),
                )),
                endpoint_proofs,
            }
        })
        .collect();
    publish_skew_topology_with_support_curves(
        branches,
        Vec::new(),
        Vec::new(),
        Vec::new(),
        Vec::new(),
        vec![touching],
    )
}

fn contact_topology_result_incomplete(
    scope: &mut OperationScope<'_, '_>,
) -> CertifiedSkewCylinderIntersections {
    CertifiedSkewCylinderIntersections {
        raw: contact_topology_incomplete(scope),
        strict_miss: None,
        branches: None,
        isolated_contacts: Vec::new(),
        through_contacts: Vec::new(),
        support_contacts: Vec::new(),
        folded_support_curves: Vec::new(),
        touching_support_curves: Vec::new(),
    }
}

fn intersect_strict_positive_two_sheet(
    strict_positive: SkewCylinderStrictPositiveTwoSheetAdmissionCertificate,
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
    reversed: bool,
    tolerance: f64,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<CertifiedSkewCylinderIntersections> {
    if strict_positive.formula_cylinders() != cylinders {
        return Err(GraphSurfaceIntersectionError::BranchCertificate(
            IntersectionCertificateError::InvalidTraceFamily,
        ));
    }
    scope.ledger_mut().charge(
        SKEW_CYLINDER_TWO_SHEET_WORK,
        SKEW_CYLINDER_TWO_SHEET_EXACT_WORK,
    )?;
    let certified = [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper].map(|sheet| {
        certify_paired_skew_cylinder_branch_residuals(cylinders, ranges, sheet, tolerance)
    });
    if let [Ok(lower), Ok(upper)] = &certified {
        return publish_whole_sheets(vec![*lower, *upper], reversed);
    }

    if ranges
        .iter()
        .any(|window| window[0].width() != core::f64::consts::TAU)
    {
        return Ok(branch_certificate_failure(&certified, scope));
    }
    scope.ledger_mut().charge(
        SKEW_CYLINDER_AXIAL_CLIP_WORK,
        SKEW_CYLINDER_AXIAL_CLIP_EXACT_WORK,
    )?;
    let canonical_to_source = if reversed { [1, 0] } else { [0, 1] };
    let topologies = match collect_skew_cylinder_axial_bound_topologies(
        cylinders,
        ranges,
        canonical_to_source,
    ) {
        Ok(occupancy) => occupancy,
        Err(SkewCylinderAxialRootFailure::IdenticallyOnBound) => {
            return Ok(CertifiedSkewCylinderIntersections {
                raw: clipped_topology_incomplete(scope),
                strict_miss: None,
                branches: None,
                isolated_contacts: Vec::new(),
                through_contacts: Vec::new(),
                support_contacts: Vec::new(),
                folded_support_curves: Vec::new(),
                touching_support_curves: Vec::new(),
            });
        }
        Err(_) => {
            return Ok(CertifiedSkewCylinderIntersections {
                raw: numeric_resolution(scope, SKEW_CYLINDER_AXIAL_CLIP_WORK),
                strict_miss: None,
                branches: None,
                isolated_contacts: Vec::new(),
                through_contacts: Vec::new(),
                support_contacts: Vec::new(),
                folded_support_curves: Vec::new(),
                touching_support_curves: Vec::new(),
            });
        }
    };
    let topology_input = SkewCylinderOpenSpanTopologyInput {
        topologies: &topologies,
        ranges,
        canonical_to_source,
        coincidence_tolerance: tolerance,
    };
    let root_cluster_plan = match plan_skew_cylinder_root_clusters(topology_input) {
        Ok(plan) => plan,
        Err(_) => {
            return Ok(CertifiedSkewCylinderIntersections {
                raw: clipped_topology_incomplete(scope),
                strict_miss: None,
                branches: None,
                isolated_contacts: Vec::new(),
                through_contacts: Vec::new(),
                support_contacts: Vec::new(),
                folded_support_curves: Vec::new(),
                touching_support_curves: Vec::new(),
            });
        }
    };
    if root_cluster_plan.work() > 0 {
        scope
            .ledger_mut()
            .charge(SKEW_CYLINDER_ROOT_CLUSTER_WORK, root_cluster_plan.work())?;
    }
    let finite_topology = match classify_skew_cylinder_open_spans(topology_input) {
        Ok(topology) => topology,
        Err(SkewCylinderOpenSpanFailure::ExactRootRelationIndeterminate) => {
            return Ok(CertifiedSkewCylinderIntersections {
                raw: numeric_resolution(scope, SKEW_CYLINDER_ROOT_CLUSTER_WORK),
                strict_miss: None,
                branches: None,
                isolated_contacts: Vec::new(),
                through_contacts: Vec::new(),
                support_contacts: Vec::new(),
                folded_support_curves: Vec::new(),
                touching_support_curves: Vec::new(),
            });
        }
        Err(_) => {
            return Ok(CertifiedSkewCylinderIntersections {
                raw: clipped_topology_incomplete(scope),
                strict_miss: None,
                branches: None,
                isolated_contacts: Vec::new(),
                through_contacts: Vec::new(),
                support_contacts: Vec::new(),
                folded_support_curves: Vec::new(),
                touching_support_curves: Vec::new(),
            });
        }
    };
    if finite_topology.root_cluster_query_plan() != root_cluster_plan {
        return Ok(CertifiedSkewCylinderIntersections {
            raw: clipped_topology_incomplete(scope),
            strict_miss: None,
            branches: None,
            isolated_contacts: Vec::new(),
            through_contacts: Vec::new(),
            support_contacts: Vec::new(),
            folded_support_curves: Vec::new(),
            touching_support_curves: Vec::new(),
        });
    }
    for event in [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper]
        .into_iter()
        .flat_map(|sheet| finite_topology.root_events(sheet))
    {
        match event.kind() {
            SkewCylinderFiniteWindowRootEventKind::Boundary if event.root_count() == 1 => {}
            SkewCylinderFiniteWindowRootEventKind::Isolated if event.root_count() > 0 => {}
            SkewCylinderFiniteWindowRootEventKind::Contact if event.root_count() > 0 => {}
            SkewCylinderFiniteWindowRootEventKind::Contact => {
                return Ok(CertifiedSkewCylinderIntersections {
                    raw: contact_topology_incomplete(scope),
                    strict_miss: None,
                    branches: None,
                    isolated_contacts: Vec::new(),
                    through_contacts: Vec::new(),
                    support_contacts: Vec::new(),
                    folded_support_curves: Vec::new(),
                    touching_support_curves: Vec::new(),
                });
            }
            SkewCylinderFiniteWindowRootEventKind::Boundary
            | SkewCylinderFiniteWindowRootEventKind::Isolated => {
                return Ok(CertifiedSkewCylinderIntersections {
                    raw: clipped_topology_incomplete(scope),
                    strict_miss: None,
                    branches: None,
                    isolated_contacts: Vec::new(),
                    through_contacts: Vec::new(),
                    support_contacts: Vec::new(),
                    folded_support_curves: Vec::new(),
                    touching_support_curves: Vec::new(),
                });
            }
        }
    }
    publish_finite_window_topology(
        strict_positive,
        finite_topology,
        certified,
        cylinders,
        ranges,
        reversed,
        tolerance,
        scope,
    )
}

#[allow(clippy::too_many_arguments)]
fn publish_finite_window_topology(
    strict_positive: SkewCylinderStrictPositiveTwoSheetAdmissionCertificate,
    finite_topology: SkewCylinderFiniteWindowTopologyCertificate,
    certified: [Result<PairedSkewCylinderBranchResidualCertificate, IntersectionCertificateError>;
        2],
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
    reversed: bool,
    tolerance: f64,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<CertifiedSkewCylinderIntersections> {
    let sheets = [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper];
    let open_span_count = sheets
        .into_iter()
        .map(|sheet| match finite_topology.sheet(sheet) {
            SkewCylinderFiniteSheetTopology::Open(spans) => spans.len(),
            SkewCylinderFiniteSheetTopology::Outside | SkewCylinderFiniteSheetTopology::Whole => 0,
        })
        .sum::<usize>();
    let isolated_count = sheets
        .into_iter()
        .map(|sheet| {
            finite_topology
                .root_events(sheet)
                .iter()
                .filter(|event| event.kind() == SkewCylinderFiniteWindowRootEventKind::Isolated)
                .count()
        })
        .sum::<usize>();
    let through_contact_count = sheets
        .into_iter()
        .map(|sheet| {
            finite_topology
                .root_events(sheet)
                .iter()
                .filter(|event| event.kind() == SkewCylinderFiniteWindowRootEventKind::Contact)
                .count()
        })
        .sum::<usize>();
    if open_span_count > 0 {
        scope.ledger_mut().charge(
            SKEW_CYLINDER_OPEN_SPAN_WORK,
            SKEW_CYLINDER_OPEN_SPAN_EXACT_WORK_PER_BRANCH * open_span_count as u64,
        )?;
    }
    let whole_contact_recertification_count = sheets
        .into_iter()
        .zip(certified.iter())
        .filter(|(sheet, certificate)| {
            certificate.is_err()
                && matches!(
                    finite_topology.sheet(*sheet),
                    SkewCylinderFiniteSheetTopology::Whole
                )
                && finite_topology
                    .root_events(*sheet)
                    .iter()
                    .any(|event| event.kind() == SkewCylinderFiniteWindowRootEventKind::Contact)
        })
        .count();
    if whole_contact_recertification_count > 0 {
        scope.ledger_mut().charge(
            SKEW_CYLINDER_TWO_SHEET_WORK,
            SKEW_CYLINDER_THROUGH_CONTACT_EXACT_WORK_PER_BRANCH
                * whole_contact_recertification_count as u64,
        )?;
    }

    let mut branches = Vec::with_capacity(2 + open_span_count);
    let mut family_members = Vec::with_capacity(open_span_count);
    for (sheet, whole_certificate) in sheets.into_iter().zip(certified.iter()) {
        match finite_topology.sheet(sheet) {
            SkewCylinderFiniteSheetTopology::Outside => {}
            SkewCylinderFiniteSheetTopology::Whole => {
                let has_contact = finite_topology
                    .root_events(sheet)
                    .iter()
                    .any(|event| event.kind() == SkewCylinderFiniteWindowRootEventKind::Contact);
                let certificate = match whole_certificate {
                    Ok(certificate) => *certificate,
                    Err(_) if has_contact => {
                        match certify_paired_skew_cylinder_branch_residuals(
                            cylinders,
                            contact_residual_ranges(ranges),
                            sheet,
                            tolerance,
                        ) {
                            Ok(certificate) => certificate,
                            Err(failure) => {
                                return Ok(single_branch_certificate_failure(&failure, scope));
                            }
                        }
                    }
                    Err(failure) => return Ok(single_branch_certificate_failure(failure, scope)),
                };
                branches.push(CertifiedSkewCylinderBranch {
                    proof: CertifiedSkewCylinderBranchProof::TwoSheet(Box::new(if reversed {
                        certificate.swapped()
                    } else {
                        certificate
                    })),
                    endpoint_proofs: [None; 2],
                });
            }
            SkewCylinderFiniteSheetTopology::Open(spans) => {
                let has_contact = finite_topology
                    .root_events(sheet)
                    .iter()
                    .any(|event| event.kind() == SkewCylinderFiniteWindowRootEventKind::Contact);
                for span in spans.iter().copied() {
                    if span.sheet != sheet {
                        return Ok(CertifiedSkewCylinderIntersections {
                            raw: numeric_resolution(scope, SKEW_CYLINDER_AXIAL_CLIP_WORK),
                            strict_miss: None,
                            branches: None,
                            isolated_contacts: Vec::new(),
                            through_contacts: Vec::new(),
                            support_contacts: Vec::new(),
                            folded_support_curves: Vec::new(),
                            touching_support_curves: Vec::new(),
                        });
                    }
                    let open_span = match certify_open_span_pcurve_transport(
                        cylinders,
                        if has_contact {
                            contact_residual_ranges(ranges)
                        } else {
                            ranges
                        },
                        ranges[0][0],
                        span,
                        reversed,
                        tolerance,
                    ) {
                        Ok(certificate) => certificate,
                        Err(failure) => {
                            return Ok(open_span_certificate_failure(&failure, scope));
                        }
                    };
                    family_members.push(PersistentSkewCylinderFiniteWindowMemberInput {
                        residual: open_span.residual_certificate(),
                        root_corridors: open_span.root_corridors(),
                    });
                    branches.push(CertifiedSkewCylinderBranch {
                        proof: CertifiedSkewCylinderBranchProof::OpenSpan(Box::new(open_span)),
                        endpoint_proofs: [span.start, span.end].map(graph_endpoint_proof),
                    });
                }
            }
        }
    }
    let mut isolated_contacts = Vec::with_capacity(isolated_count);
    let mut through_contacts = Vec::with_capacity(through_contact_count);
    if open_span_count > 0 || isolated_count > 0 || through_contact_count > 0 {
        let family = match certify_persistent_skew_cylinder_finite_window_family(
            strict_positive,
            &finite_topology,
            &family_members,
            tolerance,
        ) {
            Ok(family) => family,
            Err(failure) => return Ok(open_span_certificate_failure(&failure, scope)),
        };
        let mut ordinal = 0;
        for branch in &mut branches {
            if let CertifiedSkewCylinderBranchProof::OpenSpan(certificate) = &mut branch.proof {
                let membership = family.membership(ordinal).ok_or(
                    GraphSurfaceIntersectionError::BranchCertificate(
                        IntersectionCertificateError::InvalidTraceFamily,
                    ),
                )?;
                **certificate = certificate
                    .bind_finite_window_family(membership)
                    .map_err(GraphSurfaceIntersectionError::BranchCertificate)?;
                ordinal += 1;
            }
        }
        if ordinal != family.member_count() {
            return Err(GraphSurfaceIntersectionError::BranchCertificate(
                IntersectionCertificateError::InvalidTraceFamily,
            ));
        }
        for branch in &mut branches {
            let whole_contact = match &branch.proof {
                CertifiedSkewCylinderBranchProof::TwoSheet(residual)
                    if (0..family.root_event_count(residual.sheet())).any(|ordinal| {
                        family
                            .root_event(residual.sheet(), ordinal)
                            .is_some_and(|event| {
                                event.kind()
                                == kgraph::PersistentSkewCylinderFiniteWindowRootEventKind::Contact
                            })
                    }) =>
                {
                    Some(SkewCylinderWholeContactBranchCertificate::mint(
                        **residual, family,
                    ))
                }
                CertifiedSkewCylinderBranchProof::TwoSheet(_)
                | CertifiedSkewCylinderBranchProof::WholeContact(_)
                | CertifiedSkewCylinderBranchProof::OpenSpan(_)
                | CertifiedSkewCylinderBranchProof::FoldedSupport(_)
                | CertifiedSkewCylinderBranchProof::TouchingSupport(_) => None,
            };
            if let Some(certificate) = whole_contact {
                branch.proof = CertifiedSkewCylinderBranchProof::WholeContact(Box::new(
                    certificate.map_err(GraphSurfaceIntersectionError::BranchCertificate)?,
                ));
            }
        }
        for sheet in sheets {
            for event_ordinal in 0..family.root_event_count(sheet) {
                let Some(event) = family.root_event(sheet, event_ordinal) else {
                    return Err(GraphSurfaceIntersectionError::BranchCertificate(
                        IntersectionCertificateError::InvalidTraceFamily,
                    ));
                };
                match event.kind() {
                    kgraph::PersistentSkewCylinderFiniteWindowRootEventKind::Isolated => {
                        let certificate =
                            family
                                .isolated_point_certificate(sheet, event_ordinal)
                                .ok_or(GraphSurfaceIntersectionError::BranchCertificate(
                                    IntersectionCertificateError::InvalidTraceFamily,
                                ))?;
                        isolated_contacts.push(
                            SkewCylinderIsolatedContact::mint(certificate)
                                .map_err(GraphSurfaceIntersectionError::BranchCertificate)?,
                        );
                    }
                    kgraph::PersistentSkewCylinderFiniteWindowRootEventKind::Contact => {
                        let certificate = family
                            .through_contact_certificate(sheet, event_ordinal)
                            .ok_or(GraphSurfaceIntersectionError::BranchCertificate(
                                IntersectionCertificateError::InvalidTraceFamily,
                            ))?;
                        through_contacts.push(
                            SkewCylinderThroughContact::mint(certificate)
                                .map_err(GraphSurfaceIntersectionError::BranchCertificate)?,
                        );
                    }
                    kgraph::PersistentSkewCylinderFiniteWindowRootEventKind::Boundary => {}
                }
            }
        }
        if isolated_contacts.len() != isolated_count
            || through_contacts.len() != through_contact_count
        {
            return Err(GraphSurfaceIntersectionError::BranchCertificate(
                IntersectionCertificateError::InvalidTraceFamily,
            ));
        }
    }
    publish_skew_topology(branches, isolated_contacts, through_contacts, Vec::new())
}

fn contact_residual_ranges(mut ranges: [[ParamRange; 2]; 2]) -> [[ParamRange; 2]; 2] {
    // The existing trusted residual theorem needs strict axial enclosure. The
    // exact finite-window family separately proves closed authored-window
    // occupancy, so this auxiliary proof uses one finite universal envelope
    // only for evaluator/residual certification; it never changes topology.
    let bound = f64::MAX / 4.0;
    for window in &mut ranges {
        window[1] = ParamRange::new(-bound, bound);
    }
    ranges
}

fn certify_open_span_pcurve_transport(
    cylinders: [Cylinder; 2],
    residual_ranges: [[ParamRange; 2]; 2],
    authored_longitude: ParamRange,
    span: SkewCylinderOpenSpan,
    reversed: bool,
    tolerance: f64,
) -> Result<SkewCylinderOpenSpanBranchCertificate, IntersectionCertificateError> {
    let certificate = certify_paired_skew_cylinder_branch_subrange_residuals(
        cylinders,
        residual_ranges,
        span.range,
        span.sheet,
        tolerance,
    )?;
    let certificate = if reversed {
        certificate.swapped()
    } else {
        certificate
    };
    let [lower_root, upper_root] = span.root_longitude_intervals(authored_longitude).ok_or(
        IntersectionCertificateError::UnsupportedCarrierParameterization {
            reason: ROOT_CORRIDOR_REASON,
        },
    )?;
    let lower_corridor = certificate.certify_lower_pcurve_root_corridor(lower_root)?;
    let upper_corridor = certificate.certify_upper_pcurve_root_corridor(upper_root)?;
    SkewCylinderOpenSpanBranchCertificate::mint(certificate, [lower_corridor, upper_corridor])
}

fn publish_whole_sheets(
    certificates: Vec<PairedSkewCylinderBranchResidualCertificate>,
    reversed: bool,
) -> GraphSurfaceIntersectionResult<CertifiedSkewCylinderIntersections> {
    publish_skew_topology(
        certificates
            .into_iter()
            .map(|certificate| CertifiedSkewCylinderBranch {
                proof: CertifiedSkewCylinderBranchProof::TwoSheet(Box::new(if reversed {
                    certificate.swapped()
                } else {
                    certificate
                })),
                endpoint_proofs: [None; 2],
            })
            .collect(),
        Vec::new(),
        Vec::new(),
        Vec::new(),
    )
}

fn publish_skew_topology(
    branches: Vec<CertifiedSkewCylinderBranch>,
    isolated_contacts: Vec<SkewCylinderIsolatedContact>,
    through_contacts: Vec<SkewCylinderThroughContact>,
    support_contacts: Vec<SkewCylinderSupportContact>,
) -> GraphSurfaceIntersectionResult<CertifiedSkewCylinderIntersections> {
    publish_skew_topology_with_folded(
        branches,
        isolated_contacts,
        through_contacts,
        support_contacts,
        Vec::new(),
    )
}

fn publish_skew_topology_with_folded(
    branches: Vec<CertifiedSkewCylinderBranch>,
    isolated_contacts: Vec<SkewCylinderIsolatedContact>,
    through_contacts: Vec<SkewCylinderThroughContact>,
    support_contacts: Vec<SkewCylinderSupportContact>,
    folded_support_curves: Vec<SkewCylinderFoldedSupportCurve>,
) -> GraphSurfaceIntersectionResult<CertifiedSkewCylinderIntersections> {
    publish_skew_topology_with_support_curves(
        branches,
        isolated_contacts,
        through_contacts,
        support_contacts,
        folded_support_curves,
        Vec::new(),
    )
}

fn publish_skew_topology_with_support_curves(
    branches: Vec<CertifiedSkewCylinderBranch>,
    isolated_contacts: Vec<SkewCylinderIsolatedContact>,
    through_contacts: Vec<SkewCylinderThroughContact>,
    support_contacts: Vec<SkewCylinderSupportContact>,
    folded_support_curves: Vec<SkewCylinderFoldedSupportCurve>,
    touching_support_curves: Vec<SkewCylinderTouchingSupportCurve>,
) -> GraphSurfaceIntersectionResult<CertifiedSkewCylinderIntersections> {
    if !isolated_contacts.is_empty() && !support_contacts.is_empty() {
        return Err(GraphSurfaceIntersectionError::BranchCertificate(
            IntersectionCertificateError::InvalidTraceFamily,
        ));
    }
    let points = isolated_contacts
        .iter()
        .map(|contact| contact.raw_point())
        .chain(
            support_contacts
                .iter()
                .map(SkewCylinderSupportContact::raw_point),
        )
        .collect::<Vec<_>>();
    let curves = branches
        .iter()
        .map(|branch| raw_skew_curve(&branch.proof.residual()))
        .collect::<Vec<_>>();
    let raw = if points.is_empty() && curves.is_empty() {
        SurfaceSurfaceIntersections::complete_empty()
    } else {
        SurfaceSurfaceIntersections::canonicalized_complete(points, curves)
            .map_err(IntersectionError::from)
            .map_err(GraphSurfaceIntersectionError::Intersection)?
    };
    let branches = align_skew_branches(&raw, branches)?;
    let isolated_contacts = if support_contacts.is_empty() {
        align_skew_isolated_contacts(&raw, isolated_contacts)?
    } else {
        isolated_contacts
    };
    let support_contacts = if isolated_contacts.is_empty() {
        align_skew_support_contacts(&raw, support_contacts)?
    } else {
        support_contacts
    };
    Ok(CertifiedSkewCylinderIntersections {
        raw,
        strict_miss: None,
        branches: Some(branches),
        isolated_contacts,
        through_contacts,
        support_contacts,
        folded_support_curves,
        touching_support_curves,
    })
}

fn align_skew_support_contacts(
    raw: &SurfaceSurfaceIntersections,
    mut contacts: Vec<SkewCylinderSupportContact>,
) -> GraphSurfaceIntersectionResult<Vec<SkewCylinderSupportContact>> {
    let mut aligned = Vec::with_capacity(raw.points.len());
    for point in &raw.points {
        let mut matches = contacts
            .iter()
            .enumerate()
            .filter(|(_, contact)| contact.raw_point() == *point);
        let Some((index, _)) = matches.next() else {
            return Err(GraphSurfaceIntersectionError::BranchCertificate(
                IntersectionCertificateError::InvalidTraceFamily,
            ));
        };
        if matches.next().is_some() {
            return Err(GraphSurfaceIntersectionError::BranchCertificate(
                IntersectionCertificateError::InvalidTraceFamily,
            ));
        }
        drop(matches);
        aligned.push(contacts.remove(index));
    }
    if !contacts.is_empty() {
        return Err(GraphSurfaceIntersectionError::BranchCertificate(
            IntersectionCertificateError::InvalidTraceFamily,
        ));
    }
    Ok(aligned)
}

fn align_skew_isolated_contacts(
    raw: &SurfaceSurfaceIntersections,
    mut contacts: Vec<SkewCylinderIsolatedContact>,
) -> GraphSurfaceIntersectionResult<Vec<SkewCylinderIsolatedContact>> {
    let mut aligned = Vec::with_capacity(raw.points.len());
    for point in &raw.points {
        let mut matches = contacts
            .iter()
            .enumerate()
            .filter(|(_, contact)| contact.raw_point() == *point);
        let Some((index, _)) = matches.next() else {
            return Err(GraphSurfaceIntersectionError::BranchCertificate(
                IntersectionCertificateError::InvalidTraceFamily,
            ));
        };
        if matches.next().is_some() {
            return Err(GraphSurfaceIntersectionError::BranchCertificate(
                IntersectionCertificateError::InvalidTraceFamily,
            ));
        }
        drop(matches);
        aligned.push(contacts.remove(index));
    }
    if !contacts.is_empty() {
        return Err(GraphSurfaceIntersectionError::BranchCertificate(
            IntersectionCertificateError::InvalidTraceFamily,
        ));
    }
    Ok(aligned)
}

fn align_skew_branches(
    raw: &SurfaceSurfaceIntersections,
    mut branches: Vec<CertifiedSkewCylinderBranch>,
) -> GraphSurfaceIntersectionResult<Vec<CertifiedSkewCylinderBranch>> {
    let mut aligned = Vec::with_capacity(raw.curves.len());
    for curve in &raw.curves {
        let SurfaceIntersectionCurve::SkewCylinder(carrier) = curve.curve else {
            return Err(GraphSurfaceIntersectionError::BranchCertificate(
                IntersectionCertificateError::InvalidTraceFamily,
            ));
        };
        let mut matches = branches.iter().enumerate().filter(|(_, branch)| {
            let certificate = branch.proof.residual();
            certificate.carrier() == carrier && certificate.carrier_range() == curve.curve_range
        });
        let Some((index, _)) = matches.next() else {
            return Err(GraphSurfaceIntersectionError::BranchCertificate(
                IntersectionCertificateError::InvalidTraceFamily,
            ));
        };
        if matches.next().is_some() {
            return Err(GraphSurfaceIntersectionError::BranchCertificate(
                IntersectionCertificateError::InvalidTraceFamily,
            ));
        }
        drop(matches);
        aligned.push(branches.remove(index));
    }
    if !branches.is_empty() {
        return Err(GraphSurfaceIntersectionError::BranchCertificate(
            IntersectionCertificateError::InvalidTraceFamily,
        ));
    }
    Ok(aligned)
}

fn branch_certificate_failure(
    results: &[Result<PairedSkewCylinderBranchResidualCertificate, IntersectionCertificateError>],
    scope: &mut OperationScope<'_, '_>,
) -> CertifiedSkewCylinderIntersections {
    let unsupported = results.iter().any(|result| {
        matches!(
            result,
            Err(
                IntersectionCertificateError::UnsupportedCarrierParameterization { .. }
                    | IntersectionCertificateError::InvalidCarrierRange
            )
        )
    });
    CertifiedSkewCylinderIntersections {
        raw: if unsupported {
            two_sheet_incomplete(scope)
        } else {
            numeric_resolution(scope, SKEW_CYLINDER_TWO_SHEET_WORK)
        },
        strict_miss: None,
        branches: None,
        isolated_contacts: Vec::new(),
        through_contacts: Vec::new(),
        support_contacts: Vec::new(),
        folded_support_curves: Vec::new(),
        touching_support_curves: Vec::new(),
    }
}

fn single_branch_certificate_failure(
    failure: &IntersectionCertificateError,
    scope: &mut OperationScope<'_, '_>,
) -> CertifiedSkewCylinderIntersections {
    let unsupported = matches!(
        failure,
        IntersectionCertificateError::UnsupportedCarrierParameterization { .. }
            | IntersectionCertificateError::InvalidCarrierRange
    );
    CertifiedSkewCylinderIntersections {
        raw: if unsupported {
            two_sheet_incomplete(scope)
        } else {
            numeric_resolution(scope, SKEW_CYLINDER_TWO_SHEET_WORK)
        },
        strict_miss: None,
        branches: None,
        isolated_contacts: Vec::new(),
        through_contacts: Vec::new(),
        support_contacts: Vec::new(),
        folded_support_curves: Vec::new(),
        touching_support_curves: Vec::new(),
    }
}

fn open_span_certificate_failure(
    failure: &IntersectionCertificateError,
    scope: &mut OperationScope<'_, '_>,
) -> CertifiedSkewCylinderIntersections {
    let unsupported = matches!(
        failure,
        IntersectionCertificateError::UnsupportedCarrierParameterization { .. }
            | IntersectionCertificateError::InvalidCarrierRange
    );
    CertifiedSkewCylinderIntersections {
        raw: if unsupported {
            clipped_topology_incomplete(scope)
        } else {
            numeric_resolution(scope, SKEW_CYLINDER_OPEN_SPAN_WORK)
        },
        strict_miss: None,
        branches: None,
        isolated_contacts: Vec::new(),
        through_contacts: Vec::new(),
        support_contacts: Vec::new(),
        folded_support_curves: Vec::new(),
        touching_support_curves: Vec::new(),
    }
}

fn graph_endpoint_proof(
    proof: SkewCylinderOpenSpanEndpointProof,
) -> Option<IntersectionBranchEndpointProof> {
    if proof.event.kind() != SkewCylinderFiniteWindowRootEventKind::Boundary
        || proof.event.root_count() != 1
    {
        return None;
    }
    let root = proof.event.root(0)?;
    Some(IntersectionBranchEndpointProof::SkewCylinderAxialRoot(
        SkewCylinderAxialRootEndpointProof {
            source_operand: root.provenance.source_operand,
            boundary: match root.provenance.boundary {
                SkewCylinderAxialBoundary::Lower => SkewCylinderAxialBoundaryProof::Lower,
                SkewCylinderAxialBoundary::Upper => SkewCylinderAxialBoundaryProof::Upper,
            },
            bound: root.provenance.value,
            sheet: root.sheet,
            cyclic_ordinal: root.cyclic_ordinal,
            half_angle_chart: match root.bracket.chart {
                SkewCylinderHalfAngleChart::Tangent => SkewCylinderHalfAngleChartProof::Tangent,
                SkewCylinderHalfAngleChart::Cotangent => SkewCylinderHalfAngleChartProof::Cotangent,
            },
            half_angle_bracket: [root.bracket.lo, root.bracket.hi],
            before: match root.before {
                SkewCylinderAxialRelation::Below => SkewCylinderAxialRelationProof::Below,
                SkewCylinderAxialRelation::Above => SkewCylinderAxialRelationProof::Above,
            },
            after: match root.after {
                SkewCylinderAxialRelation::Below => SkewCylinderAxialRelationProof::Below,
                SkewCylinderAxialRelation::Above => SkewCylinderAxialRelationProof::Above,
            },
            inside_side: match proof.inside_side {
                SkewCylinderRootInsideSide::Before => SkewCylinderRootInsideSideProof::Before,
                SkewCylinderRootInsideSide::After => SkewCylinderRootInsideSideProof::After,
            },
            inside_parameter: proof.carrier_parameter,
        },
    ))
}

fn raw_skew_curve(
    certificate: &PairedSkewCylinderBranchResidualCertificate,
) -> SurfaceSurfaceCurve {
    let carrier = certificate.carrier();
    let range = certificate.carrier_range();
    let traces = certificate.traces();
    let endpoint = |trace: kgraph::SkewCylinderBranchTrace, parameter| {
        let uv = trace.pcurve().eval(parameter);
        [uv.x, uv.y]
    };
    SurfaceSurfaceCurve {
        curve: SurfaceIntersectionCurve::SkewCylinder(carrier),
        curve_range: range,
        uv_a_start: endpoint(traces[0], range.lo),
        uv_a_end: endpoint(traces[0], range.hi),
        uv_b_start: endpoint(traces[1], range.lo),
        uv_b_end: endpoint(traces[1], range.hi),
        kind: ContactKind::Transverse,
    }
}

// The strict-positive admission certificate stays inline so the established
// Copy certificate contract survives value handoff without indirection.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq)]
enum DiscriminantAdmission {
    StrictPositive(SkewCylinderStrictPositiveTwoSheetAdmissionCertificate),
    StrictNegative,
    Contact(Box<kgraph::SkewCylinderDiscriminantContactTopologyCertificate>),
    NumericResolution,
}

fn classify_one_parameterization(cylinders: [Cylinder; 2]) -> DiscriminantAdmission {
    match classify_skew_cylinder_exact_discriminant(cylinders, SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK)
    {
        Ok(SkewCylinderExactDiscriminantTopology::StrictPositive(certificate)) => {
            DiscriminantAdmission::StrictPositive(certificate)
        }
        Ok(SkewCylinderExactDiscriminantTopology::StrictNegative) => {
            DiscriminantAdmission::StrictNegative
        }
        Ok(SkewCylinderExactDiscriminantTopology::Contact(contact)) => {
            DiscriminantAdmission::Contact(contact)
        }
        Err(_) => DiscriminantAdmission::NumericResolution,
    }
}

fn canonical_pair(
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
) -> ([Cylinder; 2], [[ParamRange; 2]; 2], bool) {
    if compare_cylinder_windows(&cylinders[0], ranges[0], &cylinders[1], ranges[1]).is_gt() {
        ([cylinders[1], cylinders[0]], [ranges[1], ranges[0]], true)
    } else {
        (cylinders, ranges, false)
    }
}

fn axes_are_exactly_nonparallel(cylinders: [Cylinder; 2]) -> bool {
    let first = cylinders[0].frame().z().to_array();
    let second = cylinders[1].frame().z().to_array();
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        .into_iter()
        .any(|axis| orient3d(first, second, axis, [0.0; 3]) != Orientation::Zero)
}

fn two_sheet_incomplete(scope: &mut OperationScope<'_, '_>) -> SurfaceSurfaceIntersections {
    scope.diagnose(
        SKEW_CYLINDER_TWO_SHEET_WORK,
        SKEW_CYLINDER_TWO_SHEET_INCOMPLETE,
        DiagnosticKind::ProofIncomplete,
        TWO_SHEET_REASON,
    );
    SurfaceSurfaceIntersections::indeterminate_empty_with_evidence(
        TWO_SHEET_REASON,
        vec![IncompleteEvidence {
            code: SKEW_CYLINDER_TWO_SHEET_INCOMPLETE,
            stage: SKEW_CYLINDER_TWO_SHEET_WORK,
            cause: IncompleteCause::ProofMethodUnavailable {
                capability: SKEW_CYLINDER_TWO_SHEET_BRANCH_CARRIER,
            },
            message: TWO_SHEET_REASON,
        }],
    )
}

fn clipped_topology_incomplete(scope: &mut OperationScope<'_, '_>) -> SurfaceSurfaceIntersections {
    scope.diagnose(
        SKEW_CYLINDER_AXIAL_CLIP_WORK,
        SKEW_CYLINDER_CLIPPED_TOPOLOGY_INCOMPLETE,
        DiagnosticKind::ProofIncomplete,
        CLIPPED_TOPOLOGY_REASON,
    );
    SurfaceSurfaceIntersections::indeterminate_empty_with_evidence(
        CLIPPED_TOPOLOGY_REASON,
        vec![IncompleteEvidence {
            code: SKEW_CYLINDER_CLIPPED_TOPOLOGY_INCOMPLETE,
            stage: SKEW_CYLINDER_AXIAL_CLIP_WORK,
            cause: IncompleteCause::ProofMethodUnavailable {
                capability: SKEW_CYLINDER_CLIPPED_BRANCH_TOPOLOGY,
            },
            message: CLIPPED_TOPOLOGY_REASON,
        }],
    )
}

fn contact_topology_incomplete(scope: &mut OperationScope<'_, '_>) -> SurfaceSurfaceIntersections {
    scope.diagnose(
        SKEW_CYLINDER_DISCRIMINANT_WORK,
        SKEW_CYLINDER_CONTACT_TOPOLOGY_INCOMPLETE,
        DiagnosticKind::ProofIncomplete,
        CONTACT_TOPOLOGY_REASON,
    );
    SurfaceSurfaceIntersections::indeterminate_empty_with_evidence(
        CONTACT_TOPOLOGY_REASON,
        vec![IncompleteEvidence {
            code: SKEW_CYLINDER_CONTACT_TOPOLOGY_INCOMPLETE,
            stage: SKEW_CYLINDER_DISCRIMINANT_WORK,
            cause: IncompleteCause::ProofMethodUnavailable {
                capability: SKEW_CYLINDER_CONTACT_ROOT_TOPOLOGY,
            },
            message: CONTACT_TOPOLOGY_REASON,
        }],
    )
}

fn numeric_resolution(
    scope: &mut OperationScope<'_, '_>,
    stage: StageId,
) -> SurfaceSurfaceIntersections {
    scope.record_numeric_resolution(stage);
    scope.diagnose(
        stage,
        SKEW_CYLINDER_DISCRIMINANT_NUMERIC_RESOLUTION,
        DiagnosticKind::NumericResolution,
        NUMERIC_RESOLUTION_REASON,
    );
    SurfaceSurfaceIntersections::indeterminate_empty_with_evidence(
        NUMERIC_RESOLUTION_REASON,
        vec![IncompleteEvidence {
            code: SKEW_CYLINDER_DISCRIMINANT_NUMERIC_RESOLUTION,
            stage,
            cause: IncompleteCause::NumericResolution,
            message: NUMERIC_RESOLUTION_REASON,
        }],
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use kgeom::frame::Frame;
    use kgeom::vec::{Point3, Vec3};

    #[test]
    fn reversed_parameterization_recovers_from_one_sided_exact_envelope_refusal() {
        let first = Cylinder::new(Frame::world(), 2.0).unwrap();
        let second = Cylinder::new(
            Frame::new(
                Point3::new(0.0, 8.0, 0.0),
                Vec3::new(1.0, 1.0, 2.0_f64.powi(-500)),
                Vec3::new(1.0, -1.0, 0.0),
            )
            .unwrap(),
            1.0,
        )
        .unwrap();

        assert_eq!(
            classify_one_parameterization([first, second]),
            DiscriminantAdmission::NumericResolution
        );
        assert_eq!(
            classify_one_parameterization([second, first]),
            DiscriminantAdmission::StrictNegative
        );
    }
}
