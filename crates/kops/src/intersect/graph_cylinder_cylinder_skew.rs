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
//! coincident, seam-wrapping, and failed exact classifications remain typed
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
use kgraph::{
    IntersectionCertificateError, PairedSkewCylinderBranchResidualCertificate,
    PersistentSkewCylinderFiniteWindowMemberInput, SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    SKEW_CYLINDER_BRANCH_CERTIFICATE_WORK, SKEW_CYLINDER_BRANCH_PCURVE_ROOT_CORRIDOR_WORK,
    SKEW_CYLINDER_ROOT_CLUSTER_MAX_EXACT_WORK, SkewCylinderExactDiscriminantTopology,
    SkewCylinderFiniteSheetTopology, SkewCylinderFiniteWindowRootEventKind,
    SkewCylinderFiniteWindowTopologyCertificate, SkewCylinderOpenSpan,
    SkewCylinderOpenSpanEndpointProof, SkewCylinderOpenSpanFailure,
    SkewCylinderOpenSpanTopologyInput, SkewCylinderRootInsideSide, SkewCylinderSheet,
    SkewCylinderStrictPositiveTwoSheetAdmissionCertificate,
    certify_paired_skew_cylinder_branch_residuals,
    certify_paired_skew_cylinder_branch_subrange_residuals,
    certify_persistent_skew_cylinder_finite_window_family,
    classify_skew_cylinder_exact_discriminant, classify_skew_cylinder_open_spans,
    plan_skew_cylinder_root_clusters,
};

use super::cylinder_cylinder::{compare_cylinder_windows, validate_ranges};
use super::error::IntersectionError;
use super::graph_branch_certificate::SkewCylinderOpenSpanBranchCertificate;
use super::graph_skew_cylinder_endpoint::{
    IntersectionBranchEndpointProof, SkewCylinderAxialBoundaryProof,
    SkewCylinderAxialRelationProof, SkewCylinderAxialRootEndpointProof,
    SkewCylinderHalfAngleChartProof, SkewCylinderRootInsideSideProof,
};
use super::graph_surface::{GraphSurfaceIntersectionError, GraphSurfaceIntersectionResult};
use super::result::{
    ContactKind, SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
};
use super::skew_cylinder_sheet_occupancy::{
    SKEW_CYLINDER_AXIAL_BOUNDS_EXACT_WORK, collect_skew_cylinder_axial_bound_topologies,
};
use kgraph::{
    SkewCylinderAxialBoundary, SkewCylinderAxialRelation, SkewCylinderAxialRootFailure,
    SkewCylinderHalfAngleChart,
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

/// Complete graph inputs produced by the exact skew-cylinder admission.
pub(super) struct CertifiedSkewCylinderIntersections {
    pub(super) raw: SurfaceSurfaceIntersections,
    pub(super) strict_miss: Option<SkewCylinderStrictDiscriminantMiss>,
    pub(super) branches: Option<Vec<CertifiedSkewCylinderBranch>>,
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
    OpenSpan(Box<SkewCylinderOpenSpanBranchCertificate>),
}

impl CertifiedSkewCylinderBranchProof {
    pub(super) fn residual(&self) -> PairedSkewCylinderBranchResidualCertificate {
        match self {
            Self::TwoSheet(certificate) => **certificate,
            Self::OpenSpan(certificate) => certificate.residual_certificate(),
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
        DiscriminantAdmission::StrictPositive(_) | DiscriminantAdmission::StrictNegative => {
            (first_admission, false)
        }
        DiscriminantAdmission::NumericResolution => (
            classify_one_parameterization([cylinders[1], cylinders[0]]),
            true,
        ),
        DiscriminantAdmission::Contact => {
            let reversed = classify_one_parameterization([cylinders[1], cylinders[0]]);
            // A projection fold may look like Contact in one ruling chart
            // while the reverse chart proves two regular sheets. Conversely,
            // a contradictory reverse miss cannot supersede retained contact.
            match reversed {
                DiscriminantAdmission::StrictPositive(_) => (reversed, true),
                DiscriminantAdmission::StrictNegative
                | DiscriminantAdmission::Contact
                | DiscriminantAdmission::NumericResolution => {
                    (DiscriminantAdmission::Contact, false)
                }
            }
        }
    };

    match admission {
        DiscriminantAdmission::StrictNegative => Ok(CertifiedSkewCylinderIntersections {
            raw: SurfaceSurfaceIntersections::complete_empty(),
            strict_miss: Some(SkewCylinderStrictDiscriminantMiss::certified()),
            branches: None,
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
        DiscriminantAdmission::Contact => Ok(CertifiedSkewCylinderIntersections {
            raw: contact_topology_incomplete(scope),
            strict_miss: None,
            branches: None,
        }),
        DiscriminantAdmission::NumericResolution => Ok(CertifiedSkewCylinderIntersections {
            raw: numeric_resolution(scope, SKEW_CYLINDER_DISCRIMINANT_WORK),
            strict_miss: None,
            branches: None,
        }),
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
            });
        }
        Err(_) => {
            return Ok(CertifiedSkewCylinderIntersections {
                raw: numeric_resolution(scope, SKEW_CYLINDER_AXIAL_CLIP_WORK),
                strict_miss: None,
                branches: None,
            });
        }
    };
    let topology_input = SkewCylinderOpenSpanTopologyInput {
        topologies: &topologies,
        ranges,
        canonical_to_source,
    };
    let root_cluster_plan = match plan_skew_cylinder_root_clusters(topology_input) {
        Ok(plan) => plan,
        Err(_) => {
            return Ok(CertifiedSkewCylinderIntersections {
                raw: clipped_topology_incomplete(scope),
                strict_miss: None,
                branches: None,
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
            });
        }
        Err(_) => {
            return Ok(CertifiedSkewCylinderIntersections {
                raw: clipped_topology_incomplete(scope),
                strict_miss: None,
                branches: None,
            });
        }
    };
    if finite_topology.root_cluster_query_plan() != root_cluster_plan
        || [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper]
            .into_iter()
            .flat_map(|sheet| finite_topology.root_events(sheet))
            .any(|event| {
                event.kind() != SkewCylinderFiniteWindowRootEventKind::Boundary
                    || event.root_count() != 1
            })
    {
        return Ok(CertifiedSkewCylinderIntersections {
            raw: clipped_topology_incomplete(scope),
            strict_miss: None,
            branches: None,
        });
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
    if open_span_count > 0 {
        scope.ledger_mut().charge(
            SKEW_CYLINDER_OPEN_SPAN_WORK,
            SKEW_CYLINDER_OPEN_SPAN_EXACT_WORK_PER_BRANCH * open_span_count as u64,
        )?;
    }

    let mut branches = Vec::with_capacity(2 + open_span_count);
    let mut family_members = Vec::with_capacity(open_span_count);
    for (sheet, whole_certificate) in sheets.into_iter().zip(certified.iter()) {
        match finite_topology.sheet(sheet) {
            SkewCylinderFiniteSheetTopology::Outside => {}
            SkewCylinderFiniteSheetTopology::Whole => match whole_certificate {
                Ok(certificate) => branches.push(CertifiedSkewCylinderBranch {
                    proof: CertifiedSkewCylinderBranchProof::TwoSheet(Box::new(if reversed {
                        certificate.swapped()
                    } else {
                        *certificate
                    })),
                    endpoint_proofs: [None; 2],
                }),
                Err(failure) => return Ok(single_branch_certificate_failure(failure, scope)),
            },
            SkewCylinderFiniteSheetTopology::Open(spans) => {
                for span in spans.iter().copied() {
                    if span.sheet != sheet {
                        return Ok(CertifiedSkewCylinderIntersections {
                            raw: numeric_resolution(scope, SKEW_CYLINDER_AXIAL_CLIP_WORK),
                            strict_miss: None,
                            branches: None,
                        });
                    }
                    let open_span = match certify_open_span_pcurve_transport(
                        cylinders, ranges, span, reversed, tolerance,
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
    if open_span_count > 0 {
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
    }
    publish_skew_branches(branches)
}

fn certify_open_span_pcurve_transport(
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
    span: SkewCylinderOpenSpan,
    reversed: bool,
    tolerance: f64,
) -> Result<SkewCylinderOpenSpanBranchCertificate, IntersectionCertificateError> {
    let certificate = certify_paired_skew_cylinder_branch_subrange_residuals(
        cylinders, ranges, span.range, span.sheet, tolerance,
    )?;
    let certificate = if reversed {
        certificate.swapped()
    } else {
        certificate
    };
    let [lower_root, upper_root] = span.root_longitude_intervals(ranges[0][0]).ok_or(
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
    publish_skew_branches(
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
    )
}

fn publish_skew_branches(
    branches: Vec<CertifiedSkewCylinderBranch>,
) -> GraphSurfaceIntersectionResult<CertifiedSkewCylinderIntersections> {
    let curves = branches
        .iter()
        .map(|branch| raw_skew_curve(&branch.proof.residual()))
        .collect::<Vec<_>>();
    let raw = if curves.is_empty() {
        SurfaceSurfaceIntersections::complete_empty()
    } else {
        SurfaceSurfaceIntersections::canonicalized_complete(Vec::new(), curves)
            .map_err(IntersectionError::from)
            .map_err(GraphSurfaceIntersectionError::Intersection)?
    };
    let branches = align_skew_branches(&raw, branches)?;
    Ok(CertifiedSkewCylinderIntersections {
        raw,
        strict_miss: None,
        branches: Some(branches),
    })
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
#[derive(Debug, Clone, Copy, PartialEq)]
enum DiscriminantAdmission {
    StrictPositive(SkewCylinderStrictPositiveTwoSheetAdmissionCertificate),
    StrictNegative,
    Contact,
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
        Ok(SkewCylinderExactDiscriminantTopology::Contact) => DiscriminantAdmission::Contact,
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
