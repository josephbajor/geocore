//! Operation-local branch certificate families.
//!
//! Each variant retains the immutable proof minted by the owning graph
//! adapter. Persistence remains a separate descriptor contract, so direct
//! analytic Cylinder branches stay operation-local for now.

use kgraph::{
    IntersectionCertificateError, PairedCylinderCylinderRulingResidualCertificate,
    PairedPlaneCylinderCircleResidualCertificate, PairedPlaneCylinderRulingResidualCertificate,
    PairedSkewCylinderBranchResidualCertificate, SkewCylinderBranchGuardedEnd,
    SkewCylinderBranchPcurveCellCertificate, SkewCylinderBranchPcurveRootCorridorCertificate,
    VerifiedIntersectionCertificate, VerifiedNurbsIntersectionCertificate,
};

/// Sealed operation-local proof for one bounded skew-cylinder component.
///
/// The retained residual certificate stays compact. Arrangement consumers
/// reissue any of its 256 guarded pcurve cells by index; only the two
/// physical-root continuation corridors are stored here. Corridor order is
/// always `[lower/start, upper/end]` in canonical carrier parameter, while
/// each corridor's pcurve array follows the branch's current caller source
/// order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderOpenSpanBranchCertificate {
    residual: PairedSkewCylinderBranchResidualCertificate,
    root_corridors: [SkewCylinderBranchPcurveRootCorridorCertificate; 2],
}

impl SkewCylinderOpenSpanBranchCertificate {
    pub(super) fn mint(
        residual: PairedSkewCylinderBranchResidualCertificate,
        root_corridors: [SkewCylinderBranchPcurveRootCorridorCertificate; 2],
    ) -> Result<Self, IntersectionCertificateError> {
        let range = residual.carrier_range();
        let [lower, upper] = root_corridors;
        let lower_root = lower.root_parameter();
        let upper_root = upper.root_parameter();
        let lower_cell = lower.corridor();
        let upper_cell = upper.corridor();
        let expected_operands = residual.traces().map(|trace| trace.pcurve().operand());
        let corridors_match_trace_order = root_corridors.iter().all(|corridor| {
            corridor.root_pcurves().map(|pcurve| pcurve.operand()) == expected_operands
                && corridor.corridor().pcurves().map(|pcurve| pcurve.operand()) == expected_operands
        });
        if lower.guarded_end() != SkewCylinderBranchGuardedEnd::Lower
            || upper.guarded_end() != SkewCylinderBranchGuardedEnd::Upper
            || lower_root.hi() >= range.lo
            || upper_root.lo() <= range.hi
            || lower_cell.parameter() != kcore::interval::Interval::new(lower_root.lo(), range.lo)
            || upper_cell.parameter() != kcore::interval::Interval::new(range.hi, upper_root.hi())
            || !corridors_match_trace_order
        {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
        Ok(Self {
            residual,
            root_corridors,
        })
    }

    /// Compact paired residual proof for the guarded open span.
    pub const fn residual_certificate(self) -> PairedSkewCylinderBranchResidualCertificate {
        self.residual
    }

    /// Physical-root continuation evidence ordered `[lower/start, upper/end]`.
    pub const fn root_corridors(self) -> [SkewCylinderBranchPcurveRootCorridorCertificate; 2] {
        self.root_corridors
    }

    /// Reissue one sealed guarded pcurve cell by its fixed partition index.
    pub fn certify_pcurve_cell(
        &self,
        index: usize,
    ) -> Result<SkewCylinderBranchPcurveCellCertificate, IntersectionCertificateError> {
        self.residual.certify_pcurve_cell(index)
    }
}

/// Active-range proof retained by one operation-local branch.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum IntersectionBranchCertificate {
    /// Existing exact analytic line/circle proof family.
    Analytic(Box<VerifiedIntersectionCertificate>),
    /// Whole-period Plane/Cylinder circle proof.
    PlaneCylinderCircle(Box<PairedPlaneCylinderCircleResidualCertificate>),
    /// Finite exact-family Plane/Cylinder ruling proof.
    PlaneCylinderRuling(Box<PairedPlaneCylinderRulingResidualCertificate>),
    /// Finite exact-family Cylinder/Cylinder ruling proof.
    CylinderCylinderRuling(Box<PairedCylinderCylinderRulingResidualCertificate>),
    /// Certified procedural full-cycle sheet of a strict-positive skew pair.
    SkewCylinderTwoSheet(Box<PairedSkewCylinderBranchResidualCertificate>),
    /// Non-wrapping skew span with guarded and physical-root pcurve evidence.
    SkewCylinderOpenSpan(Box<SkewCylinderOpenSpanBranchCertificate>),
    /// Operation-generated degree-1 analytic/NURBS trace proof.
    Nurbs(Box<VerifiedNurbsIntersectionCertificate>),
}

impl IntersectionBranchCertificate {
    pub(crate) const fn is_operation_local_cylinder(&self) -> bool {
        matches!(
            self,
            Self::PlaneCylinderCircle(_)
                | Self::PlaneCylinderRuling(_)
                | Self::CylinderCylinderRuling(_)
                | Self::SkewCylinderTwoSheet(_)
                | Self::SkewCylinderOpenSpan(_)
        )
    }

    /// Conservative paired residual bounds in operand order.
    pub fn residual_bounds(&self) -> [f64; 2] {
        match self {
            Self::Analytic(certificate) => certificate.residual_bounds(),
            Self::PlaneCylinderCircle(certificate) => certificate.residual_bounds(),
            Self::PlaneCylinderRuling(certificate) => certificate.residual_bounds(),
            Self::CylinderCylinderRuling(certificate) => certificate.residual_bounds(),
            Self::SkewCylinderTwoSheet(certificate) => certificate.residual_bounds(),
            Self::SkewCylinderOpenSpan(certificate) => {
                certificate.residual_certificate().residual_bounds()
            }
            Self::Nurbs(certificate) => certificate.residual_bounds(),
        }
    }

    /// Model-space tolerance used by the proof.
    pub fn tolerance(&self) -> f64 {
        match self {
            Self::Analytic(certificate) => certificate.tolerance(),
            Self::PlaneCylinderCircle(certificate) => certificate.tolerance(),
            Self::PlaneCylinderRuling(certificate) => certificate.tolerance(),
            Self::CylinderCylinderRuling(certificate) => certificate.tolerance(),
            Self::SkewCylinderTwoSheet(certificate) => certificate.tolerance(),
            Self::SkewCylinderOpenSpan(certificate) => {
                certificate.residual_certificate().tolerance()
            }
            Self::Nurbs(certificate) => certificate.tolerance(),
        }
    }

    /// Borrow the analytic plane-line proof when it matches.
    pub fn as_plane_line(&self) -> Option<kgraph::PairedPlaneLineResidualCertificate> {
        match self {
            Self::Analytic(certificate) => certificate.as_plane_line(),
            Self::PlaneCylinderCircle(_)
            | Self::PlaneCylinderRuling(_)
            | Self::CylinderCylinderRuling(_)
            | Self::SkewCylinderTwoSheet(_)
            | Self::SkewCylinderOpenSpan(_)
            | Self::Nurbs(_) => None,
        }
    }

    /// Borrow the analytic plane/sphere proof when it matches.
    pub fn as_plane_sphere_circle(
        &self,
    ) -> Option<kgraph::PairedPlaneSphereCircleResidualCertificate> {
        match self {
            Self::Analytic(certificate) => certificate.as_plane_sphere_circle(),
            Self::PlaneCylinderCircle(_)
            | Self::PlaneCylinderRuling(_)
            | Self::CylinderCylinderRuling(_)
            | Self::SkewCylinderTwoSheet(_)
            | Self::SkewCylinderOpenSpan(_)
            | Self::Nurbs(_) => None,
        }
    }

    /// Borrow the whole-period Plane/Cylinder circle proof when it matches.
    pub fn as_plane_cylinder_circle(&self) -> Option<PairedPlaneCylinderCircleResidualCertificate> {
        match self {
            Self::PlaneCylinderCircle(certificate) => Some(**certificate),
            Self::Analytic(_)
            | Self::PlaneCylinderRuling(_)
            | Self::CylinderCylinderRuling(_)
            | Self::SkewCylinderTwoSheet(_)
            | Self::SkewCylinderOpenSpan(_)
            | Self::Nurbs(_) => None,
        }
    }

    /// Borrow the finite Plane/Cylinder ruling proof when it matches.
    pub fn as_plane_cylinder_ruling(&self) -> Option<PairedPlaneCylinderRulingResidualCertificate> {
        match self {
            Self::PlaneCylinderRuling(certificate) => Some(**certificate),
            Self::Analytic(_)
            | Self::PlaneCylinderCircle(_)
            | Self::CylinderCylinderRuling(_)
            | Self::SkewCylinderTwoSheet(_)
            | Self::SkewCylinderOpenSpan(_)
            | Self::Nurbs(_) => None,
        }
    }

    /// Borrow the finite Cylinder/Cylinder ruling proof when it matches.
    pub fn as_cylinder_cylinder_ruling(
        &self,
    ) -> Option<PairedCylinderCylinderRulingResidualCertificate> {
        match self {
            Self::CylinderCylinderRuling(certificate) => Some(**certificate),
            Self::Analytic(_)
            | Self::PlaneCylinderCircle(_)
            | Self::PlaneCylinderRuling(_)
            | Self::SkewCylinderTwoSheet(_)
            | Self::SkewCylinderOpenSpan(_)
            | Self::Nurbs(_) => None,
        }
    }

    /// Borrow the certified skew Cylinder/Cylinder two-sheet proof when it matches.
    pub fn as_skew_cylinder_two_sheet(
        &self,
    ) -> Option<PairedSkewCylinderBranchResidualCertificate> {
        match self {
            Self::SkewCylinderTwoSheet(certificate) => Some(**certificate),
            Self::Analytic(_)
            | Self::PlaneCylinderCircle(_)
            | Self::PlaneCylinderRuling(_)
            | Self::CylinderCylinderRuling(_)
            | Self::SkewCylinderOpenSpan(_)
            | Self::Nurbs(_) => None,
        }
    }

    /// Borrow the independently certified skew-cylinder subrange proof.
    pub fn as_skew_cylinder_open_span(
        &self,
    ) -> Option<PairedSkewCylinderBranchResidualCertificate> {
        match self {
            Self::SkewCylinderOpenSpan(certificate) => Some(certificate.residual_certificate()),
            Self::Analytic(_)
            | Self::PlaneCylinderCircle(_)
            | Self::PlaneCylinderRuling(_)
            | Self::CylinderCylinderRuling(_)
            | Self::SkewCylinderTwoSheet(_)
            | Self::Nurbs(_) => None,
        }
    }

    /// Borrow the sealed bounded-span proof including both root corridors.
    pub fn as_skew_cylinder_open_span_branch(
        &self,
    ) -> Option<SkewCylinderOpenSpanBranchCertificate> {
        match self {
            Self::SkewCylinderOpenSpan(certificate) => Some(**certificate),
            Self::Analytic(_)
            | Self::PlaneCylinderCircle(_)
            | Self::PlaneCylinderRuling(_)
            | Self::CylinderCylinderRuling(_)
            | Self::SkewCylinderTwoSheet(_)
            | Self::Nurbs(_) => None,
        }
    }

    /// Borrow the operation-generated analytic/NURBS proof when it matches.
    pub fn as_nurbs(&self) -> Option<&VerifiedNurbsIntersectionCertificate> {
        match self {
            Self::Analytic(_)
            | Self::PlaneCylinderCircle(_)
            | Self::PlaneCylinderRuling(_)
            | Self::CylinderCylinderRuling(_)
            | Self::SkewCylinderTwoSheet(_)
            | Self::SkewCylinderOpenSpan(_) => None,
            Self::Nurbs(certificate) => Some(certificate.as_ref()),
        }
    }
}
