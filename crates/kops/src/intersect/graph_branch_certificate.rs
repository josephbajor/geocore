//! Operation-local branch certificate families.
//!
//! Each variant retains the immutable proof minted by the owning graph
//! adapter. Persistence remains a separate descriptor contract, so direct
//! analytic Cylinder branches stay operation-local for now.

use kgraph::{
    IntersectionCertificateError, PairedCylinderCylinderRulingResidualCertificate,
    PairedPlaneCylinderCircleResidualCertificate, PairedPlaneCylinderRulingResidualCertificate,
    PairedSkewCylinderBranchResidualCertificate,
    PersistentSkewCylinderFiniteWindowFamilyCertificate,
    PersistentSkewCylinderFiniteWindowFamilyMembershipCertificate,
    PersistentSkewCylinderFiniteWindowRootEventKind,
    PersistentSkewCylinderFiniteWindowSheetOccupancy,
    PersistentSkewCylinderFoldedSupportCertificate, SkewCylinderBranchGuardedEnd,
    SkewCylinderBranchPcurveCellCertificate, SkewCylinderBranchPcurveRootCorridorCertificate,
    VerifiedIntersectionCertificate, VerifiedNurbsIntersectionCertificate,
};

/// One lower/upper member of an exact two-root folded support component.
#[derive(Debug, Clone, PartialEq)]
pub struct SkewCylinderFoldedSupportBranchCertificate {
    residual: PairedSkewCylinderBranchResidualCertificate,
    folded: PersistentSkewCylinderFoldedSupportCertificate,
}

impl SkewCylinderFoldedSupportBranchCertificate {
    pub(super) fn mint(
        residual: PairedSkewCylinderBranchResidualCertificate,
        folded: PersistentSkewCylinderFoldedSupportCertificate,
    ) -> Result<Self, IntersectionCertificateError> {
        let ordinal = match residual.sheet() {
            kgraph::SkewCylinderSheet::Lower => 0,
            kgraph::SkewCylinderSheet::Upper => 1,
        };
        let formula = folded.formula_residuals()[ordinal];
        let same_order = residual == formula;
        let swapped_order = residual == formula.swapped();
        if (!same_order && !swapped_order)
            || residual.carrier_range() != folded.guarded_range()
            || folded.required_edge_tolerance() > folded.tolerance()
        {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
        Ok(Self { residual, folded })
    }

    /// Guarded paired residual member in caller source order.
    pub const fn residual_certificate(&self) -> PairedSkewCylinderBranchResidualCertificate {
        self.residual
    }

    /// Shared exact two-root support topology.
    pub const fn folded_certificate(&self) -> &PersistentSkewCylinderFoldedSupportCertificate {
        &self.folded
    }
}

/// Full-cycle residual proof composed with exact closed finite-window contact
/// topology for the same immutable sheet.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderWholeContactBranchCertificate {
    residual: PairedSkewCylinderBranchResidualCertificate,
    family: PersistentSkewCylinderFiniteWindowFamilyCertificate,
}

impl SkewCylinderWholeContactBranchCertificate {
    pub(super) fn mint(
        residual: PairedSkewCylinderBranchResidualCertificate,
        family: PersistentSkewCylinderFiniteWindowFamilyCertificate,
    ) -> Result<Self, IntersectionCertificateError> {
        let sheet = residual.sheet();
        let has_contact = (0..family.root_event_count(sheet)).any(|ordinal| {
            family.root_event(sheet, ordinal).is_some_and(|event| {
                event.kind() == PersistentSkewCylinderFiniteWindowRootEventKind::Contact
            })
        });
        if family.sheet_occupancy(sheet) != PersistentSkewCylinderFiniteWindowSheetOccupancy::Whole
            || !has_contact
            || residual.carrier().cylinders() != family.formula_cylinders()
            || residual.carrier_range() != family.formula_windows()[0][0]
            || residual.traces().map(|trace| trace.surface()) != family.source_cylinders()
            || residual.tolerance().to_bits() != family.tolerance().to_bits()
        {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
        Ok(Self { residual, family })
    }

    /// Whole-cycle paired residual proof for the represented analytic branch.
    pub const fn residual_certificate(self) -> PairedSkewCylinderBranchResidualCertificate {
        self.residual
    }

    /// Exact finite-window family proving closed whole occupancy and contacts.
    pub const fn finite_window_family(self) -> PersistentSkewCylinderFiniteWindowFamilyCertificate {
        self.family
    }
}

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
    finite_window_family_membership:
        Option<PersistentSkewCylinderFiniteWindowFamilyMembershipCertificate>,
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
            finite_window_family_membership: None,
        })
    }

    pub(super) fn bind_finite_window_family(
        mut self,
        membership: PersistentSkewCylinderFiniteWindowFamilyMembershipCertificate,
    ) -> Result<Self, IntersectionCertificateError> {
        let member = membership.member();
        if member.sheet() != self.residual.sheet()
            || member.guarded_range() != self.residual.carrier_range()
            || member.root_parameter_enclosures()
                != self
                    .root_corridors
                    .map(|corridor| corridor.root_parameter())
            || member.residual_bounds()
                != combined_residual_bounds(self.residual, self.root_corridors)
            || member.tolerance().to_bits() != self.residual.tolerance().to_bits()
        {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
        self.finite_window_family_membership = Some(membership);
        Ok(self)
    }

    /// Compact paired residual proof for the guarded open span.
    pub const fn residual_certificate(self) -> PairedSkewCylinderBranchResidualCertificate {
        self.residual
    }

    /// Physical-root continuation evidence ordered `[lower/start, upper/end]`.
    pub const fn root_corridors(self) -> [SkewCylinderBranchPcurveRootCorridorCertificate; 2] {
        self.root_corridors
    }

    /// Complete finite-window family and immutable member ordinal.
    pub const fn finite_window_family_membership(
        self,
    ) -> Option<PersistentSkewCylinderFiniteWindowFamilyMembershipCertificate> {
        self.finite_window_family_membership
    }

    /// Reissue one sealed guarded pcurve cell by its fixed partition index.
    pub fn certify_pcurve_cell(
        &self,
        index: usize,
    ) -> Result<SkewCylinderBranchPcurveCellCertificate, IntersectionCertificateError> {
        self.residual.certify_pcurve_cell(index)
    }
}

fn combined_residual_bounds(
    residual: PairedSkewCylinderBranchResidualCertificate,
    corridors: [SkewCylinderBranchPcurveRootCorridorCertificate; 2],
) -> [f64; 2] {
    let mut bounds = residual.residual_bounds();
    for corridor in corridors {
        for (bound, corridor_bound) in bounds.iter_mut().zip(corridor.corridor().residual_bounds())
        {
            *bound = bound.max(corridor_bound);
        }
    }
    bounds
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
    /// Full-cycle skew sheet with exact branch-attached finite-window contacts.
    SkewCylinderWholeContact(Box<SkewCylinderWholeContactBranchCertificate>),
    /// Non-wrapping skew span with guarded and physical-root pcurve evidence.
    SkewCylinderOpenSpan(Box<SkewCylinderOpenSpanBranchCertificate>),
    /// One guarded member of a two-root folded support component.
    SkewCylinderFoldedSupport(Box<SkewCylinderFoldedSupportBranchCertificate>),
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
                | Self::SkewCylinderWholeContact(_)
                | Self::SkewCylinderOpenSpan(_)
                | Self::SkewCylinderFoldedSupport(_)
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
            Self::SkewCylinderWholeContact(certificate) => {
                certificate.residual_certificate().residual_bounds()
            }
            Self::SkewCylinderOpenSpan(certificate) => {
                certificate.residual_certificate().residual_bounds()
            }
            Self::SkewCylinderFoldedSupport(certificate) => {
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
            Self::SkewCylinderWholeContact(certificate) => {
                certificate.residual_certificate().tolerance()
            }
            Self::SkewCylinderOpenSpan(certificate) => {
                certificate.residual_certificate().tolerance()
            }
            Self::SkewCylinderFoldedSupport(certificate) => {
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
            | Self::SkewCylinderWholeContact(_)
            | Self::SkewCylinderOpenSpan(_)
            | Self::SkewCylinderFoldedSupport(_)
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
            | Self::SkewCylinderWholeContact(_)
            | Self::SkewCylinderOpenSpan(_)
            | Self::SkewCylinderFoldedSupport(_)
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
            | Self::SkewCylinderWholeContact(_)
            | Self::SkewCylinderOpenSpan(_)
            | Self::SkewCylinderFoldedSupport(_)
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
            | Self::SkewCylinderWholeContact(_)
            | Self::SkewCylinderOpenSpan(_)
            | Self::SkewCylinderFoldedSupport(_)
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
            | Self::SkewCylinderWholeContact(_)
            | Self::SkewCylinderOpenSpan(_)
            | Self::SkewCylinderFoldedSupport(_)
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
            | Self::SkewCylinderWholeContact(_)
            | Self::SkewCylinderOpenSpan(_)
            | Self::SkewCylinderFoldedSupport(_)
            | Self::Nurbs(_) => None,
        }
    }

    /// Borrow a whole skew branch's closed finite-window contact proof.
    pub fn as_skew_cylinder_whole_contact(
        &self,
    ) -> Option<SkewCylinderWholeContactBranchCertificate> {
        match self {
            Self::SkewCylinderWholeContact(certificate) => Some(**certificate),
            Self::Analytic(_)
            | Self::PlaneCylinderCircle(_)
            | Self::PlaneCylinderRuling(_)
            | Self::CylinderCylinderRuling(_)
            | Self::SkewCylinderTwoSheet(_)
            | Self::SkewCylinderOpenSpan(_)
            | Self::SkewCylinderFoldedSupport(_)
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
            | Self::SkewCylinderWholeContact(_)
            | Self::SkewCylinderFoldedSupport(_)
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
            | Self::SkewCylinderWholeContact(_)
            | Self::SkewCylinderFoldedSupport(_)
            | Self::Nurbs(_) => None,
        }
    }

    /// Borrow one member's folded support proof and its shared root topology.
    pub fn as_skew_cylinder_folded_support(
        &self,
    ) -> Option<&SkewCylinderFoldedSupportBranchCertificate> {
        match self {
            Self::SkewCylinderFoldedSupport(certificate) => Some(certificate),
            _ => None,
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
            | Self::SkewCylinderWholeContact(_)
            | Self::SkewCylinderOpenSpan(_)
            | Self::SkewCylinderFoldedSupport(_) => None,
            Self::Nurbs(certificate) => Some(certificate.as_ref()),
        }
    }
}
