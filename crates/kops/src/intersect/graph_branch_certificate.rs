//! Operation-local whole-branch certificate families.
//!
//! Each variant retains the immutable proof minted by the owning graph
//! adapter. Persistence remains a separate descriptor contract, so direct
//! analytic Cylinder branches stay operation-local for now.

use kgraph::{
    PairedCylinderCylinderRulingResidualCertificate, PairedPlaneCylinderCircleResidualCertificate,
    PairedPlaneCylinderRulingResidualCertificate, PairedSkewCylinderBranchResidualCertificate,
    VerifiedIntersectionCertificate, VerifiedNurbsIntersectionCertificate,
};

/// Whole-range proof retained by one operation-local branch.
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
            | Self::SkewCylinderTwoSheet(_) => None,
            Self::Nurbs(certificate) => Some(certificate.as_ref()),
        }
    }
}
