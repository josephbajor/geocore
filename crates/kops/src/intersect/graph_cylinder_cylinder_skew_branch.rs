//! Promotion of certified procedural skew Cylinder/Cylinder sheets and spans.
//!
//! Discovery, finite-window containment, sheet identity, and paired residual
//! proof are all retained by the kgraph certificate. This adapter only checks
//! that the raw operation-local branch names the same immutable carrier before
//! exposing graph descriptors in caller source order. Exact full cycles become
//! closed branches; independently certified strict subranges become open.

use kgraph::{Curve2dDescriptor, CurveDescriptor, SkewCylinderBranchCarrier};

use super::graph_cylinder_cylinder_skew::CertifiedSkewCylinderBranchProof;
use super::graph_surface::{
    GraphSurfaceIntersectionError, GraphSurfaceIntersectionResult, IntersectionBranchCertificate,
    IntersectionBranchTopology, VerifiedBranchPayload,
};
use super::result::{ContactKind, SurfaceSurfaceCurve};

pub(super) fn build_verified_skew_cylinder_branch(
    raw_carrier: SkewCylinderBranchCarrier,
    raw_branch: &SurfaceSurfaceCurve,
    proof: CertifiedSkewCylinderBranchProof,
) -> GraphSurfaceIntersectionResult<VerifiedBranchPayload> {
    let certificate = proof.residual();
    if raw_branch.kind != ContactKind::Transverse
        || raw_branch.curve_range != certificate.carrier_range()
        || raw_carrier != certificate.carrier()
        || raw_carrier.sheet() != certificate.sheet()
    {
        return Err(GraphSurfaceIntersectionError::BranchCertificate(
            kgraph::IntersectionCertificateError::InvalidTraceFamily,
        ));
    }
    let traces = certificate.traces();
    let full_cycle = certificate.carrier_range().width() == core::f64::consts::TAU;
    Ok(VerifiedBranchPayload {
        carrier: CurveDescriptor::SkewCylinderBranch(raw_carrier),
        carrier_range: certificate.carrier_range(),
        topology: if full_cycle {
            IntersectionBranchTopology::Closed
        } else {
            IntersectionBranchTopology::Open
        },
        pcurves: traces.map(|trace| Curve2dDescriptor::SkewCylinderBranch(trace.pcurve())),
        parameter_maps: certificate.parameter_maps(),
        certificate: match (full_cycle, proof) {
            (true, CertifiedSkewCylinderBranchProof::TwoSheet(certificate)) => {
                IntersectionBranchCertificate::SkewCylinderTwoSheet(certificate)
            }
            (false, CertifiedSkewCylinderBranchProof::OpenSpan(certificate)) => {
                IntersectionBranchCertificate::SkewCylinderOpenSpan(certificate)
            }
            (true, CertifiedSkewCylinderBranchProof::OpenSpan(_))
            | (false, CertifiedSkewCylinderBranchProof::TwoSheet(_)) => {
                return Err(GraphSurfaceIntersectionError::BranchCertificate(
                    kgraph::IntersectionCertificateError::InvalidTraceFamily,
                ));
            }
        },
    })
}
