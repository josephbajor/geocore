//! Promotion of certified procedural full-cycle skew Cylinder/Cylinder sheets.
//!
//! Discovery, finite-window containment, sheet identity, and paired residual
//! proof are all retained by the kgraph certificate. This adapter only checks
//! that the raw operation-local branch names the same immutable carrier before
//! exposing graph descriptors in caller source order.

use kgraph::{
    Curve2dDescriptor, CurveDescriptor, PairedSkewCylinderBranchResidualCertificate,
    SkewCylinderBranchCarrier,
};

use super::graph_surface::{
    GraphSurfaceIntersectionError, GraphSurfaceIntersectionResult, IntersectionBranchCertificate,
    IntersectionBranchTopology, VerifiedBranchPayload,
};
use super::result::{ContactKind, SurfaceSurfaceCurve};

pub(super) fn build_verified_skew_cylinder_branch(
    raw_carrier: SkewCylinderBranchCarrier,
    raw_branch: &SurfaceSurfaceCurve,
    certificate: PairedSkewCylinderBranchResidualCertificate,
) -> GraphSurfaceIntersectionResult<VerifiedBranchPayload> {
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
    Ok(VerifiedBranchPayload {
        carrier: CurveDescriptor::SkewCylinderBranch(raw_carrier),
        carrier_range: certificate.carrier_range(),
        topology: IntersectionBranchTopology::Closed,
        pcurves: traces.map(|trace| Curve2dDescriptor::SkewCylinderBranch(trace.pcurve())),
        parameter_maps: certificate.parameter_maps(),
        certificate: IntersectionBranchCertificate::SkewCylinderTwoSheet(Box::new(certificate)),
    })
}
