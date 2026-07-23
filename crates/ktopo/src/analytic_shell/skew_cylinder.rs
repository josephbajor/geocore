//! Sealed preflight for persistent bounded skew-cylinder composites.

use super::{
    AnalyticEdgeDeclaration, AnalyticEdgeProof, AnalyticShellCurve, AnalyticShellEdge,
    AnalyticShellPcurve, AnalyticShellPlanError, AnalyticShellSurface, UseCandidate,
};
use kcore::tolerance::{LINEAR_RESOLUTION, Tolerances};
use kgeom::curve::Curve;
use kgeom::vec::Point3;
use kgraph::{AffineParamMap1d, IntersectionCertificateError};

pub(super) fn validate_edge_declaration(
    edge: AnalyticShellEdge,
) -> Result<(), AnalyticShellPlanError> {
    match (edge.carrier, edge.persistent_skew_certificate) {
        (AnalyticShellCurve::PersistentSkewCylinderOpenSpan(carrier), Some(certificate))
            if carrier == certificate.carrier()
                && edge.range == certificate.logical_range()
                && edge.vertices[0] != edge.vertices[1] =>
        {
            Tolerances::default()
                .entity_tolerance(certificate.required_edge_tolerance().max(LINEAR_RESOLUTION))
                .map_err(|_| AnalyticShellPlanError::InvalidGeometry {
                    reason: "persistent skew-cylinder edge tolerance is invalid",
                })?;
            Ok(())
        }
        (AnalyticShellCurve::PersistentSkewCylinderOpenSpan(_), _) => {
            Err(AnalyticShellPlanError::InvalidGeometry {
                reason: "persistent skew-cylinder carrier was not derived from its sealed certificate",
            })
        }
        (_, Some(_)) => Err(AnalyticShellPlanError::InvalidGeometry {
            reason: "persistent skew-cylinder certificate has a mismatched carrier",
        }),
        _ => Ok(()),
    }
}

pub(super) fn endpoint_matches(edge: AnalyticShellEdge, endpoint: usize, position: Point3) -> bool {
    let Some(certificate) = edge.persistent_skew_certificate else {
        return false;
    };
    let expected = certificate.endpoint_points()[endpoint];
    let parameter = if endpoint == 0 {
        certificate.logical_range().lo
    } else {
        certificate.logical_range().hi
    };
    point_bits_equal(position, expected)
        && super::certify_endpoint_incidence(
            position,
            certificate.carrier().eval(parameter),
            certificate.required_edge_tolerance(),
        )
}

pub(super) fn certify_pair(
    declaration: AnalyticEdgeDeclaration,
    uses: [UseCandidate; 2],
) -> Result<AnalyticEdgeProof, IntersectionCertificateError> {
    let AnalyticEdgeDeclaration::Bounded(edge) = declaration else {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    };
    let Some(certificate) = edge.persistent_skew_certificate else {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    };
    if edge.carrier != AnalyticShellCurve::PersistentSkewCylinderOpenSpan(certificate.carrier())
        || edge.range != certificate.logical_range()
    {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }

    if !pair_matches(certificate, uses)? {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    Ok(AnalyticEdgeProof::PersistentSkewCylinderOpenSpan(
        certificate,
    ))
}

/// Return the uses in certificate source order.
///
/// Numeric face keys carry no geometric meaning. The source order is selected
/// only by exact cylinder and persistent-pcurve descriptor agreement. Integer
/// period chart shifts remain free because they do not change the graph-bound
/// pcurve; generic loop/domain preflight validates their lifted coordinates.
pub(super) fn order_pair(
    declaration: AnalyticEdgeDeclaration,
    uses: [UseCandidate; 2],
) -> Result<[UseCandidate; 2], IntersectionCertificateError> {
    let AnalyticEdgeDeclaration::Bounded(edge) = declaration else {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    };
    let Some(certificate) = edge.persistent_skew_certificate else {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    };
    let direct = pair_matches(certificate, uses)?;
    let swapped = pair_matches(certificate, [uses[1], uses[0]])?;
    match (direct, swapped) {
        (true, false) => Ok(uses),
        (false, true) => Ok([uses[1], uses[0]]),
        _ => Err(IntersectionCertificateError::InvalidTraceFamily),
    }
}

fn pair_matches(
    certificate: kgraph::PersistentSkewCylinderOpenSpanCertificate,
    uses: [UseCandidate; 2],
) -> Result<bool, IntersectionCertificateError> {
    let traces = certificate.residual_certificate().traces();
    let pcurves = certificate.pcurves();
    let identity = AffineParamMap1d::new(1.0, 0.0)?;
    Ok((0..2).all(|index| {
        uses[index].surface == AnalyticShellSurface::Cylinder(traces[index].surface())
            && uses[index].pcurve.curve
                == AnalyticShellPcurve::PersistentSkewCylinderOpenSpan(pcurves[index])
            && uses[index].pcurve.edge_to_pcurve == identity
            && uses[index].pcurve.closure_winding.is_none()
    }))
}

fn point_bits_equal(left: Point3, right: Point3) -> bool {
    left.x.to_bits() == right.x.to_bits()
        && left.y.to_bits() == right.y.to_bits()
        && left.z.to_bits() == right.z.to_bits()
}
