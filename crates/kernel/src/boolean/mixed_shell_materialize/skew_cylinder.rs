//! Sealed plan-to-topology adapter for bounded skew-cylinder composites.

use kgeom::param::ParamRange;
use kgeom::vec::Point3;
use kgraph::{
    PERSISTENT_SKEW_CYLINDER_OPEN_SPAN_WORK, PersistentSkewCylinderOpenSpanCertificate,
    PersistentSkewCylinderOpenSpanOrientation, certify_persistent_skew_cylinder_open_span,
};
use ktopo::analytic_shell::{
    AnalyticEdgeKey, AnalyticPcurveUse, AnalyticShellSkewCylinderOpenSpan, AnalyticVertexKey,
};
use ktopo::entity::PcurveChart;

use super::{MixedShellMaterializationError, PhysicalCarrier, PhysicalEdge, section_edge};
use crate::boolean::mixed_shell_plan::{
    MixedPcurveLineage, MixedSectionEdgePlan, MixedShellProofPlan, MixedSourceFaceKey,
};

/// One charged, certified composite waiting for stable topology keys.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct CertifiedSkewCylinderSpan {
    certificate: PersistentSkewCylinderOpenSpanCertificate,
}

impl CertifiedSkewCylinderSpan {
    pub(super) const fn endpoint_points(self) -> [Point3; 2] {
        self.certificate.endpoint_points()
    }

    pub(super) const fn logical_range(self) -> ParamRange {
        self.certificate.logical_range()
    }

    pub(super) fn declarations(
        self,
        edge: AnalyticEdgeKey,
        vertices: [AnalyticVertexKey; 2],
    ) -> AnalyticShellSkewCylinderOpenSpan {
        AnalyticShellSkewCylinderOpenSpan::new(edge, vertices, self.certificate)
    }
}

/// Exact precharge represented by the persistent physical edges.
pub(super) fn physical_work(
    plan: &MixedShellProofPlan,
    edges: &[PhysicalEdge],
) -> Result<u64, MixedShellMaterializationError> {
    let mut count = 0_u64;
    for physical in edges {
        let PhysicalCarrier::Section(fragment) = physical.carrier() else {
            continue;
        };
        if section_edge(plan, fragment)?.skew_persistence().is_some() {
            count = count
                .checked_add(1)
                .ok_or(MixedShellMaterializationError::WorkCountOverflow)?;
        }
    }
    count
        .checked_mul(PERSISTENT_SKEW_CYLINDER_OPEN_SPAN_WORK)
        .ok_or(MixedShellMaterializationError::WorkCountOverflow)
}

/// Mint the persistent graph proof once for one physical Section fragment.
///
/// The operation caller must already have charged the reusable blueprint's
/// complete `work()`, including its explicit `persistent_skew_work()`.
pub(super) fn certify(
    edge: &MixedSectionEdgePlan,
) -> Result<CertifiedSkewCylinderSpan, MixedShellMaterializationError> {
    let fragment = edge.fragment_index();
    let input = edge.skew_persistence().ok_or(
        MixedShellMaterializationError::MissingPersistentSkewInput(fragment),
    )?;
    let canonical_roots = input.physical_roots();
    let logical_roots = if input.reversed() {
        [canonical_roots[1], canonical_roots[0]]
    } else {
        canonical_roots
    };
    if [logical_roots[0].endpoint(), logical_roots[1].endpoint()] != edge.endpoints() {
        return Err(
            MixedShellMaterializationError::PersistentSkewEndpointIdentityMismatch(fragment),
        );
    }
    let orientation = if input.reversed() {
        PersistentSkewCylinderOpenSpanOrientation::Reversed
    } else {
        PersistentSkewCylinderOpenSpanOrientation::Forward
    };
    let certificate = certify_persistent_skew_cylinder_open_span(
        input.residual_certificate(),
        input.root_corridors(),
        input.physical_endpoint_points(),
        orientation,
    )
    .map_err(MixedShellMaterializationError::PersistentSkewCertificate)?;
    if certificate.work() != PERSISTENT_SKEW_CYLINDER_OPEN_SPAN_WORK
        || certificate.logical_range() != ParamRange::new(0.0, 1.0)
        || certificate.endpoint_points() != [logical_roots[0].point(), logical_roots[1].point()]
    {
        return Err(MixedShellMaterializationError::InvalidAnalyticGeometry);
    }
    Ok(CertifiedSkewCylinderSpan { certificate })
}

/// Select one sealed pcurve in source operand order and retain its chart lift.
pub(super) fn pcurve_for_use(
    edge: &MixedSectionEdgePlan,
    face: MixedSourceFaceKey,
    lineage: &MixedPcurveLineage,
    pcurves: [AnalyticPcurveUse; 2],
) -> Result<AnalyticPcurveUse, MixedShellMaterializationError> {
    let MixedPcurveLineage::Section {
        branch,
        operand,
        cylinder_period_shift,
    } = lineage
    else {
        return Err(
            MixedShellMaterializationError::PersistentSkewLineageMismatch {
                fragment: edge.fragment_index(),
                face,
            },
        );
    };
    if *operand > 1
        || *branch != edge.fragment().branch()
        || face.operand() != *operand
        || edge.carrier_faces()[*operand] != face
    {
        return Err(
            MixedShellMaterializationError::PersistentSkewLineageMismatch {
                fragment: edge.fragment_index(),
                face,
            },
        );
    }
    let shift = i32::try_from(*cylinder_period_shift)
        .map_err(|_| MixedShellMaterializationError::PeriodShiftOverflow)?;
    Ok(pcurves[*operand].with_chart(PcurveChart::shifted([shift, 0])))
}
