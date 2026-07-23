//! Exact internally tangent Cylinder-support certificate over finite windows.
//!
//! The kops classifier proves the directed containment of the two unequal
//! infinite radial supports. This adapter additionally requires exact live
//! source-boundary binding, proves strict positive finite axial overlap, and
//! retains the complete total preorder of all four topology-owned endpoints.

use core::cmp::Ordering;

use kcore::predicates::Orientation;
use kops::intersect::{
    ParallelCylinderInternalTangency, ParallelCylinderRadialRelation,
    classify_parallel_cylinder_radial_relation,
};

use super::super::axial_interval_sweep::{
    AuthoredAxialEndpoint, AxialEndpointComparison, AxialEndpointContributor, AxialIntervalOperand,
    CertifiedAxialEndpointPreorder,
};
use super::super::curved_source::CertifiedCylinderSource;
use super::{
    NormalizedAxialIntervals, ParallelCylinderAxialBoundaryWitness, ParallelCylinderRelationGap,
    axial_compare, source_overlap_ends,
};

/// Exact internal radial tangency plus the complete finite endpoint order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct CertifiedParallelCylinderInternalRadialTangency {
    contained_operand: usize,
    boundaries: [ParallelCylinderAxialBoundaryWitness; 4],
    axial_parameter_bits: [[u64; 2]; 2],
    preorder: CertifiedAxialEndpointPreorder,
}

impl CertifiedParallelCylinderInternalRadialTangency {
    /// Operand whose smaller radial disk is contained by its peer.
    pub(crate) const fn contained_operand(&self) -> usize {
        self.contained_operand
    }

    /// Operand whose larger radial disk contains its peer.
    pub(crate) const fn containing_operand(&self) -> usize {
        1 - self.contained_operand
    }

    /// Source boundaries in `[Left Start, Left End, Right Start, Right End]` order.
    pub(crate) const fn boundaries(&self) -> &[ParallelCylinderAxialBoundaryWitness; 4] {
        &self.boundaries
    }

    /// Exact side-pcurve height for one certified authored boundary.
    pub(crate) const fn axial_parameter(&self, operand: usize, boundary: usize) -> Option<f64> {
        if operand < 2 && boundary < 2 {
            Some(f64::from_bits(self.axial_parameter_bits[operand][boundary]))
        } else {
            None
        }
    }

    /// Complete exact total preorder of the four authored endpoint identities.
    pub(crate) const fn preorder(&self) -> &CertifiedAxialEndpointPreorder {
        &self.preorder
    }
}

/// Bind exact directed internal tangency to strict positive finite overlap.
pub(super) fn certify_internal_radial_tangency(
    cylinders: [&CertifiedCylinderSource; 2],
    normalized: &NormalizedAxialIntervals,
) -> Result<Option<CertifiedParallelCylinderInternalRadialTangency>, ParallelCylinderRelationGap> {
    let containment = match classify_parallel_cylinder_radial_relation([
        cylinders[0].cylinder(),
        cylinders[1].cylinder(),
    ]) {
        ParallelCylinderRadialRelation::ExactInternalTangent(containment) => containment,
        _ => return Ok(None),
    };

    // Full source validity permits a fixed incidence envelope when an authored
    // ring center is only a rounded side-cylinder evaluation. Such a boundary
    // cannot become finite-window authority for analytic reconstruction.
    if normalized
        .supports
        .iter()
        .flatten()
        .any(|support| support.envelope != 0.0)
    {
        return Err(ParallelCylinderRelationGap::SourceBoundaryBinding);
    }

    // Axial gap and contact exit before this helper. Retain an independent
    // exact proof that this certificate owns a positive finite overlap.
    source_overlap_ends(normalized)?;

    let contributors = [
        AxialEndpointContributor::new(AxialIntervalOperand::Left, AuthoredAxialEndpoint::Start),
        AxialEndpointContributor::new(AxialIntervalOperand::Left, AuthoredAxialEndpoint::End),
        AxialEndpointContributor::new(AxialIntervalOperand::Right, AuthoredAxialEndpoint::Start),
        AxialEndpointContributor::new(AxialIntervalOperand::Right, AuthoredAxialEndpoint::End),
    ];
    let endpoint_pairs = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)];
    let comparisons = endpoint_pairs.map(|(first, second)| {
        let first_support = normalized.supports[first / 2][first % 2];
        let second_support = normalized.supports[second / 2][second % 2];
        let ordering = axial_compare(
            normalized.common_axis,
            first_support.point,
            second_support.point,
        )
        .map(orientation_ordering)?;
        Ok(AxialEndpointComparison::new(
            contributors[first],
            contributors[second],
            ordering,
        ))
    });
    let [first, second, third, fourth, fifth, sixth] = comparisons;
    let comparisons = [first?, second?, third?, fourth?, fifth?, sixth?];
    let preorder = CertifiedAxialEndpointPreorder::from_comparisons(comparisons)
        .map_err(|_| ParallelCylinderRelationGap::AxialEndpointOrder)?;

    let boundaries = core::array::from_fn(|index| {
        let operand = index / 2;
        let boundary = index % 2;
        let source = cylinders[operand].boundaries()[boundary];
        ParallelCylinderAxialBoundaryWitness {
            operand,
            boundary,
            cap_face: source.cap_face(),
            edge: source.edge(),
        }
    });
    let contained_operand = match containment {
        ParallelCylinderInternalTangency::FirstContainsSecond => 1,
        ParallelCylinderInternalTangency::SecondContainsFirst => 0,
    };
    Ok(Some(CertifiedParallelCylinderInternalRadialTangency {
        contained_operand,
        boundaries,
        axial_parameter_bits: normalized
            .supports
            .map(|boundaries| boundaries.map(|support| support.side_parameter.to_bits())),
        preorder,
    }))
}

const fn orientation_ordering(orientation: Orientation) -> Ordering {
    match orientation {
        Orientation::Negative => Ordering::Less,
        Orientation::Zero => Ordering::Equal,
        Orientation::Positive => Ordering::Greater,
    }
}
