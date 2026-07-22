//! Exact common-cylinder-support certificate over four finite axial endpoints.
//!
//! The radial classifier proves only the infinite Cylinder supports. This
//! adapter additionally requires every live ring center to be an exact source
//! cylinder evaluation, proves strict positive finite overlap, and retains the
//! complete six-comparison preorder of all four topology-owned endpoints.

use core::cmp::Ordering;

use kcore::predicates::Orientation;
use kops::intersect::{ParallelCylinderRadialRelation, classify_parallel_cylinder_radial_relation};

use super::super::axial_interval_sweep::{
    AuthoredAxialEndpoint, AxialEndpointComparison, AxialEndpointContributor, AxialIntervalOperand,
    CertifiedAxialEndpointPreorder,
};
use super::super::curved_source::CertifiedCylinderSource;
use super::{
    NormalizedAxialIntervals, ParallelCylinderAxialBoundaryWitness, ParallelCylinderRelationGap,
    axial_compare, source_overlap_ends,
};

/// Exact common radial support plus the complete finite axial endpoint order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct CertifiedParallelCylinderCommonSupport {
    boundaries: [ParallelCylinderAxialBoundaryWitness; 4],
    preorder: CertifiedAxialEndpointPreorder,
}

impl CertifiedParallelCylinderCommonSupport {
    /// Source boundaries in `[Left Start, Left End, Right Start, Right End]` order.
    pub(crate) const fn boundaries(&self) -> &[ParallelCylinderAxialBoundaryWitness; 4] {
        &self.boundaries
    }

    /// Complete exact total preorder of the four authored endpoint identities.
    pub(crate) const fn preorder(&self) -> &CertifiedAxialEndpointPreorder {
        &self.preorder
    }
}

/// Bind exact common infinite support to strict positive finite axial overlap.
pub(super) fn certify_common_support(
    cylinders: [&CertifiedCylinderSource; 2],
    normalized: &NormalizedAxialIntervals,
) -> Result<Option<CertifiedParallelCylinderCommonSupport>, ParallelCylinderRelationGap> {
    if classify_parallel_cylinder_radial_relation([
        cylinders[0].cylinder(),
        cylinders[1].cylinder(),
    ]) != ParallelCylinderRadialRelation::ExactCommonSupport
    {
        return Ok(None);
    }

    // Full validity permits a fixed incidence envelope when an authored ring
    // center is only a rounded side-cylinder evaluation. Such a ring cannot
    // become exact common-support reconstruction authority.
    if normalized
        .supports
        .iter()
        .flatten()
        .any(|support| support.envelope != 0.0)
    {
        return Err(ParallelCylinderRelationGap::SourceBoundaryBinding);
    }

    // Gap and contact already exit before this helper. Retain an independent
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
    Ok(Some(CertifiedParallelCylinderCommonSupport {
        boundaries,
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
