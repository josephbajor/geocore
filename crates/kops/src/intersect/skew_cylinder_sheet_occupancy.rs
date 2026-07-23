//! Atomic finite-window bound collection for strict-positive skew sheets.
//!
//! The caller owns one ledger reservation for all four exact axial-bound
//! queries, so no prefix can publish a partial finite-window result. A sibling
//! pure topology merge consumes the complete set.

use kgeom::param::ParamRange;
use kgeom::surface::Cylinder;

use kgraph::{
    SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK, SkewCylinderAxialBoundProvenance,
    SkewCylinderAxialBoundTopology, SkewCylinderAxialBoundary, SkewCylinderAxialRootFailure,
    classify_skew_cylinder_axial_bound,
};

/// Atomic work for the two lower and two upper axial-bound queries.
pub(super) const SKEW_CYLINDER_AXIAL_BOUNDS_EXACT_WORK: u64 =
    4 * SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK;

/// Collect all four caller-authored bounds after the owner reserves the whole
/// atomic work unit.
pub(super) fn collect_skew_cylinder_axial_bound_topologies(
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
    canonical_to_source: [usize; 2],
) -> Result<[SkewCylinderAxialBoundTopology; 4], SkewCylinderAxialRootFailure> {
    let mut topologies = Vec::with_capacity(4);
    for canonical_operand in 0..2 {
        let source_operand = canonical_to_source[canonical_operand];
        for (boundary, value) in [
            (
                SkewCylinderAxialBoundary::Lower,
                ranges[canonical_operand][1].lo,
            ),
            (
                SkewCylinderAxialBoundary::Upper,
                ranges[canonical_operand][1].hi,
            ),
        ] {
            topologies.push(classify_skew_cylinder_axial_bound(
                cylinders,
                canonical_to_source,
                SkewCylinderAxialBoundProvenance {
                    source_operand,
                    boundary,
                    value,
                },
                SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
            )?);
        }
    }
    topologies
        .try_into()
        .map_err(|_| SkewCylinderAxialRootFailure::InconsistentTopology)
}
