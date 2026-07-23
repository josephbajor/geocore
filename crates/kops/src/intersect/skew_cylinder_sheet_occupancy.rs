//! Root-free finite-window occupancy for strict-positive skew-cylinder sheets.
//!
//! Four exact axial-bound queries classify each complete infinite-support
//! sheet as wholly contained, wholly outside, or clipped. The caller owns one
//! atomic ledger reservation for all four queries, so no prefix can publish a
//! partial finite-window result.

use kgeom::param::ParamRange;
use kgeom::surface::Cylinder;
use kgraph::SkewCylinderSheet;

use super::skew_cylinder_axial_roots::{
    SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK, SkewCylinderAxialBoundProvenance,
    SkewCylinderAxialBoundTopology, SkewCylinderAxialBoundary, SkewCylinderAxialRelation,
    SkewCylinderAxialRootFailure, classify_skew_cylinder_axial_bound,
};

/// Atomic work for the two lower and two upper axial-bound queries.
pub(super) const SKEW_CYLINDER_SHEET_OCCUPANCY_EXACT_WORK: u64 =
    4 * SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK;

/// Complete-cycle relation of one sheet to both finite axial windows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum SkewCylinderSheetOccupancy {
    Outside,
    Contained,
    Clipped,
}

/// Classify both ordered sheets from four exact caller-authored bounds.
pub(super) fn classify_skew_cylinder_sheet_occupancy(
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
    canonical_to_source: [usize; 2],
) -> Result<[SkewCylinderSheetOccupancy; 2], SkewCylinderAxialRootFailure> {
    let sheets = [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper];
    let mut clipped = [false; 2];
    let mut contained = [true; 2];

    for canonical_operand in 0..2 {
        let source_operand = canonical_to_source[canonical_operand];
        for (boundary, value, required) in [
            (
                SkewCylinderAxialBoundary::Lower,
                ranges[canonical_operand][1].lo,
                SkewCylinderAxialRelation::Above,
            ),
            (
                SkewCylinderAxialBoundary::Upper,
                ranges[canonical_operand][1].hi,
                SkewCylinderAxialRelation::Below,
            ),
        ] {
            let topology = classify_skew_cylinder_axial_bound(
                cylinders,
                canonical_to_source,
                SkewCylinderAxialBoundProvenance {
                    source_operand,
                    boundary,
                    value,
                },
                SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
            )?;
            for (index, sheet) in sheets.into_iter().enumerate() {
                if topology.roots.iter().any(|root| root.sheet == sheet) {
                    clipped[index] = true;
                } else {
                    contained[index] &= root_free_relation(&topology, index)? == required;
                }
            }
        }
    }

    Ok(core::array::from_fn(|index| {
        if clipped[index] {
            SkewCylinderSheetOccupancy::Clipped
        } else if contained[index] {
            SkewCylinderSheetOccupancy::Contained
        } else {
            SkewCylinderSheetOccupancy::Outside
        }
    }))
}

fn root_free_relation(
    topology: &SkewCylinderAxialBoundTopology,
    sheet_index: usize,
) -> Result<SkewCylinderAxialRelation, SkewCylinderAxialRootFailure> {
    let relation = topology
        .open_cell_relations
        .first()
        .map(|relations| relations[sheet_index])
        .ok_or(SkewCylinderAxialRootFailure::InconsistentTopology)?;
    if topology
        .open_cell_relations
        .iter()
        .all(|relations| relations[sheet_index] == relation)
    {
        Ok(relation)
    } else {
        Err(SkewCylinderAxialRootFailure::InconsistentTopology)
    }
}
