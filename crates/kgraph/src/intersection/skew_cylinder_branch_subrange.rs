//! Independent residual certification for bounded skew-cylinder sheet arcs.

use super::*;

const EDGE_PROOF_CELLS: usize = 96;
const EDGE_BINARY_SCALES: usize = 48;

/// Certify one strict, non-wrapping subrange of a finite skew-cylinder sheet.
///
/// `carrier_range` is expressed in the canonical first cylinder's authored
/// longitude chart. It must lie strictly inside that complete-period window;
/// callers retain any exact axial-root endpoint provenance separately. The
/// proof is constructed directly over the requested range and does not require
/// a whole-cycle sheet certificate.
pub fn certify_paired_skew_cylinder_branch_subrange_residuals(
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
    carrier_range: ParamRange,
    sheet: SkewCylinderSheet,
    tolerance: f64,
) -> Result<PairedSkewCylinderBranchResidualCertificate, IntersectionCertificateError> {
    validate_inputs(cylinders, ranges, tolerance)?;
    if !carrier_range.is_finite() || carrier_range.width() <= 0.0 {
        return Err(IntersectionCertificateError::InvalidCarrierRange);
    }
    let authored = ranges[0][0];
    if carrier_range.lo <= authored.lo || carrier_range.hi >= authored.hi {
        return Err(unsupported(
            "skew Cylinder/Cylinder subrange must lie strictly inside one canonical authored longitude window without wrapping",
        ));
    }
    if !axes_are_exactly_nonparallel(cylinders) {
        return Err(unsupported(
            "skew Cylinder/Cylinder branch requires exact-predicate nonparallel axes",
        ));
    }

    let algebra = build_algebra(cylinders, carrier_range, sheet)
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    certify_validated_branch(algebra, ranges, tolerance)
}

pub(super) fn proof_cell_boundary(range: ParamRange, index: usize, boundary_graded: bool) -> f64 {
    if index == 0 {
        return range.lo;
    }
    if index == SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS {
        return range.hi;
    }
    if !boundary_graded {
        return range.lo
            + range.width() * index as f64 / SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS as f64;
    }

    const CENTER_CELLS: usize = SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS - 2 * EDGE_PROOF_CELLS;
    let fraction = if index <= EDGE_PROOF_CELLS {
        0.25 * edge_ramp_fraction(index)
    } else if index >= SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS - EDGE_PROOF_CELLS {
        1.0 - 0.25 * edge_ramp_fraction(SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS - index)
    } else {
        0.25 + 0.5 * (index - EDGE_PROOF_CELLS) as f64 / CENTER_CELLS as f64
    };
    range.lo + range.width() * fraction
}

fn edge_ramp_fraction(step: usize) -> f64 {
    debug_assert!((1..=EDGE_PROOF_CELLS).contains(&step));
    // The edge cells span forty-eight binary scales. Linear interpolation
    // between adjacent powers of two keeps every cell narrower than its
    // distance from the retained endpoint, avoiding harmonic dependency
    // leakage without changing the fixed 256-cell logical work contract.
    let scaled = (EDGE_PROOF_CELLS - step) * EDGE_BINARY_SCALES;
    let whole = scaled / (EDGE_PROOF_CELLS - 1);
    let remainder = scaled % (EDGE_PROOF_CELLS - 1);
    let mut power = 1.0;
    for _ in 0..whole {
        power *= 0.5;
    }
    power * (1.0 - remainder as f64 / (2 * (EDGE_PROOF_CELLS - 1)) as f64)
}
