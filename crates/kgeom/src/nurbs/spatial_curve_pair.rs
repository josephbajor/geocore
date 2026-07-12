//! Conservative unique-root certificates for genuinely spatial curve pairs.

use super::NurbsCurve;
use super::curve_pair::{
    CurvePairProjectionPlane, certify_p_matrix, certify_p_matrix_in_ranges, has_exact_common_corner,
};
use crate::param::ParamRange;

/// Certify a unique full 3D root supplied by an exact shared parameter corner.
///
/// The shared endpoint is an exact existence witness. A strict interval
/// P-matrix for one two-coordinate difference map makes that projected map
/// injective on the complete parameter rectangle, so no second full 3D root
/// can exist. Failure is inconclusive and must retain the caller's candidate.
pub(super) fn certify_spatial_common_corner(
    first: &NurbsCurve,
    second: &NurbsCurve,
) -> Option<(CurvePairProjectionPlane, f64)> {
    if !has_exact_common_corner(first, second) {
        return None;
    }

    certify_injective_projection(first, second)
}

/// Find a coordinate difference map that is injective on the whole cell.
pub(super) fn certify_injective_projection(
    first: &NurbsCurve,
    second: &NurbsCurve,
) -> Option<(CurvePairProjectionPlane, f64)> {
    for (plane, axes) in [
        (CurvePairProjectionPlane::Xy, [0, 1]),
        (CurvePairProjectionPlane::Xz, [0, 2]),
        (CurvePairProjectionPlane::Yz, [1, 2]),
    ] {
        for axes in [axes, [axes[1], axes[0]]] {
            for signs in [[1.0, 1.0], [1.0, -1.0], [-1.0, 1.0], [-1.0, -1.0]] {
                if let Some(bound) = certify_p_matrix(first, second, axes, signs) {
                    return Some((plane, bound));
                }
            }
        }
    }
    None
}

/// Find an injective coordinate difference map over finite source ranges.
pub(super) fn certify_injective_projection_in_ranges(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
) -> Option<(CurvePairProjectionPlane, f64)> {
    for (plane, axes) in [
        (CurvePairProjectionPlane::Xy, [0, 1]),
        (CurvePairProjectionPlane::Xz, [0, 2]),
        (CurvePairProjectionPlane::Yz, [1, 2]),
    ] {
        for axes in [axes, [axes[1], axes[0]]] {
            for signs in [[1.0, 1.0], [1.0, -1.0], [-1.0, 1.0], [-1.0, -1.0]] {
                if let Some(bound) = certify_p_matrix_in_ranges(
                    first,
                    first_range,
                    second,
                    second_range,
                    axes,
                    signs,
                ) {
                    return Some((plane, bound));
                }
            }
        }
    }
    None
}
