//! Exact interior-knot witnesses for spatial curve-pair roots.
//!
//! The curve-pair certificate facade uses these full-multiplicity source
//! witnesses directly.

use super::NurbsCurve;
use super::curve_pair::{CurvePairProjectionPlane, certify_p_matrix};
use crate::param::ParamRange;
use crate::vec::Point3;

/// Certify exactly one full 3D root using an exact interior-knot witness.
///
/// An interior knot of multiplicity equal to the curve degree has exactly one
/// nonzero basis function at the knot. Its value is therefore the associated
/// stored Euclidean control point for both polynomial and positive-weight
/// rational curves. Bit-equal witness control points prove 3D existence
/// without relying on floating-point evaluation or a coordinate projection.
///
/// Uniqueness is certified independently: after row sign changes, an interval
/// enclosure of one projected difference Jacobian must be a strict P-matrix on
/// the complete parameter rectangle. The interval family also contains every
/// componentwise average Jacobian along a parameter-space line segment, so the
/// P-matrix univalence argument remains valid across the witness's piecewise-C1
/// knot. It permits at most one projected zero, hence at most one full 3D root.
/// Any failed condition is inconclusive.
pub(crate) fn certify_spatial_interior_root(
    first: &NurbsCurve,
    second: &NurbsCurve,
) -> Option<(CurvePairProjectionPlane, f64)> {
    if !first.knots().is_clamped() || !second.knots().is_clamped() {
        return None;
    }
    exact_interior_witness_in_ranges(
        first,
        first.knots().domain(),
        second,
        second.knots().domain(),
    )?;

    for (projection_plane, axes) in [
        (CurvePairProjectionPlane::Xy, [0, 1]),
        (CurvePairProjectionPlane::Xz, [0, 2]),
        (CurvePairProjectionPlane::Yz, [1, 2]),
    ] {
        for axes in [axes, [axes[1], axes[0]]] {
            for signs in [[1.0, 1.0], [1.0, -1.0], [-1.0, 1.0], [-1.0, -1.0]] {
                if let Some(determinant_lower_bound) = certify_p_matrix(first, second, axes, signs)
                {
                    return Some((projection_plane, determinant_lower_bound));
                }
            }
        }
    }
    None
}

/// Find a bit-exact common point represented by full-multiplicity source knots.
pub(super) fn exact_interior_witness_in_ranges(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
) -> Option<(f64, f64)> {
    let first_witnesses = full_multiplicity_interior_points(first)
        .into_iter()
        .filter(|(parameter, _)| first_range.contains(*parameter))
        .collect::<Vec<_>>();
    let second_witnesses = full_multiplicity_interior_points(second)
        .into_iter()
        .filter(|(parameter, _)| second_range.contains(*parameter))
        .collect::<Vec<_>>();
    first_witnesses
        .into_iter()
        .find_map(|(first_parameter, first_point)| {
            second_witnesses
                .iter()
                .find(|(_, second_point)| first_point == *second_point)
                .map(|(second_parameter, _)| (first_parameter, *second_parameter))
        })
}

fn full_multiplicity_interior_points(curve: &NurbsCurve) -> Vec<(f64, Point3)> {
    let degree = curve.degree();
    let knots = curve.knots();
    let domain = knots.domain();
    let values = knots.as_slice();
    let mut witnesses = Vec::new();
    let mut index = degree + 1;
    while index + degree < values.len() {
        let parameter = values[index];
        let mut end = index + 1;
        while end < values.len() && values[end] == parameter {
            end += 1;
        }
        if domain.lo < parameter && parameter < domain.hi && end - index == degree {
            let span = end - 1;
            witnesses.push((parameter, curve.points()[span - degree]));
        }
        index = end;
    }
    witnesses
}
