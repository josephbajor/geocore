//! Conservative coordinate enclosures for intersections of three planes.
//!
//! The input plane witnesses are finite dyadic data. Outward interval
//! arithmetic encloses their homogeneous coefficients and Cramer solve. A
//! result is returned only when the solve is unique, sufficiently conditioned,
//! and wholly contained by the caller's model-space size box.

use crate::interval::Interval;
use crate::predicates::OrientedPlanePoints;

type PlaneIntervals = [Interval; 4];

/// Certified enclosure of one unique, usable three-plane intersection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlaneTripleEnclosure {
    coordinates: [Interval; 3],
    reciprocal_condition_lower_bound: f64,
}

impl PlaneTripleEnclosure {
    /// Outward coordinate intervals containing the exact intersection.
    pub const fn coordinates(self) -> [Interval; 3] {
        self.coordinates
    }

    /// Certified lower bound for `1 / (||A||_inf * ||A^-1||_inf)`.
    pub const fn reciprocal_condition_lower_bound(self) -> f64 {
        self.reciprocal_condition_lower_bound
    }
}

/// Fail-closed reason that a three-plane enclosure was not certified.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlaneTripleEnclosureError {
    /// The size-box or reciprocal-condition threshold is invalid.
    InvalidLimits,
    /// A witness contains a non-finite coordinate.
    NonFinitePlane,
    /// A witness point lies outside the supplied size box.
    PlaneWitnessOutsideSizeBox,
    /// A plane is degenerate or interval arithmetic cannot certify its normal.
    UncertifiedPlane,
    /// The three planes are dependent or interval arithmetic cannot certify a unique solve.
    UncertifiedIntersection,
    /// The certified reciprocal-condition lower bound misses the requested threshold.
    IllConditioned,
    /// The complete coordinate enclosure is not inside the supplied size box.
    IntersectionOutsideSizeBox,
}

/// Enclose the unique intersection of three oriented planes.
///
/// `size_box_half` must be finite and positive. Every witness point and the
/// complete returned coordinate enclosure must lie in the closed box
/// `[-size_box_half, size_box_half]^3`. `minimum_reciprocal_condition` must be
/// finite and in `(0, 1]`; equality with the certified lower bound is usable.
///
/// The three whole witnesses are canonicalized before arithmetic, so their
/// caller order cannot affect result bits. No midpoint or tolerance decision
/// is made here: an uncertain denominator, overflowed enclosure, or marginal
/// condition estimate is an explicit refusal.
pub fn enclose_plane_triple_intersection(
    mut planes: [OrientedPlanePoints; 3],
    size_box_half: f64,
    minimum_reciprocal_condition: f64,
) -> Result<PlaneTripleEnclosure, PlaneTripleEnclosureError> {
    validate_limits(size_box_half, minimum_reciprocal_condition)?;
    validate_witnesses(&planes, size_box_half)?;
    planes.sort_by(compare_plane_witnesses);

    let planes = planes
        .map(interval_oriented_plane)
        .into_iter()
        .collect::<Result<Vec<_>, _>>()?;
    let planes: [PlaneIntervals; 3] = planes
        .try_into()
        .map_err(|_| PlaneTripleEnclosureError::UncertifiedIntersection)?;
    let spatial = planes.map(|plane| [plane[0], plane[1], plane[2]]);
    let determinant = interval_det3(spatial);
    let determinant_magnitude = strict_magnitude_lower(determinant)
        .ok_or(PlaneTripleEnclosureError::UncertifiedIntersection)?;

    let matrix_norm = matrix_infinity_norm_upper(spatial)
        .ok_or(PlaneTripleEnclosureError::UncertifiedIntersection)?;
    let adjugate_norm = adjugate_infinity_norm_upper(spatial)
        .ok_or(PlaneTripleEnclosureError::UncertifiedIntersection)?;
    let condition_denominator = Interval::point(matrix_norm) * Interval::point(adjugate_norm);
    let reciprocal_condition_lower_bound = Interval::point(determinant_magnitude)
        .checked_div(condition_denominator)
        .map(Interval::lo)
        .filter(|value| value.is_finite() && *value > 0.0 && *value <= 1.0)
        .ok_or(PlaneTripleEnclosureError::UncertifiedIntersection)?;
    if reciprocal_condition_lower_bound < minimum_reciprocal_condition {
        return Err(PlaneTripleEnclosureError::IllConditioned);
    }

    let constants = [-planes[0][3], -planes[1][3], -planes[2][3]];
    let coordinates: [Option<Interval>; 3] = core::array::from_fn(|column| {
        let mut numerator_matrix = spatial;
        for row in 0..3 {
            numerator_matrix[row][column] = constants[row];
        }
        interval_det3(numerator_matrix).checked_div(determinant)
    });
    let coordinates = coordinates
        .into_iter()
        .collect::<Option<Vec<_>>>()
        .ok_or(PlaneTripleEnclosureError::UncertifiedIntersection)?;
    let coordinates: [Interval; 3] = coordinates
        .try_into()
        .map_err(|_| PlaneTripleEnclosureError::UncertifiedIntersection)?;
    if coordinates.iter().any(|coordinate| {
        !interval_is_finite(*coordinate)
            || coordinate.lo() < -size_box_half
            || coordinate.hi() > size_box_half
    }) {
        return Err(PlaneTripleEnclosureError::IntersectionOutsideSizeBox);
    }

    Ok(PlaneTripleEnclosure {
        coordinates,
        reciprocal_condition_lower_bound,
    })
}

fn validate_limits(
    size_box_half: f64,
    minimum_reciprocal_condition: f64,
) -> Result<(), PlaneTripleEnclosureError> {
    if !size_box_half.is_finite()
        || size_box_half <= 0.0
        || !minimum_reciprocal_condition.is_finite()
        || minimum_reciprocal_condition <= 0.0
        || minimum_reciprocal_condition > 1.0
    {
        Err(PlaneTripleEnclosureError::InvalidLimits)
    } else {
        Ok(())
    }
}

fn validate_witnesses(
    planes: &[OrientedPlanePoints; 3],
    size_box_half: f64,
) -> Result<(), PlaneTripleEnclosureError> {
    for coordinate in planes.iter().flatten().flatten() {
        if !coordinate.is_finite() {
            return Err(PlaneTripleEnclosureError::NonFinitePlane);
        }
        if coordinate.abs() > size_box_half {
            return Err(PlaneTripleEnclosureError::PlaneWitnessOutsideSizeBox);
        }
    }
    Ok(())
}

fn compare_plane_witnesses(
    first: &OrientedPlanePoints,
    second: &OrientedPlanePoints,
) -> core::cmp::Ordering {
    first
        .iter()
        .flatten()
        .zip(second.iter().flatten())
        .map(|(first, second)| first.total_cmp(second))
        .find(|ordering| !ordering.is_eq())
        .unwrap_or(core::cmp::Ordering::Equal)
}

fn interval_oriented_plane(
    points: OrientedPlanePoints,
) -> Result<PlaneIntervals, PlaneTripleEnclosureError> {
    let u =
        [0, 1, 2].map(|axis| Interval::point(points[1][axis]) - Interval::point(points[0][axis]));
    let v =
        [0, 1, 2].map(|axis| Interval::point(points[2][axis]) - Interval::point(points[0][axis]));
    let conventional = [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ];
    if conventional
        .iter()
        .all(|component| component.contains_zero())
    {
        return Err(PlaneTripleEnclosureError::UncertifiedPlane);
    }
    let spatial = conventional.map(|component| -component);
    let constant = conventional[0] * Interval::point(points[0][0])
        + conventional[1] * Interval::point(points[0][1])
        + conventional[2] * Interval::point(points[0][2]);
    let plane = [spatial[0], spatial[1], spatial[2], constant];
    plane
        .iter()
        .all(|component| interval_is_finite(*component))
        .then_some(plane)
        .ok_or(PlaneTripleEnclosureError::UncertifiedPlane)
}

fn interval_det3(matrix: [[Interval; 3]; 3]) -> Interval {
    matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
}

fn interval_is_finite(interval: Interval) -> bool {
    interval.lo().is_finite() && interval.hi().is_finite()
}

fn interval_abs_upper(interval: Interval) -> Option<f64> {
    let upper = interval.lo().abs().max(interval.hi().abs());
    upper.is_finite().then_some(upper)
}

fn strict_magnitude_lower(interval: Interval) -> Option<f64> {
    let lower = if interval.lo() > 0.0 {
        interval.lo()
    } else if interval.hi() < 0.0 {
        -interval.hi()
    } else {
        return None;
    };
    (lower.is_finite() && lower > 0.0).then_some(lower)
}

fn matrix_infinity_norm_upper(matrix: [[Interval; 3]; 3]) -> Option<f64> {
    matrix
        .into_iter()
        .map(|row| {
            row.into_iter()
                .try_fold(Interval::point(0.0), |sum, value| {
                    Some(sum + Interval::point(interval_abs_upper(value)?))
                })
        })
        .collect::<Option<Vec<_>>>()?
        .into_iter()
        .map(Interval::hi)
        .reduce(f64::max)
        .filter(|norm| norm.is_finite() && *norm > 0.0)
}

fn adjugate_infinity_norm_upper(matrix: [[Interval; 3]; 3]) -> Option<f64> {
    let cofactor = |row: usize, column: usize| {
        let rows = [(row + 1) % 3, (row + 2) % 3];
        let columns = [(column + 1) % 3, (column + 2) % 3];
        matrix[rows[0]][columns[0]] * matrix[rows[1]][columns[1]]
            - matrix[rows[0]][columns[1]] * matrix[rows[1]][columns[0]]
    };
    (0..3)
        .map(|adjugate_row| {
            (0..3).try_fold(Interval::point(0.0), |sum, adjugate_column| {
                Some(
                    sum + Interval::point(interval_abs_upper(cofactor(
                        adjugate_column,
                        adjugate_row,
                    ))?),
                )
            })
        })
        .collect::<Option<Vec<_>>>()?
        .into_iter()
        .map(Interval::hi)
        .reduce(f64::max)
        .filter(|norm| norm.is_finite() && *norm > 0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn det3(matrix: [[i128; 3]; 3]) -> i128 {
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
            - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
            + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    }

    fn cross(first: [i64; 3], second: [i64; 3]) -> [i64; 3] {
        [
            first[1] * second[2] - first[2] * second[1],
            first[2] * second[0] - first[0] * second[2],
            first[0] * second[1] - first[1] * second[0],
        ]
    }

    fn add(first: [i64; 3], second: [i64; 3]) -> [i64; 3] {
        core::array::from_fn(|axis| first[axis] + second[axis])
    }

    fn plane_through(normal: [i64; 3], point: [i64; 3]) -> [[i64; 3]; 3] {
        let basis = if normal[0] != 0 || normal[1] != 0 {
            [0, 0, 1]
        } else {
            [1, 0, 0]
        };
        let first = cross(normal, basis);
        let second = cross(normal, first);
        [point, add(point, first), add(point, second)]
    }

    fn floating(points: [[i64; 3]; 3]) -> OrientedPlanePoints {
        points.map(|point| point.map(|coordinate| coordinate as f64))
    }

    fn oriented_plane(points: [[i64; 3]; 3]) -> [i128; 4] {
        let u: [i128; 3] =
            core::array::from_fn(|axis| i128::from(points[1][axis] - points[0][axis]));
        let v: [i128; 3] =
            core::array::from_fn(|axis| i128::from(points[2][axis] - points[0][axis]));
        let conventional = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        [
            -conventional[0],
            -conventional[1],
            -conventional[2],
            conventional
                .iter()
                .enumerate()
                .map(|(axis, coefficient)| coefficient * i128::from(points[0][axis]))
                .sum(),
        ]
    }

    fn cramer(planes: [[[i64; 3]; 3]; 3]) -> Option<([i128; 3], i128)> {
        let planes = planes.map(oriented_plane);
        let spatial = planes.map(|plane| [plane[0], plane[1], plane[2]]);
        let denominator = det3(spatial);
        if denominator == 0 {
            return None;
        }
        let constants = [-planes[0][3], -planes[1][3], -planes[2][3]];
        let numerators = core::array::from_fn(|column| {
            let mut matrix = spatial;
            for row in 0..3 {
                matrix[row][column] = constants[row];
            }
            det3(matrix)
        });
        Some((numerators, denominator))
    }

    fn axis_plane(axis: usize, coordinate: f64) -> OrientedPlanePoints {
        match axis {
            0 => [
                [coordinate, 0.0, 0.0],
                [coordinate, 1.0, 0.0],
                [coordinate, 0.0, 1.0],
            ],
            1 => [
                [0.0, coordinate, 0.0],
                [0.0, coordinate, 1.0],
                [1.0, coordinate, 0.0],
            ],
            _ => [
                [0.0, 0.0, coordinate],
                [1.0, 0.0, coordinate],
                [0.0, 1.0, coordinate],
            ],
        }
    }

    #[test]
    fn randomized_integer_systems_enclose_independent_i128_cramer_solutions() {
        let mut state = 0xD1B5_4A32_D192_ED03_u64;
        let mut next = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        let mut accepted = 0;
        for _ in 0..8_000 {
            let solution = core::array::from_fn(|_| (next() % 31) as i64 - 15);
            let normals =
                core::array::from_fn(|_| core::array::from_fn(|_| (next() % 11) as i64 - 5));
            if normals.contains(&[0; 3]) {
                continue;
            }
            let integer_planes = normals.map(|normal| plane_through(normal, solution));
            let Some((numerators, denominator)) = cramer(integer_planes) else {
                continue;
            };
            for axis in 0..3 {
                assert_eq!(numerators[axis], i128::from(solution[axis]) * denominator);
            }
            let Ok(enclosure) = enclose_plane_triple_intersection(
                integer_planes.map(floating),
                1_024.0,
                128.0 * f64::EPSILON,
            ) else {
                continue;
            };
            for (coordinate, expected) in enclosure.coordinates().into_iter().zip(solution) {
                assert!(coordinate.contains(expected as f64));
            }
            accepted += 1;
        }
        assert!(accepted > 5_000, "accepted only {accepted} systems");
    }

    #[test]
    fn defining_plane_permutation_preserves_all_result_bits() {
        let solution = [7, -11, 3];
        let planes = [[2, -3, 5], [-7, 4, 3], [6, 5, -2]]
            .map(|normal| floating(plane_through(normal, solution)));
        let permutations = [
            [planes[0], planes[1], planes[2]],
            [planes[0], planes[2], planes[1]],
            [planes[1], planes[0], planes[2]],
            [planes[1], planes[2], planes[0]],
            [planes[2], planes[0], planes[1]],
            [planes[2], planes[1], planes[0]],
        ];
        let expected =
            enclose_plane_triple_intersection(permutations[0], 1_024.0, f64::EPSILON).unwrap();
        for permutation in permutations.into_iter().skip(1) {
            let actual =
                enclose_plane_triple_intersection(permutation, 1_024.0, f64::EPSILON).unwrap();
            assert_eq!(actual, expected);
            assert_eq!(
                actual.reciprocal_condition_lower_bound().to_bits(),
                expected.reciprocal_condition_lower_bound().to_bits()
            );
            for (actual, expected) in actual.coordinates().into_iter().zip(expected.coordinates()) {
                assert_eq!(actual.lo().to_bits(), expected.lo().to_bits());
                assert_eq!(actual.hi().to_bits(), expected.hi().to_bits());
            }
        }
    }

    #[test]
    fn conditioning_threshold_is_inclusive_and_fails_closed_above_the_bound() {
        let planes = [axis_plane(0, 2.0), axis_plane(1, -3.0), axis_plane(2, 5.0)];
        let initial = enclose_plane_triple_intersection(planes, 64.0, f64::EPSILON).unwrap();
        let bound = initial.reciprocal_condition_lower_bound();
        assert_eq!(
            enclose_plane_triple_intersection(planes, 64.0, bound).unwrap(),
            initial
        );
        assert_eq!(
            enclose_plane_triple_intersection(planes, 64.0, bound.next_up()),
            Err(PlaneTripleEnclosureError::IllConditioned)
        );
    }

    #[test]
    fn invalid_degenerate_uncertain_and_out_of_box_inputs_are_refused() {
        let valid = [axis_plane(0, 2.0), axis_plane(1, -3.0), axis_plane(2, 5.0)];
        for (size, condition) in [
            (0.0, f64::EPSILON),
            (f64::INFINITY, f64::EPSILON),
            (64.0, 0.0),
            (64.0, f64::NAN),
            (64.0, 1.0 + f64::EPSILON),
        ] {
            assert_eq!(
                enclose_plane_triple_intersection(valid, size, condition),
                Err(PlaneTripleEnclosureError::InvalidLimits)
            );
        }

        let mut nonfinite = valid;
        nonfinite[0][0][0] = f64::NAN;
        assert_eq!(
            enclose_plane_triple_intersection(nonfinite, 64.0, f64::EPSILON),
            Err(PlaneTripleEnclosureError::NonFinitePlane)
        );

        let mut witness_outside = valid;
        witness_outside[0][0][1] = 65.0;
        assert_eq!(
            enclose_plane_triple_intersection(witness_outside, 64.0, f64::EPSILON),
            Err(PlaneTripleEnclosureError::PlaneWitnessOutsideSizeBox)
        );

        let degenerate = [
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
            valid[1],
            valid[2],
        ];
        assert_eq!(
            enclose_plane_triple_intersection(degenerate, 64.0, f64::EPSILON),
            Err(PlaneTripleEnclosureError::UncertifiedPlane)
        );

        let dependent = [axis_plane(0, 2.0), axis_plane(0, 3.0), axis_plane(2, 5.0)];
        assert_eq!(
            enclose_plane_triple_intersection(dependent, 64.0, f64::EPSILON),
            Err(PlaneTripleEnclosureError::UncertifiedIntersection)
        );

        let outside_intersection = [
            axis_plane(2, 0.0),
            axis_plane(1, 0.0),
            [[10.0, 0.0, 9.0], [9.0, 0.0, 10.0], [10.0, 1.0, 9.0]],
        ];
        assert_eq!(
            enclose_plane_triple_intersection(outside_intersection, 10.0, f64::EPSILON),
            Err(PlaneTripleEnclosureError::IntersectionOutsideSizeBox)
        );
    }
}
