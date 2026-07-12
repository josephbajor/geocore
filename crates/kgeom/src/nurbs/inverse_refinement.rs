//! Checked inverse knot refinement for NURBS curves.
//!
//! Floating-point knot insertion is not algebraically reversible: solving the
//! insertion equations can produce a nearby predecessor that represents a
//! different curve.  This module therefore treats inverse solutions only as
//! candidates.  A candidate is returned only when inserting the removed knot
//! through [`NurbsCurve::with_knot_inserted`] reproduces the stored refined
//! representation exactly.

use super::NurbsCurve;
use crate::vec::Point3;
use std::collections::HashSet;

/// Default maximum number of exact predecessor representations explored by
/// [`checked_refinement_ancestors`].
pub const CHECKED_REFINEMENT_ANCESTOR_LIMIT: usize = 4_096;

#[derive(Clone, Copy)]
enum Control {
    Polynomial([f64; 3]),
    Homogeneous([f64; 4]),
}

#[derive(PartialEq, Eq, Hash)]
struct CurveBits {
    degree: usize,
    knots: Vec<u64>,
    points: Vec<[u64; 3]>,
    weights: Option<Vec<u64>>,
}

impl CurveBits {
    fn new(curve: &NurbsCurve) -> Self {
        Self {
            degree: curve.degree(),
            knots: curve
                .knots()
                .as_slice()
                .iter()
                .map(|&value| canonical_bits(value))
                .collect(),
            points: curve
                .points()
                .iter()
                .map(|point| {
                    [
                        canonical_bits(point.x),
                        canonical_bits(point.y),
                        canonical_bits(point.z),
                    ]
                })
                .collect(),
            weights: curve
                .weights()
                .map(|weights| weights.iter().map(|&value| canonical_bits(value)).collect()),
        }
    }
}

impl Control {
    fn solve_right(self, blended: Self, alpha: f64) -> Option<Self> {
        let inverse = 1.0 / alpha;
        let left_scale = 1.0 - alpha;
        match (self, blended) {
            (Self::Polynomial(left), Self::Polynomial(blended)) => {
                Some(Self::Polynomial(core::array::from_fn(|axis| {
                    (blended[axis] - left_scale * left[axis]) * inverse
                })))
            }
            (Self::Homogeneous(left), Self::Homogeneous(blended)) => {
                Some(Self::Homogeneous(core::array::from_fn(|axis| {
                    (blended[axis] - left_scale * left[axis]) * inverse
                })))
            }
            _ => None,
        }
    }

    fn solve_left(self, blended: Self, alpha: f64) -> Option<Self> {
        let inverse = 1.0 / (1.0 - alpha);
        match (self, blended) {
            (Self::Polynomial(right), Self::Polynomial(blended)) => {
                Some(Self::Polynomial(core::array::from_fn(|axis| {
                    (blended[axis] - alpha * right[axis]) * inverse
                })))
            }
            (Self::Homogeneous(right), Self::Homogeneous(blended)) => {
                Some(Self::Homogeneous(core::array::from_fn(|axis| {
                    (blended[axis] - alpha * right[axis]) * inverse
                })))
            }
            _ => None,
        }
    }
}

/// Return every checked predecessor obtained by removing one occurrence of
/// the exact interior knot `u`.
///
/// The returned set contains at most two curves: inverse insertion equations
/// are solved once from each boundary of the affected control-point window.
/// Neither solution is trusted on its own.  Each is rebuilt as a validated
/// NURBS curve, refined again using the production insertion operation, and
/// retained only if that result equals `curve` exactly.
pub fn checked_remove_knot(curve: &NurbsCurve, u: f64) -> Vec<NurbsCurve> {
    let domain = curve.knots().domain();
    let refined_multiplicity = curve.knots().multiplicity(u);
    if !(domain.lo < u && u < domain.hi) || refined_multiplicity == 0 {
        return Vec::new();
    }

    let mut coarse_knots = curve.knots().as_slice().to_vec();
    let Some(position) = coarse_knots.iter().position(|&knot| knot == u) else {
        return Vec::new();
    };
    coarse_knots.remove(position);

    let degree = curve.degree();
    let coarse_multiplicity = refined_multiplicity - 1;
    let coarse_span = coarse_knots.partition_point(|&knot| knot <= u) - 1;
    let left_fixed = coarse_span - degree;
    let right_fixed = coarse_span - coarse_multiplicity;
    let refined = controls(curve);

    let mut candidates = Vec::with_capacity(2);

    let mut from_left = vec![refined[0]; refined.len() - 1];
    from_left[..=left_fixed].copy_from_slice(&refined[..=left_fixed]);
    from_left[right_fixed..].copy_from_slice(&refined[right_fixed + 1..]);
    for index in left_fixed + 1..right_fixed {
        let alpha = insertion_alpha(&coarse_knots, degree, u, index);
        let Some(control) = from_left[index - 1].solve_right(refined[index], alpha) else {
            return Vec::new();
        };
        from_left[index] = control;
    }
    retain_if_exact_reinsertion(curve, &coarse_knots, u, &from_left, &mut candidates);

    let mut from_right = vec![refined[0]; refined.len() - 1];
    from_right[..=left_fixed].copy_from_slice(&refined[..=left_fixed]);
    from_right[right_fixed..].copy_from_slice(&refined[right_fixed + 1..]);
    for index in (left_fixed + 1..right_fixed).rev() {
        let next = index + 1;
        let alpha = insertion_alpha(&coarse_knots, degree, u, next);
        let Some(control) = from_right[next].solve_left(refined[next], alpha) else {
            return candidates;
        };
        from_right[index] = control;
    }
    retain_if_exact_reinsertion(curve, &coarse_knots, u, &from_right, &mut candidates);

    candidates
}

/// Enumerate all representations reachable through checked exact knot
/// removals, including `curve` itself.
///
/// `state_limit` bounds exploration.  `None` means the bound was reached
/// before the exact predecessor graph was exhausted; callers must not use a
/// partial graph to claim non-equivalence.  Every returned edge has already
/// passed exact production reinsertion, so matching any returned ancestor of
/// two curves is sufficient evidence that both descend from that exact
/// representation.
pub fn checked_refinement_ancestors(
    curve: &NurbsCurve,
    state_limit: usize,
) -> Option<Vec<NurbsCurve>> {
    if state_limit == 0 {
        return None;
    }
    let mut ancestors = vec![curve.clone()];
    let mut seen = HashSet::from([CurveBits::new(curve)]);
    let mut next = 0;
    while next < ancestors.len() {
        let current = ancestors[next].clone();
        next += 1;
        for knot in distinct_interior_knots(&current) {
            for predecessor in checked_remove_knot(&current, knot) {
                if !seen.insert(CurveBits::new(&predecessor)) {
                    continue;
                }
                if ancestors.len() == state_limit {
                    return None;
                }
                ancestors.push(predecessor);
            }
        }
    }
    Some(ancestors)
}

fn canonical_bits(value: f64) -> u64 {
    if value == 0.0 {
        0.0f64.to_bits()
    } else {
        value.to_bits()
    }
}

fn insertion_alpha(knots: &[f64], degree: usize, u: f64, index: usize) -> f64 {
    (u - knots[index]) / (knots[index + degree] - knots[index])
}

fn controls(curve: &NurbsCurve) -> Vec<Control> {
    match curve.weights() {
        Some(weights) => curve
            .points()
            .iter()
            .zip(weights)
            .map(|(point, &weight)| {
                Control::Homogeneous([point.x * weight, point.y * weight, point.z * weight, weight])
            })
            .collect(),
        None => curve
            .points()
            .iter()
            .map(|point| Control::Polynomial([point.x, point.y, point.z]))
            .collect(),
    }
}

fn rebuild(curve: &NurbsCurve, knots: &[f64], controls: &[Control]) -> Option<NurbsCurve> {
    match curve.weights() {
        Some(_) => {
            let mut points = Vec::with_capacity(controls.len());
            let mut weights = Vec::with_capacity(controls.len());
            for control in controls {
                let Control::Homogeneous([x, y, z, weight]) = *control else {
                    return None;
                };
                if !weight.is_finite() || weight <= 0.0 {
                    return None;
                }
                points.push(Point3::new(x / weight, y / weight, z / weight));
                weights.push(weight);
            }
            NurbsCurve::new(curve.degree(), knots.to_vec(), points, Some(weights)).ok()
        }
        None => {
            let points = controls
                .iter()
                .map(|control| match *control {
                    Control::Polynomial([x, y, z]) => Some(Point3::new(x, y, z)),
                    Control::Homogeneous(_) => None,
                })
                .collect::<Option<Vec<_>>>()?;
            NurbsCurve::new(curve.degree(), knots.to_vec(), points, None).ok()
        }
    }
}

fn retain_if_exact_reinsertion(
    refined: &NurbsCurve,
    coarse_knots: &[f64],
    removed: f64,
    controls: &[Control],
    candidates: &mut Vec<NurbsCurve>,
) {
    let Some(candidate) = rebuild(refined, coarse_knots, controls) else {
        return;
    };
    if candidate
        .with_knot_inserted(removed, 1)
        .is_ok_and(|reinserted| reinserted == *refined)
        && !candidates.contains(&candidate)
    {
        candidates.push(candidate);
    }
}

fn distinct_interior_knots(curve: &NurbsCurve) -> Vec<f64> {
    let domain = curve.knots().domain();
    let mut knots = Vec::new();
    for &knot in curve.knots().as_slice() {
        if domain.lo < knot && knot < domain.hi && knots.last() != Some(&knot) {
            knots.push(knot);
        }
    }
    knots
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rational_quadratic() -> NurbsCurve {
        NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 2.0, 0.0),
                Point3::new(3.0, 0.0, 0.0),
            ],
            Some(vec![1.0, 1.5, 2.0]),
        )
        .unwrap()
    }

    #[test]
    fn independently_refined_rational_quadratics_recover_common_ancestor() {
        let coarse = rational_quadratic();
        let left = coarse.with_knot_inserted(0.25, 1).unwrap();
        let right = coarse.with_knot_inserted(0.75, 1).unwrap();

        let left_ancestors =
            checked_refinement_ancestors(&left, CHECKED_REFINEMENT_ANCESTOR_LIMIT).unwrap();
        let right_ancestors =
            checked_refinement_ancestors(&right, CHECKED_REFINEMENT_ANCESTOR_LIMIT).unwrap();

        assert!(left_ancestors.contains(&coarse));
        assert!(right_ancestors.contains(&coarse));
        assert!(
            left_ancestors
                .iter()
                .any(|ancestor| right_ancestors.contains(ancestor))
        );
    }

    #[test]
    fn altered_refinement_is_not_accepted() {
        let refined = rational_quadratic().with_knot_inserted(0.25, 1).unwrap();
        let mut points = refined.points().to_vec();
        points[1].y += 1.0e-12;
        let altered = NurbsCurve::new(
            refined.degree(),
            refined.knots().as_slice().to_vec(),
            points,
            refined.weights().map(<[f64]>::to_vec),
        )
        .unwrap();

        assert!(checked_remove_knot(&altered, 0.25).is_empty());
    }

    #[test]
    fn absent_and_endpoint_knots_are_not_removable() {
        let curve = rational_quadratic();
        assert!(checked_remove_knot(&curve, 0.5).is_empty());
        assert!(checked_remove_knot(&curve, 0.0).is_empty());
        assert!(checked_remove_knot(&curve, 1.0).is_empty());
    }

    #[test]
    fn repeated_rational_insertion_recovers_through_single_checked_edges() {
        let coarse = rational_quadratic();
        let refined = coarse.with_knot_inserted(0.25, 2).unwrap();
        let ancestors = checked_refinement_ancestors(&refined, 16).unwrap();
        assert!(ancestors.contains(&coarse));
    }

    #[test]
    fn exploration_limit_is_fail_closed() {
        let refined = rational_quadratic().with_knot_inserted(0.25, 1).unwrap();
        assert!(checked_refinement_ancestors(&refined, 1).is_none());
    }
}
