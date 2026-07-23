//! Bounded exact topology for cyclic second-harmonic functions.
//!
//! A second harmonic is reduced to quartics in complementary tangent and
//! cotangent half-angle charts. Both charts are isolated over a bounded
//! overlap. Deterministic ownership then turns their finite root brackets
//! into one canonical traversal of the projective circle without evaluating
//! an inverse trigonometric function.

use std::cmp::Ordering;

use super::bounded_polynomial::{
    EndpointSide, ExactPolynomial, RootBracket, RootIsolation, RootIsolationFailure,
};

pub(super) use super::bounded_polynomial::ExactScalar as ExactTrigScalar;

const CHART_BOUND: f64 = 2.0;
const OWNERSHIP_BOUND: f64 = 1.0;

/// Fixed admission reservation for one complete cyclic topology query.
///
/// The reservation covers both bounded quartic isolations, repeated-root
/// classification, and exact signs on every resulting open cell. It is
/// checked before quartic construction, so an undersized request cannot
/// publish a partial chart result. The polynomial authority retains its own
/// fixed arithmetic, Sturm-chain, subdivision, and expansion bounds.
pub(super) const CYCLIC_SECOND_HARMONIC_EXACT_WORK: u64 = 64;

/// Exact coefficients of
/// `a0 + a1 cos(u) + b1 sin(u) + a2 cos(2u) + b2 sin(2u)`.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct SecondHarmonicCoefficients {
    a0: ExactTrigScalar,
    a1: ExactTrigScalar,
    b1: ExactTrigScalar,
    a2: ExactTrigScalar,
    b2: ExactTrigScalar,
}

impl SecondHarmonicCoefficients {
    pub(super) fn new(
        a0: ExactTrigScalar,
        a1: ExactTrigScalar,
        b1: ExactTrigScalar,
        a2: ExactTrigScalar,
        b2: ExactTrigScalar,
    ) -> Self {
        Self { a0, a1, b1, a2, b2 }
    }

    #[cfg(test)]
    fn from_f64(coefficients: [f64; 5]) -> Result<Self, RootIsolationFailure> {
        let [a0, a1, b1, a2, b2] = coefficients;
        Ok(Self::new(
            ExactTrigScalar::from_f64(a0)?,
            ExactTrigScalar::from_f64(a1)?,
            ExactTrigScalar::from_f64(b1)?,
            ExactTrigScalar::from_f64(a2)?,
            ExactTrigScalar::from_f64(b2)?,
        ))
    }

    fn is_identically_zero(&self) -> bool {
        self.a0.is_zero()
            && self.a1.is_zero()
            && self.b1.is_zero()
            && self.a2.is_zero()
            && self.b2.is_zero()
    }
}

/// A strict exact sign on an open cyclic cell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum StrictSign {
    Negative,
    Positive,
}

impl StrictSign {
    fn from_expansion_sign(sign: i8) -> Result<Self, CyclicSecondHarmonicFailure> {
        match sign {
            -1 => Ok(Self::Negative),
            1 => Ok(Self::Positive),
            _ => Err(CyclicSecondHarmonicFailure::InconsistentChartTopology),
        }
    }

    #[cfg(test)]
    fn inverted(self) -> Self {
        match self {
            Self::Negative => Self::Positive,
            Self::Positive => Self::Negative,
        }
    }
}

/// The projective half-angle chart owning a canonical root bracket.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum HalfAngleChart {
    /// `t = tan(u / 2)`, owning `|t| <= 1`.
    Tangent,
    /// `s = cot(u / 2)`, owning `|s| < 1`.
    Cotangent,
}

/// One canonical closed bracket containing exactly one cyclic root.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct CyclicRootBracket {
    pub(super) chart: HalfAngleChart,
    pub(super) lo: f64,
    pub(super) hi: f64,
}

/// One distinct cyclic root and its exact repeated-root state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct CyclicRoot {
    pub(super) bracket: CyclicRootBracket,
    pub(super) repeated: bool,
}

/// Complete topology of one exact cyclic second harmonic.
#[derive(Debug, Clone, PartialEq)]
pub(super) enum CyclicSecondHarmonicTopology {
    /// All five source coefficients are exactly zero.
    IdenticallyZero,
    /// A nonzero function with canonical roots and exact cyclic cell signs.
    ///
    /// With roots, `open_cell_signs[i]` is the sign immediately after root
    /// `i` through the open cell leading to the next root, modulo the root
    /// count. A root-free cycle has one sign.
    Nonzero {
        roots: Vec<CyclicRoot>,
        open_cell_signs: Vec<StrictSign>,
    },
}

impl CyclicSecondHarmonicTopology {
    /// Return a strict whole-cycle sign exactly when no root exists.
    pub(super) fn strict_full_cycle_sign(&self) -> Option<StrictSign> {
        match self {
            Self::Nonzero {
                roots,
                open_cell_signs,
            } if roots.is_empty() => open_cell_signs.first().copied(),
            Self::IdenticallyZero | Self::Nonzero { .. } => None,
        }
    }
}

/// Stable fail-closed causes for cyclic topology construction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum CyclicSecondHarmonicFailure {
    /// The caller did not reserve the fixed complete-query allowance.
    WorkLimit { required: u64, provided: u64 },
    /// Exact scalar, Sturm, subdivision, or parameter isolation failed.
    ExactArithmetic(RootIsolationFailure),
    /// Complementary charts did not produce one coherent cyclic topology.
    InconsistentChartTopology,
}

impl From<RootIsolationFailure> for CyclicSecondHarmonicFailure {
    fn from(failure: RootIsolationFailure) -> Self {
        Self::ExactArithmetic(failure)
    }
}

#[derive(Debug, Clone)]
struct OwnedRoot {
    root: CyclicRoot,
    before: StrictSign,
    after: StrictSign,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum OwnershipPosition {
    Interior,
    Exterior,
    ExactSeam,
}

/// Classify all distinct roots and open-cell signs over one projective cycle.
pub(super) fn classify_cyclic_second_harmonic(
    coefficients: &SecondHarmonicCoefficients,
    work_limit: u64,
) -> Result<CyclicSecondHarmonicTopology, CyclicSecondHarmonicFailure> {
    if work_limit < CYCLIC_SECOND_HARMONIC_EXACT_WORK {
        return Err(CyclicSecondHarmonicFailure::WorkLimit {
            required: CYCLIC_SECOND_HARMONIC_EXACT_WORK,
            provided: work_limit,
        });
    }
    if coefficients.is_identically_zero() {
        return Ok(CyclicSecondHarmonicTopology::IdenticallyZero);
    }
    if let Ok(Some(sign)) = constant_dominance_sign(coefficients) {
        return Ok(CyclicSecondHarmonicTopology::Nonzero {
            roots: Vec::new(),
            open_cell_signs: vec![sign],
        });
    }

    let tangent_coefficients = tangent_half_angle_coefficients(coefficients)?;
    let cotangent_coefficients = [
        tangent_coefficients[4].clone(),
        tangent_coefficients[3].clone(),
        tangent_coefficients[2].clone(),
        tangent_coefficients[1].clone(),
        tangent_coefficients[0].clone(),
    ];
    let tangent = ExactPolynomial::new(Vec::from(tangent_coefficients))?;
    let cotangent = ExactPolynomial::new(Vec::from(cotangent_coefficients))?;

    let tangent_roots = complete_roots(&tangent)?;
    let cotangent_roots = complete_roots(&cotangent)?;
    let mut roots = Vec::with_capacity(4);
    for bracket in owned_chart_roots(tangent_roots, HalfAngleChart::Tangent)? {
        roots.push(owned_root(&tangent, HalfAngleChart::Tangent, bracket)?);
    }
    for bracket in owned_chart_roots(cotangent_roots, HalfAngleChart::Cotangent)? {
        roots.push(owned_root(&cotangent, HalfAngleChart::Cotangent, bracket)?);
    }
    if roots.len() > 4 {
        return Err(CyclicSecondHarmonicFailure::InconsistentChartTopology);
    }
    roots.sort_by(cyclic_root_order);

    let open_cell_signs = exact_open_cell_signs(&roots, &tangent)?;
    Ok(CyclicSecondHarmonicTopology::Nonzero {
        roots: roots.into_iter().map(|owned| owned.root).collect(),
        open_cell_signs,
    })
}

fn constant_dominance_sign(
    coefficients: &SecondHarmonicCoefficients,
) -> Result<Option<StrictSign>, RootIsolationFailure> {
    let mut harmonic_bound = ExactTrigScalar::zero();
    for coefficient in [
        &coefficients.a1,
        &coefficients.b1,
        &coefficients.a2,
        &coefficients.b2,
    ] {
        let magnitude = if coefficient.sign() < 0 {
            coefficient.negate()?
        } else {
            coefficient.clone()
        };
        harmonic_bound = harmonic_bound.add(&magnitude)?;
    }
    let constant_magnitude = if coefficients.a0.sign() < 0 {
        coefficients.a0.negate()?
    } else {
        coefficients.a0.clone()
    };
    if constant_magnitude.sub(&harmonic_bound)?.sign() <= 0 {
        return Ok(None);
    }
    Ok(Some(if coefficients.a0.sign() < 0 {
        StrictSign::Negative
    } else {
        StrictSign::Positive
    }))
}

fn tangent_half_angle_coefficients(
    coefficients: &SecondHarmonicCoefficients,
) -> Result<[ExactTrigScalar; 5], RootIsolationFailure> {
    let q0 = coefficients
        .a0
        .add(&coefficients.a1)?
        .add(&coefficients.a2)?;
    let q1 = coefficients
        .b1
        .scale(2.0)?
        .add(&coefficients.b2.scale(4.0)?)?;
    let q2 = coefficients
        .a0
        .scale(2.0)?
        .sub(&coefficients.a2.scale(6.0)?)?;
    let q3 = coefficients
        .b1
        .scale(2.0)?
        .sub(&coefficients.b2.scale(4.0)?)?;
    let q4 = coefficients
        .a0
        .sub(&coefficients.a1)?
        .add(&coefficients.a2)?;
    Ok([q0, q1, q2, q3, q4])
}

fn complete_roots(
    polynomial: &ExactPolynomial,
) -> Result<Vec<RootBracket>, CyclicSecondHarmonicFailure> {
    match polynomial.isolate(-CHART_BOUND, CHART_BOUND) {
        RootIsolation::Complete(roots) => Ok(roots),
        RootIsolation::Ambiguous(failure) => Err(failure.into()),
    }
}

fn owned_chart_roots(
    roots: Vec<RootBracket>,
    chart: HalfAngleChart,
) -> Result<Vec<RootBracket>, CyclicSecondHarmonicFailure> {
    let mut owned = Vec::with_capacity(roots.len());
    for root in roots {
        let retain = match (chart, ownership_position(root)?) {
            (
                HalfAngleChart::Tangent,
                OwnershipPosition::Interior | OwnershipPosition::ExactSeam,
            )
            | (HalfAngleChart::Cotangent, OwnershipPosition::Interior) => true,
            (HalfAngleChart::Tangent, OwnershipPosition::Exterior)
            | (
                HalfAngleChart::Cotangent,
                OwnershipPosition::Exterior | OwnershipPosition::ExactSeam,
            ) => false,
        };
        if retain {
            owned.push(root);
        }
    }
    Ok(owned)
}

fn ownership_position(root: RootBracket) -> Result<OwnershipPosition, CyclicSecondHarmonicFailure> {
    if root.lo == root.hi && (root.lo == -OWNERSHIP_BOUND || root.lo == OWNERSHIP_BOUND) {
        return Ok(OwnershipPosition::ExactSeam);
    }
    if (root.lo < -OWNERSHIP_BOUND && root.hi > -OWNERSHIP_BOUND)
        || (root.lo < OWNERSHIP_BOUND && root.hi > OWNERSHIP_BOUND)
    {
        return Err(CyclicSecondHarmonicFailure::InconsistentChartTopology);
    }
    if root.lo >= -OWNERSHIP_BOUND && root.hi <= OWNERSHIP_BOUND {
        Ok(OwnershipPosition::Interior)
    } else {
        Ok(OwnershipPosition::Exterior)
    }
}

fn owned_root(
    polynomial: &ExactPolynomial,
    chart: HalfAngleChart,
    bracket: RootBracket,
) -> Result<OwnedRoot, CyclicSecondHarmonicFailure> {
    let (numeric_left, numeric_right) = numeric_side_signs(polynomial, bracket)?;
    let (before, after) = match chart {
        HalfAngleChart::Tangent => (numeric_left, numeric_right),
        HalfAngleChart::Cotangent => (numeric_right, numeric_left),
    };
    Ok(OwnedRoot {
        root: CyclicRoot {
            bracket: CyclicRootBracket {
                chart,
                lo: bracket.lo,
                hi: bracket.hi,
            },
            repeated: polynomial.root_is_repeated(bracket)?,
        },
        before,
        after,
    })
}

fn numeric_side_signs(
    polynomial: &ExactPolynomial,
    bracket: RootBracket,
) -> Result<(StrictSign, StrictSign), CyclicSecondHarmonicFailure> {
    let (left, right) = if bracket.lo == bracket.hi {
        (
            polynomial.side_sign(bracket.lo, EndpointSide::Left)?,
            polynomial.side_sign(bracket.hi, EndpointSide::Right)?,
        )
    } else {
        (
            polynomial.evaluate(bracket.lo)?.sign(),
            polynomial.evaluate(bracket.hi)?.sign(),
        )
    };
    Ok((
        StrictSign::from_expansion_sign(left)?,
        StrictSign::from_expansion_sign(right)?,
    ))
}

fn cyclic_root_order(lhs: &OwnedRoot, rhs: &OwnedRoot) -> Ordering {
    let lhs_sector = cyclic_sector(lhs.root.bracket);
    let rhs_sector = cyclic_sector(rhs.root.bracket);
    lhs_sector.cmp(&rhs_sector).then_with(|| {
        let lhs_parameter = bracket_representative(lhs.root.bracket);
        let rhs_parameter = bracket_representative(rhs.root.bracket);
        match lhs_sector {
            0 | 3 => lhs_parameter.total_cmp(&rhs_parameter),
            1 | 2 => rhs_parameter.total_cmp(&lhs_parameter),
            _ => Ordering::Equal,
        }
    })
}

fn cyclic_sector(bracket: CyclicRootBracket) -> u8 {
    match bracket.chart {
        HalfAngleChart::Tangent if bracket.lo >= 0.0 => 0,
        HalfAngleChart::Cotangent if bracket.lo >= 0.0 => 1,
        HalfAngleChart::Cotangent => 2,
        HalfAngleChart::Tangent => 3,
    }
}

fn bracket_representative(bracket: CyclicRootBracket) -> f64 {
    if bracket.lo == bracket.hi {
        bracket.lo
    } else {
        bracket.lo / 2.0 + bracket.hi / 2.0
    }
}

fn exact_open_cell_signs(
    roots: &[OwnedRoot],
    tangent: &ExactPolynomial,
) -> Result<Vec<StrictSign>, CyclicSecondHarmonicFailure> {
    if roots.is_empty() {
        return Ok(vec![StrictSign::from_expansion_sign(
            tangent.evaluate(0.0)?.sign(),
        )?]);
    }

    for (root, next) in roots
        .iter()
        .zip(roots.iter().cycle().skip(1))
        .take(roots.len())
    {
        if root.after != next.before {
            return Err(CyclicSecondHarmonicFailure::InconsistentChartTopology);
        }
    }
    Ok(roots.iter().map(|root| root.after).collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn classify(coefficients: [f64; 5]) -> CyclicSecondHarmonicTopology {
        classify_cyclic_second_harmonic(
            &SecondHarmonicCoefficients::from_f64(coefficients).unwrap(),
            CYCLIC_SECOND_HARMONIC_EXACT_WORK,
        )
        .unwrap()
    }

    fn nonzero(topology: &CyclicSecondHarmonicTopology) -> (&[CyclicRoot], &[StrictSign]) {
        match topology {
            CyclicSecondHarmonicTopology::Nonzero {
                roots,
                open_cell_signs,
            } => (roots, open_cell_signs),
            CyclicSecondHarmonicTopology::IdenticallyZero => {
                panic!("expected a nonzero harmonic")
            }
        }
    }

    #[test]
    fn constants_have_one_exact_whole_cycle_sign() {
        let positive = classify([3.0, 0.0, 0.0, 0.0, 0.0]);
        let negative = classify([-3.0, 0.0, 0.0, 0.0, 0.0]);
        assert_eq!(
            positive.strict_full_cycle_sign(),
            Some(StrictSign::Positive)
        );
        assert_eq!(
            negative.strict_full_cycle_sign(),
            Some(StrictSign::Negative)
        );
        assert!(nonzero(&positive).0.is_empty());
        assert!(nonzero(&negative).0.is_empty());
    }

    #[test]
    fn cosine_second_harmonic_has_four_ordered_simple_roots() {
        let topology = classify([0.0, 0.0, 0.0, 1.0, 0.0]);
        let (roots, signs) = nonzero(&topology);
        assert_eq!(roots.len(), 4);
        assert!(roots.iter().all(|root| !root.repeated));
        assert_eq!(
            roots
                .iter()
                .map(|root| root.bracket.chart)
                .collect::<Vec<_>>(),
            vec![
                HalfAngleChart::Tangent,
                HalfAngleChart::Cotangent,
                HalfAngleChart::Cotangent,
                HalfAngleChart::Tangent,
            ]
        );
        assert_eq!(
            signs,
            &[
                StrictSign::Negative,
                StrictSign::Positive,
                StrictSign::Negative,
                StrictSign::Positive,
            ]
        );
        assert_eq!(topology.strict_full_cycle_sign(), None);
    }

    #[test]
    fn one_plus_cosine_second_harmonic_has_two_repeated_seam_roots() {
        let topology = classify([1.0, 0.0, 0.0, 1.0, 0.0]);
        let (roots, signs) = nonzero(&topology);
        assert_eq!(
            roots,
            &[
                CyclicRoot {
                    bracket: CyclicRootBracket {
                        chart: HalfAngleChart::Tangent,
                        lo: 1.0,
                        hi: 1.0,
                    },
                    repeated: true,
                },
                CyclicRoot {
                    bracket: CyclicRootBracket {
                        chart: HalfAngleChart::Tangent,
                        lo: -1.0,
                        hi: -1.0,
                    },
                    repeated: true,
                },
            ]
        );
        assert_eq!(signs, &[StrictSign::Positive, StrictSign::Positive]);
    }

    #[test]
    fn skew_cylinder_repeated_seam_discriminant_is_not_a_chart_failure() {
        // 4 * (2^2 - (sin(u) - 3)^2)
        let topology = classify([-22.0, 0.0, 24.0, 2.0, 0.0]);
        let (roots, signs) = nonzero(&topology);
        assert_eq!(
            roots,
            &[CyclicRoot {
                bracket: CyclicRootBracket {
                    chart: HalfAngleChart::Tangent,
                    lo: 1.0,
                    hi: 1.0,
                },
                repeated: true,
            }]
        );
        assert_eq!(signs, &[StrictSign::Negative]);
    }

    #[test]
    fn chart_overlap_ownership_and_projective_infinity_are_canonical() {
        // Its tangent-chart quartic is
        // (t - 1/2)(t - 2) = t^2 - (5/2)t + 1.
        // The degree-two projective root at infinity is u = pi.
        let topology = classify([0.5, 0.5, -0.625, 0.0, -0.3125]);
        let (roots, signs) = nonzero(&topology);
        assert_eq!(
            roots,
            &[
                CyclicRoot {
                    bracket: CyclicRootBracket {
                        chart: HalfAngleChart::Tangent,
                        lo: 0.5,
                        hi: 0.5,
                    },
                    repeated: false,
                },
                CyclicRoot {
                    bracket: CyclicRootBracket {
                        chart: HalfAngleChart::Cotangent,
                        lo: 0.5,
                        hi: 0.5,
                    },
                    repeated: false,
                },
                CyclicRoot {
                    bracket: CyclicRootBracket {
                        chart: HalfAngleChart::Cotangent,
                        lo: 0.0,
                        hi: 0.0,
                    },
                    repeated: true,
                },
            ]
        );
        assert_eq!(
            signs,
            &[
                StrictSign::Negative,
                StrictSign::Positive,
                StrictSign::Positive,
            ]
        );

        assert_eq!(
            ownership_position(RootBracket {
                lo: 1.0_f64.next_down(),
                hi: 1.0_f64.next_up(),
            }),
            Err(CyclicSecondHarmonicFailure::InconsistentChartTopology)
        );
    }

    #[test]
    fn exact_zero_is_not_a_root_free_strict_sign() {
        let topology = classify([0.0; 5]);
        assert_eq!(topology, CyclicSecondHarmonicTopology::IdenticallyZero);
        assert_eq!(topology.strict_full_cycle_sign(), None);
    }

    #[test]
    fn one_ulp_neighbors_of_repeated_contact_have_distinct_topology() {
        let below = classify([1.0_f64.next_down(), 0.0, 0.0, 1.0, 0.0]);
        let exact = classify([1.0, 0.0, 0.0, 1.0, 0.0]);
        let above = classify([1.0_f64.next_up(), 0.0, 0.0, 1.0, 0.0]);

        assert_eq!(nonzero(&below).0.len(), 4);
        assert!(nonzero(&below).0.iter().all(|root| !root.repeated));
        assert_eq!(nonzero(&exact).0.len(), 2);
        assert!(nonzero(&exact).0.iter().all(|root| root.repeated));
        assert_eq!(above.strict_full_cycle_sign(), Some(StrictSign::Positive));
    }

    #[test]
    fn unsafe_source_and_derived_extremes_fail_closed() {
        assert_eq!(
            SecondHarmonicCoefficients::from_f64([f64::from_bits(1), 0.0, 0.0, 0.0, 0.0,]),
            Err(RootIsolationFailure::UnsafeArithmeticEnvelope)
        );

        let huge = ExactTrigScalar::from_f64(2.0_f64.powi(400)).unwrap();
        let zero = ExactTrigScalar::zero();
        let coefficients =
            SecondHarmonicCoefficients::new(zero.clone(), zero.clone(), zero.clone(), huge, zero);
        assert_eq!(
            classify_cyclic_second_harmonic(&coefficients, CYCLIC_SECOND_HARMONIC_EXACT_WORK,),
            Err(CyclicSecondHarmonicFailure::ExactArithmetic(
                RootIsolationFailure::UnsafeArithmeticEnvelope,
            ))
        );
    }

    #[test]
    fn output_is_deterministic_and_common_scale_canonical() {
        let source = [0.5, 0.5, -0.625, 0.0, -0.3125];
        let baseline = classify(source);
        assert_eq!(classify(source), baseline);
        assert_eq!(
            classify(source.map(|coefficient| coefficient * 8.0)),
            baseline
        );

        let inverted = classify(source.map(|coefficient| coefficient * -4.0));
        let (baseline_roots, baseline_signs) = nonzero(&baseline);
        let (inverted_roots, inverted_signs) = nonzero(&inverted);
        assert_eq!(inverted_roots, baseline_roots);
        assert_eq!(
            inverted_signs,
            baseline_signs
                .iter()
                .copied()
                .map(StrictSign::inverted)
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn fixed_work_reservation_has_an_exact_n_and_n_minus_one_frontier() {
        let coefficients = SecondHarmonicCoefficients::from_f64([0.0, 0.0, 0.0, 1.0, 0.0]).unwrap();
        assert!(
            classify_cyclic_second_harmonic(&coefficients, CYCLIC_SECOND_HARMONIC_EXACT_WORK,)
                .is_ok()
        );
        assert_eq!(
            classify_cyclic_second_harmonic(&coefficients, CYCLIC_SECOND_HARMONIC_EXACT_WORK - 1,),
            Err(CyclicSecondHarmonicFailure::WorkLimit {
                required: CYCLIC_SECOND_HARMONIC_EXACT_WORK,
                provided: CYCLIC_SECOND_HARMONIC_EXACT_WORK - 1,
            })
        );
        let zero = SecondHarmonicCoefficients::from_f64([0.0; 5]).unwrap();
        assert!(matches!(
            classify_cyclic_second_harmonic(&zero, CYCLIC_SECOND_HARMONIC_EXACT_WORK - 1,),
            Err(CyclicSecondHarmonicFailure::WorkLimit { .. })
        ));
    }
}
