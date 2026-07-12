//! Bounded exact point evaluation for spatial NURBS existence witnesses.

use super::NurbsCurve;
use kcore::expansion;

const MAX_DEGREE: usize = 8;
const MAX_EXPANSION_COMPONENTS: usize = 4_096;
const MAX_PRODUCT_TERMS: usize = 1_024;
const SAFE_COMPONENT_MIN: f64 = f64::from_bits(((1023 - 500) as u64) << 52);
const SAFE_COMPONENT_MAX: f64 = f64::from_bits(((1023 + 500) as u64) << 52);
const SAFE_PRODUCT_MIN: f64 = f64::from_bits(((1023 - 400) as u64) << 52);
const SAFE_PRODUCT_MAX: f64 = f64::from_bits(((1023 + 400) as u64) << 52);

/// Exact equality witness at two in-domain curve parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ExactSpatialSampleWitness {
    first_parameter: f64,
    second_parameter: f64,
}

impl ExactSpatialSampleWitness {
    /// Exact parameter on the first curve.
    pub(crate) const fn first_parameter(self) -> f64 {
        self.first_parameter
    }

    /// Exact parameter on the second curve.
    pub(crate) const fn second_parameter(self) -> f64 {
        self.second_parameter
    }
}

/// Prove that two in-domain NURBS samples are exactly the same 3D point.
///
/// Every finite `f64` input is an exact dyadic rational. De Boor evaluation is
/// performed with rational values whose numerators and denominators are exact
/// floating-point expansions, so no rounded point comparison participates in
/// the proof. Polynomial and positive-weight rational curves are supported.
///
/// Arithmetic growth, degree, and exponent corridors are deliberately
/// bounded. Any unsupported input, non-exactly-equal point, or exhausted bound
/// returns `None`; callers must keep the candidate region in all such cases.
pub(crate) fn certify_exact_spatial_sample(
    first: &NurbsCurve,
    first_parameter: f64,
    second: &NurbsCurve,
    second_parameter: f64,
) -> Option<ExactSpatialSampleWitness> {
    let first_domain = first.knots().domain();
    let second_domain = second.knots().domain();
    if !(first_parameter.is_finite()
        && second_parameter.is_finite()
        && first_domain.contains(first_parameter)
        && second_domain.contains(second_parameter))
    {
        return None;
    }
    let first_point = exact_homogeneous_sample(first, first_parameter)?;
    let second_point = exact_homogeneous_sample(second, second_parameter)?;
    if (0..3).all(|axis| {
        exact_fraction_products_equal(
            &first_point[axis],
            &second_point[3],
            &second_point[axis],
            &first_point[3],
        ) == Some(true)
    }) {
        Some(ExactSpatialSampleWitness {
            first_parameter,
            second_parameter,
        })
    } else {
        None
    }
}

#[derive(Clone)]
struct ExactFraction {
    numerator: Vec<f64>,
    denominator: Vec<f64>,
}

impl ExactFraction {
    fn from_f64(value: f64) -> Option<Self> {
        safe_raw(value).then(|| Self {
            numerator: vec![value],
            denominator: vec![1.0],
        })
    }

    fn difference(first: f64, second: f64) -> Option<Self> {
        if !safe_raw(first) || !safe_raw(second) {
            return None;
        }
        let (rounded, residue) = expansion::two_diff(first, second);
        let numerator = expansion::from_two(rounded, residue);
        safe_expansion(&numerator).then(|| Self {
            numerator,
            denominator: vec![1.0],
        })
    }

    fn add(&self, other: &Self) -> Option<Self> {
        let left = exact_mul(&self.numerator, &other.denominator)?;
        let right = exact_mul(&other.numerator, &self.denominator)?;
        Self::new(
            exact_sum(&left, &right)?,
            exact_mul(&self.denominator, &other.denominator)?,
        )
    }

    fn sub(&self, other: &Self) -> Option<Self> {
        let left = exact_mul(&self.numerator, &other.denominator)?;
        let right = expansion::negate(&exact_mul(&other.numerator, &self.denominator)?);
        Self::new(
            exact_sum(&left, &right)?,
            exact_mul(&self.denominator, &other.denominator)?,
        )
    }

    fn mul(&self, other: &Self) -> Option<Self> {
        Self::new(
            exact_mul(&self.numerator, &other.numerator)?,
            exact_mul(&self.denominator, &other.denominator)?,
        )
    }

    fn checked_div(&self, other: &Self) -> Option<Self> {
        if expansion::sign(&other.numerator) == 0 {
            return None;
        }
        Self::new(
            exact_mul(&self.numerator, &other.denominator)?,
            exact_mul(&self.denominator, &other.numerator)?,
        )
    }

    fn new(numerator: Vec<f64>, denominator: Vec<f64>) -> Option<Self> {
        (safe_expansion(&numerator)
            && safe_expansion(&denominator)
            && expansion::sign(&denominator) != 0)
            .then_some(Self {
                numerator,
                denominator,
            })
    }
}

fn exact_homogeneous_sample(curve: &NurbsCurve, parameter: f64) -> Option<[ExactFraction; 4]> {
    let degree = curve.degree();
    if degree > MAX_DEGREE {
        return None;
    }
    let knots = curve.knots();
    let span = knots.find_span(parameter);
    let weights = curve.weights();
    let mut work = Vec::with_capacity(degree + 1);
    for index in span - degree..=span {
        let weight = ExactFraction::from_f64(weights.map_or(1.0, |values| values[index]))?;
        work.push([
            ExactFraction::from_f64(curve.points()[index].x)?.mul(&weight)?,
            ExactFraction::from_f64(curve.points()[index].y)?.mul(&weight)?,
            ExactFraction::from_f64(curve.points()[index].z)?.mul(&weight)?,
            weight,
        ]);
    }

    let parameter = ExactFraction::from_f64(parameter)?;
    let one = ExactFraction::from_f64(1.0)?;
    for level in 1..=degree {
        for local in (level..=degree).rev() {
            let control_index = span - degree + local;
            let numerator =
                parameter.sub(&ExactFraction::from_f64(knots.as_slice()[control_index])?)?;
            let denominator = ExactFraction::difference(
                knots.as_slice()[control_index + degree - level + 1],
                knots.as_slice()[control_index],
            )?;
            let alpha = numerator.checked_div(&denominator)?;
            let one_minus_alpha = one.sub(&alpha)?;
            let left = work[local - 1].clone();
            let right = work[local].clone();
            work[local] = [
                blend_component(&left[0], &right[0], &one_minus_alpha, &alpha)?,
                blend_component(&left[1], &right[1], &one_minus_alpha, &alpha)?,
                blend_component(&left[2], &right[2], &one_minus_alpha, &alpha)?,
                blend_component(&left[3], &right[3], &one_minus_alpha, &alpha)?,
            ];
            if work[local].iter().any(|value| {
                !safe_expansion(&value.numerator) || !safe_expansion(&value.denominator)
            }) {
                return None;
            }
        }
    }
    work.pop()
}

fn blend_component(
    left: &ExactFraction,
    right: &ExactFraction,
    one_minus_alpha: &ExactFraction,
    alpha: &ExactFraction,
) -> Option<ExactFraction> {
    one_minus_alpha.mul(left)?.add(&alpha.mul(right)?)
}

fn exact_fraction_products_equal(
    first: &ExactFraction,
    second: &ExactFraction,
    third: &ExactFraction,
    fourth: &ExactFraction,
) -> Option<bool> {
    let left = first.mul(second)?;
    let right = third.mul(fourth)?;
    let left_cross = exact_mul(&left.numerator, &right.denominator)?;
    let right_cross = exact_mul(&right.numerator, &left.denominator)?;
    let difference = exact_sum(&left_cross, &expansion::negate(&right_cross))?;
    Some(expansion::sign(&difference) == 0)
}

fn exact_sum(first: &[f64], second: &[f64]) -> Option<Vec<f64>> {
    if !safe_expansion(first)
        || !safe_expansion(second)
        || first.len().checked_add(second.len())? > MAX_EXPANSION_COMPONENTS
    {
        return None;
    }
    let result = expansion::sum(first, second);
    safe_expansion(&result).then_some(result)
}

fn exact_mul(first: &[f64], second: &[f64]) -> Option<Vec<f64>> {
    if !safe_expansion(first)
        || !safe_expansion(second)
        || first.len().checked_mul(second.len())? > MAX_PRODUCT_TERMS
    {
        return None;
    }
    for &a in first {
        for &b in second {
            if a == 0.0 || b == 0.0 {
                continue;
            }
            let product = a.abs() * b.abs();
            if !(SAFE_PRODUCT_MIN..=SAFE_PRODUCT_MAX).contains(&product) {
                return None;
            }
        }
    }
    let result = expansion::mul(first, second);
    (result.len() <= MAX_EXPANSION_COMPONENTS && safe_expansion(&result)).then_some(result)
}

fn safe_raw(value: f64) -> bool {
    value == 0.0
        || (value.is_finite()
            && SAFE_COMPONENT_MIN <= value.abs()
            && value.abs() <= SAFE_COMPONENT_MAX)
}

fn safe_expansion(values: &[f64]) -> bool {
    !values.is_empty()
        && values.len() <= MAX_EXPANSION_COMPONENTS
        && values.iter().copied().all(safe_raw)
}
