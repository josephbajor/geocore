//! Bounded exact isolation for low-degree real polynomials.
//!
//! The coefficients are exact floating-point expansions, not rounded
//! approximations of source geometry.  A signed pseudo-remainder Sturm chain
//! counts distinct roots without introducing inexact coefficient divisions.
//! Every arithmetic and subdivision path has a fixed resource bound: callers
//! get an explicit ambiguous result instead of an incomplete root list.

use kcore::expansion;

const MAX_DEGREE: usize = 4;
const MAX_EXPANSION_COMPONENTS: usize = 1_024;
const MAX_EXPANSION_PRODUCT_WORK: usize = 65_536;
const MAX_STURM_POLYNOMIALS: usize = MAX_DEGREE + 1;
const MAX_ISOLATION_CELLS: usize = 4_096;
const MAX_ISOLATION_DEPTH: usize = 192;

// `expansion::scale` and `expansion::mul` use Dekker splitting internally.
// Bound component magnitudes to prevent split overflow and require products
// above 2^-800, leaving roughly 100 binary exponents for an exact-product
// residue before the normal-number floor. Every produced component is checked
// again, so cancellation or accumulation outside that envelope fails closed.
const EXACT_COMPONENT_MAX: f64 = f64::from_bits(((1023 + 500) as u64) << 52);
const EXACT_PRODUCT_MIN: f64 = f64::from_bits(((1023 - 800) as u64) << 52);
const EXACT_PRODUCT_MAX: f64 = f64::from_bits(((1023 + 400) as u64) << 52);

/// A stable reason why exact construction or bounded isolation could not
/// finish.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RootIsolationFailure {
    /// A source coefficient, scale, evaluation point, or range was non-finite.
    NonFiniteInput,
    /// The supplied polynomial exceeded the supported quartic degree.
    DegreeLimit,
    /// The identically zero polynomial has no isolatable finite root set.
    ZeroPolynomial,
    /// Exact expansion arithmetic left its safe normal-number envelope.
    UnsafeArithmeticEnvelope,
    /// A canonical expansion exceeded the fixed component budget.
    ExpansionLimit,
    /// The requested isolation range was reversed.
    InvalidRange,
    /// The signed pseudo-remainder sequence violated its degree/work bound.
    SturmChainLimit,
    /// Deterministic subdivision exhausted its cell or depth budget.
    IsolationLimit,
    /// No representable strict midpoint remained for an unresolved root cell.
    ParameterResolution,
}

/// Exact dyadic value represented by a canonical floating-point expansion.
///
/// Construction and arithmetic are checked because the underlying expansion
/// primitives intentionally assume that intermediate `f64` operations remain
/// finite.
#[derive(Debug, Clone, PartialEq)]
pub struct ExactScalar {
    components: Vec<f64>,
}

impl ExactScalar {
    pub fn from_f64(value: f64) -> Result<Self, RootIsolationFailure> {
        if !value.is_finite() {
            return Err(RootIsolationFailure::NonFiniteInput);
        }
        checked_components(vec![value])
    }

    pub fn zero() -> Self {
        Self {
            components: vec![0.0],
        }
    }

    pub fn add(&self, rhs: &Self) -> Result<Self, RootIsolationFailure> {
        checked_sum(&self.components, &rhs.components)
    }

    pub fn sub(&self, rhs: &Self) -> Result<Self, RootIsolationFailure> {
        let negated = checked_negate(&rhs.components)?;
        checked_sum(&self.components, &negated)
    }

    pub fn mul(&self, rhs: &Self) -> Result<Self, RootIsolationFailure> {
        if self.is_zero() || rhs.is_zero() {
            return Ok(Self::zero());
        }
        checked_mul(&self.components, &rhs.components)
    }

    pub fn scale(&self, factor: f64) -> Result<Self, RootIsolationFailure> {
        if !factor.is_finite() {
            return Err(RootIsolationFailure::NonFiniteInput);
        }
        if self.is_zero() || factor == 0.0 {
            return Ok(Self::zero());
        }
        if self.components.len().saturating_mul(2) > MAX_EXPANSION_PRODUCT_WORK {
            return Err(RootIsolationFailure::ExpansionLimit);
        }
        for &component in &self.components {
            checked_product_pair(component, factor)?;
        }
        checked_components(expansion::scale(&self.components, factor))
    }

    pub fn negate(&self) -> Result<Self, RootIsolationFailure> {
        checked_components(checked_negate(&self.components)?)
    }

    pub fn sign(&self) -> i8 {
        expansion::sign(&self.components)
    }

    pub fn is_zero(&self) -> bool {
        self.sign() == 0
    }
}

/// Exact polynomial with coefficients in ascending power order.
#[derive(Debug, Clone, PartialEq)]
pub struct ExactPolynomial {
    coefficients: Vec<ExactScalar>,
}

impl ExactPolynomial {
    pub fn new(coefficients: Vec<ExactScalar>) -> Result<Self, RootIsolationFailure> {
        let polynomial = Self::from_coefficients_allow_zero(coefficients)?;
        if polynomial.is_zero() {
            Err(RootIsolationFailure::ZeroPolynomial)
        } else {
            Ok(polynomial)
        }
    }

    pub fn derivative(&self) -> Result<Self, RootIsolationFailure> {
        if self.degree() == 0 {
            return Ok(Self::zero());
        }
        let mut coefficients = Vec::with_capacity(self.coefficients.len() - 1);
        for (power, coefficient) in self.coefficients.iter().enumerate().skip(1) {
            coefficients.push(coefficient.scale(power as f64)?);
        }
        Self::from_coefficients_allow_zero(coefficients)
    }

    /// Isolate every distinct real root in the closed finite range.
    ///
    /// Repeated roots appear once.  Point brackets are produced for exactly
    /// representable roots; all other brackets are refined to adjacent `f64`
    /// values (or until the deterministic depth/work bound is exhausted).
    pub fn isolate(&self, lo: f64, hi: f64) -> RootIsolation {
        match self.try_isolate(lo, hi) {
            Ok(roots) => RootIsolation::Complete(roots),
            Err(
                RootIsolationFailure::UnsafeArithmeticEnvelope
                | RootIsolationFailure::ExpansionLimit,
            ) => match self.try_isolate_bernstein(lo, hi) {
                Ok(roots) => RootIsolation::Complete(roots),
                Err(failure) => RootIsolation::Ambiguous(failure),
            },
            Err(failure) => RootIsolation::Ambiguous(failure),
        }
    }

    /// Isolate every distinct root whose multiplicity is at least two.
    ///
    /// The last nonzero polynomial in the `P, P'` pseudo-remainder chain is a
    /// positive scaling of `gcd(P, P')`. A constant gcd proves that all roots
    /// are simple; otherwise its bounded roots are exactly the repeated roots
    /// of `P`.
    pub fn isolate_repeated_roots(&self, lo: f64, hi: f64) -> RootIsolation {
        if self.is_zero() {
            return RootIsolation::Ambiguous(RootIsolationFailure::ZeroPolynomial);
        }
        let repeated = self
            .sturm_chain()
            .and_then(|chain| {
                chain
                    .last()
                    .cloned()
                    .ok_or(RootIsolationFailure::SturmChainLimit)
            })
            .and_then(|gcd| {
                if gcd.degree() == 0 {
                    Ok(Vec::new())
                } else {
                    gcd.try_isolate(lo, hi)
                }
            });
        match repeated {
            Ok(roots) => RootIsolation::Complete(roots),
            Err(failure) => RootIsolation::Ambiguous(failure),
        }
    }

    /// Prove whether the one root owned by `root` has multiplicity at least
    /// two in this polynomial.
    ///
    /// `RootBracket` endpoints are either the exact root itself or certified
    /// non-roots, so isolating `gcd(P, P')` over the same closed bracket cannot
    /// accidentally select a neighboring root at a shared numeric boundary.
    pub fn root_is_repeated(&self, root: RootBracket) -> Result<bool, RootIsolationFailure> {
        if self.is_zero() {
            return Err(RootIsolationFailure::ZeroPolynomial);
        }
        let derivative = self.derivative()?;
        if derivative
            .strict_sign_on_interval(root.lo, root.hi)?
            .is_some()
        {
            return Ok(false);
        }
        let chain = self.sturm_chain()?;
        let gcd = chain.last().ok_or(RootIsolationFailure::SturmChainLimit)?;
        if gcd.degree() == 0 {
            return Ok(false);
        }
        match gcd.try_isolate(root.lo, root.hi)?.as_slice() {
            [] => Ok(false),
            [_] => Ok(true),
            _ => Err(RootIsolationFailure::SturmChainLimit),
        }
    }

    fn from_coefficients_allow_zero(
        mut coefficients: Vec<ExactScalar>,
    ) -> Result<Self, RootIsolationFailure> {
        if coefficients.is_empty() {
            coefficients.push(ExactScalar::zero());
        }
        while coefficients.len() > 1 && coefficients.last().is_some_and(ExactScalar::is_zero) {
            coefficients.pop();
        }
        if coefficients.len() - 1 > MAX_DEGREE {
            return Err(RootIsolationFailure::DegreeLimit);
        }
        Ok(Self { coefficients })
    }

    fn zero() -> Self {
        Self {
            coefficients: vec![ExactScalar::zero()],
        }
    }

    fn is_zero(&self) -> bool {
        self.coefficients.len() == 1 && self.coefficients[0].is_zero()
    }

    fn degree(&self) -> usize {
        self.coefficients.len() - 1
    }

    fn leading_coefficient(&self) -> &ExactScalar {
        self.coefficients
            .last()
            .expect("a polynomial always has a coefficient")
    }

    pub fn evaluate(&self, parameter: f64) -> Result<ExactScalar, RootIsolationFailure> {
        if !parameter.is_finite() {
            return Err(RootIsolationFailure::NonFiniteInput);
        }
        let mut value = self.leading_coefficient().clone();
        for coefficient in self.coefficients.iter().rev().skip(1) {
            value = value.scale(parameter)?.add(coefficient)?;
        }
        Ok(value)
    }

    pub fn side_sign(
        &self,
        parameter: f64,
        side: EndpointSide,
    ) -> Result<i8, RootIsolationFailure> {
        let mut derivative = self.clone();
        for order in 0..=self.degree() {
            let sign = derivative.evaluate(parameter)?.sign();
            if sign != 0 {
                return Ok(if side == EndpointSide::Left && order % 2 == 1 {
                    -sign
                } else {
                    sign
                });
            }
            derivative = derivative.derivative()?;
        }
        Ok(0)
    }

    fn sturm_chain(&self) -> Result<Vec<Self>, RootIsolationFailure> {
        let mut chain = vec![self.clone()];
        let derivative = self.derivative()?;
        if derivative.is_zero() {
            return Ok(chain);
        }
        chain.push(derivative);

        while chain
            .last()
            .is_some_and(|polynomial| polynomial.degree() > 0)
        {
            if chain.len() >= MAX_STURM_POLYNOMIALS {
                return Err(RootIsolationFailure::SturmChainLimit);
            }
            let next =
                signed_negative_pseudo_remainder(&chain[chain.len() - 2], &chain[chain.len() - 1])?;
            if next.is_zero() {
                break;
            }
            chain.push(next);
        }
        Ok(chain)
    }

    fn try_isolate(&self, lo: f64, hi: f64) -> Result<Vec<RootBracket>, RootIsolationFailure> {
        if self.is_zero() {
            return Err(RootIsolationFailure::ZeroPolynomial);
        }
        if !lo.is_finite() || !hi.is_finite() || lo > hi {
            return Err(RootIsolationFailure::InvalidRange);
        }

        if lo == hi {
            return if self.evaluate(lo)?.is_zero() {
                Ok(vec![RootBracket { lo, hi }])
            } else {
                Ok(Vec::new())
            };
        }

        let chain = self.sturm_chain()?;
        let mut roots = Vec::with_capacity(MAX_DEGREE);
        let lo_is_root = self.evaluate(lo)?.is_zero();
        let hi_is_root = self.evaluate(hi)?.is_zero();
        if lo_is_root {
            roots.push(RootBracket { lo, hi: lo });
        }

        let interior_count = roots_in_open_interval(&chain, lo, hi)?;
        let mut cells = vec![IsolationCell {
            lo,
            hi,
            count: interior_count,
            depth: 0,
        }];
        let mut visited = 0_usize;

        while let Some(cell) = cells.pop() {
            visited += 1;
            if visited > MAX_ISOLATION_CELLS {
                return Err(RootIsolationFailure::IsolationLimit);
            }
            if cell.count == 0 {
                continue;
            }
            if cell.count == 1 && adjacent(cell.lo, cell.hi) {
                if self.evaluate(cell.lo)?.is_zero() || self.evaluate(cell.hi)?.is_zero() {
                    return Err(RootIsolationFailure::ParameterResolution);
                }
                roots.push(RootBracket {
                    lo: cell.lo,
                    hi: cell.hi,
                });
                continue;
            }
            if cell.depth >= MAX_ISOLATION_DEPTH {
                return Err(RootIsolationFailure::IsolationLimit);
            }

            let midpoint = strict_midpoint(cell.lo, cell.hi)
                .ok_or(RootIsolationFailure::ParameterResolution)?;
            let midpoint_is_root = self.evaluate(midpoint)?.is_zero();
            if midpoint_is_root {
                roots.push(RootBracket {
                    lo: midpoint,
                    hi: midpoint,
                });
            }

            let left_count = roots_in_open_interval(&chain, cell.lo, midpoint)?;
            let right_count = roots_in_open_interval(&chain, midpoint, cell.hi)?;
            let midpoint_count = usize::from(midpoint_is_root);
            if left_count + midpoint_count + right_count != cell.count {
                return Err(RootIsolationFailure::SturmChainLimit);
            }

            let next_depth = cell.depth + 1;
            if right_count > 0 {
                if right_count == 1 && adjacent(midpoint, cell.hi) {
                    if self.evaluate(midpoint)?.is_zero() || self.evaluate(cell.hi)?.is_zero() {
                        return Err(RootIsolationFailure::ParameterResolution);
                    }
                    roots.push(RootBracket {
                        lo: midpoint,
                        hi: cell.hi,
                    });
                } else {
                    cells.push(IsolationCell {
                        lo: midpoint,
                        hi: cell.hi,
                        count: right_count,
                        depth: next_depth,
                    });
                }
            }
            if left_count > 0 {
                if left_count == 1 && adjacent(cell.lo, midpoint) {
                    if self.evaluate(cell.lo)?.is_zero() || self.evaluate(midpoint)?.is_zero() {
                        return Err(RootIsolationFailure::ParameterResolution);
                    }
                    roots.push(RootBracket {
                        lo: cell.lo,
                        hi: midpoint,
                    });
                } else {
                    cells.push(IsolationCell {
                        lo: cell.lo,
                        hi: midpoint,
                        count: left_count,
                        depth: next_depth,
                    });
                }
            }
        }

        if hi_is_root {
            roots.push(RootBracket { lo: hi, hi });
        }
        roots.sort_by(|a, b| a.lo.total_cmp(&b.lo).then(a.hi.total_cmp(&b.hi)));
        roots.dedup();
        if roots
            .windows(2)
            .any(|pair| pair[0].representative() >= pair[1].representative())
        {
            return Err(RootIsolationFailure::ParameterResolution);
        }
        Ok(roots)
    }

    /// Exact Bernstein/Descartes fallback for simple roots when pseudo-
    /// remainder coefficient growth leaves the expansion product envelope.
    /// Sign variation zero excludes roots and variation one proves exactly
    /// one root counting multiplicity. Unresolved repeated clusters still
    /// fail closed at the shared depth/cell bounds.
    fn try_isolate_bernstein(
        &self,
        lo: f64,
        hi: f64,
    ) -> Result<Vec<RootBracket>, RootIsolationFailure> {
        if self.is_zero() {
            return Err(RootIsolationFailure::ZeroPolynomial);
        }
        if !lo.is_finite() || !hi.is_finite() || lo > hi {
            return Err(RootIsolationFailure::InvalidRange);
        }
        if lo == hi {
            return if self.evaluate(lo)?.is_zero() {
                Ok(vec![RootBracket { lo, hi }])
            } else {
                Ok(Vec::new())
            };
        }

        let mut roots = Vec::with_capacity(self.degree());
        if self.evaluate(lo)?.is_zero() {
            roots.push(RootBracket { lo, hi: lo });
        }
        let hi_is_root = self.evaluate(hi)?.is_zero();
        let mut cells = vec![BernsteinIsolationCell {
            lo,
            hi,
            controls: self.bernstein_controls(lo, hi)?,
            depth: 0,
        }];
        let mut visited = 0_usize;

        while let Some(cell) = cells.pop() {
            visited += 1;
            if visited > MAX_ISOLATION_CELLS {
                return Err(RootIsolationFailure::IsolationLimit);
            }
            let variations = sign_variations(&cell.controls);
            if variations == 0 {
                continue;
            }
            if adjacent(cell.lo, cell.hi) {
                if variations != 1
                    || self.evaluate(cell.lo)?.is_zero()
                    || self.evaluate(cell.hi)?.is_zero()
                {
                    return Err(RootIsolationFailure::ParameterResolution);
                }
                roots.push(RootBracket {
                    lo: cell.lo,
                    hi: cell.hi,
                });
                continue;
            }
            if cell.depth >= MAX_ISOLATION_DEPTH {
                return Err(RootIsolationFailure::IsolationLimit);
            }

            let midpoint = strict_midpoint(cell.lo, cell.hi)
                .ok_or(RootIsolationFailure::ParameterResolution)?;
            if self.evaluate(midpoint)?.is_zero() {
                roots.push(RootBracket {
                    lo: midpoint,
                    hi: midpoint,
                });
            }
            let [left, right] = split_bernstein_controls(&cell.controls)?;
            let next_depth = cell.depth + 1;
            if sign_variations(&right) > 0 {
                cells.push(BernsteinIsolationCell {
                    lo: midpoint,
                    hi: cell.hi,
                    controls: right,
                    depth: next_depth,
                });
            }
            if sign_variations(&left) > 0 {
                cells.push(BernsteinIsolationCell {
                    lo: cell.lo,
                    hi: midpoint,
                    controls: left,
                    depth: next_depth,
                });
            }
        }

        if hi_is_root {
            roots.push(RootBracket { lo: hi, hi });
        }
        roots.sort_by(|a, b| a.lo.total_cmp(&b.lo).then(a.hi.total_cmp(&b.hi)));
        roots.dedup();
        if roots
            .windows(2)
            .any(|pair| pair[0].representative() >= pair[1].representative())
        {
            return Err(RootIsolationFailure::ParameterResolution);
        }
        Ok(roots)
    }

    fn strict_sign_on_interval(
        &self,
        lo: f64,
        hi: f64,
    ) -> Result<Option<i8>, RootIsolationFailure> {
        let controls = self.bernstein_controls(lo, hi)?;
        let signs = controls.iter().map(ExactScalar::sign).collect::<Vec<_>>();
        Ok(if signs.iter().all(|sign| *sign > 0) {
            Some(1)
        } else if signs.iter().all(|sign| *sign < 0) {
            Some(-1)
        } else {
            None
        })
    }

    fn bernstein_controls(
        &self,
        lo: f64,
        hi: f64,
    ) -> Result<Vec<ExactScalar>, RootIsolationFailure> {
        let width = hi - lo;
        let exact_width = ExactScalar::from_f64(hi)?.sub(&ExactScalar::from_f64(lo)?)?;
        if exact_width != ExactScalar::from_f64(width)? {
            return Err(RootIsolationFailure::UnsafeArithmeticEnvelope);
        }
        let degree = self.degree();
        let mut affine = vec![ExactScalar::zero(); degree + 1];
        for (power, coefficient) in self.coefficients.iter().enumerate() {
            for (target_power, slot) in affine.iter_mut().enumerate().take(power + 1) {
                let mut term = coefficient.clone();
                for _ in 0..power - target_power {
                    term = term.scale(lo)?;
                }
                for _ in 0..target_power {
                    term = term.scale(width)?;
                }
                term = term.scale(binomial(power, target_power) as f64)?;
                *slot = slot.add(&term)?;
            }
        }

        let factorials = [1_usize, 1, 2, 6, 24];
        let mut controls = Vec::with_capacity(degree + 1);
        for control in 0..=degree {
            let mut value = ExactScalar::zero();
            for (power, coefficient) in affine.iter().enumerate().take(control + 1) {
                let factor =
                    binomial(control, power) * factorials[power] * factorials[degree - power];
                value = value.add(&coefficient.scale(factor as f64)?)?;
            }
            controls.push(value);
        }
        Ok(controls)
    }
}

/// A closed interval containing exactly one distinct root.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RootBracket {
    pub lo: f64,
    pub hi: f64,
}

impl RootBracket {
    pub fn representative(self) -> f64 {
        if self.lo == self.hi {
            self.lo
        } else {
            self.lo / 2.0 + self.hi / 2.0
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum RootIsolation {
    Complete(Vec<RootBracket>),
    Ambiguous(RootIsolationFailure),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EndpointSide {
    Left,
    Right,
}

#[derive(Debug, Clone, Copy)]
struct IsolationCell {
    lo: f64,
    hi: f64,
    count: usize,
    depth: usize,
}

#[derive(Debug, Clone)]
struct BernsteinIsolationCell {
    lo: f64,
    hi: f64,
    controls: Vec<ExactScalar>,
    depth: usize,
}

fn binomial(n: usize, k: usize) -> usize {
    let k = k.min(n - k);
    let mut value = 1_usize;
    for index in 0..k {
        value = value * (n - index) / (index + 1);
    }
    value
}

fn sign_variations(controls: &[ExactScalar]) -> usize {
    let mut previous = 0_i8;
    let mut variations = 0_usize;
    for sign in controls.iter().map(ExactScalar::sign) {
        if sign == 0 {
            continue;
        }
        if previous != 0 && sign != previous {
            variations += 1;
        }
        previous = sign;
    }
    variations
}

fn split_bernstein_controls(
    controls: &[ExactScalar],
) -> Result<[Vec<ExactScalar>; 2], RootIsolationFailure> {
    let degree = controls
        .len()
        .checked_sub(1)
        .ok_or(RootIsolationFailure::DegreeLimit)?;
    let mut row = controls.to_vec();
    let mut left = vec![ExactScalar::zero(); degree + 1];
    let mut right = vec![ExactScalar::zero(); degree + 1];
    left[0] = row[0].clone();
    right[degree] = row[degree].clone();
    for level in 1..=degree {
        row = row
            .windows(2)
            .map(|pair| pair[0].add(&pair[1])?.scale(0.5))
            .collect::<Result<Vec<_>, RootIsolationFailure>>()?;
        left[level] = row[0].clone();
        right[degree - level] = row[row.len() - 1].clone();
    }
    Ok([left, right])
}

fn checked_components(components: Vec<f64>) -> Result<ExactScalar, RootIsolationFailure> {
    if components.is_empty()
        || components
            .iter()
            .any(|component| *component != 0.0 && !component.is_normal())
    {
        return Err(RootIsolationFailure::UnsafeArithmeticEnvelope);
    }
    if components.len() > MAX_EXPANSION_COMPONENTS {
        return Err(RootIsolationFailure::ExpansionLimit);
    }
    Ok(ExactScalar { components })
}

fn checked_sum(lhs: &[f64], rhs: &[f64]) -> Result<ExactScalar, RootIsolationFailure> {
    if lhs.len().saturating_add(rhs.len()) > MAX_EXPANSION_COMPONENTS * 2 {
        return Err(RootIsolationFailure::ExpansionLimit);
    }
    checked_components(expansion::sum(lhs, rhs))
}

fn checked_mul(lhs: &[f64], rhs: &[f64]) -> Result<ExactScalar, RootIsolationFailure> {
    if lhs.len().saturating_mul(rhs.len()) > MAX_EXPANSION_PRODUCT_WORK {
        return Err(RootIsolationFailure::ExpansionLimit);
    }
    for &lhs_component in lhs {
        for &rhs_component in rhs {
            checked_product_pair(lhs_component, rhs_component)?;
        }
    }
    checked_components(expansion::mul(lhs, rhs))
}

fn checked_negate(components: &[f64]) -> Result<Vec<f64>, RootIsolationFailure> {
    let negated = expansion::negate(components);
    if negated
        .iter()
        .any(|component| *component != 0.0 && !component.is_normal())
    {
        Err(RootIsolationFailure::UnsafeArithmeticEnvelope)
    } else {
        Ok(negated)
    }
}

fn checked_product_pair(lhs: f64, rhs: f64) -> Result<(), RootIsolationFailure> {
    if lhs == 0.0 || rhs == 0.0 {
        return Ok(());
    }
    if !lhs.is_normal()
        || !rhs.is_normal()
        || lhs.abs() > EXACT_COMPONENT_MAX
        || rhs.abs() > EXACT_COMPONENT_MAX
    {
        return Err(RootIsolationFailure::UnsafeArithmeticEnvelope);
    }
    let product = lhs.abs() * rhs.abs();
    if !product.is_normal() || !(EXACT_PRODUCT_MIN..=EXACT_PRODUCT_MAX).contains(&product) {
        return Err(RootIsolationFailure::UnsafeArithmeticEnvelope);
    }
    Ok(())
}

/// Negative remainder with the sign of pseudo-division's leading-coefficient
/// factor removed.  The result is therefore a positive scalar multiple of the
/// ordinary negative remainder and is valid in a Sturm sequence.
fn signed_negative_pseudo_remainder(
    dividend: &ExactPolynomial,
    divisor: &ExactPolynomial,
) -> Result<ExactPolynomial, RootIsolationFailure> {
    debug_assert!(!divisor.is_zero());
    debug_assert!(dividend.degree() >= divisor.degree());

    let divisor_degree = divisor.degree();
    let iterations = dividend.degree() - divisor_degree + 1;
    let divisor_leading = divisor.leading_coefficient();
    let mut remainder = dividend.clone();

    for _ in 0..iterations {
        if remainder.is_zero() || remainder.degree() < divisor_degree {
            remainder = scale_polynomial(&remainder, divisor_leading)?;
            continue;
        }

        let degree_difference = remainder.degree() - divisor_degree;
        let remainder_leading = remainder.leading_coefficient().clone();
        let scaled_remainder = scale_polynomial(&remainder, divisor_leading)?;
        let scaled_divisor =
            scale_and_shift_polynomial(divisor, &remainder_leading, degree_difference)?;
        remainder = subtract_polynomials(&scaled_remainder, &scaled_divisor)?;
    }

    let pseudo_factor_sign = if divisor_leading.sign() < 0 && iterations % 2 == 1 {
        -1
    } else {
        1
    };
    if pseudo_factor_sign > 0 {
        negate_polynomial(&remainder)
    } else {
        Ok(remainder)
    }
}

fn scale_polynomial(
    polynomial: &ExactPolynomial,
    factor: &ExactScalar,
) -> Result<ExactPolynomial, RootIsolationFailure> {
    if polynomial.is_zero() || factor.is_zero() {
        return Ok(ExactPolynomial::zero());
    }
    let coefficients = polynomial
        .coefficients
        .iter()
        .map(|coefficient| coefficient.mul(factor))
        .collect::<Result<Vec<_>, _>>()?;
    ExactPolynomial::from_coefficients_allow_zero(coefficients)
}

fn scale_and_shift_polynomial(
    polynomial: &ExactPolynomial,
    factor: &ExactScalar,
    shift: usize,
) -> Result<ExactPolynomial, RootIsolationFailure> {
    let mut coefficients = vec![ExactScalar::zero(); shift];
    for coefficient in &polynomial.coefficients {
        coefficients.push(coefficient.mul(factor)?);
    }
    ExactPolynomial::from_coefficients_allow_zero(coefficients)
}

fn subtract_polynomials(
    lhs: &ExactPolynomial,
    rhs: &ExactPolynomial,
) -> Result<ExactPolynomial, RootIsolationFailure> {
    let coefficient_count = lhs.coefficients.len().max(rhs.coefficients.len());
    let mut coefficients = Vec::with_capacity(coefficient_count);
    for index in 0..coefficient_count {
        let lhs_coefficient = lhs
            .coefficients
            .get(index)
            .cloned()
            .unwrap_or_else(ExactScalar::zero);
        let rhs_coefficient = rhs
            .coefficients
            .get(index)
            .cloned()
            .unwrap_or_else(ExactScalar::zero);
        coefficients.push(lhs_coefficient.sub(&rhs_coefficient)?);
    }
    ExactPolynomial::from_coefficients_allow_zero(coefficients)
}

fn negate_polynomial(
    polynomial: &ExactPolynomial,
) -> Result<ExactPolynomial, RootIsolationFailure> {
    let coefficients = polynomial
        .coefficients
        .iter()
        .map(ExactScalar::negate)
        .collect::<Result<Vec<_>, _>>()?;
    ExactPolynomial::from_coefficients_allow_zero(coefficients)
}

fn roots_in_open_interval(
    chain: &[ExactPolynomial],
    lo: f64,
    hi: f64,
) -> Result<usize, RootIsolationFailure> {
    if lo >= hi {
        return Ok(0);
    }
    let lo_variations = variations_at(chain, lo, EndpointSide::Right)?;
    let hi_variations = variations_at(chain, hi, EndpointSide::Left)?;
    lo_variations
        .checked_sub(hi_variations)
        .ok_or(RootIsolationFailure::SturmChainLimit)
}

fn variations_at(
    chain: &[ExactPolynomial],
    parameter: f64,
    side: EndpointSide,
) -> Result<usize, RootIsolationFailure> {
    let mut previous = 0_i8;
    let mut variations = 0_usize;
    for polynomial in chain {
        let sign = polynomial.side_sign(parameter, side)?;
        if sign == 0 {
            continue;
        }
        if previous != 0 && sign != previous {
            variations += 1;
        }
        previous = sign;
    }
    Ok(variations)
}

fn strict_midpoint(lo: f64, hi: f64) -> Option<f64> {
    let midpoint = lo / 2.0 + hi / 2.0;
    (midpoint.is_finite() && lo < midpoint && midpoint < hi).then_some(midpoint)
}

fn adjacent(lo: f64, hi: f64) -> bool {
    lo.next_up() >= hi
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scalar(value: f64) -> ExactScalar {
        ExactScalar::from_f64(value).unwrap()
    }

    fn polynomial(coefficients: &[f64]) -> ExactPolynomial {
        ExactPolynomial::new(coefficients.iter().copied().map(scalar).collect()).unwrap()
    }

    fn complete_roots(polynomial: &ExactPolynomial, lo: f64, hi: f64) -> Vec<RootBracket> {
        match polynomial.isolate(lo, hi) {
            RootIsolation::Complete(roots) => roots,
            RootIsolation::Ambiguous(failure) => panic!("unexpected ambiguity: {failure:?}"),
        }
    }

    fn assert_brackets_root(brackets: &[RootBracket], expected: &[f64], tolerance: f64) {
        assert_eq!(brackets.len(), expected.len());
        for (bracket, expected) in brackets.iter().zip(expected) {
            assert!(bracket.lo <= *expected && *expected <= bracket.hi);
            assert!((bracket.representative() - expected).abs() <= tolerance);
        }
    }

    #[test]
    fn isolates_four_distinct_roots() {
        // (x + 3)(x + 1)(x - 2)(x - 4)
        let roots = complete_roots(&polynomial(&[24.0, 14.0, -13.0, -2.0, 1.0]), -5.0, 5.0);
        assert_brackets_root(&roots, &[-3.0, -1.0, 2.0, 4.0], f64::EPSILON);
    }

    #[test]
    fn negative_polynomial_scale_preserves_sturm_counts() {
        // The first pseudo-division uses a negative derivative leading
        // coefficient here, exercising the pseudo-remainder sign correction.
        let roots = complete_roots(&polynomial(&[-24.0, -14.0, 13.0, 2.0, -1.0]), -5.0, 5.0);
        assert_brackets_root(&roots, &[-3.0, -1.0, 2.0, 4.0], f64::EPSILON);
    }

    #[test]
    fn certifies_no_real_roots() {
        let roots = complete_roots(&polynomial(&[1.0, 0.0, 1.0]), -100.0, 100.0);
        assert!(roots.is_empty());
    }

    #[test]
    fn constant_derivative_zero_polynomial_is_ambiguous() {
        let derivative = polynomial(&[3.0]).derivative().unwrap();
        assert_eq!(
            derivative.isolate(-1.0, 1.0),
            RootIsolation::Ambiguous(RootIsolationFailure::ZeroPolynomial)
        );
    }

    #[test]
    fn repeated_rational_root_is_reported_once() {
        // (x - 1)^2 (x + 2)
        let polynomial = polynomial(&[2.0, -3.0, 0.0, 1.0]);
        let roots = complete_roots(&polynomial, -4.0, 4.0);
        assert_brackets_root(&roots, &[-2.0, 1.0], f64::EPSILON);
        assert!(!polynomial.root_is_repeated(roots[0]).unwrap());
        assert!(polynomial.root_is_repeated(roots[1]).unwrap());
        let repeated = match polynomial.isolate_repeated_roots(-4.0, 4.0) {
            RootIsolation::Complete(roots) => roots,
            RootIsolation::Ambiguous(failure) => {
                panic!("unexpected repeated-root ambiguity: {failure:?}")
            }
        };
        assert_brackets_root(&repeated, &[1.0], f64::EPSILON);
    }

    #[test]
    fn repeated_irrational_roots_are_distinctly_isolated() {
        // (x^2 - 2)^2
        let polynomial = polynomial(&[4.0, 0.0, -4.0, 0.0, 1.0]);
        let roots = complete_roots(&polynomial, -2.0, 2.0);
        let sqrt_two = 2.0_f64.sqrt();
        assert_brackets_root(&roots, &[-sqrt_two, sqrt_two], 2.0 * f64::EPSILON);
        assert!(
            roots
                .iter()
                .all(|root| polynomial.root_is_repeated(*root).unwrap())
        );
        let repeated = match polynomial.isolate_repeated_roots(-2.0, 2.0) {
            RootIsolation::Complete(roots) => roots,
            RootIsolation::Ambiguous(failure) => {
                panic!("unexpected repeated-root ambiguity: {failure:?}")
            }
        };
        assert_brackets_root(&repeated, &[-sqrt_two, sqrt_two], 2.0 * f64::EPSILON);
    }

    #[test]
    fn separates_roots_closer_than_one_nanounit() {
        let a = 1.0;
        let b = 1.0 + 2.0_f64.powi(-32);
        // Exact product (x-a)(x-b), constructed without rounding the product
        // of the source roots into a single coefficient operation.
        let minus_a = scalar(a).negate().unwrap();
        let minus_b = scalar(b).negate().unwrap();
        let constant = minus_a.mul(&minus_b).unwrap();
        let linear = minus_a.add(&minus_b).unwrap();
        let p = ExactPolynomial::new(vec![constant, linear, scalar(1.0)]).unwrap();
        let roots = complete_roots(&p, 0.5, 1.5);
        assert_brackets_root(&roots, &[a, b], f64::EPSILON);
        assert_eq!(
            p.isolate_repeated_roots(0.5, 1.5),
            RootIsolation::Complete(Vec::new())
        );
    }

    #[test]
    fn bernstein_fallback_isolates_four_irrational_roots_after_sturm_envelope_refusal() {
        // Substituting sin(u)=2t/(1+t^2) into
        // 1.8^2 + sin(u)^2 - 4 produces this exact even quartic. Its expanded
        // dyadic coefficients drive pseudo-remainders outside the conservative
        // product envelope, while Bernstein sign variation still proves all
        // four simple roots without sampling.
        let height = scalar(1.8);
        let constant = height.mul(&height).unwrap().sub(&scalar(4.0)).unwrap();
        let quadratic = constant.scale(2.0).unwrap().add(&scalar(4.0)).unwrap();
        let p = ExactPolynomial::new(vec![
            constant.clone(),
            scalar(0.0),
            quadratic,
            scalar(0.0),
            constant,
        ])
        .unwrap();
        assert_eq!(
            p.try_isolate(-2.0, 2.0),
            Err(RootIsolationFailure::UnsafeArithmeticEnvelope)
        );

        let sine = (4.0 - 1.8_f64 * 1.8).sqrt();
        let cosine = (1.0 - sine * sine).sqrt();
        let tangent = sine / (1.0 + cosine);
        let expected = [-1.0 / tangent, -tangent, tangent, 1.0 / tangent];
        let roots = complete_roots(&p, -2.0, 2.0);
        assert_brackets_root(&roots, &expected, 4.0 * f64::EPSILON);
        assert!(roots.iter().all(|root| !p.root_is_repeated(*root).unwrap()));
    }

    #[test]
    fn includes_closed_range_endpoint_roots() {
        let roots = complete_roots(&polynomial(&[0.0, -1.0, 0.0, 1.0]), -1.0, 1.0);
        assert_eq!(
            roots,
            vec![
                RootBracket { lo: -1.0, hi: -1.0 },
                RootBracket { lo: 0.0, hi: 0.0 },
                RootBracket { lo: 1.0, hi: 1.0 },
            ]
        );
    }

    #[test]
    fn unsafe_extreme_arithmetic_is_explicitly_ambiguous() {
        let huge = scalar(1.0e308);
        assert_eq!(
            huge.mul(&huge),
            Err(RootIsolationFailure::UnsafeArithmeticEnvelope)
        );

        let p = polynomial(&[1.0, 1.0]);
        assert_eq!(
            p.isolate(-1.0e308, 1.0e308),
            RootIsolation::Ambiguous(RootIsolationFailure::UnsafeArithmeticEnvelope)
        );
    }

    #[test]
    fn subnormal_inputs_and_products_are_explicitly_ambiguous() {
        assert_eq!(
            ExactScalar::from_f64(f64::from_bits(1)),
            Err(RootIsolationFailure::UnsafeArithmeticEnvelope)
        );

        let too_small = scalar(2.0_f64.powi(-450));
        assert_eq!(
            too_small.mul(&too_small),
            Err(RootIsolationFailure::UnsafeArithmeticEnvelope)
        );
        assert_eq!(
            too_small.scale(2.0_f64.powi(-400)),
            Err(RootIsolationFailure::UnsafeArithmeticEnvelope)
        );
    }

    #[test]
    fn product_near_low_exact_envelope_remains_available() {
        let lhs = scalar(2.0_f64.powi(-400));
        let rhs = scalar(2.0_f64.powi(-399));
        assert_eq!(lhs.mul(&rhs).unwrap().sign(), 1);
    }

    #[test]
    fn caller_supplied_adjacent_range_can_isolate_one_open_root() {
        let q = scalar(1.0).add(&scalar(2.0_f64.powi(-53))).unwrap();
        let p = ExactPolynomial::new(vec![q.negate().unwrap(), scalar(0.0), scalar(1.0)]).unwrap();
        let lo = 1.0_f64;
        let hi = lo.next_up();
        assert_eq!(
            p.isolate(lo, hi),
            RootIsolation::Complete(vec![RootBracket { lo, hi }])
        );
    }

    #[test]
    fn adjacent_bracket_never_reuses_an_exact_endpoint_root() {
        // q = 1 + 2^-53 exactly, so sqrt(q) lies strictly between the adjacent
        // floats 1 and 1.next_up().  In (x - 1)(x^2 - q), the exact root at
        // 1 is discovered at the first midpoint.  A closed adjacent bracket
        // for sqrt(q) would also contain that already-recorded root and would
        // therefore not isolate one distinct root.
        let q = scalar(1.0).add(&scalar(2.0_f64.powi(-53))).unwrap();
        let p = ExactPolynomial::new(vec![
            q.clone(),
            q.negate().unwrap(),
            scalar(-1.0),
            scalar(1.0),
        ])
        .unwrap();
        assert_eq!(
            p.isolate(0.0, 2.0),
            RootIsolation::Ambiguous(RootIsolationFailure::ParameterResolution)
        );
    }

    #[test]
    fn distinct_adjacent_brackets_require_distinct_representatives() {
        // The exact roots 1 +/- 2^-54 fall on opposite sides of 1 but inside
        // its two neighboring-float gaps.  Their disjoint isolating brackets
        // have midpoint representatives that both round to 1.0, so exposing
        // those representatives as two numeric roots would collapse topology.
        let delta = scalar(2.0_f64.powi(-54));
        let constant = scalar(1.0).sub(&delta.mul(&delta).unwrap()).unwrap();
        let p = ExactPolynomial::new(vec![constant, scalar(-2.0), scalar(1.0)]).unwrap();
        assert_eq!(
            p.isolate(0.0, 2.0),
            RootIsolation::Ambiguous(RootIsolationFailure::ParameterResolution)
        );
    }
}
