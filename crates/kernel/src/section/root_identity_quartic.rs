//! Complete interval-certified roots for periodic quartic restrictions.

use kcore::interval::Interval;

use super::root_identity::{
    RootIdentityGap, excludes_zero, finite, strict_sort, twice_atan_interval,
};

const QUARTIC_ROOT_MAX_DEPTH: usize = 24;
const QUARTIC_ROOT_MAX_NODES: usize = 16_384;

// Each compact half-angle branch may visit at most `MAX_NODES`. Precharging
// both complete covers makes the semantic result independent of early exits
// while bounding the recursive work below the default Section allowance.
pub(super) const CIRCLE_CYLINDER_QUARTIC_WORK: u64 = 2 * QUARTIC_ROOT_MAX_NODES as u64;

/// Certify all roots of a periodic quartic half-angle numerator.
///
/// Positive and negative half angles are independently compactified by
/// `h = +/-s/(1-s)`, `s in [0, 1]`. The transformed polynomial has the same
/// finite roots because its positive denominator was cleared. A recursive
/// interval cover discards only cells whose polynomial range excludes zero;
/// every retained root cell must additionally have a one-signed derivative
/// and opposite endpoint signs, proving existence and uniqueness. The
/// accepted zero, two, or four cells are therefore a complete simple-root
/// proof, not candidate samples.
pub(super) fn certify_periodic_quartic_roots(
    quartic: [Interval; 5],
) -> core::result::Result<Vec<Interval>, RootIdentityGap> {
    if quartic[0].contains_zero() {
        return Err(RootIdentityGap::ParameterSeamContact);
    }
    // `h = +/-infinity` is the ordinary source parameter `PI`, but is a chart
    // boundary for both compactifications. Do not infer its multiplicity from
    // a degree drop.
    if quartic[4].contains_zero() {
        return Err(RootIdentityGap::TangentialOrUnresolvedMultiplicity);
    }

    let mut compact_roots = Vec::new();
    for half_angle_sign in [1_i8, -1_i8] {
        let compact = compactified_quartic(quartic, half_angle_sign)?;
        for root in isolate_compact_quartic(compact)? {
            compact_roots.push((half_angle_sign, root));
        }
    }
    // A seam-free periodic scalar crosses zero an even number of times, and
    // the half-angle numerator's degree bounds the complete count by four.
    if !matches!(compact_roots.len(), 0 | 2 | 4) {
        return Err(RootIdentityGap::TangentialOrUnresolvedMultiplicity);
    }

    let tau = core::f64::consts::TAU;
    let mut roots = Vec::with_capacity(compact_roots.len());
    for (half_angle_sign, compact) in compact_roots {
        if compact.lo() <= 0.0 || compact.hi() >= 1.0 {
            return Err(RootIdentityGap::ParameterSeamContact);
        }
        let magnitude = compact
            .checked_div(Interval::point(1.0) - compact)
            .ok_or(RootIdentityGap::ArithmeticGuard)?;
        let principal = if half_angle_sign > 0 {
            twice_atan_interval(magnitude)?
        } else {
            twice_atan_interval(-magnitude)? + Interval::point(tau)
        };
        if principal.lo() <= 0.0 || principal.hi() >= tau {
            return Err(RootIdentityGap::ParameterSeamContact);
        }
        roots.push(principal);
    }
    strict_sort(&mut roots)?;
    Ok(roots)
}

fn compactified_quartic(
    quartic: [Interval; 5],
    half_angle_sign: i8,
) -> core::result::Result<[Interval; 5], RootIdentityGap> {
    const BINOMIAL: [[u8; 5]; 5] = [
        [1, 0, 0, 0, 0],
        [1, 1, 0, 0, 0],
        [1, 2, 1, 0, 0],
        [1, 3, 3, 1, 0],
        [1, 4, 6, 4, 1],
    ];
    let mut transformed: [Option<Interval>; 5] = [None; 5];
    for (power, coefficient) in quartic.into_iter().enumerate() {
        let signed = if half_angle_sign < 0 && power % 2 == 1 {
            -coefficient
        } else {
            coefficient
        };
        for (residual_power, binomial) in BINOMIAL[4 - power].iter().enumerate().take(5 - power) {
            let scalar = f64::from(*binomial);
            let mut term = signed * Interval::point(scalar);
            if residual_power % 2 == 1 {
                term = -term;
            }
            let output_power = power + residual_power;
            transformed[output_power] = Some(match transformed[output_power] {
                Some(accumulated) => accumulated + term,
                None => term,
            });
        }
    }
    let transformed = transformed.map(|coefficient| coefficient.unwrap_or(Interval::point(0.0)));
    transformed
        .iter()
        .all(|coefficient| finite(*coefficient))
        .then_some(transformed)
        .ok_or(RootIdentityGap::ArithmeticGuard)
}

#[derive(Default)]
struct CompactRootCover {
    certified: Vec<Interval>,
    unresolved: Vec<Interval>,
    visited: usize,
    exhausted: bool,
}

fn isolate_compact_quartic(
    coefficients: [Interval; 5],
) -> core::result::Result<Vec<Interval>, RootIdentityGap> {
    let derivative = quartic_derivative(coefficients);
    let mut cover = CompactRootCover::default();
    classify_compact_interval(
        coefficients,
        derivative,
        Interval::new(0.0, 1.0),
        0,
        &mut cover,
    );
    if cover.exhausted {
        return Err(RootIdentityGap::TangentialOrUnresolvedMultiplicity);
    }

    let mut unresolved = merge_touching_intervals(cover.unresolved);
    for candidate in unresolved.drain(..) {
        let derivative_range = polynomial_value(derivative, candidate);
        let Some(_) = strict_interval_sign(derivative_range) else {
            return Err(RootIdentityGap::TangentialOrUnresolvedMultiplicity);
        };
        let left = strict_interval_sign(polynomial_value(
            coefficients,
            Interval::point(candidate.lo()),
        ));
        let right = strict_interval_sign(polynomial_value(
            coefficients,
            Interval::point(candidate.hi()),
        ));
        match (left, right) {
            (Some(a), Some(b)) if a != b => {
                cover
                    .certified
                    .push(refine_compact_root(coefficients, candidate, a, b));
            }
            (Some(a), Some(b)) if a == b => {}
            _ => return Err(RootIdentityGap::TangentialOrUnresolvedMultiplicity),
        }
    }
    strict_sort(&mut cover.certified)?;
    Ok(cover.certified)
}

fn classify_compact_interval(
    coefficients: [Interval; 5],
    derivative: [Interval; 4],
    domain: Interval,
    depth: usize,
    cover: &mut CompactRootCover,
) {
    if cover.exhausted {
        return;
    }
    if cover.visited == QUARTIC_ROOT_MAX_NODES {
        cover.exhausted = true;
        return;
    }
    cover.visited += 1;
    if excludes_zero(polynomial_value(coefficients, domain)) {
        return;
    }

    let derivative_sign = strict_interval_sign(polynomial_value(derivative, domain));
    let left_sign =
        strict_interval_sign(polynomial_value(coefficients, Interval::point(domain.lo())));
    let right_sign =
        strict_interval_sign(polynomial_value(coefficients, Interval::point(domain.hi())));
    if derivative_sign.is_some()
        && let (Some(left), Some(right)) = (left_sign, right_sign)
    {
        if left != right {
            cover
                .certified
                .push(refine_compact_root(coefficients, domain, left, right));
        }
        return;
    }
    if depth == QUARTIC_ROOT_MAX_DEPTH {
        cover.unresolved.push(domain);
        return;
    }
    let midpoint = 0.5 * domain.lo() + 0.5 * domain.hi();
    if midpoint <= domain.lo() || midpoint >= domain.hi() {
        cover.unresolved.push(domain);
        return;
    }
    classify_compact_interval(
        coefficients,
        derivative,
        Interval::new(domain.lo(), midpoint),
        depth + 1,
        cover,
    );
    classify_compact_interval(
        coefficients,
        derivative,
        Interval::new(midpoint, domain.hi()),
        depth + 1,
        cover,
    );
}

fn refine_compact_root(
    coefficients: [Interval; 5],
    mut bracket: Interval,
    mut left_sign: i8,
    right_sign: i8,
) -> Interval {
    debug_assert_ne!(left_sign, right_sign);
    for _ in 0..80 {
        let midpoint = 0.5 * bracket.lo() + 0.5 * bracket.hi();
        if midpoint <= bracket.lo() || midpoint >= bracket.hi() {
            break;
        }
        match strict_interval_sign(polynomial_value(coefficients, Interval::point(midpoint))) {
            Some(sign) if sign == left_sign => {
                bracket = Interval::new(midpoint, bracket.hi());
                left_sign = sign;
            }
            Some(sign) if sign == right_sign => {
                bracket = Interval::new(bracket.lo(), midpoint);
            }
            _ => break,
        }
    }
    bracket
}

fn merge_touching_intervals(mut intervals: Vec<Interval>) -> Vec<Interval> {
    intervals.sort_by(|a, b| a.lo().total_cmp(&b.lo()).then(a.hi().total_cmp(&b.hi())));
    let mut merged: Vec<Interval> = Vec::with_capacity(intervals.len());
    for interval in intervals {
        if let Some(last) = merged.last_mut()
            && interval.lo() <= last.hi()
        {
            *last = Interval::new(last.lo(), last.hi().max(interval.hi()));
            continue;
        }
        merged.push(interval);
    }
    merged
}

fn quartic_derivative(coefficients: [Interval; 5]) -> [Interval; 4] {
    core::array::from_fn(|power| coefficients[power + 1] * Interval::point((power + 1) as f64))
}

fn polynomial_value<const N: usize>(coefficients: [Interval; N], argument: Interval) -> Interval {
    let mut value = coefficients[N - 1];
    for coefficient in coefficients[..N - 1].iter().rev() {
        value = value * argument + *coefficient;
    }
    value
}

fn strict_interval_sign(value: Interval) -> Option<i8> {
    if value.lo() > 0.0 {
        Some(1)
    } else if value.hi() < 0.0 {
        Some(-1)
    } else {
        None
    }
}
