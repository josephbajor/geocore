//! Exact parameter-correspondence lifts for spatial curve-pair roots.
//!
//! A zero of a two-coordinate projection is a full 3D zero when an exact
//! scalar carrier shared by both source representations forces the normalized
//! parameters to correspond and the omitted scalar coordinate is shared under
//! that same correspondence.  Strict source-range derivative intervals make
//! the carrier injective; Poincare--Miranda and a range P-matrix independently
//! prove existence and uniqueness of the projected zero.

use super::NurbsCurve;
use super::curve_pair::{
    CurvePairAlgebraicSearchConfig, CurvePairProjectionPlane, certify_p_matrix_in_ranges,
};
use crate::curve::Curve;
use crate::param::ParamRange;
use kcore::expansion;
use kcore::interval::Interval;

const SAFE_COMPONENT_MIN: f64 = f64::from_bits(((1023 - 500) as u64) << 52);
const SAFE_COMPONENT_MAX: f64 = f64::from_bits(((1023 + 500) as u64) << 52);
const SAFE_PRODUCT_MIN: f64 = f64::from_bits(((1023 - 400) as u64) << 52);
const SAFE_PRODUCT_MAX: f64 = f64::from_bits(((1023 + 400) as u64) << 52);
const STABLE_PRIMITIVE_FORM_PREFIX_COEFFICIENT: i8 = 6;
const MAX_SUPPORTED_PRIMITIVE_FORM_COEFFICIENT: i8 = 14;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ParameterOrientation {
    Same,
    Reversed,
}

/// Lift one projected unique root to an exact spatial root without sampling.
///
/// Only the two original source representations and outward interval bounds
/// over their requested ranges participate.  Any unsupported representation,
/// non-exact correspondence, unsafe exact-product corridor, or inconclusive
/// interval proof returns `None`.
pub(super) fn certify_algebraic_spatial_root(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
    search: CurvePairAlgebraicSearchConfig,
) -> Option<(CurvePairProjectionPlane, f64)> {
    if !first.knots().is_clamped()
        || !second.knots().is_clamped()
        || first.degree() != second.degree()
        || first.knots().as_slice().len() != second.knots().as_slice().len()
        || first.points().len() != second.points().len()
    {
        return None;
    }

    for orientation in [ParameterOrientation::Same, ParameterOrientation::Reversed] {
        if !normalized_knots_correspond(first, second, orientation)
            || !weights_are_globally_proportional(first, second, orientation)
        {
            continue;
        }

        for carrier_axis in 0..3 {
            if !scalar_controls_correspond(first, second, carrier_axis, orientation) {
                continue;
            }
            let Some(carrier) = certify_carrier_correspondence(
                first,
                first_range,
                second,
                second_range,
                carrier_axis,
                orientation,
            ) else {
                continue;
            };

            for omitted_axis in (0..3).filter(|&axis| axis != carrier_axis) {
                if !scalar_controls_correspond(first, second, omitted_axis, orientation) {
                    continue;
                }
                let crossing_axis = (0..3)
                    .find(|&axis| axis != carrier_axis && axis != omitted_axis)
                    .expect("three coordinate axes have one remaining member");
                let plane = projection_plane(carrier_axis, crossing_axis);
                if let Some(bound) = certify_with_carrier_as_first_equation(
                    first,
                    first_range,
                    second,
                    second_range,
                    carrier_axis,
                    crossing_axis,
                    carrier,
                ) {
                    return Some((plane, bound));
                }
                if let Some(bound) = certify_with_carrier_as_second_equation(
                    first,
                    first_range,
                    second,
                    second_range,
                    carrier_axis,
                    crossing_axis,
                    carrier,
                    orientation,
                ) {
                    return Some((plane, bound));
                }
            }
        }

        if let Some(certificate) = certify_primitive_integer_form_spatial_root(
            first,
            first_range,
            second,
            second_range,
            orientation,
            search,
        ) {
            return Some(certificate);
        }
    }
    None
}

/// Broaden the coordinate-scalar lift to exact primitive integer linear forms.
///
/// Carrier coefficients are nonzero, primitive, bounded by the validated
/// configured magnitude, and normalized so the first projected coefficient is
/// positive. Residual coefficients have the same bound and gcd normalization,
/// with a positive omitted-coordinate coefficient. These rules enumerate
/// every form in the bounded family once, modulo a nonzero integer scale and
/// global sign. The compatibility magnitude-twelve family remains the exact
/// prefix of the optional magnitude-thirteen and magnitude-fourteen shells.
///
/// A shared projected carrier is equal at every projected zero even when no
/// coordinate scalar or unit-coefficient form corresponds. Injectivity fixes
/// the normalized parameter correspondence. A shared residual with nonzero
/// omitted coefficient then forces the omitted coordinate to agree.
fn certify_primitive_integer_form_spatial_root(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
    orientation: ParameterOrientation,
    search: CurvePairAlgebraicSearchConfig,
) -> Option<(CurvePairProjectionPlane, f64)> {
    for coefficient_bound in configured_primitive_form_bounds(search) {
        if let Some(certificate) = certify_primitive_integer_form_spatial_root_at_bound(
            first,
            first_range,
            second,
            second_range,
            orientation,
            coefficient_bound,
        ) {
            return Some(certificate);
        }
    }
    None
}

fn configured_primitive_form_bounds(
    search: CurvePairAlgebraicSearchConfig,
) -> core::ops::RangeInclusive<i8> {
    let maximum = i8::try_from(search.maximum_primitive_form_coefficient())
        .expect("validated primitive-form coefficient ceiling fits i8");
    debug_assert!(maximum <= MAX_SUPPORTED_PRIMITIVE_FORM_COEFFICIENT);
    STABLE_PRIMITIVE_FORM_PREFIX_COEFFICIENT..=maximum
}

#[allow(clippy::too_many_arguments)]
fn certify_primitive_integer_form_spatial_root_at_bound(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
    orientation: ParameterOrientation,
    coefficient_bound: i8,
) -> Option<(CurvePairProjectionPlane, f64)> {
    'omitted: for omitted_axis in 0..3 {
        let [first_axis, second_axis] = match omitted_axis {
            0 => [1, 2],
            1 => [0, 2],
            2 => [0, 1],
            _ => unreachable!("coordinate axes are in 0..3"),
        };
        let plane = projection_plane(first_axis, second_axis);

        for first_coefficient in 1..=coefficient_bound {
            for second_coefficient in -coefficient_bound..=coefficient_bound {
                if second_coefficient == 0 {
                    continue;
                }
                let carrier = linear_form([
                    (first_axis, first_coefficient),
                    (second_axis, second_coefficient),
                ]);
                if !coefficients_are_primitive(carrier) {
                    continue;
                }
                if !linear_form_controls_correspond(first, second, carrier, orientation)
                    || !linear_form_is_strictly_monotone(
                        first,
                        first_range,
                        second,
                        second_range,
                        carrier,
                        orientation,
                    )
                {
                    continue;
                }

                for omitted_coefficient in 1..=coefficient_bound {
                    for first_residual_coefficient in -coefficient_bound..=coefficient_bound {
                        for second_residual_coefficient in -coefficient_bound..=coefficient_bound {
                            let residual = linear_form([
                                (omitted_axis, omitted_coefficient),
                                (first_axis, first_residual_coefficient),
                                (second_axis, second_residual_coefficient),
                            ]);
                            if (coefficient_bound > STABLE_PRIMITIVE_FORM_PREFIX_COEFFICIENT
                                && !form_pair_reaches_bound(carrier, residual, coefficient_bound))
                                || !coefficients_are_primitive(residual)
                                || !linear_form_controls_correspond(
                                    first,
                                    second,
                                    residual,
                                    orientation,
                                )
                            {
                                continue;
                            }
                            for axes in [[first_axis, second_axis], [second_axis, first_axis]] {
                                let bound =
                                    super::curve_pair::certify_projected_unique_root_in_ranges(
                                        first,
                                        first_range,
                                        second,
                                        second_range,
                                        axes,
                                    );
                                if let Some(bound) = bound {
                                    return Some((plane, bound));
                                }
                            }
                            // The projected Poincare/P-matrix proof depends
                            // only on this omitted axis, not on which exact
                            // carrier/residual pair supplied the 3D lift.
                            continue 'omitted;
                        }
                    }
                }
            }
        }
    }
    None
}

fn form_pair_reaches_bound(carrier: [i8; 3], residual: [i8; 3], bound: i8) -> bool {
    carrier
        .into_iter()
        .chain(residual)
        .any(|coefficient| coefficient.unsigned_abs() == bound as u8)
}

fn linear_form<const N: usize>(terms: [(usize, i8); N]) -> [i8; 3] {
    let mut coefficients = [0; 3];
    for (axis, coefficient) in terms {
        coefficients[axis] = coefficient;
    }
    coefficients
}

fn coefficients_are_primitive(coefficients: [i8; 3]) -> bool {
    coefficients
        .into_iter()
        .map(i8::unsigned_abs)
        .fold(0, greatest_common_divisor)
        == 1
}

fn greatest_common_divisor(mut first: u8, mut second: u8) -> u8 {
    while second != 0 {
        (first, second) = (second, first % second);
    }
    first
}

fn linear_form_controls_correspond(
    first: &NurbsCurve,
    second: &NurbsCurve,
    coefficients: [i8; 3],
    orientation: ParameterOrientation,
) -> bool {
    let count = first.points().len();
    (0..count).all(|index| {
        let second_index = corresponding_index(index, count, orientation);
        let Some(first_value) = exact_linear_form(first, index, coefficients) else {
            return false;
        };
        let Some(second_value) = exact_linear_form(second, second_index, coefficients) else {
            return false;
        };
        exact_expansions_equal(&first_value, &second_value)
    })
}

fn exact_linear_form(curve: &NurbsCurve, index: usize, coefficients: [i8; 3]) -> Option<Vec<f64>> {
    let mut result = vec![0.0];
    for (axis, coefficient) in coefficients.into_iter().enumerate() {
        let value = component(curve, index, axis);
        let term = match coefficient {
            -14..=-2 | 2..=14 => {
                if !safe_expansion(&[value]) {
                    return None;
                }
                expansion::scale(&[value], f64::from(coefficient))
            }
            -1 => vec![-value],
            0 => continue,
            1 => vec![value],
            _ => return None,
        };
        if !safe_expansion(&term) {
            return None;
        }
        result = expansion::sum(&result, &term);
        if !safe_expansion(&result) {
            return None;
        }
    }
    Some(result)
}

fn exact_expansions_equal(first: &[f64], second: &[f64]) -> bool {
    if !safe_expansion(first) || !safe_expansion(second) {
        return false;
    }
    let difference = expansion::sum(first, &expansion::negate(second));
    safe_expansion(&difference) && expansion::sign(&difference) == 0
}

fn linear_form_is_strictly_monotone(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
    coefficients: [i8; 3],
    orientation: ParameterOrientation,
) -> bool {
    let Some(mapped_second_range) = mapped_range(
        second_range,
        second.param_range(),
        first.param_range(),
        orientation,
    ) else {
        return false;
    };
    let Some(derivative) = linear_form_derivative_interval(
        first,
        bounding_range(first_range, mapped_second_range),
        coefficients,
    ) else {
        return false;
    };
    derivative.lo() > 0.0 || derivative.hi() < 0.0
}

fn linear_form_derivative_interval(
    curve: &NurbsCurve,
    range: ParamRange,
    coefficients: [i8; 3],
) -> Option<Interval> {
    super::source_range_interval::derivative_signed_linear_form_interval(curve, range, coefficients)
}

#[derive(Debug, Clone, Copy)]
struct CarrierCorrespondence {
    derivative_sign: f64,
    mapped_second_range: ParamRange,
}

fn certify_carrier_correspondence(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
    axis: usize,
    orientation: ParameterOrientation,
) -> Option<CarrierCorrespondence> {
    let mapped_second_range = mapped_range(
        second_range,
        second.param_range(),
        first.param_range(),
        orientation,
    )?;
    let derivative = super::source_range_interval::derivative_component_interval(
        first,
        bounding_range(first_range, mapped_second_range),
        axis,
    )?;
    let derivative_sign = if derivative.lo() > 0.0 {
        1.0
    } else if derivative.hi() < 0.0 {
        -1.0
    } else {
        return None;
    };
    Some(CarrierCorrespondence {
        derivative_sign,
        mapped_second_range,
    })
}

fn certify_with_carrier_as_first_equation(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
    carrier_axis: usize,
    crossing_axis: usize,
    carrier: CarrierCorrespondence,
) -> Option<f64> {
    if first_range.lo > carrier.mapped_second_range.lo
        || first_range.hi < carrier.mapped_second_range.hi
    {
        return None;
    }
    let crossing_sign =
        second_parameter_face_orientation(first, first_range, second, second_range, crossing_axis)?;
    certify_p_matrix_in_ranges(
        first,
        first_range,
        second,
        second_range,
        [carrier_axis, crossing_axis],
        [carrier.derivative_sign, crossing_sign],
    )
}

#[allow(clippy::too_many_arguments)]
fn certify_with_carrier_as_second_equation(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
    carrier_axis: usize,
    crossing_axis: usize,
    carrier: CarrierCorrespondence,
    orientation: ParameterOrientation,
) -> Option<f64> {
    if carrier.mapped_second_range.lo > first_range.lo
        || carrier.mapped_second_range.hi < first_range.hi
    {
        return None;
    }
    let crossing_sign =
        first_parameter_face_orientation(first, first_range, second, second_range, crossing_axis)?;
    let carrier_sign = match orientation {
        ParameterOrientation::Same => -carrier.derivative_sign,
        ParameterOrientation::Reversed => carrier.derivative_sign,
    };
    certify_p_matrix_in_ranges(
        first,
        first_range,
        second,
        second_range,
        [crossing_axis, carrier_axis],
        [crossing_sign, carrier_sign],
    )
}

fn first_parameter_face_orientation(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
    axis: usize,
) -> Option<f64> {
    let low =
        super::source_range_interval::position_component_interval(
            first,
            ParamRange::new(first_range.lo, first_range.lo),
            axis,
        )? - super::source_range_interval::position_component_interval(second, second_range, axis)?;
    let high =
        super::source_range_interval::position_component_interval(
            first,
            ParamRange::new(first_range.hi, first_range.hi),
            axis,
        )? - super::source_range_interval::position_component_interval(second, second_range, axis)?;
    interval_face_orientation(low, high)
}

fn second_parameter_face_orientation(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
    axis: usize,
) -> Option<f64> {
    let low = super::source_range_interval::position_component_interval(first, first_range, axis)?
        - super::source_range_interval::position_component_interval(
            second,
            ParamRange::new(second_range.lo, second_range.lo),
            axis,
        )?;
    let high = super::source_range_interval::position_component_interval(first, first_range, axis)?
        - super::source_range_interval::position_component_interval(
            second,
            ParamRange::new(second_range.hi, second_range.hi),
            axis,
        )?;
    interval_face_orientation(low, high)
}

fn interval_face_orientation(low: Interval, high: Interval) -> Option<f64> {
    if low.hi() <= 0.0 && high.lo() >= 0.0 {
        Some(1.0)
    } else if low.lo() >= 0.0 && high.hi() <= 0.0 {
        Some(-1.0)
    } else {
        None
    }
}

fn bounding_range(first: ParamRange, second: ParamRange) -> ParamRange {
    ParamRange::new(first.lo.min(second.lo), first.hi.max(second.hi))
}

fn mapped_range(
    range: ParamRange,
    source_domain: ParamRange,
    target_domain: ParamRange,
    orientation: ParameterOrientation,
) -> Option<ParamRange> {
    let first = map_parameter(range.lo, source_domain, target_domain, orientation)?;
    let last = map_parameter(range.hi, source_domain, target_domain, orientation)?;
    Some(ParamRange::new(first.min(last), first.max(last)))
}

fn map_parameter(
    parameter: f64,
    source_domain: ParamRange,
    target_domain: ParamRange,
    orientation: ParameterOrientation,
) -> Option<f64> {
    if !safe_domain(source_domain)
        || !safe_domain(target_domain)
        || !source_domain.contains(parameter)
    {
        return None;
    }
    let normalized = (parameter - source_domain.lo) / source_domain.width();
    let normalized = match orientation {
        ParameterOrientation::Same => normalized,
        ParameterOrientation::Reversed => 1.0 - normalized,
    };
    let mapped = target_domain.lo + normalized * target_domain.width();
    (mapped.is_finite()
        && target_domain.contains(mapped)
        && exact_normalized_parameters_equal(
            parameter,
            source_domain,
            mapped,
            target_domain,
            orientation,
        ))
    .then_some(mapped)
}

fn normalized_knots_correspond(
    first: &NurbsCurve,
    second: &NurbsCurve,
    orientation: ParameterOrientation,
) -> bool {
    let first_domain = first.param_range();
    let second_domain = second.param_range();
    if !safe_domain(first_domain) || !safe_domain(second_domain) {
        return false;
    }
    let second_knots: Box<dyn Iterator<Item = &f64>> = match orientation {
        ParameterOrientation::Same => Box::new(second.knots().as_slice().iter()),
        ParameterOrientation::Reversed => Box::new(second.knots().as_slice().iter().rev()),
    };
    first.knots().as_slice().iter().zip(second_knots).all(
        |(&first_parameter, &second_parameter)| {
            exact_normalized_parameters_equal(
                first_parameter,
                first_domain,
                second_parameter,
                second_domain,
                orientation,
            )
        },
    )
}

fn scalar_controls_correspond(
    first: &NurbsCurve,
    second: &NurbsCurve,
    axis: usize,
    orientation: ParameterOrientation,
) -> bool {
    let count = first.points().len();
    (0..count).all(|index| {
        let second_index = corresponding_index(index, count, orientation);
        component(first, index, axis) == component(second, second_index, axis)
    })
}

fn weights_are_globally_proportional(
    first: &NurbsCurve,
    second: &NurbsCurve,
    orientation: ParameterOrientation,
) -> bool {
    let first_weights = first.weights();
    let second_weights = second.weights();
    let first_weight = |index| first_weights.map_or(1.0, |weights| weights[index]);
    let second_weight = |index| second_weights.map_or(1.0, |weights| weights[index]);
    let count = first.points().len();
    let second_base_index = corresponding_index(0, count, orientation);
    let first_base = first_weight(0);
    let second_base = second_weight(second_base_index);
    (0..count).all(|index| {
        let second_index = corresponding_index(index, count, orientation);
        exact_products_equal(
            first_weight(index),
            second_base,
            second_weight(second_index),
            first_base,
        )
    })
}

fn corresponding_index(index: usize, count: usize, orientation: ParameterOrientation) -> usize {
    match orientation {
        ParameterOrientation::Same => index,
        ParameterOrientation::Reversed => count - 1 - index,
    }
}

fn exact_products_equal(first: f64, second: f64, third: f64, fourth: f64) -> bool {
    exact_expansion_products_equal(&[first], &[second], &[third], &[fourth]) == Some(true)
}

fn exact_normalized_parameters_equal(
    first_parameter: f64,
    first_domain: ParamRange,
    second_parameter: f64,
    second_domain: ParamRange,
    orientation: ParameterOrientation,
) -> bool {
    let Some(first_offset) = exact_difference(first_parameter, first_domain.lo) else {
        return false;
    };
    let Some(second_offset) = (match orientation {
        ParameterOrientation::Same => exact_difference(second_parameter, second_domain.lo),
        ParameterOrientation::Reversed => exact_difference(second_domain.hi, second_parameter),
    }) else {
        return false;
    };
    let Some(first_width) = exact_difference(first_domain.hi, first_domain.lo) else {
        return false;
    };
    let Some(second_width) = exact_difference(second_domain.hi, second_domain.lo) else {
        return false;
    };
    exact_expansion_products_equal(&first_offset, &second_width, &second_offset, &first_width)
        == Some(true)
}

fn exact_difference(first: f64, second: f64) -> Option<Vec<f64>> {
    if !first.is_finite() || !second.is_finite() {
        return None;
    }
    let (difference, tail) = expansion::two_diff(first, second);
    let result = expansion::from_two(difference, tail);
    safe_expansion(&result).then_some(result)
}

fn exact_expansion_products_equal(
    first: &[f64],
    second: &[f64],
    third: &[f64],
    fourth: &[f64],
) -> Option<bool> {
    if !safe_product_inputs(first, second) || !safe_product_inputs(third, fourth) {
        return None;
    }
    let left = expansion::mul(first, second);
    let right = expansion::mul(third, fourth);
    if !safe_expansion(&left) || !safe_expansion(&right) {
        return None;
    }
    let difference = expansion::sum(&left, &expansion::negate(&right));
    safe_expansion(&difference).then(|| expansion::sign(&difference) == 0)
}

fn safe_product_inputs(first: &[f64], second: &[f64]) -> bool {
    safe_expansion(first)
        && safe_expansion(second)
        && first.iter().all(|&a| {
            second.iter().all(|&b| {
                a == 0.0
                    || b == 0.0
                    || (a.abs() * b.abs()).is_finite()
                        && (SAFE_PRODUCT_MIN..=SAFE_PRODUCT_MAX).contains(&(a.abs() * b.abs()))
            })
        })
}

fn safe_expansion(values: &[f64]) -> bool {
    !values.is_empty()
        && values.iter().all(|&value| {
            value == 0.0
                || value.is_finite()
                    && (SAFE_COMPONENT_MIN..=SAFE_COMPONENT_MAX).contains(&value.abs())
        })
}

fn safe_domain(domain: ParamRange) -> bool {
    domain.is_finite()
        && domain.width().is_finite()
        && domain.width() > 0.0
        && (SAFE_COMPONENT_MIN..=SAFE_COMPONENT_MAX).contains(&domain.width())
}

fn component(curve: &NurbsCurve, index: usize, axis: usize) -> f64 {
    let point = curve.points()[index];
    match axis {
        0 => point.x,
        1 => point.y,
        2 => point.z,
        _ => unreachable!("coordinate axes are in 0..3"),
    }
}

fn projection_plane(first_axis: usize, second_axis: usize) -> CurvePairProjectionPlane {
    match (first_axis.min(second_axis), first_axis.max(second_axis)) {
        (0, 1) => CurvePairProjectionPlane::Xy,
        (0, 2) => CurvePairProjectionPlane::Xz,
        (1, 2) => CurvePairProjectionPlane::Yz,
        _ => unreachable!("distinct coordinate axes select one projection plane"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vec::Point3;

    #[test]
    fn magnitude_fourteen_form_arithmetic_outside_the_exact_corridor_fails_closed() {
        let curve = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                Point3::new(f64::MAX, 1.0, 0.0),
                Point3::new(f64::MAX, 2.0, 0.0),
            ],
            None,
        )
        .unwrap();
        assert!(exact_linear_form(&curve, 0, [12, 1, 0]).is_none());
        assert!(exact_linear_form(&curve, 0, [13, 0, 0]).is_none());
        assert!(exact_linear_form(&curve, 0, [-13, 0, 0]).is_none());
        assert!(exact_linear_form(&curve, 0, [14, 0, 0]).is_none());
        assert!(exact_linear_form(&curve, 0, [-14, 0, 0]).is_none());
        assert!(linear_form_derivative_interval(&curve, curve.param_range(), [12, 1, 0]).is_none());
        assert!(linear_form_derivative_interval(&curve, curve.param_range(), [13, 0, 0]).is_none());
        assert!(
            linear_form_derivative_interval(&curve, curve.param_range(), [-13, 0, 0]).is_none()
        );
        assert!(linear_form_derivative_interval(&curve, curve.param_range(), [14, 0, 0]).is_none());
        assert!(
            linear_form_derivative_interval(&curve, curve.param_range(), [-14, 0, 0]).is_none()
        );

        let finite = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point3::new(1.0, 2.0, 0.0), Point3::new(2.0, 3.0, 0.0)],
            None,
        )
        .unwrap();
        assert!(exact_linear_form(&finite, 0, [13, -1, 0]).is_some());
        assert!(exact_linear_form(&finite, 0, [-13, 1, 0]).is_some());
        assert!(exact_linear_form(&finite, 0, [14, -1, 0]).is_some());
        assert!(exact_linear_form(&finite, 0, [-14, 1, 0]).is_some());
        assert!(exact_linear_form(&finite, 0, [15, 0, 0]).is_none());
        assert!(exact_linear_form(&finite, 0, [-15, 0, 0]).is_none());
        assert!(
            linear_form_derivative_interval(&finite, finite.param_range(), [13, -1, 0]).is_some()
        );
        assert!(
            linear_form_derivative_interval(&finite, finite.param_range(), [14, -1, 0]).is_some()
        );
        assert!(
            linear_form_derivative_interval(&finite, finite.param_range(), [15, 0, 0]).is_none()
        );
    }

    #[test]
    fn configured_shells_extend_the_compatibility_search_order_by_stable_prefixes() {
        let compatibility =
            configured_primitive_form_bounds(CurvePairAlgebraicSearchConfig::default())
                .collect::<Vec<_>>();
        let magnitude_thirteen =
            configured_primitive_form_bounds(CurvePairAlgebraicSearchConfig::new(13).unwrap())
                .collect::<Vec<_>>();
        let magnitude_fourteen =
            configured_primitive_form_bounds(CurvePairAlgebraicSearchConfig::new(14).unwrap())
                .collect::<Vec<_>>();

        assert_eq!(compatibility, vec![6, 7, 8, 9, 10, 11, 12]);
        assert_eq!(
            &magnitude_thirteen[..compatibility.len()],
            compatibility.as_slice()
        );
        assert_eq!(magnitude_thirteen.last(), Some(&13));
        assert_eq!(
            &magnitude_fourteen[..magnitude_thirteen.len()],
            magnitude_thirteen.as_slice()
        );
        assert_eq!(magnitude_fourteen.last(), Some(&14));
    }

    #[test]
    fn primitive_normalization_removes_sign_and_scale_duplicates() {
        assert!(coefficients_are_primitive([2, 1, 0]));
        assert!(coefficients_are_primitive([3, -2, 0]));
        assert!(coefficients_are_primitive([4, -3, 0]));
        assert!(coefficients_are_primitive([5, -4, 0]));
        assert!(coefficients_are_primitive([6, -5, 0]));
        assert!(coefficients_are_primitive([7, -6, 0]));
        assert!(coefficients_are_primitive([8, -7, 0]));
        assert!(coefficients_are_primitive([9, -8, 0]));
        assert!(coefficients_are_primitive([10, -9, 0]));
        assert!(coefficients_are_primitive([11, -10, 0]));
        assert!(coefficients_are_primitive([12, -11, 0]));
        assert!(coefficients_are_primitive([-1, 0, 2]));
        assert!(!coefficients_are_primitive([2, -2, 0]));
        assert!(!coefficients_are_primitive([0, 0, 0]));

        let compatibility_maximum = i8::try_from(
            CurvePairAlgebraicSearchConfig::default().maximum_primitive_form_coefficient(),
        )
        .unwrap();
        let carrier_count = (1..=compatibility_maximum)
            .flat_map(|first| {
                (-compatibility_maximum..=compatibility_maximum)
                    .map(move |second| [first, second, 0])
            })
            .filter(|coefficients| coefficients[1] != 0)
            .filter(|&coefficients| coefficients_are_primitive(coefficients))
            .count();
        assert_eq!(carrier_count, 182);

        let residual_count = (1..=compatibility_maximum)
            .flat_map(|omitted| {
                (-compatibility_maximum..=compatibility_maximum).flat_map(move |first| {
                    (-compatibility_maximum..=compatibility_maximum)
                        .map(move |second| [first, second, omitted])
                })
            })
            .filter(|&coefficients| coefficients_are_primitive(coefficients))
            .count();
        assert_eq!(residual_count, 6_153);

        let magnitude_thirteen = 13;
        let magnitude_thirteen_carrier_count = (1..=magnitude_thirteen)
            .flat_map(|first| {
                (-magnitude_thirteen..=magnitude_thirteen).map(move |second| [first, second, 0])
            })
            .filter(|coefficients| coefficients[1] != 0)
            .filter(|&coefficients| coefficients_are_primitive(coefficients))
            .count();
        let magnitude_thirteen_residual_count = (1..=magnitude_thirteen)
            .flat_map(|omitted| {
                (-magnitude_thirteen..=magnitude_thirteen).flat_map(move |first| {
                    (-magnitude_thirteen..=magnitude_thirteen)
                        .map(move |second| [first, second, omitted])
                })
            })
            .filter(|&coefficients| coefficients_are_primitive(coefficients))
            .count();
        assert_eq!(magnitude_thirteen_carrier_count, 230);
        assert_eq!(magnitude_thirteen_residual_count, 8_121);
        assert_eq!(magnitude_thirteen_carrier_count - carrier_count, 48);
        assert_eq!(magnitude_thirteen_residual_count - residual_count, 1_968);

        let supported_maximum = MAX_SUPPORTED_PRIMITIVE_FORM_COEFFICIENT;
        let supported_carrier_count = (1..=supported_maximum)
            .flat_map(|first| {
                (-supported_maximum..=supported_maximum).map(move |second| [first, second, 0])
            })
            .filter(|coefficients| coefficients[1] != 0)
            .filter(|&coefficients| coefficients_are_primitive(coefficients))
            .count();
        let supported_residual_count = (1..=supported_maximum)
            .flat_map(|omitted| {
                (-supported_maximum..=supported_maximum).flat_map(move |first| {
                    (-supported_maximum..=supported_maximum)
                        .map(move |second| [first, second, omitted])
                })
            })
            .filter(|&coefficients| coefficients_are_primitive(coefficients))
            .count();
        assert_eq!(supported_carrier_count, 254);
        assert_eq!(supported_residual_count, 9_825);
        assert_eq!(
            supported_carrier_count - magnitude_thirteen_carrier_count,
            24
        );
        assert_eq!(
            supported_residual_count - magnitude_thirteen_residual_count,
            1_704
        );
    }
}
