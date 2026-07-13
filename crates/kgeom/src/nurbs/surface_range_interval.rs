//! Source-representation interval bounds over finite NURBS surface rectangles.

use super::NurbsSurface;
use crate::aabb::Aabb3;
use crate::param::ParamRange;
use crate::surface::Dir;
use crate::vec::Vec3;
use kcore::interval::Interval;

/// Original-source point and first-partial enclosure over one parameter box.
///
/// The point enclosure is evaluated at [`Self::center`]. The partials enclose
/// the complete requested box and therefore support a centered mean-value
/// proof without constructing a restricted or refined control net.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NurbsSurfaceSourceDifferentialEnclosure {
    center: [f64; 2],
    position: [Interval; 3],
    derivative_u: [Interval; 3],
    derivative_v: [Interval; 3],
}

impl NurbsSurfaceSourceDifferentialEnclosure {
    /// Parameter-space center at which the point enclosure was evaluated.
    pub const fn center(self) -> [f64; 2] {
        self.center
    }

    /// Outward point-coordinate intervals at [`Self::center`].
    pub const fn position(self) -> [Interval; 3] {
        self.position
    }

    /// Outward first-partial coordinate intervals over the complete box.
    pub const fn derivative_u(self) -> [Interval; 3] {
        self.derivative_u
    }

    /// Outward first-partial coordinate intervals over the complete box.
    pub const fn derivative_v(self) -> [Interval; 3] {
        self.derivative_v
    }
}

/// Conservative number of original-source tensor span slots inspected by one
/// position-range enclosure.
///
/// Let `R = (nu - pu) * (nv - pv)`, including repeated/empty span slots. One
/// enclosure admits the support scan (`R`), direct position scan (`R`), the
/// centered point rectangle (`1`), and both derivative scans, each of which
/// evaluates source position and one derivative rectangle (`2R + 2R`). Thus
/// the deterministic preflight cost is `6R + 1` Work units. Charging all slots
/// is conservative for partial ranges and inconclusive arithmetic.
pub(super) fn position_range_work_units(surface: &NurbsSurface) -> Option<usize> {
    source_tensor_span_slots(surface)?
        .checked_mul(6)?
        .checked_add(1)
}

/// Conservative work admitted for one original-source differential enclosure.
///
/// This intentionally uses the existing position-range upper bound, which
/// includes more source-span scans than the narrower differential accessor.
pub fn source_differential_enclosure_work_units(surface: &NurbsSurface) -> Option<usize> {
    position_range_work_units(surface)
}

/// Enclose a source NURBS point and both first partials without sampling.
///
/// `range` may collapse in either direction, which is required for a trace
/// whose affine pcurve holds one surface coordinate constant. `center` must
/// lie in the closed box. Every interval is evaluated from the original knot
/// vectors and homogeneous controls with outward-rounded arithmetic.
pub fn source_differential_enclosure(
    surface: &NurbsSurface,
    range: [ParamRange; 2],
    center: [f64; 2],
) -> Option<NurbsSurfaceSourceDifferentialEnclosure> {
    let domains = [
        surface.knots(Dir::U).domain(),
        surface.knots(Dir::V).domain(),
    ];
    if range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
        || center.iter().any(|value| !value.is_finite())
        || (0..2).any(|axis| {
            range[axis].lo < domains[axis].lo
                || range[axis].hi > domains[axis].hi
                || !range[axis].contains(center[axis])
        })
    {
        return None;
    }
    let position = position_components_at(surface, center)?;
    let derivative_u = derivative_component_intervals(surface, range, Dir::U)?;
    let derivative_v = derivative_component_intervals(surface, range, Dir::V)?;
    Some(NurbsSurfaceSourceDifferentialEnclosure {
        center,
        position,
        derivative_u,
        derivative_v,
    })
}

/// Number of source tensor span slots, including repeated/empty slots.
pub(super) fn source_tensor_span_slots(surface: &NurbsSurface) -> Option<usize> {
    let (control_count_u, control_count_v) = surface.net_size();
    control_count_u
        .checked_sub(surface.degree_u())?
        .checked_mul(control_count_v.checked_sub(surface.degree_v())?)
}

/// Conservatively enclose source-surface positions over a closed parameter
/// rectangle without constructing a restricted or subdivided control net.
///
/// Every overlapping source knot-span rectangle is evaluated directly with
/// interval de Boor arithmetic in homogeneous coordinates. If any arithmetic
/// is inconclusive, the active original-control support hull is retained so
/// callers fail open rather than exclude source geometry.
pub(super) fn position_range_aabb(surface: &NurbsSurface, range: [ParamRange; 2]) -> Aabb3 {
    let source_hull =
        source_support_hull(surface, range).unwrap_or_else(|| Aabb3::from_points(surface.points()));
    let Some(direct) = position_component_intervals(surface, range) else {
        return source_hull;
    };
    let mut bounded = intersect(source_hull, component_box(direct));
    if let Some(centered) = centered_mean_value_components(surface, range) {
        bounded = intersect(bounded, component_box(centered));
    }
    if bounded.is_empty() {
        source_hull
    } else {
        bounded
    }
}

fn component_box([x, y, z]: [Interval; 3]) -> Aabb3 {
    Aabb3 {
        min: Vec3::new(x.lo(), y.lo(), z.lo()),
        max: Vec3::new(x.hi(), y.hi(), z.hi()),
    }
}

fn intersect(first: Aabb3, second: Aabb3) -> Aabb3 {
    Aabb3 {
        min: first.min.max(second.min),
        max: first.max.min(second.max),
    }
}

fn source_support_hull(surface: &NurbsSurface, range: [ParamRange; 2]) -> Option<Aabb3> {
    let domains = [
        surface.knots(Dir::U).domain(),
        surface.knots(Dir::V).domain(),
    ];
    if range
        .iter()
        .any(|range| !range.is_finite() || range.width() <= 0.0)
        || (0..2).any(|axis| range[axis].lo < domains[axis].lo || range[axis].hi > domains[axis].hi)
    {
        return None;
    }

    let knots_u = surface.knots(Dir::U).as_slice();
    let knots_v = surface.knots(Dir::V).as_slice();
    let degree_u = surface.degree_u();
    let degree_v = surface.degree_v();
    let (control_count_u, control_count_v) = surface.net_size();
    let mut hull = Aabb3::empty();
    for span_u in degree_u..control_count_u {
        if knots_u[span_u] >= knots_u[span_u + 1]
            || range[0].lo.max(knots_u[span_u]) >= range[0].hi.min(knots_u[span_u + 1])
        {
            continue;
        }
        for span_v in degree_v..control_count_v {
            if knots_v[span_v] >= knots_v[span_v + 1]
                || range[1].lo.max(knots_v[span_v]) >= range[1].hi.min(knots_v[span_v + 1])
            {
                continue;
            }
            for control_u in span_u - degree_u..=span_u {
                let start = control_u * control_count_v + span_v - degree_v;
                let end = control_u * control_count_v + span_v + 1;
                hull = hull.union(Aabb3::from_points(surface.points().get(start..end)?));
            }
        }
    }
    (!hull.is_empty()).then_some(hull)
}

fn position_component_intervals(
    surface: &NurbsSurface,
    range: [ParamRange; 2],
) -> Option<[Interval; 3]> {
    let domains = [
        surface.knots(Dir::U).domain(),
        surface.knots(Dir::V).domain(),
    ];
    if range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
        || (0..2).any(|axis| range[axis].lo < domains[axis].lo || range[axis].hi > domains[axis].hi)
    {
        return None;
    }

    let controls = homogeneous_controls(surface)?;
    let knots_u = surface.knots(Dir::U).as_slice();
    let knots_v = surface.knots(Dir::V).as_slice();
    let degree_u = surface.degree_u();
    let degree_v = surface.degree_v();
    let (control_count_u, control_count_v) = surface.net_size();
    let mut result: Option<[Interval; 3]> = None;

    for span_u in degree_u..control_count_u {
        if knots_u[span_u] >= knots_u[span_u + 1] {
            continue;
        }
        let Some((local_u_lo, local_u_hi)) =
            local_span_range(range[0], knots_u[span_u], knots_u[span_u + 1])
        else {
            continue;
        };
        for span_v in degree_v..control_count_v {
            if knots_v[span_v] >= knots_v[span_v + 1] {
                continue;
            }
            let Some((local_v_lo, local_v_hi)) =
                local_span_range(range[1], knots_v[span_v], knots_v[span_v + 1])
            else {
                continue;
            };

            let position = interval_surface_de_boor(
                knots_u,
                degree_u,
                span_u,
                Interval::new(local_u_lo, local_u_hi),
                knots_v,
                degree_v,
                span_v,
                Interval::new(local_v_lo, local_v_hi),
                &controls,
                control_count_v,
            )?;
            let components = if surface.weights().is_none() {
                [position[0], position[1], position[2]]
            } else {
                if position[3].lo() <= 0.0 {
                    return None;
                }
                [
                    position[0].checked_div(position[3])?,
                    position[1].checked_div(position[3])?,
                    position[2].checked_div(position[3])?,
                ]
            };
            if !components.iter().copied().all(finite) {
                return None;
            }
            result = Some(match result {
                Some(current) => core::array::from_fn(|axis| hull(current[axis], components[axis])),
                None => components,
            });
        }
    }
    result
}

/// Centered mean-value form. Unlike direct interval de Boor evaluation, its
/// over-estimation contracts quadratically near a stationary point, which is
/// essential for bounded adaptive exclusion of shallow tangencies.
fn centered_mean_value_components(
    surface: &NurbsSurface,
    range: [ParamRange; 2],
) -> Option<[Interval; 3]> {
    if range
        .iter()
        .any(|range| !range.is_finite() || range.width() <= 0.0)
    {
        return None;
    }
    let midpoint = [
        range[0].lo + 0.5 * range[0].width(),
        range[1].lo + 0.5 * range[1].width(),
    ];
    if !(range[0].contains(midpoint[0]) && range[1].contains(midpoint[1])) {
        return None;
    }
    let center = position_components_at(surface, midpoint)?;
    let derivative_u = derivative_component_intervals(surface, range, Dir::U)?;
    let derivative_v = derivative_component_intervals(surface, range, Dir::V)?;
    let delta_u = Interval::new(range[0].lo, range[0].hi) - Interval::point(midpoint[0]);
    let delta_v = Interval::new(range[1].lo, range[1].hi) - Interval::point(midpoint[1]);
    let result = core::array::from_fn(|axis| {
        center[axis] + derivative_u[axis] * delta_u + derivative_v[axis] * delta_v
    });
    result.iter().copied().all(finite).then_some(result)
}

fn position_components_at(surface: &NurbsSurface, parameter: [f64; 2]) -> Option<[Interval; 3]> {
    let knots_u = surface.knots(Dir::U);
    let knots_v = surface.knots(Dir::V);
    let controls = homogeneous_controls(surface)?;
    let position = interval_surface_de_boor(
        knots_u.as_slice(),
        surface.degree_u(),
        knots_u.find_span(parameter[0]),
        Interval::point(parameter[0]),
        knots_v.as_slice(),
        surface.degree_v(),
        knots_v.find_span(parameter[1]),
        Interval::point(parameter[1]),
        &controls,
        surface.net_size().1,
    )?;
    euclidean_components(position, surface.weights().is_some())
}

fn derivative_component_intervals(
    surface: &NurbsSurface,
    range: [ParamRange; 2],
    direction: Dir,
) -> Option<[Interval; 3]> {
    let degree_u = surface.degree_u();
    let degree_v = surface.degree_v();
    if (direction == Dir::U && degree_u == 0) || (direction == Dir::V && degree_v == 0) {
        return Some([Interval::point(0.0); 3]);
    }
    let knots_u = surface.knots(Dir::U).as_slice();
    let knots_v = surface.knots(Dir::V).as_slice();
    let (control_count_u, control_count_v) = surface.net_size();
    let homogeneous = homogeneous_controls(surface)?;
    let (derivative, derivative_count_v) =
        homogeneous_derivative_controls(surface, &homogeneous, direction)?;
    let mut result: Option<[Interval; 3]> = None;

    for span_u in degree_u..control_count_u {
        if knots_u[span_u] >= knots_u[span_u + 1] {
            continue;
        }
        let Some((local_u_lo, local_u_hi)) =
            local_span_range(range[0], knots_u[span_u], knots_u[span_u + 1])
        else {
            continue;
        };
        for span_v in degree_v..control_count_v {
            if knots_v[span_v] >= knots_v[span_v + 1] {
                continue;
            }
            let Some((local_v_lo, local_v_hi)) =
                local_span_range(range[1], knots_v[span_v], knots_v[span_v + 1])
            else {
                continue;
            };
            let parameter_u = Interval::new(local_u_lo, local_u_hi);
            let parameter_v = Interval::new(local_v_lo, local_v_hi);
            let position = interval_surface_de_boor(
                knots_u,
                degree_u,
                span_u,
                parameter_u,
                knots_v,
                degree_v,
                span_v,
                parameter_v,
                &homogeneous,
                control_count_v,
            )?;
            let homogeneous_derivative = match direction {
                Dir::U => interval_surface_de_boor(
                    &knots_u[1..knots_u.len() - 1],
                    degree_u - 1,
                    span_u - 1,
                    parameter_u,
                    knots_v,
                    degree_v,
                    span_v,
                    parameter_v,
                    &derivative,
                    derivative_count_v,
                )?,
                Dir::V => interval_surface_de_boor(
                    knots_u,
                    degree_u,
                    span_u,
                    parameter_u,
                    &knots_v[1..knots_v.len() - 1],
                    degree_v - 1,
                    span_v - 1,
                    parameter_v,
                    &derivative,
                    derivative_count_v,
                )?,
            };
            let components = if surface.weights().is_none() {
                [
                    homogeneous_derivative[0],
                    homogeneous_derivative[1],
                    homogeneous_derivative[2],
                ]
            } else {
                let weight = position[3];
                if weight.lo() <= 0.0 {
                    return None;
                }
                let weight_squared = weight.square();
                let mut components = [Interval::point(0.0); 3];
                for axis in 0..3 {
                    components[axis] = (homogeneous_derivative[axis] * weight
                        - position[axis] * homogeneous_derivative[3])
                        .checked_div(weight_squared)?;
                }
                components
            };
            if !components.iter().copied().all(finite) {
                return None;
            }
            result = Some(match result {
                Some(current) => core::array::from_fn(|axis| hull(current[axis], components[axis])),
                None => components,
            });
        }
    }
    result
}

#[allow(clippy::too_many_arguments)]
fn interval_surface_de_boor(
    knots_u: &[f64],
    degree_u: usize,
    span_u: usize,
    parameter_u: Interval,
    knots_v: &[f64],
    degree_v: usize,
    span_v: usize,
    parameter_v: Interval,
    controls: &[[Interval; 4]],
    control_count_v: usize,
) -> Option<[Interval; 4]> {
    let base_u = span_u.checked_sub(degree_u)?;
    let base_v = span_v.checked_sub(degree_v)?;
    let mut evaluated_u = Vec::with_capacity(degree_v + 1);
    for control_v in base_v..=span_v {
        let local_u = (base_u..=span_u)
            .map(|control_u| {
                controls
                    .get(control_u * control_count_v + control_v)
                    .copied()
            })
            .collect::<Option<Vec<_>>>()?;
        evaluated_u.push(interval_de_boor_local(
            knots_u,
            degree_u,
            span_u,
            parameter_u,
            &local_u,
        )?);
    }
    interval_de_boor_local(knots_v, degree_v, span_v, parameter_v, &evaluated_u)
}

fn interval_de_boor_local(
    knots: &[f64],
    degree: usize,
    span: usize,
    parameter: Interval,
    local_controls: &[[Interval; 4]],
) -> Option<[Interval; 4]> {
    if local_controls.len() != degree + 1 {
        return None;
    }
    let base = span.checked_sub(degree)?;
    let mut work = local_controls.to_vec();
    for level in 1..=degree {
        for local in (level..=degree).rev() {
            let control_index = base + local;
            let denominator = Interval::point(knots[control_index + degree - level + 1])
                - Interval::point(knots[control_index]);
            let alpha =
                (parameter - Interval::point(knots[control_index])).checked_div(denominator)?;
            // The exact alpha lies in [0, 1] on this fixed nonempty source
            // span. Intersect only outward-rounding spill.
            let alpha_lo = alpha.lo().max(0.0);
            let alpha_hi = alpha.hi().min(1.0);
            if alpha_lo > alpha_hi {
                return None;
            }
            let alpha = Interval::new(alpha_lo, alpha_hi);
            let blended = core::array::from_fn(|axis| {
                interval_blend(work[local - 1][axis], work[local][axis], alpha)
            });
            if !blended.iter().copied().all(finite) {
                return None;
            }
            work[local] = blended;
        }
    }
    work.get(degree).copied()
}

fn homogeneous_controls(surface: &NurbsSurface) -> Option<Vec<[Interval; 4]>> {
    surface
        .points()
        .iter()
        .enumerate()
        .map(|(index, point)| {
            let weight = surface.weights().map_or(1.0, |weights| weights[index]);
            let weight = Interval::point(weight);
            let control = [
                Interval::point(point.x) * weight,
                Interval::point(point.y) * weight,
                Interval::point(point.z) * weight,
                weight,
            ];
            control.iter().copied().all(finite).then_some(control)
        })
        .collect()
}

fn homogeneous_derivative_controls(
    surface: &NurbsSurface,
    controls: &[[Interval; 4]],
    direction: Dir,
) -> Option<(Vec<[Interval; 4]>, usize)> {
    let (control_count_u, control_count_v) = surface.net_size();
    match direction {
        Dir::U => {
            let degree = surface.degree_u();
            let knots = surface.knots(Dir::U).as_slice();
            let mut derivative = Vec::with_capacity((control_count_u - 1) * control_count_v);
            for control_u in 0..control_count_u - 1 {
                let denominator = Interval::point(knots[control_u + degree + 1])
                    - Interval::point(knots[control_u + 1]);
                let scale = Interval::point(degree as f64).checked_div(denominator)?;
                for control_v in 0..control_count_v {
                    let first = controls[control_u * control_count_v + control_v];
                    let second = controls[(control_u + 1) * control_count_v + control_v];
                    let value = core::array::from_fn(|axis| scale * (second[axis] - first[axis]));
                    if !value.iter().copied().all(finite) {
                        return None;
                    }
                    derivative.push(value);
                }
            }
            Some((derivative, control_count_v))
        }
        Dir::V => {
            let degree = surface.degree_v();
            let knots = surface.knots(Dir::V).as_slice();
            let derivative_count_v = control_count_v - 1;
            let mut derivative = Vec::with_capacity(control_count_u * derivative_count_v);
            for control_u in 0..control_count_u {
                for control_v in 0..derivative_count_v {
                    let denominator = Interval::point(knots[control_v + degree + 1])
                        - Interval::point(knots[control_v + 1]);
                    let scale = Interval::point(degree as f64).checked_div(denominator)?;
                    let first = controls[control_u * control_count_v + control_v];
                    let second = controls[control_u * control_count_v + control_v + 1];
                    let value = core::array::from_fn(|axis| scale * (second[axis] - first[axis]));
                    if !value.iter().copied().all(finite) {
                        return None;
                    }
                    derivative.push(value);
                }
            }
            Some((derivative, derivative_count_v))
        }
    }
}

fn euclidean_components(homogeneous: [Interval; 4], rational: bool) -> Option<[Interval; 3]> {
    if !rational {
        return Some([homogeneous[0], homogeneous[1], homogeneous[2]]);
    }
    if homogeneous[3].lo() <= 0.0 {
        return None;
    }
    Some([
        homogeneous[0].checked_div(homogeneous[3])?,
        homogeneous[1].checked_div(homogeneous[3])?,
        homogeneous[2].checked_div(homogeneous[3])?,
    ])
}

fn interval_blend(first: Interval, second: Interval, alpha: Interval) -> Interval {
    let blend_at = |parameter| {
        let parameter = Interval::point(parameter);
        (Interval::point(1.0) - parameter) * first + parameter * second
    };
    let low = blend_at(alpha.lo());
    let high = blend_at(alpha.hi());
    Interval::new(low.lo().min(high.lo()), low.hi().max(high.hi()))
}

fn hull(first: Interval, second: Interval) -> Interval {
    Interval::new(first.lo().min(second.lo()), first.hi().max(second.hi()))
}

fn local_span_range(range: ParamRange, span_lo: f64, span_hi: f64) -> Option<(f64, f64)> {
    let local_lo = range.lo.max(span_lo);
    let local_hi = range.hi.min(span_hi);
    // Positive-width boxes retain the original strict-overlap rule so a
    // boundary-only neighboring span cannot widen ordinary BVH bounds. A
    // genuinely collapsed trace coordinate admits its closed point span(s).
    (local_lo < local_hi || (range.width() == 0.0 && local_lo == local_hi))
        .then_some((local_lo, local_hi))
}

fn finite(interval: Interval) -> bool {
    interval.lo().is_finite() && interval.hi().is_finite()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::surface::Surface;
    use crate::vec::Point3;

    fn test_surface(weights: Option<Vec<f64>>) -> NurbsSurface {
        let mut points = Vec::new();
        for u in 0..4 {
            for v in 0..3 {
                points.push(Point3::new(
                    f64::from(u),
                    f64::from(v),
                    f64::from((u * 5 + v * 3) % 7) - 3.0,
                ));
            }
        }
        NurbsSurface::new(
            2,
            2,
            vec![0.0, 0.0, 0.0, 0.4, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            points,
            weights,
        )
        .unwrap()
    }

    #[test]
    fn source_rectangle_bounds_enclose_polynomial_and_rational_samples() {
        for surface in [
            test_surface(None),
            test_surface(Some(
                (0..12).map(|index| 0.5 + f64::from(index) / 10.0).collect(),
            )),
        ] {
            let range = [ParamRange::new(0.23, 0.71), ParamRange::new(0.17, 0.83)];
            let bounds = position_range_aabb(&surface, range);
            for i in 0..=20 {
                for j in 0..=20 {
                    let uv = [
                        range[0].lerp(f64::from(i) / 20.0),
                        range[1].lerp(f64::from(j) / 20.0),
                    ];
                    assert!(bounds.contains(surface.eval(uv)));
                }
            }
        }
    }

    #[test]
    fn source_derivative_intervals_enclose_polynomial_and_rational_samples() {
        for surface in [
            test_surface(None),
            test_surface(Some(
                (0..12).map(|index| 0.5 + f64::from(index) / 10.0).collect(),
            )),
        ] {
            let range = [ParamRange::new(0.23, 0.71), ParamRange::new(0.17, 0.83)];
            let bounds_u = derivative_component_intervals(&surface, range, Dir::U).unwrap();
            let bounds_v = derivative_component_intervals(&surface, range, Dir::V).unwrap();
            for i in 0..=20 {
                for j in 0..=20 {
                    let uv = [
                        range[0].lerp(f64::from(i) / 20.0),
                        range[1].lerp(f64::from(j) / 20.0),
                    ];
                    let derivatives = surface.eval_derivs(uv, 1);
                    for (bounds, value) in [(bounds_u, derivatives.du), (bounds_v, derivatives.dv)]
                    {
                        assert!(bounds[0].contains(value.x));
                        assert!(bounds[1].contains(value.y));
                        assert!(bounds[2].contains(value.z));
                    }
                }
            }
        }
    }

    #[test]
    fn public_source_differential_enclosure_supports_constant_trace_coordinates() {
        for surface in [
            test_surface(None),
            test_surface(Some(
                (0..12).map(|index| 0.5 + f64::from(index) / 10.0).collect(),
            )),
        ] {
            let range = [ParamRange::new(0.23, 0.71), ParamRange::new(0.4, 0.4)];
            let center = [0.47, 0.4];
            let enclosure = surface
                .source_differential_enclosure(range, center)
                .unwrap();
            assert_eq!(enclosure.center(), center);
            let point = surface.eval(center);
            for (interval, value) in enclosure.position().into_iter().zip(point.to_array()) {
                assert!(interval.contains(value));
            }
            for i in 0..=20 {
                let uv = [range[0].lerp(f64::from(i) / 20.0), range[1].lo];
                let derivatives = surface.eval_derivs(uv, 1);
                for (bounds, value) in [
                    (enclosure.derivative_u(), derivatives.du),
                    (enclosure.derivative_v(), derivatives.dv),
                ] {
                    for (interval, component) in bounds.into_iter().zip(value.to_array()) {
                        assert!(interval.contains(component));
                    }
                }
            }
            assert_eq!(surface.source_differential_enclosure_work_units(), Some(13));
        }
    }

    #[test]
    fn inconclusive_homogeneous_arithmetic_fails_open_to_source_support() {
        let huge = f64::MAX / 1.5;
        let points = vec![
            Point3::new(huge, 0.0, 0.0),
            Point3::new(huge, 1.0, 0.0),
            Point3::new(huge, 0.0, 1.0),
            Point3::new(huge, 1.0, 1.0),
        ];
        let surface = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            points.clone(),
            Some(vec![2.0; 4]),
        )
        .unwrap();
        assert_eq!(
            position_range_aabb(&surface, surface.param_range()),
            Aabb3::from_points(&points)
        );
    }
}
