//! Shared SSI emission from one support curve's two curve/surface hit sets.

use super::parameter::fit_scalar_parameter;
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint,
    SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfacePoint,
    accept_surface_surface_candidate,
};
use kcore::tolerance::Tolerances;
use kgeom::param::ParamRange;
use kgeom::surface::Surface;
use kgeom::vec::Point3;

/// Pair-owned data needed to turn two complete curve/surface results into SSI
/// branches and isolated points on their shared support curve.
pub(super) struct SupportCurvePairConfig<'a> {
    pub curve: &'a SurfaceIntersectionCurve,
    pub curve_range: ParamRange,
    pub first_hit: &'a CurveSurfaceIntersections,
    pub second_hit: &'a CurveSurfaceIntersections,
    pub kind: ContactKind,
    pub parameter_tolerance: f64,
    pub parameter_period: Option<f64>,
    pub branch_tolerance: f64,
    pub first_surface: &'a dyn Surface,
    pub second_surface: &'a dyn Surface,
    pub first_uv: &'a dyn Fn(Point3) -> Option<[f64; 2]>,
    pub second_uv: &'a dyn Fn(Point3) -> Option<[f64; 2]>,
    pub tolerances: Tolerances,
}

/// Emit the common positive-length intervals and isolated contacts from two
/// complete curve/surface hit sets.
///
/// This helper owns no completion or accounting policy. Pair solvers retain
/// final result construction, and provisional sub-solvers must not enter this
/// exact analytic path.
pub(super) fn emit_support_curve_pair(
    config: SupportCurvePairConfig<'_>,
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
) {
    debug_assert!(config.first_hit.is_complete());
    debug_assert!(config.second_hit.is_complete());

    for first_overlap in &config.first_hit.overlaps {
        for second_overlap in &config.second_hit.overlaps {
            let lo = first_overlap.curve.lo.max(second_overlap.curve.lo);
            let hi = first_overlap.curve.hi.min(second_overlap.curve.hi);
            if hi - lo > config.parameter_tolerance {
                let Some(uv_a_start) = (config.first_uv)(config.curve.eval(lo)) else {
                    continue;
                };
                let Some(uv_a_end) = (config.first_uv)(config.curve.eval(hi)) else {
                    continue;
                };
                let Some(uv_b_start) = (config.second_uv)(config.curve.eval(lo)) else {
                    continue;
                };
                let Some(uv_b_end) = (config.second_uv)(config.curve.eval(hi)) else {
                    continue;
                };
                emit_curve(
                    curves,
                    SurfaceSurfaceCurve {
                        curve: config.curve.clone(),
                        curve_range: ParamRange::new(lo, hi),
                        uv_a_start,
                        uv_a_end,
                        uv_b_start,
                        uv_b_end,
                        kind: config.kind,
                    },
                    config.branch_tolerance,
                );
            } else if (hi - lo).abs() <= config.parameter_tolerance {
                emit_point_at(
                    &config,
                    points,
                    ((lo + hi) / 2.0)
                        .clamp(config.curve.param_range().lo, config.curve.param_range().hi),
                );
            }
        }
    }

    emit_isolated_points(&config, points);
}

fn emit_isolated_points(
    config: &SupportCurvePairConfig<'_>,
    points: &mut Vec<SurfaceSurfacePoint>,
) {
    for point in &config.first_hit.points {
        if hit_contains_parameter(config, config.second_hit, point.t_curve) {
            emit_point_at(config, points, point.t_curve);
        }
    }
    for point in &config.second_hit.points {
        if hit_contains_parameter(config, config.first_hit, point.t_curve) {
            emit_point_at(config, points, point.t_curve);
        }
    }
    for first_point in &config.first_hit.points {
        for second_point in &config.second_hit.points {
            if curve_parameters_match(config, first_point, second_point) {
                emit_point_at(config, points, first_point.t_curve);
            }
        }
    }
}

fn emit_point_at(
    config: &SupportCurvePairConfig<'_>,
    points: &mut Vec<SurfaceSurfacePoint>,
    parameter: f64,
) {
    let Some(parameter) =
        fit_scalar_parameter(parameter, config.curve_range, config.parameter_tolerance)
    else {
        return;
    };
    let point = config.curve.eval(parameter);
    let Some(uv_a) = (config.first_uv)(point) else {
        return;
    };
    let Some(uv_b) = (config.second_uv)(point) else {
        return;
    };
    if let Some(point) = accept_surface_surface_candidate(
        config.first_surface,
        uv_a,
        config.second_surface,
        uv_b,
        config.kind,
        config.tolerances,
    ) {
        emit_point(points, point, config.tolerances);
    }
}

fn hit_contains_parameter(
    config: &SupportCurvePairConfig<'_>,
    hit: &CurveSurfaceIntersections,
    parameter: f64,
) -> bool {
    hit.overlaps.iter().any(|overlap| {
        overlap_contains_parameter(
            overlap,
            parameter,
            config.parameter_tolerance,
            config.parameter_period,
        )
    }) || hit.points.iter().any(|point| {
        curve_parameter_distance(point.t_curve, parameter, config.parameter_period)
            <= config.parameter_tolerance.max(config.tolerances.angular())
    })
}

fn overlap_contains_parameter(
    overlap: &CurveSurfaceOverlap,
    parameter: f64,
    tolerance: f64,
    period: Option<f64>,
) -> bool {
    let contains = |candidate: f64| {
        candidate >= overlap.curve.lo - tolerance && candidate <= overlap.curve.hi + tolerance
    };
    match period {
        Some(period) => [parameter, parameter - period, parameter + period]
            .into_iter()
            .any(contains),
        None => contains(parameter),
    }
}

fn curve_parameters_match(
    config: &SupportCurvePairConfig<'_>,
    first: &CurveSurfacePoint,
    second: &CurveSurfacePoint,
) -> bool {
    curve_parameter_distance(first.t_curve, second.t_curve, config.parameter_period)
        <= config.parameter_tolerance.max(config.tolerances.angular())
        || first.point.dist(second.point) <= config.tolerances.linear()
}

fn curve_parameter_distance(first: f64, second: f64, period: Option<f64>) -> f64 {
    let difference = (first - second).abs();
    period.map_or(difference, |period| {
        difference.min((period - difference).abs())
    })
}

fn emit_point(
    points: &mut Vec<SurfaceSurfacePoint>,
    candidate: SurfaceSurfacePoint,
    tolerances: Tolerances,
) {
    if !points
        .iter()
        .any(|point| point.point.dist(candidate.point) <= tolerances.linear())
    {
        points.push(candidate);
    }
}

fn emit_curve(
    curves: &mut Vec<SurfaceSurfaceCurve>,
    candidate: SurfaceSurfaceCurve,
    tolerance: f64,
) {
    if !curves.iter().any(|curve| {
        (curve.curve_range.lo - candidate.curve_range.lo).abs() <= tolerance
            && (curve.curve_range.hi - candidate.curve_range.hi).abs() <= tolerance
            && curve
                .curve
                .eval(curve.curve_range.lo)
                .dist(candidate.curve.eval(candidate.curve_range.lo))
                <= tolerance
            && curve
                .curve
                .eval(curve.curve_range.hi)
                .dist(candidate.curve.eval(candidate.curve_range.hi))
                <= tolerance
    }) {
        curves.push(candidate);
    }
}
