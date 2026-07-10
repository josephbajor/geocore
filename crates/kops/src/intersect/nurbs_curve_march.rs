use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint,
    accept_curve_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::surface::Surface;
use kgeom::vec::{Point3, Vec3};

const MIN_STEPS: usize = 96;
const MAX_STEPS: usize = 512;
const MAX_BISECTION_STEPS: usize = 80;
const COMPLETION_REASON: &str =
    "fixed-grid NURBS curve/surface marching does not prove complete coverage";

#[derive(Clone, Copy)]
pub(super) struct CurveMarchConfig<'a> {
    pub curve: &'a NurbsCurve,
    pub curve_range: ParamRange,
    pub surface: &'a dyn Surface,
    pub surface_range: [ParamRange; 2],
    pub tolerances: Tolerances,
    pub signed_distance: &'a dyn Fn(Point3) -> f64,
    pub surface_uv: &'a dyn Fn(Point3) -> Option<[f64; 2]>,
    pub surface_normal: &'a dyn Fn([f64; 2]) -> Option<Vec3>,
    pub finite_curve_range_reason: &'static str,
    pub finite_surface_range_reason: &'static str,
    pub clamped_curve_reason: &'static str,
    pub domain_range_reason: &'static str,
}

#[derive(Debug, Clone, Copy)]
struct Sample {
    t: f64,
    point: Point3,
    distance: f64,
}

fn provisional_result(
    points: Vec<CurveSurfacePoint>,
    overlaps: Vec<CurveSurfaceOverlap>,
) -> Result<CurveSurfaceIntersections> {
    CurveSurfaceIntersections::canonicalized_indeterminate(points, overlaps, COMPLETION_REASON)
}

pub(super) fn march_nurbs_curve_surface_intersection(
    config: CurveMarchConfig<'_>,
) -> Result<CurveSurfaceIntersections> {
    validate_ranges(config)?;

    let curve_range = clamp_to_domain(config.curve_range, config.curve.param_range());
    let parameter_tol = parameter_tolerance(curve_range, config.tolerances);
    if curve_range.width() <= parameter_tol {
        return single_parameter_intersection(config, curve_range.lo);
    }

    let samples = sample_curve(config, curve_range);
    if samples
        .iter()
        .all(|sample| sample.distance.abs() <= config.tolerances.linear())
    {
        return contained_curve_intersections(config, &samples, parameter_tol);
    }

    let mut points = Vec::new();
    if let Some(first) = samples.first()
        && first.distance.abs() <= config.tolerances.linear()
    {
        push_root_candidate(config, first.t, None, &mut points);
    }
    if let Some(last) = samples.last()
        && last.distance.abs() <= config.tolerances.linear()
    {
        push_root_candidate(config, last.t, None, &mut points);
    }
    for pair in samples.windows(2) {
        let [a, b] = pair else {
            continue;
        };
        if same_sign(a.distance, b.distance) {
            continue;
        }
        if a.distance.abs() <= config.tolerances.linear()
            || b.distance.abs() <= config.tolerances.linear()
        {
            continue;
        }
        let root = bisect_root(config, a.t, b.t, a.distance, parameter_tol);
        push_root_candidate(config, root, None, &mut points);
    }
    for triple in samples.windows(3) {
        let [a, b, c] = triple else {
            continue;
        };
        let b_abs = b.distance.abs();
        if b_abs > a.distance.abs() || b_abs > c.distance.abs() {
            continue;
        }
        let root = minimize_abs_distance(config, a.t, c.t, parameter_tol);
        let forced_kind = same_sign(a.distance, c.distance).then_some(ContactKind::Tangent);
        push_root_candidate(config, root, forced_kind, &mut points);
    }

    provisional_result(points, Vec::new())
}

fn single_parameter_intersection(
    config: CurveMarchConfig<'_>,
    t_curve: f64,
) -> Result<CurveSurfaceIntersections> {
    if (config.signed_distance)(config.curve.eval(t_curve)).abs() > config.tolerances.linear() {
        return Ok(CurveSurfaceIntersections::indeterminate_empty(
            COMPLETION_REASON,
        ));
    }
    let mut points = Vec::new();
    push_root_candidate(config, t_curve, None, &mut points);
    provisional_result(points, Vec::new())
}

fn contained_curve_intersections(
    config: CurveMarchConfig<'_>,
    samples: &[Sample],
    parameter_tol: f64,
) -> Result<CurveSurfaceIntersections> {
    let mut overlaps = Vec::new();
    for pair in samples.windows(2) {
        let [a, b] = pair else {
            continue;
        };
        if b.t - a.t <= parameter_tol {
            continue;
        }
        let mid_t = (a.t + b.t) / 2.0;
        let mid = config.curve.eval(mid_t);
        let Some(uv_start) = (config.surface_uv)(a.point) else {
            continue;
        };
        let Some(uv_end) = (config.surface_uv)(b.point) else {
            continue;
        };
        if (config.surface_uv)(mid).is_none() {
            continue;
        }
        overlaps.push(CurveSurfaceOverlap {
            curve: ParamRange::new(a.t, b.t),
            uv_start,
            uv_end,
        });
    }
    merge_overlaps(&mut overlaps, config.curve_range, config.tolerances);
    provisional_result(Vec::new(), overlaps)
}

fn sample_curve(config: CurveMarchConfig<'_>, curve_range: ParamRange) -> Vec<Sample> {
    let span_hint = config
        .curve
        .knots()
        .control_count()
        .saturating_sub(config.curve.degree())
        .max(1);
    let steps = (span_hint * config.curve.degree().max(1) * 32).clamp(MIN_STEPS, MAX_STEPS);
    (0..=steps)
        .map(|i| {
            let t = curve_range.lerp(i as f64 / steps as f64);
            let point = config.curve.eval(t);
            Sample {
                t,
                point,
                distance: (config.signed_distance)(point),
            }
        })
        .collect()
}

fn bisect_root(
    config: CurveMarchConfig<'_>,
    mut lo: f64,
    mut hi: f64,
    mut f_lo: f64,
    parameter_tol: f64,
) -> f64 {
    let mut root = (lo + hi) / 2.0;
    for _ in 0..MAX_BISECTION_STEPS {
        root = (lo + hi) / 2.0;
        let f_mid = (config.signed_distance)(config.curve.eval(root));
        if f_mid.abs() <= config.tolerances.linear() || hi - lo <= parameter_tol {
            break;
        }
        if same_sign(f_lo, f_mid) {
            lo = root;
            f_lo = f_mid;
        } else {
            hi = root;
        }
    }
    root
}

fn minimize_abs_distance(
    config: CurveMarchConfig<'_>,
    mut lo: f64,
    mut hi: f64,
    parameter_tol: f64,
) -> f64 {
    for _ in 0..MAX_BISECTION_STEPS {
        if hi - lo <= parameter_tol {
            break;
        }
        let third = (hi - lo) / 3.0;
        let left = lo + third;
        let right = hi - third;
        let f_left = (config.signed_distance)(config.curve.eval(left)).abs();
        let f_right = (config.signed_distance)(config.curve.eval(right)).abs();
        if f_left <= f_right {
            hi = right;
        } else {
            lo = left;
        }
    }
    (lo + hi) / 2.0
}

fn push_root_candidate(
    config: CurveMarchConfig<'_>,
    t_curve: f64,
    forced_kind: Option<ContactKind>,
    points: &mut Vec<CurveSurfacePoint>,
) {
    let point = config.curve.eval(t_curve);
    if (config.signed_distance)(point).abs() > config.tolerances.linear() {
        return;
    }
    let Some(uv) = (config.surface_uv)(point) else {
        return;
    };
    let Some(point) = accept_curve_surface_candidate(
        config.curve,
        t_curve,
        config.surface,
        uv,
        forced_kind
            .map(|kind| forced_contact_kind(config, t_curve, uv, kind))
            .unwrap_or_else(|| contact_kind(config, t_curve, uv)),
        config.tolerances,
    ) else {
        return;
    };
    push_distinct_point(points, point, config.curve_range, config.tolerances);
}

fn forced_contact_kind(
    config: CurveMarchConfig<'_>,
    t_curve: f64,
    uv: [f64; 2],
    kind: ContactKind,
) -> ContactKind {
    let tangent = config.curve.eval_derivs(t_curve, 1).d[1];
    if tangent.norm() <= config.tolerances.linear() || (config.surface_normal)(uv).is_none() {
        ContactKind::Singular
    } else {
        kind
    }
}

fn contact_kind(config: CurveMarchConfig<'_>, t_curve: f64, uv: [f64; 2]) -> ContactKind {
    let tangent = config.curve.eval_derivs(t_curve, 1).d[1];
    let tangent_norm = tangent.norm();
    let Some(normal) = (config.surface_normal)(uv) else {
        return ContactKind::Singular;
    };
    if tangent_norm <= config.tolerances.linear() {
        ContactKind::Singular
    } else if tangent.dot(normal).abs() > tangent_norm * config.tolerances.angular() {
        ContactKind::Transverse
    } else {
        ContactKind::Tangent
    }
}

fn push_distinct_point(
    points: &mut Vec<CurveSurfacePoint>,
    candidate: CurveSurfacePoint,
    range: ParamRange,
    tolerances: Tolerances,
) {
    let parameter_tol = parameter_tolerance(range, tolerances);
    if !points.iter().any(|point| {
        (point.t_curve - candidate.t_curve).abs() <= parameter_tol
            || point.point.dist(candidate.point) <= tolerances.linear()
    }) {
        points.push(candidate);
    }
}

fn merge_overlaps(
    overlaps: &mut Vec<CurveSurfaceOverlap>,
    global_range: ParamRange,
    tolerances: Tolerances,
) {
    overlaps.sort_by(|a, b| a.curve.lo.total_cmp(&b.curve.lo));
    let parameter_tol = parameter_tolerance(global_range, tolerances);
    let mut merged: Vec<CurveSurfaceOverlap> = Vec::new();
    for overlap in overlaps.drain(..) {
        if let Some(last) = merged.last_mut()
            && overlap.curve.lo <= last.curve.hi + parameter_tol
        {
            last.curve = ParamRange::new(last.curve.lo, last.curve.hi.max(overlap.curve.hi));
            last.uv_end = overlap.uv_end;
            continue;
        }
        merged.push(overlap);
    }
    *overlaps = merged;
}

fn same_sign(a: f64, b: f64) -> bool {
    (a < 0.0 && b < 0.0) || (a > 0.0 && b > 0.0)
}

fn clamp_to_domain(range: ParamRange, domain: ParamRange) -> ParamRange {
    ParamRange::new(
        range.lo.clamp(domain.lo, domain.hi),
        range.hi.clamp(domain.lo, domain.hi),
    )
}

fn parameter_tolerance(range: ParamRange, tolerances: Tolerances) -> f64 {
    (range.width().abs() * 1e-10)
        .max(tolerances.angular())
        .max(1e-12)
}

fn validate_ranges(config: CurveMarchConfig<'_>) -> Result<()> {
    if !config.curve_range.is_finite() || config.curve_range.width() < 0.0 {
        return Err(Error::InvalidGeometry {
            reason: config.finite_curve_range_reason,
        });
    }
    if config
        .surface_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: config.finite_surface_range_reason,
        });
    }
    if !config.curve.knots().is_clamped() {
        return Err(Error::InvalidGeometry {
            reason: config.clamped_curve_reason,
        });
    }
    let domain = config.curve.param_range();
    let parameter_tol = parameter_tolerance(domain, config.tolerances);
    if config.curve_range.lo < domain.lo - parameter_tol
        || config.curve_range.hi > domain.hi + parameter_tol
    {
        return Err(Error::InvalidGeometry {
            reason: config.domain_range_reason,
        });
    }
    Ok(())
}
