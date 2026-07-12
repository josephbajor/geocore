use super::parameter::{fit_parameter_pair, validate_curve_surface_ranges};
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint,
    accept_curve_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::Point3;

const MAX_ROOT_DEPTH: usize = 72;
const MAX_CLIP_DEPTH: usize = 72;
const LEAF_SAMPLES: usize = 16;
const COMPLETION_REASON: &str =
    "NURBS/plane Bezier clipping leaf fallbacks do not yet report complete coverage";

fn provisional_result(
    points: Vec<CurveSurfacePoint>,
    overlaps: Vec<CurveSurfaceOverlap>,
) -> Result<CurveSurfaceIntersections> {
    CurveSurfaceIntersections::canonicalized_indeterminate(points, overlaps, COMPLETION_REASON)
}

/// Intersect a clamped NURBS curve restricted to a finite range with a finite
/// plane parameter window.
///
/// Isolated contacts are found by Bezier-span convex-hull clipping against the
/// plane. Spans whose control points lie in the plane become contained
/// overlaps clipped against the plane's `(u, v)` box.
pub fn intersect_bounded_nurbs_plane(
    curve: &NurbsCurve,
    curve_range: ParamRange,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    validate_ranges(curve, curve_range, plane_range, tolerances)?;

    let curve_range = clamp_to_domain(curve_range, curve.param_range());
    let parameter_tol = parameter_tolerance(curve_range, tolerances);
    if curve_range.width() <= parameter_tol {
        return single_parameter_intersection(
            curve,
            curve_range.lo,
            plane,
            plane_range,
            tolerances,
        );
    }

    let bounded = restrict_curve_to_range(curve, curve_range, parameter_tol)?;
    let mut points = Vec::new();
    let mut overlaps = Vec::new();
    for bezier in bounded.to_beziers()? {
        let distances = signed_control_distances(&bezier, plane);
        let (min_distance, max_distance) = min_max(&distances);
        if min_distance > tolerances.linear() || max_distance < -tolerances.linear() {
            continue;
        }
        if distances
            .iter()
            .all(|distance| distance.abs() <= tolerances.linear())
        {
            collect_contained_intervals(
                &bezier,
                plane,
                plane_range,
                tolerances,
                curve_range,
                &mut overlaps,
                0,
            )?;
        } else {
            collect_isolated_roots(
                &bezier,
                plane,
                plane_range,
                tolerances,
                curve_range,
                &mut points,
                0,
            )?;
        }
    }

    merge_overlaps(&mut overlaps, curve_range, tolerances);
    provisional_result(points, overlaps)
}

fn collect_isolated_roots(
    bezier: &NurbsCurve,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
    global_range: ParamRange,
    points: &mut Vec<CurveSurfacePoint>,
    depth: usize,
) -> Result<()> {
    let distances = signed_control_distances(bezier, plane);
    let (min_distance, max_distance) = min_max(&distances);
    if min_distance > tolerances.linear() || max_distance < -tolerances.linear() {
        return Ok(());
    }

    let range = bezier.param_range();
    let f_lo = signed_distance(bezier.eval(range.lo), plane);
    let f_hi = signed_distance(bezier.eval(range.hi), plane);
    push_root_candidate(bezier, range.lo, plane, plane_range, tolerances, points);
    push_root_candidate(bezier, range.hi, plane, plane_range, tolerances, points);

    let variations = sign_variations(&distances, tolerances.linear());
    if variations == 0 {
        return Ok(());
    }
    if variations == 1 && f_lo * f_hi < 0.0 {
        let t = bisect_root(bezier, plane, range.lo, range.hi, f_lo, f_hi, tolerances);
        push_root_candidate(bezier, t, plane, plane_range, tolerances, points);
        return Ok(());
    }

    if range.width() <= parameter_tolerance(global_range, tolerances) || depth >= MAX_ROOT_DEPTH {
        if let Some(t) = best_leaf_root(bezier, plane, range, tolerances) {
            push_root_candidate(bezier, t, plane, plane_range, tolerances, points);
        }
        return Ok(());
    }

    let mid = range.lerp(0.5);
    let (left, right) = bezier.split_at(mid)?;
    collect_isolated_roots(
        &left,
        plane,
        plane_range,
        tolerances,
        global_range,
        points,
        depth + 1,
    )?;
    collect_isolated_roots(
        &right,
        plane,
        plane_range,
        tolerances,
        global_range,
        points,
        depth + 1,
    )
}

fn collect_contained_intervals(
    bezier: &NurbsCurve,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
    global_range: ParamRange,
    overlaps: &mut Vec<CurveSurfaceOverlap>,
    depth: usize,
) -> Result<()> {
    let uv_bounds = control_uv_bounds(bezier, plane);
    if outside_window(uv_bounds, plane_range, tolerances) {
        return Ok(());
    }
    let range = bezier.param_range();
    if inside_window(uv_bounds, plane_range, tolerances) {
        push_contained_overlap(bezier, plane, plane_range, tolerances, overlaps);
        return Ok(());
    }

    if range.width() <= parameter_tolerance(global_range, tolerances) || depth >= MAX_CLIP_DEPTH {
        let mid_uv = plane_uv(bezier.eval(range.lerp(0.5)), plane);
        if fit_parameter_pair(mid_uv, plane_range, tolerances.linear()).is_some() {
            push_contained_overlap(bezier, plane, plane_range, tolerances, overlaps);
        }
        return Ok(());
    }

    let mid = range.lerp(0.5);
    let (left, right) = bezier.split_at(mid)?;
    collect_contained_intervals(
        &left,
        plane,
        plane_range,
        tolerances,
        global_range,
        overlaps,
        depth + 1,
    )?;
    collect_contained_intervals(
        &right,
        plane,
        plane_range,
        tolerances,
        global_range,
        overlaps,
        depth + 1,
    )
}

fn single_parameter_intersection(
    curve: &NurbsCurve,
    t_curve: f64,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    if signed_distance(curve.eval(t_curve), plane).abs() > tolerances.linear() {
        return Ok(CurveSurfaceIntersections::indeterminate_empty(
            COMPLETION_REASON,
        ));
    }
    let Some(uv) = fit_parameter_pair(
        plane_uv(curve.eval(t_curve), plane),
        plane_range,
        tolerances.linear(),
    ) else {
        return Ok(CurveSurfaceIntersections::indeterminate_empty(
            COMPLETION_REASON,
        ));
    };
    let points = accept_curve_surface_candidate(
        curve,
        t_curve,
        plane,
        uv,
        contact_kind(curve, t_curve, plane, tolerances),
        tolerances,
    )
    .into_iter()
    .collect();
    provisional_result(points, Vec::new())
}

fn restrict_curve_to_range(
    curve: &NurbsCurve,
    range: ParamRange,
    parameter_tol: f64,
) -> Result<NurbsCurve> {
    let mut bounded = curve.clone();
    let domain = bounded.param_range();
    if range.lo > domain.lo + parameter_tol {
        bounded = bounded.split_at(range.lo)?.1;
    }
    let domain = bounded.param_range();
    if range.hi < domain.hi - parameter_tol {
        bounded = bounded.split_at(range.hi)?.0;
    }
    Ok(bounded)
}

fn push_root_candidate(
    curve: &NurbsCurve,
    t_curve: f64,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
    points: &mut Vec<CurveSurfacePoint>,
) {
    if signed_distance(curve.eval(t_curve), plane).abs() > tolerances.linear() {
        return;
    }
    let Some(uv) = fit_parameter_pair(
        plane_uv(curve.eval(t_curve), plane),
        plane_range,
        tolerances.linear(),
    ) else {
        return;
    };
    let Some(point) = accept_curve_surface_candidate(
        curve,
        t_curve,
        plane,
        uv,
        contact_kind(curve, t_curve, plane, tolerances),
        tolerances,
    ) else {
        return;
    };
    push_distinct_point(points, point, curve.param_range(), tolerances);
}

fn push_contained_overlap(
    curve: &NurbsCurve,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
    overlaps: &mut Vec<CurveSurfaceOverlap>,
) {
    let range = curve.param_range();
    if range.width() <= parameter_tolerance(range, tolerances) {
        return;
    }
    let Some(uv_start) = fit_parameter_pair(
        plane_uv(curve.eval(range.lo), plane),
        plane_range,
        tolerances.linear(),
    ) else {
        return;
    };
    let Some(uv_end) = fit_parameter_pair(
        plane_uv(curve.eval(range.hi), plane),
        plane_range,
        tolerances.linear(),
    ) else {
        return;
    };
    overlaps.push(CurveSurfaceOverlap {
        curve: range,
        uv_start,
        uv_end,
    });
}

fn bisect_root(
    curve: &NurbsCurve,
    plane: &Plane,
    mut lo: f64,
    mut hi: f64,
    mut f_lo: f64,
    mut f_hi: f64,
    tolerances: Tolerances,
) -> f64 {
    let parameter_tol = parameter_tolerance(curve.param_range(), tolerances);
    for _ in 0..80 {
        let mid = (lo + hi) / 2.0;
        let f_mid = signed_distance(curve.eval(mid), plane);
        if f_mid.abs() <= tolerances.linear() || hi - lo <= parameter_tol {
            return mid;
        }
        if same_sign(f_lo, f_mid) {
            lo = mid;
            f_lo = f_mid;
        } else {
            hi = mid;
            f_hi = f_mid;
        }
        if f_hi.abs() <= tolerances.linear() {
            return hi;
        }
    }
    (lo + hi) / 2.0
}

fn best_leaf_root(
    curve: &NurbsCurve,
    plane: &Plane,
    range: ParamRange,
    tolerances: Tolerances,
) -> Option<f64> {
    let mut best = (f64::INFINITY, range.lo);
    for i in 0..=LEAF_SAMPLES {
        let t = range.lerp(i as f64 / LEAF_SAMPLES as f64);
        let distance = signed_distance(curve.eval(t), plane).abs();
        if distance < best.0 {
            best = (distance, t);
        }
    }
    (best.0 <= tolerances.linear()).then_some(best.1)
}

fn signed_control_distances(curve: &NurbsCurve, plane: &Plane) -> Vec<f64> {
    curve
        .points()
        .iter()
        .map(|&point| signed_distance(point, plane))
        .collect()
}

fn signed_distance(point: Point3, plane: &Plane) -> f64 {
    plane.frame().to_local(point).z
}

fn contact_kind(
    curve: &NurbsCurve,
    t_curve: f64,
    plane: &Plane,
    tolerances: Tolerances,
) -> ContactKind {
    let tangent = curve.eval_derivs(t_curve, 1).d[1];
    let tangent_norm = tangent.norm();
    if tangent_norm <= tolerances.linear() {
        ContactKind::Singular
    } else if tangent.dot(plane.frame().z()).abs() > tangent_norm * tolerances.angular() {
        ContactKind::Transverse
    } else {
        ContactKind::Tangent
    }
}

fn plane_uv(point: Point3, plane: &Plane) -> [f64; 2] {
    let local = plane.frame().to_local(point);
    [local.x, local.y]
}

fn control_uv_bounds(curve: &NurbsCurve, plane: &Plane) -> [[f64; 2]; 2] {
    let mut bounds = [[f64::INFINITY, f64::NEG_INFINITY]; 2];
    for point in curve.points() {
        let uv = plane_uv(*point, plane);
        for axis in 0..2 {
            bounds[axis][0] = bounds[axis][0].min(uv[axis]);
            bounds[axis][1] = bounds[axis][1].max(uv[axis]);
        }
    }
    bounds
}

fn outside_window(
    uv_bounds: [[f64; 2]; 2],
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> bool {
    (0..2).any(|axis| {
        uv_bounds[axis][1] < plane_range[axis].lo - tolerances.linear()
            || uv_bounds[axis][0] > plane_range[axis].hi + tolerances.linear()
    })
}

fn inside_window(
    uv_bounds: [[f64; 2]; 2],
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> bool {
    (0..2).all(|axis| {
        uv_bounds[axis][0] >= plane_range[axis].lo - tolerances.linear()
            && uv_bounds[axis][1] <= plane_range[axis].hi + tolerances.linear()
    })
}

fn min_max(values: &[f64]) -> (f64, f64) {
    values
        .iter()
        .copied()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), value| {
            (min.min(value), max.max(value))
        })
}

fn sign_variations(values: &[f64], tolerance: f64) -> usize {
    let mut previous = None;
    let mut variations = 0;
    for &value in values {
        if value.abs() <= tolerance {
            continue;
        }
        let sign = value.is_sign_positive();
        if previous.is_some_and(|prev| prev != sign) {
            variations += 1;
        }
        previous = Some(sign);
    }
    variations
}

fn same_sign(a: f64, b: f64) -> bool {
    (a < 0.0 && b < 0.0) || (a > 0.0 && b > 0.0)
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

fn validate_ranges(
    curve: &NurbsCurve,
    curve_range: ParamRange,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<()> {
    validate_curve_surface_ranges(
        curve_range,
        plane_range,
        "nurbs/plane intersection requires a finite non-reversed curve range",
        "nurbs/plane intersection requires finite non-reversed surface ranges",
    )?;
    if !curve.knots().is_clamped() {
        return Err(Error::InvalidGeometry {
            reason: "nurbs/plane intersection requires a clamped NURBS curve",
        });
    }
    let domain = curve.param_range();
    let parameter_tol = parameter_tolerance(domain, tolerances);
    if curve_range.lo < domain.lo - parameter_tol || curve_range.hi > domain.hi + parameter_tol {
        return Err(Error::InvalidGeometry {
            reason: "nurbs/plane intersection curve range must lie within the NURBS domain",
        });
    }
    Ok(())
}
