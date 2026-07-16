use super::parameter::{fit_parameter_pair, validate_curve_surface_ranges};
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint,
    accept_curve_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::predicates::{Orientation, affine_dot3};
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::nurbs::{
    NurbsCurve, PlaneCurveRangeRelation, classify_curve_range_against_affine_band,
    classify_curve_range_against_plane_slab,
};
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::Point3;

const MAX_ROOT_DEPTH: usize = 72;
const MAX_ROOT_NODES: usize = 65_536;
const MAX_CLIP_DEPTH: usize = 72;
const MAX_CLIP_NODES: usize = 65_536;
const LEAF_SAMPLES: usize = 16;
const COMPLETION_REASON: &str =
    "NURBS/plane source-range depth, node, and leaf limits leave coverage incomplete";

fn provisional_result(
    points: Vec<CurveSurfacePoint>,
    overlaps: Vec<CurveSurfaceOverlap>,
) -> Result<CurveSurfaceIntersections> {
    CurveSurfaceIntersections::canonicalized_indeterminate(points, overlaps, COMPLETION_REASON)
}

#[derive(Clone, Copy)]
struct NurbsPlaneProblem<'a> {
    source: &'a NurbsCurve,
    plane: &'a Plane,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
    global_range: ParamRange,
}

/// Intersect a clamped NURBS curve restricted to a finite range with a finite
/// plane parameter window.
///
/// Original-source homogeneous affine range proofs are the only authority for
/// excluding a plane side, declaring a tolerance-slab overlap, or accepting a
/// complete source subrange inside the plane's `(u, v)` window. Rounded
/// restricted, Bezier, and split controls guide parameter subdivision and
/// numeric sign variation only. Provisional isolated contacts are evaluated on
/// the original source and admitted with exact affine slab signs.
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
    let mut remaining_root_nodes = MAX_ROOT_NODES;
    let mut remaining_clip_nodes = MAX_CLIP_NODES;
    let problem = NurbsPlaneProblem {
        source: curve,
        plane,
        plane_range,
        tolerances,
        global_range: curve_range,
    };
    for bezier in bounded.to_beziers()? {
        match source_plane_relation(curve, bezier.param_range(), plane, tolerances) {
            PlaneCurveRangeRelation::Negative | PlaneCurveRangeRelation::Positive => {}
            PlaneCurveRangeRelation::WithinSlab => {
                collect_contained_intervals(
                    &bezier,
                    problem,
                    &mut overlaps,
                    &mut remaining_clip_nodes,
                    0,
                )?;
            }
            PlaneCurveRangeRelation::Candidate => {
                collect_isolated_roots(
                    &bezier,
                    problem,
                    &mut points,
                    &mut remaining_root_nodes,
                    0,
                )?;
            }
        }
    }

    merge_overlaps(&mut overlaps);
    provisional_result(points, overlaps)
}

fn collect_isolated_roots(
    numeric: &NurbsCurve,
    problem: NurbsPlaneProblem<'_>,
    points: &mut Vec<CurveSurfacePoint>,
    remaining_nodes: &mut usize,
    depth: usize,
) -> Result<()> {
    if *remaining_nodes == 0 {
        return Ok(());
    }
    *remaining_nodes -= 1;

    let range = numeric.param_range();
    let endpoint_hit = push_root_candidate(
        problem.source,
        range.lo,
        problem.plane,
        problem.plane_range,
        problem.tolerances,
        points,
    ) | push_root_candidate(
        problem.source,
        range.hi,
        problem.plane,
        problem.plane_range,
        problem.tolerances,
        points,
    );
    let variations = sign_variations(numeric, problem.plane);
    if endpoint_hit && variations == Some(0) {
        return Ok(());
    }

    match source_plane_relation(problem.source, range, problem.plane, problem.tolerances) {
        PlaneCurveRangeRelation::Negative | PlaneCurveRangeRelation::Positive => return Ok(()),
        PlaneCurveRangeRelation::WithinSlab => {
            if points.iter().any(|point| range.contains(point.t_curve)) {
                return Ok(());
            }
            let lo_sign = exact_sample_sign(problem.source.eval(range.lo), problem.plane);
            let hi_sign = exact_sample_sign(problem.source.eval(range.hi), problem.plane);
            if let (Some(lo_sign), Some(hi_sign)) = (lo_sign, hi_sign)
                && opposite_strict_signs(lo_sign, hi_sign)
                && let Some(t) = bisect_root(
                    problem.source,
                    problem.plane,
                    range.lo,
                    range.hi,
                    lo_sign,
                    hi_sign,
                    problem.tolerances,
                )
            {
                push_root_candidate(
                    problem.source,
                    t,
                    problem.plane,
                    problem.plane_range,
                    problem.tolerances,
                    points,
                );
            } else if let Some(t) =
                best_leaf_root(problem.source, problem.plane, range, problem.tolerances)
            {
                push_root_candidate(
                    problem.source,
                    t,
                    problem.plane,
                    problem.plane_range,
                    problem.tolerances,
                    points,
                );
            }
            return Ok(());
        }
        PlaneCurveRangeRelation::Candidate => {}
    }

    let lo_sign = exact_sample_sign(problem.source.eval(range.lo), problem.plane);
    let hi_sign = exact_sample_sign(problem.source.eval(range.hi), problem.plane);
    if variations == Some(1)
        && let (Some(lo_sign), Some(hi_sign)) = (lo_sign, hi_sign)
        && opposite_strict_signs(lo_sign, hi_sign)
        && let Some(t) = bisect_root(
            problem.source,
            problem.plane,
            range.lo,
            range.hi,
            lo_sign,
            hi_sign,
            problem.tolerances,
        )
    {
        push_root_candidate(
            problem.source,
            t,
            problem.plane,
            problem.plane_range,
            problem.tolerances,
            points,
        );
        return Ok(());
    }

    if range.width() <= parameter_tolerance(problem.global_range, problem.tolerances)
        || depth >= MAX_ROOT_DEPTH
    {
        if let Some(t) = best_leaf_root(problem.source, problem.plane, range, problem.tolerances) {
            push_root_candidate(
                problem.source,
                t,
                problem.plane,
                problem.plane_range,
                problem.tolerances,
                points,
            );
        }
        return Ok(());
    }

    let mid = range.lerp(0.5);
    let (left, right) = numeric.split_at(mid)?;
    collect_isolated_roots(&left, problem, points, remaining_nodes, depth + 1)?;
    collect_isolated_roots(&right, problem, points, remaining_nodes, depth + 1)
}

fn collect_contained_intervals(
    numeric: &NurbsCurve,
    problem: NurbsPlaneProblem<'_>,
    overlaps: &mut Vec<CurveSurfaceOverlap>,
    remaining_nodes: &mut usize,
    depth: usize,
) -> Result<()> {
    if *remaining_nodes == 0 {
        return Ok(());
    }
    *remaining_nodes -= 1;

    let range = numeric.param_range();
    match source_window_relation(
        problem.source,
        range,
        problem.plane,
        problem.plane_range,
        problem.tolerances,
    ) {
        SourceWindowRelation::Outside => return Ok(()),
        SourceWindowRelation::Inside => {
            push_contained_overlap(
                problem.source,
                range,
                problem.plane,
                problem.plane_range,
                problem.tolerances,
                overlaps,
            );
            return Ok(());
        }
        SourceWindowRelation::Candidate => {}
    }

    if range.width() <= parameter_tolerance(problem.global_range, problem.tolerances)
        || depth >= MAX_CLIP_DEPTH
    {
        return Ok(());
    }

    let mid = range.lerp(0.5);
    let (left, right) = numeric.split_at(mid)?;
    collect_contained_intervals(&left, problem, overlaps, remaining_nodes, depth + 1)?;
    collect_contained_intervals(&right, problem, overlaps, remaining_nodes, depth + 1)
}

fn single_parameter_intersection(
    curve: &NurbsCurve,
    t_curve: f64,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    let point = curve.eval(t_curve);
    if exact_sample_relation(point, plane, tolerances) != Some(SamplePlaneRelation::WithinSlab) {
        return Ok(CurveSurfaceIntersections::indeterminate_empty(
            COMPLETION_REASON,
        ));
    }
    let Some(uv) = fit_parameter_pair(plane_uv(point, plane), plane_range, tolerances.linear())
    else {
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
) -> bool {
    let point = curve.eval(t_curve);
    if exact_sample_relation(point, plane, tolerances) != Some(SamplePlaneRelation::WithinSlab) {
        return false;
    }
    let Some(uv) = fit_parameter_pair(plane_uv(point, plane), plane_range, tolerances.linear())
    else {
        return false;
    };
    let Some(point) = accept_curve_surface_candidate(
        curve,
        t_curve,
        plane,
        uv,
        contact_kind(curve, t_curve, plane, tolerances),
        tolerances,
    ) else {
        return false;
    };
    push_distinct_point(points, point, curve.param_range(), tolerances);
    true
}

fn push_contained_overlap(
    source: &NurbsCurve,
    range: ParamRange,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
    overlaps: &mut Vec<CurveSurfaceOverlap>,
) {
    if range.width() <= parameter_tolerance(range, tolerances) {
        return;
    }
    let Some(uv_start) = fit_parameter_pair(
        plane_uv(source.eval(range.lo), plane),
        plane_range,
        tolerances.linear(),
    ) else {
        return;
    };
    let Some(uv_end) = fit_parameter_pair(
        plane_uv(source.eval(range.hi), plane),
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
    mut lo_sign: Orientation,
    mut hi_sign: Orientation,
    tolerances: Tolerances,
) -> Option<f64> {
    let parameter_tol = parameter_tolerance(curve.param_range(), tolerances);
    for _ in 0..80 {
        let mid = (lo + hi) / 2.0;
        let point = curve.eval(mid);
        if exact_sample_relation(point, plane, tolerances) == Some(SamplePlaneRelation::WithinSlab)
        {
            return Some(mid);
        }
        if hi - lo <= parameter_tol {
            return best_leaf_root(curve, plane, ParamRange::new(lo, hi), tolerances);
        }
        let mid_sign = exact_sample_sign(point, plane)?;
        if mid_sign == Orientation::Zero {
            return Some(mid);
        }
        if mid_sign == lo_sign {
            lo = mid;
            lo_sign = mid_sign;
        } else if mid_sign == hi_sign {
            hi = mid;
            hi_sign = mid_sign;
        } else {
            return None;
        }
    }
    best_leaf_root(curve, plane, ParamRange::new(lo, hi), tolerances)
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
        let point = curve.eval(t);
        if exact_sample_relation(point, plane, tolerances) == Some(SamplePlaneRelation::WithinSlab)
        {
            let distance = signed_distance(point, plane).abs();
            if distance < best.0 {
                best = (distance, t);
            }
        }
    }
    best.0.is_finite().then_some(best.1)
}

fn source_plane_relation(
    source: &NurbsCurve,
    range: ParamRange,
    plane: &Plane,
    tolerances: Tolerances,
) -> PlaneCurveRangeRelation {
    classify_curve_range_against_plane_slab(
        source,
        range,
        plane.frame().origin(),
        plane.frame().z(),
        tolerances.linear(),
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SourceWindowRelation {
    Outside,
    Candidate,
    Inside,
}

fn source_window_relation(
    source: &NurbsCurve,
    range: ParamRange,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> SourceWindowRelation {
    let axes = [plane.frame().x(), plane.frame().y()];
    let mut candidate = false;
    for axis in 0..2 {
        let lower = plane_range[axis].lo - tolerances.linear();
        let upper = plane_range[axis].hi + tolerances.linear();
        match classify_curve_range_against_affine_band(
            source,
            range,
            plane.frame().origin(),
            axes[axis],
            lower,
            upper,
        ) {
            PlaneCurveRangeRelation::Negative | PlaneCurveRangeRelation::Positive => {
                return SourceWindowRelation::Outside;
            }
            PlaneCurveRangeRelation::Candidate => candidate = true,
            PlaneCurveRangeRelation::WithinSlab => {}
        }
    }
    if candidate {
        SourceWindowRelation::Candidate
    } else {
        SourceWindowRelation::Inside
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SamplePlaneRelation {
    Negative,
    WithinSlab,
    Positive,
}

fn exact_sample_relation(
    point: Point3,
    plane: &Plane,
    tolerances: Tolerances,
) -> Option<SamplePlaneRelation> {
    let normal = plane.frame().z().to_array();
    let point = point.to_array();
    let origin = plane.frame().origin().to_array();
    if affine_dot3(normal, point, origin, -tolerances.linear())?.sign() == Orientation::Positive {
        return Some(SamplePlaneRelation::Positive);
    }
    if affine_dot3(normal, point, origin, tolerances.linear())?.sign() == Orientation::Negative {
        return Some(SamplePlaneRelation::Negative);
    }
    Some(SamplePlaneRelation::WithinSlab)
}

fn exact_sample_sign(point: Point3, plane: &Plane) -> Option<Orientation> {
    affine_dot3(
        plane.frame().z().to_array(),
        point.to_array(),
        plane.frame().origin().to_array(),
        0.0,
    )
    .map(|classified| classified.sign())
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

fn sign_variations(curve: &NurbsCurve, plane: &Plane) -> Option<usize> {
    let mut previous = None;
    let mut variations = 0;
    for &point in curve.points() {
        let sign = exact_sample_sign(point, plane)?;
        if sign == Orientation::Zero {
            continue;
        }
        if previous.is_some_and(|prev| prev != sign) {
            variations += 1;
        }
        previous = Some(sign);
    }
    Some(variations)
}

fn opposite_strict_signs(first: Orientation, second: Orientation) -> bool {
    matches!(
        (first, second),
        (Orientation::Negative, Orientation::Positive)
            | (Orientation::Positive, Orientation::Negative)
    )
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

fn merge_overlaps(overlaps: &mut Vec<CurveSurfaceOverlap>) {
    overlaps.sort_by(|a, b| a.curve.lo.total_cmp(&b.curve.lo));
    let mut merged: Vec<CurveSurfaceOverlap> = Vec::new();
    for overlap in overlaps.drain(..) {
        if let Some(last) = merged.last_mut()
            && overlap.curve.lo <= last.curve.hi
        {
            if overlap.curve.hi > last.curve.hi {
                last.curve = ParamRange::new(last.curve.lo, overlap.curve.hi);
                last.uv_end = overlap.uv_end;
            }
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

#[cfg(test)]
mod tests {
    use super::*;

    fn overlap(lo: f64, hi: f64) -> CurveSurfaceOverlap {
        CurveSurfaceOverlap {
            curve: ParamRange::new(lo, hi),
            uv_start: [lo, 0.0],
            uv_end: [hi, 0.0],
        }
    }

    #[test]
    fn overlap_merge_requires_actual_parameter_contact() {
        let mut separated = vec![overlap(0.0, 0.4), overlap(0.4 + 5.0e-11, 1.0)];
        merge_overlaps(&mut separated);
        assert_eq!(separated.len(), 2);

        let mut touching = vec![overlap(0.0, 0.4), overlap(0.4, 1.0)];
        merge_overlaps(&mut touching);
        assert_eq!(touching, vec![overlap(0.0, 1.0)]);

        let mut nested = vec![overlap(0.0, 1.0), overlap(0.25, 0.75)];
        merge_overlaps(&mut nested);
        assert_eq!(nested, vec![overlap(0.0, 1.0)]);
    }
}
