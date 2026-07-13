use super::line_plane::intersect_bounded_line_plane;
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint,
    SurfaceIntersectionCurve, SurfaceRegionOrientation, SurfaceSurfaceCurve,
    SurfaceSurfaceIntersections, SurfaceSurfacePoint, SurfaceSurfaceRegion,
    SurfaceSurfaceRegionVertex, accept_surface_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::interval::Interval;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Curve, Line};
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Surface};
use kgeom::vec::Point3;

/// Intersect two finite plane windows.
pub fn intersect_bounded_planes(
    a: &Plane,
    a_range: [ParamRange; 2],
    b: &Plane,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(a_range, b_range)?;

    let n_a = a.frame().z();
    let n_b = b.frame().z();
    let direction = n_a.cross(n_b);
    let direction_norm = direction.norm();
    if direction_norm <= tolerances.angular() {
        let separation = (b.frame().origin() - a.frame().origin()).dot(n_a).abs();
        if separation <= tolerances.linear() {
            return intersect_coincident_plane_windows(a, a_range, b, b_range, tolerances);
        }
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    }

    let c_a = n_a.dot(a.frame().origin());
    let c_b = n_b.dot(b.frame().origin());
    let origin = ((n_b * c_a - n_a * c_b).cross(direction)) / direction.norm_sq();
    let line = Line::new(origin, direction)?;
    let Some(line_range) = candidate_line_range(&line, a, a_range, b, b_range, tolerances) else {
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    };

    let a_hit = intersect_bounded_line_plane(&line, line_range, a, a_range, tolerances)?;
    let b_hit = intersect_bounded_line_plane(&line, line_range, b, b_range, tolerances)?;

    let mut points = Vec::new();
    let mut curves = Vec::new();
    add_clipped_line_branch(
        &mut points,
        &mut curves,
        line,
        line_range,
        &a_hit,
        &b_hit,
        a,
        a_range,
        b,
        b_range,
        tolerances,
    );
    SurfaceSurfaceIntersections::canonicalized_complete(points, curves)
}

fn candidate_line_range(
    line: &Line,
    a: &Plane,
    a_range: [ParamRange; 2],
    b: &Plane,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<ParamRange> {
    let a_projection = plane_window_projection(line, a, a_range);
    let b_projection = plane_window_projection(line, b, b_range);
    let lo = a_projection.lo.max(b_projection.lo);
    let hi = a_projection.hi.min(b_projection.hi);
    if hi < lo - tolerances.linear() {
        None
    } else if hi < lo {
        let mid = (lo + hi) / 2.0;
        Some(ParamRange::new(mid, mid))
    } else {
        Some(ParamRange::new(lo, hi))
    }
}

fn plane_window_projection(line: &Line, plane: &Plane, range: [ParamRange; 2]) -> ParamRange {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for u in [range[0].lo, range[0].hi] {
        for v in [range[1].lo, range[1].hi] {
            let t = (plane.frame().point_at(u, v, 0.0) - line.origin()).dot(line.dir());
            lo = lo.min(t);
            hi = hi.max(t);
        }
    }
    ParamRange::new(lo, hi)
}

#[allow(clippy::too_many_arguments)]
fn add_clipped_line_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    line: Line,
    line_range: ParamRange,
    a_hit: &CurveSurfaceIntersections,
    b_hit: &CurveSurfaceIntersections,
    a: &Plane,
    a_range: [ParamRange; 2],
    b: &Plane,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    for a_overlap in &a_hit.overlaps {
        for b_overlap in &b_hit.overlaps {
            let lo = a_overlap.curve.lo.max(b_overlap.curve.lo);
            let hi = a_overlap.curve.hi.min(b_overlap.curve.hi);
            if hi - lo > tolerances.linear() {
                let Some(uv_a_start) = plane_uv_at(line.eval(lo), a, a_range, tolerances) else {
                    continue;
                };
                let Some(uv_a_end) = plane_uv_at(line.eval(hi), a, a_range, tolerances) else {
                    continue;
                };
                let Some(uv_b_start) = plane_uv_at(line.eval(lo), b, b_range, tolerances) else {
                    continue;
                };
                let Some(uv_b_end) = plane_uv_at(line.eval(hi), b, b_range, tolerances) else {
                    continue;
                };
                curves.push(SurfaceSurfaceCurve {
                    curve: SurfaceIntersectionCurve::Line(line),
                    curve_range: ParamRange::new(lo, hi),
                    uv_a_start,
                    uv_a_end,
                    uv_b_start,
                    uv_b_end,
                    kind: ContactKind::Transverse,
                });
            } else if (hi - lo).abs() <= tolerances.linear() {
                add_point_from_parameter(
                    points,
                    &line,
                    line_range,
                    (lo + hi) / 2.0,
                    a,
                    a_range,
                    b,
                    b_range,
                    tolerances,
                );
            }
        }
    }

    add_isolated_points(
        points, &line, line_range, a_hit, b_hit, a, a_range, b, b_range, tolerances,
    );
}

#[allow(clippy::too_many_arguments)]
fn add_isolated_points(
    points: &mut Vec<SurfaceSurfacePoint>,
    line: &Line,
    line_range: ParamRange,
    a_hit: &CurveSurfaceIntersections,
    b_hit: &CurveSurfaceIntersections,
    a: &Plane,
    a_range: [ParamRange; 2],
    b: &Plane,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    for point in &a_hit.points {
        if hit_contains_t(b_hit, point.t_curve, tolerances) {
            add_point_from_parameter(
                points,
                line,
                line_range,
                point.t_curve,
                a,
                a_range,
                b,
                b_range,
                tolerances,
            );
        }
    }
    for point in &b_hit.points {
        if hit_contains_t(a_hit, point.t_curve, tolerances) {
            add_point_from_parameter(
                points,
                line,
                line_range,
                point.t_curve,
                a,
                a_range,
                b,
                b_range,
                tolerances,
            );
        }
    }
    for a_point in &a_hit.points {
        for b_point in &b_hit.points {
            if curve_parameters_match(a_point, b_point, tolerances) {
                add_point_from_parameter(
                    points,
                    line,
                    line_range,
                    a_point.t_curve,
                    a,
                    a_range,
                    b,
                    b_range,
                    tolerances,
                );
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn add_point_from_parameter(
    points: &mut Vec<SurfaceSurfacePoint>,
    line: &Line,
    line_range: ParamRange,
    t: f64,
    a: &Plane,
    a_range: [ParamRange; 2],
    b: &Plane,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    let Some(t) = fit_scalar_parameter(t, line_range, tolerances.linear()) else {
        return;
    };
    let point = line.eval(t);
    let Some(uv_a) = plane_uv_at(point, a, a_range, tolerances) else {
        return;
    };
    let Some(uv_b) = plane_uv_at(point, b, b_range, tolerances) else {
        return;
    };
    if let Some(point) =
        accept_surface_surface_candidate(a, uv_a, b, uv_b, ContactKind::Transverse, tolerances)
    {
        push_point(points, point, tolerances);
    }
}

fn plane_uv_at(
    point: Point3,
    plane: &Plane,
    range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let local = plane.frame().to_local(point);
    Some([
        fit_scalar_parameter(local.x, range[0], tolerances.linear())?,
        fit_scalar_parameter(local.y, range[1], tolerances.linear())?,
    ])
}

fn hit_contains_t(hit: &CurveSurfaceIntersections, t: f64, tolerances: Tolerances) -> bool {
    hit.overlaps
        .iter()
        .any(|overlap| overlap_contains_t(overlap, t, tolerances))
        || hit
            .points
            .iter()
            .any(|point| (point.t_curve - t).abs() <= tolerances.linear())
}

fn overlap_contains_t(overlap: &CurveSurfaceOverlap, t: f64, tolerances: Tolerances) -> bool {
    t >= overlap.curve.lo - tolerances.linear() && t <= overlap.curve.hi + tolerances.linear()
}

fn curve_parameters_match(
    a: &CurveSurfacePoint,
    b: &CurveSurfacePoint,
    tolerances: Tolerances,
) -> bool {
    (a.t_curve - b.t_curve).abs() <= tolerances.linear()
        || a.point.dist(b.point) <= tolerances.linear()
}

fn fit_scalar_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
}

fn push_point(
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

#[derive(Clone, Copy, Debug)]
struct PairedPlaneSample {
    point: Point3,
    uv_a: [f64; 2],
    uv_b: [f64; 2],
    residual: f64,
    residual_bound: f64,
}

fn intersect_coincident_plane_windows(
    a: &Plane,
    a_range: [ParamRange; 2],
    b: &Plane,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let a_corners = rectangle_corners(a_range);
    let b_corners = rectangle_corners(b_range).map(|uv| {
        let local = a.frame().to_local(b.eval(uv));
        [local.x, local.y]
    });
    let mut candidates = Vec::new();

    for point in a_corners {
        if point_in_first_window(point, a_range, tolerances.linear())
            && point_in_second_window(point, a, b, b_range, tolerances.linear())
        {
            push_uv_candidate(&mut candidates, point, tolerances.linear());
        }
    }
    for point in b_corners {
        if point_in_first_window(point, a_range, tolerances.linear())
            && point_in_second_window(point, a, b, b_range, tolerances.linear())
        {
            push_uv_candidate(&mut candidates, point, tolerances.linear());
        }
    }
    for (a_start, a_end) in rectangle_edges(a_corners) {
        for (b_start, b_end) in rectangle_edges(b_corners) {
            for point in segment_intersections(a_start, a_end, b_start, b_end, tolerances) {
                if point_in_first_window(point, a_range, tolerances.linear())
                    && point_in_second_window(point, a, b, b_range, tolerances.linear())
                {
                    push_uv_candidate(&mut candidates, point, tolerances.linear());
                }
            }
        }
    }

    let hull = convex_hull(candidates, tolerances.linear());
    match intersection_dimension(&hull, tolerances.linear()) {
        WindowIntersectionDimension::Empty => Ok(SurfaceSurfaceIntersections::complete_empty()),
        WindowIntersectionDimension::Point => {
            let uv = centroid(&hull);
            let sample = paired_plane_sample(uv, a, a_range, b, b_range, tolerances).ok_or(
                Error::InvalidGeometry {
                    reason: "coincident plane/plane window produced non-finite paired boundary data",
                },
            )?;
            SurfaceSurfaceIntersections::canonicalized_complete(
                vec![SurfaceSurfacePoint {
                    point: sample.point,
                    uv_a: sample.uv_a,
                    uv_b: sample.uv_b,
                    residual: sample.residual,
                    kind: ContactKind::Tangent,
                }],
                Vec::new(),
            )
        }
        WindowIntersectionDimension::Curve(start, end) => {
            let start = paired_plane_sample(start, a, a_range, b, b_range, tolerances).ok_or(
                Error::InvalidGeometry {
                    reason: "coincident plane/plane window produced non-finite paired boundary data",
                },
            )?;
            let end = paired_plane_sample(end, a, a_range, b, b_range, tolerances).ok_or(
                Error::InvalidGeometry {
                    reason: "coincident plane/plane window produced non-finite paired boundary data",
                },
            )?;
            let direction = end.point - start.point;
            let length = direction.norm();
            if length <= tolerances.linear() {
                return SurfaceSurfaceIntersections::canonicalized_complete(
                    vec![SurfaceSurfacePoint {
                        point: (start.point + end.point) / 2.0,
                        uv_a: midpoint2(start.uv_a, end.uv_a),
                        uv_b: midpoint2(start.uv_b, end.uv_b),
                        residual: start.residual.max(end.residual),
                        kind: ContactKind::Tangent,
                    }],
                    Vec::new(),
                );
            }
            let line = Line::new(start.point, direction)?;
            SurfaceSurfaceIntersections::canonicalized_complete(
                Vec::new(),
                vec![SurfaceSurfaceCurve {
                    curve: SurfaceIntersectionCurve::Line(line),
                    curve_range: ParamRange::new(0.0, length),
                    uv_a_start: start.uv_a,
                    uv_a_end: end.uv_a,
                    uv_b_start: start.uv_b,
                    uv_b_end: end.uv_b,
                    kind: ContactKind::Tangent,
                }],
            )
        }
        WindowIntersectionDimension::Region => {
            let mut boundary = Vec::with_capacity(hull.len());
            let mut max_residual = 0.0_f64;
            for uv in hull {
                let sample = paired_plane_sample(uv, a, a_range, b, b_range, tolerances).ok_or(
                    Error::InvalidGeometry {
                        reason: "coincident plane/plane window produced non-finite paired boundary data",
                    },
                )?;
                max_residual = max_residual.max(sample.residual_bound);
                boundary.push(SurfaceSurfaceRegionVertex {
                    point: sample.point,
                    uv_a: sample.uv_a,
                    uv_b: sample.uv_b,
                    residual: sample.residual,
                });
            }
            let orientation = if a.frame().z().dot(b.frame().z()).is_sign_positive() {
                SurfaceRegionOrientation::Same
            } else {
                SurfaceRegionOrientation::Reversed
            };
            SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
                Vec::new(),
                Vec::new(),
                vec![SurfaceSurfaceRegion {
                    boundary,
                    orientation,
                    correspondence: super::result::SurfaceRegionCorrespondence::Polygonal,
                    max_residual,
                }],
            )
        }
    }
}

fn rectangle_corners(range: [ParamRange; 2]) -> [[f64; 2]; 4] {
    [
        [range[0].lo, range[1].lo],
        [range[0].hi, range[1].lo],
        [range[0].hi, range[1].hi],
        [range[0].lo, range[1].hi],
    ]
}

fn rectangle_edges(corners: [[f64; 2]; 4]) -> [([f64; 2], [f64; 2]); 4] {
    [
        (corners[0], corners[1]),
        (corners[1], corners[2]),
        (corners[2], corners[3]),
        (corners[3], corners[0]),
    ]
}

fn point_in_first_window(point: [f64; 2], range: [ParamRange; 2], tolerance: f64) -> bool {
    point[0] >= range[0].lo - tolerance
        && point[0] <= range[0].hi + tolerance
        && point[1] >= range[1].lo - tolerance
        && point[1] <= range[1].hi + tolerance
}

fn point_in_second_window(
    point_a: [f64; 2],
    a: &Plane,
    b: &Plane,
    range_b: [ParamRange; 2],
    tolerance: f64,
) -> bool {
    let local = b.frame().to_local(a.eval(point_a));
    point_in_first_window([local.x, local.y], range_b, tolerance)
}

fn segment_intersections(
    p: [f64; 2],
    p_end: [f64; 2],
    q: [f64; 2],
    q_end: [f64; 2],
    tolerances: Tolerances,
) -> Vec<[f64; 2]> {
    let r = sub2(p_end, p);
    let s = sub2(q_end, q);
    let r_length = norm2(r);
    let s_length = norm2(s);
    if r_length <= tolerances.linear() {
        return point_segment_contact(p, q, q_end, tolerances.linear())
            .into_iter()
            .collect();
    }
    if s_length <= tolerances.linear() {
        return point_segment_contact(q, p, p_end, tolerances.linear())
            .into_iter()
            .collect();
    }

    let denominator = cross2(r, s);
    if denominator.abs() <= tolerances.angular() * r_length * s_length {
        return Vec::new();
    }
    let q_minus_p = sub2(q, p);
    let t = cross2(q_minus_p, s) / denominator;
    let u = cross2(q_minus_p, r) / denominator;
    let t_tolerance = tolerances.linear() / r_length;
    let u_tolerance = tolerances.linear() / s_length;
    if t < -t_tolerance || t > 1.0 + t_tolerance || u < -u_tolerance || u > 1.0 + u_tolerance {
        Vec::new()
    } else {
        let from_p = add2(p, scale2(r, t.clamp(0.0, 1.0)));
        let from_q = add2(q, scale2(s, u.clamp(0.0, 1.0)));
        vec![midpoint2(from_p, from_q)]
    }
}

fn point_segment_contact(
    point: [f64; 2],
    start: [f64; 2],
    end: [f64; 2],
    tolerance: f64,
) -> Option<[f64; 2]> {
    let direction = sub2(end, start);
    let length_sq = dot2(direction, direction);
    if length_sq == 0.0 {
        return (norm2(sub2(point, start)) <= tolerance).then(|| midpoint2(point, start));
    }
    let parameter = (dot2(sub2(point, start), direction) / length_sq).clamp(0.0, 1.0);
    let closest = add2(start, scale2(direction, parameter));
    (norm2(sub2(point, closest)) <= tolerance).then(|| midpoint2(point, closest))
}

fn push_uv_candidate(candidates: &mut Vec<[f64; 2]>, candidate: [f64; 2], tolerance: f64) {
    if !candidates
        .iter()
        .any(|existing| norm2(sub2(*existing, candidate)) <= tolerance)
    {
        candidates.push(candidate);
    }
}

fn convex_hull(mut points: Vec<[f64; 2]>, tolerance: f64) -> Vec<[f64; 2]> {
    points.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    let mut unique = Vec::new();
    for point in points {
        push_uv_candidate(&mut unique, point, tolerance);
    }
    if unique.len() <= 2 {
        return unique;
    }

    let mut lower = Vec::new();
    for &point in &unique {
        while lower.len() >= 2
            && cross2(
                sub2(lower[lower.len() - 1], lower[lower.len() - 2]),
                sub2(point, lower[lower.len() - 1]),
            ) <= 0.0
        {
            lower.pop();
        }
        lower.push(point);
    }
    let mut upper = Vec::new();
    for &point in unique.iter().rev() {
        while upper.len() >= 2
            && cross2(
                sub2(upper[upper.len() - 1], upper[upper.len() - 2]),
                sub2(point, upper[upper.len() - 1]),
            ) <= 0.0
        {
            upper.pop();
        }
        upper.push(point);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

enum WindowIntersectionDimension {
    Empty,
    Point,
    Curve([f64; 2], [f64; 2]),
    Region,
}

fn intersection_dimension(points: &[[f64; 2]], tolerance: f64) -> WindowIntersectionDimension {
    if points.is_empty() {
        return WindowIntersectionDimension::Empty;
    }
    let mut farthest = (points[0], points[0]);
    let mut diameter = 0.0_f64;
    for (index, &a) in points.iter().enumerate() {
        for &b in &points[index + 1..] {
            let distance = norm2(sub2(b, a));
            if distance > diameter {
                diameter = distance;
                farthest = (a, b);
            }
        }
    }
    if diameter <= tolerance {
        return WindowIntersectionDimension::Point;
    }
    if compare_uv(farthest.1, farthest.0).is_lt() {
        farthest = (farthest.1, farthest.0);
    }
    let baseline = sub2(farthest.1, farthest.0);
    let thickness = points
        .iter()
        .map(|point| cross2(baseline, sub2(*point, farthest.0)).abs() / diameter)
        .fold(0.0_f64, f64::max);
    if thickness <= tolerance {
        WindowIntersectionDimension::Curve(farthest.0, farthest.1)
    } else {
        WindowIntersectionDimension::Region
    }
}

fn centroid(points: &[[f64; 2]]) -> [f64; 2] {
    let sum = points
        .iter()
        .fold([0.0, 0.0], |sum, point| add2(sum, *point));
    scale2(sum, 1.0 / points.len() as f64)
}

fn paired_plane_sample(
    point_a: [f64; 2],
    a: &Plane,
    a_range: [ParamRange; 2],
    b: &Plane,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<PairedPlaneSample> {
    let uv_a = [
        fit_scalar_parameter(point_a[0], a_range[0], tolerances.linear())?,
        fit_scalar_parameter(point_a[1], a_range[1], tolerances.linear())?,
    ];
    let pa = a.eval(uv_a);
    let local_b = b.frame().to_local(pa);
    let uv_b = [
        fit_scalar_parameter(local_b.x, b_range[0], tolerances.linear())?,
        fit_scalar_parameter(local_b.y, b_range[1], tolerances.linear())?,
    ];
    let pb = b.eval(uv_b);
    let residual = pa.dist(pb);
    let residual_bound = conservative_point_distance(pa, pb)?;
    Some(PairedPlaneSample {
        point: (pa + pb) / 2.0,
        uv_a,
        uv_b,
        residual,
        residual_bound,
    })
}

fn conservative_point_distance(a: Point3, b: Point3) -> Option<f64> {
    let x = Interval::point(a.x) - Interval::point(b.x);
    let y = Interval::point(a.y) - Interval::point(b.y);
    let z = Interval::point(a.z) - Interval::point(b.z);
    (x.square() + y.square() + z.square())
        .sqrt()
        .map(Interval::hi)
        .filter(|bound| bound.is_finite())
}

fn add2(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] + b[0], a[1] + b[1]]
}

fn sub2(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

fn scale2(value: [f64; 2], scale: f64) -> [f64; 2] {
    [value[0] * scale, value[1] * scale]
}

fn midpoint2(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]
}

fn dot2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn cross2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn norm2(value: [f64; 2]) -> f64 {
    dot2(value, value).sqrt()
}

fn compare_uv(a: [f64; 2], b: [f64; 2]) -> core::cmp::Ordering {
    a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1]))
}

fn validate_ranges(a_range: [ParamRange; 2], b_range: [ParamRange; 2]) -> Result<()> {
    if a_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "plane/plane intersection requires finite non-reversed first-plane ranges",
        });
    }
    if b_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "plane/plane intersection requires finite non-reversed second-plane ranges",
        });
    }
    Ok(())
}
