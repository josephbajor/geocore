use super::conic::{ConicPairConfig, ConicPlaneRelation};
use super::line_circle::intersect_bounded_line_circle;
use super::result::{ContactKind, CurveCurveIntersections};
use kcore::error::Result;
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Line};
use kgeom::param::ParamRange;
use kgeom::vec::Vec3;

/// Intersect two circles restricted to finite parameter ranges.
///
/// Handles coplanar secants/tangencies, skew-plane contacts, periodic arc
/// filtering, and positive-length coincident arc overlaps.
pub fn intersect_bounded_circles(
    a: &Circle,
    range_a: ParamRange,
    b: &Circle,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let pair = ConicPairConfig::circles(a, range_a, b, range_b, tolerances)?;
    match pair.plane_relation()? {
        ConicPlaneRelation::Parallel => intersect_parallel_plane_circles(a, b, tolerances, pair),
        ConicPlaneRelation::Crossing(line) => {
            intersect_plane_crossing_circles(a, range_a, tolerances, line, pair)
        }
    }
}

fn intersect_parallel_plane_circles(
    a: &Circle,
    b: &Circle,
    tolerances: Tolerances,
    pair: ConicPairConfig<'_>,
) -> Result<CurveCurveIntersections> {
    let center_delta = b.frame().origin() - a.frame().origin();
    if center_delta.dot(a.frame().z()).abs() > tolerances.linear() {
        return Ok(CurveCurveIntersections::complete_empty());
    }

    let local_b_center = a.frame().to_local(b.frame().origin());
    let center_distance =
        (local_b_center.x * local_b_center.x + local_b_center.y * local_b_center.y).sqrt();
    let radius_delta = (a.radius() - b.radius()).abs();
    if center_distance + radius_delta <= tolerances.linear() {
        return intersect_coincident_circles(a, b, pair);
    }
    if center_distance <= tolerances.linear() {
        return Ok(CurveCurveIntersections::complete_empty());
    }

    intersect_coplanar_distinct_circles(a, b, local_b_center, center_distance, tolerances, pair)
}

fn intersect_coplanar_distinct_circles(
    a: &Circle,
    b: &Circle,
    local_b_center: Vec3,
    center_distance: f64,
    tolerances: Tolerances,
    pair: ConicPairConfig<'_>,
) -> Result<CurveCurveIntersections> {
    let ra = a.radius();
    let rb = b.radius();
    let tangent = (center_distance - (ra + rb)).abs() <= tolerances.linear()
        || (center_distance - (ra - rb).abs()).abs() <= tolerances.linear();
    if center_distance > ra + rb + tolerances.linear()
        || center_distance < (ra - rb).abs() - tolerances.linear()
    {
        return Ok(CurveCurveIntersections::complete_empty());
    }

    let axis = Vec3::new(
        local_b_center.x / center_distance,
        local_b_center.y / center_distance,
        0.0,
    );
    let perp = Vec3::new(-axis.y, axis.x, 0.0);
    let along = (center_distance * center_distance + ra * ra - rb * rb) / (2.0 * center_distance);
    let height_sq = ra * ra - along * along;
    let offsets = if tangent || height_sq <= 0.0 {
        vec![0.0]
    } else {
        let height = height_sq.sqrt();
        vec![-height, height]
    };

    let mut points = Vec::with_capacity(offsets.len());
    for offset in offsets {
        let local = axis * along + perp * offset;
        let point = a.frame().point_at(local.x, local.y, 0.0);
        pair.push_point(point, tangent.then_some(ContactKind::Tangent), &mut points);
    }
    CurveCurveIntersections::canonicalized_complete(points, Vec::new())
}

fn intersect_plane_crossing_circles(
    a: &Circle,
    range_a: ParamRange,
    tolerances: Tolerances,
    line: Line,
    pair: ConicPairConfig<'_>,
) -> Result<CurveCurveIntersections> {
    let center_parameter = line.dir().dot(a.frame().origin() - line.origin());
    let line_range = ParamRange::new(
        center_parameter - a.radius() - tolerances.linear(),
        center_parameter + a.radius() + tolerances.linear(),
    );
    let line_circle_hits =
        intersect_bounded_line_circle(&line, line_range, a, range_a, tolerances)?;

    let mut points = Vec::with_capacity(line_circle_hits.points.len());
    for line_hit in line_circle_hits.points {
        let t_a = line_hit.t_b;
        let point = a.eval(t_a);
        let Some(t_b) = pair.fit_b(point) else {
            continue;
        };
        pair.push_parameters(t_a, t_b, None, &mut points);
    }
    CurveCurveIntersections::canonicalized_complete(points, Vec::new())
}

fn intersect_coincident_circles(
    a: &Circle,
    b: &Circle,
    pair: ConicPairConfig<'_>,
) -> Result<CurveCurveIntersections> {
    let same_normal = a.frame().z().dot(b.frame().z()) >= 0.0;
    let alpha = math::atan2(
        a.frame().y().dot(b.frame().x()),
        a.frame().x().dot(b.frame().x()),
    );
    let (sign, offset) = if same_normal {
        (1.0, -alpha)
    } else {
        (-1.0, alpha)
    };
    pair.coincident(sign, offset)
}
