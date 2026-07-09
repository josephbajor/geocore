use super::circle_ellipse::intersect_bounded_circle_ellipse;
use super::ellipse_ellipse::intersect_bounded_ellipses;
use super::line_circle::intersect_bounded_line_circle;
use super::line_ellipse::intersect_bounded_line_ellipse;
use super::line_line::intersect_bounded_lines;
use super::line_nurbs::intersect_bounded_line_nurbs;
use super::result::{CurveCurveIntersections, CurveCurveOverlap, CurveCurvePoint};
use kcore::error::{Error, Result};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;

/// Intersect two curves restricted to finite parameter ranges where needed.
///
/// This dispatches the currently supported analytic curve classes plus the
/// initial line/NURBS bridge. Unsupported curve classes fail explicitly; the
/// general subdivision/Newton curve-curve solver remains later M4 work.
pub fn intersect_bounded_curves(
    a: &dyn Curve,
    range_a: ParamRange,
    b: &dyn Curve,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    if let (Some(a), Some(b)) = (as_line(a), as_line(b)) {
        return intersect_bounded_lines(a, range_a, b, range_b, tolerances);
    }
    if let (Some(a), Some(b)) = (as_line(a), as_circle(b)) {
        return intersect_bounded_line_circle(a, range_a, b, range_b, tolerances);
    }
    if let (Some(a), Some(b)) = (as_circle(a), as_line(b)) {
        return intersect_bounded_line_circle(b, range_b, a, range_a, tolerances)
            .and_then(reverse_intersections);
    }
    if let (Some(a), Some(b)) = (as_line(a), as_ellipse(b)) {
        return intersect_bounded_line_ellipse(a, range_a, b, range_b, tolerances);
    }
    if let (Some(a), Some(b)) = (as_ellipse(a), as_line(b)) {
        return intersect_bounded_line_ellipse(b, range_b, a, range_a, tolerances)
            .and_then(reverse_intersections);
    }
    if let (Some(a), Some(b)) = (as_line(a), as_nurbs(b)) {
        return intersect_bounded_line_nurbs(a, range_a, b, range_b, tolerances);
    }
    if let (Some(a), Some(b)) = (as_nurbs(a), as_line(b)) {
        return intersect_bounded_line_nurbs(b, range_b, a, range_a, tolerances)
            .and_then(reverse_intersections);
    }
    if let (Some(a), Some(b)) = (as_circle(a), as_circle(b)) {
        return super::circle_circle::intersect_bounded_circles(a, range_a, b, range_b, tolerances);
    }
    if let (Some(a), Some(b)) = (as_circle(a), as_ellipse(b)) {
        return intersect_bounded_circle_ellipse(a, range_a, b, range_b, tolerances);
    }
    if let (Some(a), Some(b)) = (as_ellipse(a), as_circle(b)) {
        return intersect_bounded_circle_ellipse(b, range_b, a, range_a, tolerances)
            .and_then(reverse_intersections);
    }
    if let (Some(a), Some(b)) = (as_ellipse(a), as_ellipse(b)) {
        return intersect_bounded_ellipses(a, range_a, b, range_b, tolerances);
    }

    Err(Error::InvalidGeometry {
        reason: "unsupported curve/curve intersection class",
    })
}

fn as_line(curve: &dyn Curve) -> Option<&Line> {
    curve.as_any().downcast_ref()
}

fn as_circle(curve: &dyn Curve) -> Option<&Circle> {
    curve.as_any().downcast_ref()
}

fn as_ellipse(curve: &dyn Curve) -> Option<&Ellipse> {
    curve.as_any().downcast_ref()
}

fn as_nurbs(curve: &dyn Curve) -> Option<&NurbsCurve> {
    curve.as_any().downcast_ref()
}

fn reverse_intersections(hit: CurveCurveIntersections) -> Result<CurveCurveIntersections> {
    CurveCurveIntersections::canonicalized(
        hit.points.into_iter().map(reverse_point).collect(),
        hit.overlaps.into_iter().map(reverse_overlap).collect(),
    )
}

fn reverse_point(point: CurveCurvePoint) -> CurveCurvePoint {
    CurveCurvePoint {
        point: point.point,
        t_a: point.t_b,
        t_b: point.t_a,
        residual: point.residual,
        kind: point.kind,
    }
}

fn reverse_overlap(overlap: CurveCurveOverlap) -> CurveCurveOverlap {
    CurveCurveOverlap {
        a: overlap.b,
        b: overlap.a,
        orientation: overlap.orientation,
    }
}
