use super::circle_ellipse::intersect_bounded_circle_ellipse;
use super::circle_nurbs::intersect_bounded_circle_nurbs;
use super::ellipse_ellipse::intersect_bounded_ellipses;
use super::ellipse_nurbs::intersect_bounded_ellipse_nurbs;
use super::error::{IntersectionError, IntersectionResult};
use super::geometry_class::CurveDispatch;
use super::line_circle::intersect_bounded_line_circle;
use super::line_ellipse::intersect_bounded_line_ellipse;
use super::line_line::intersect_bounded_lines;
use super::line_nurbs::intersect_bounded_line_nurbs;
use super::nurbs_nurbs::intersect_bounded_nurbs_nurbs;
use super::result::CurveCurveIntersections;
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::param::ParamRange;

/// Intersect two curves restricted to finite parameter ranges where needed.
///
/// This dispatches the currently supported analytic curve classes plus the
/// initial NURBS bridges. Unsupported curve classes fail explicitly; the
/// broader subdivision/Newton curve-curve solver remains later M4 work.
pub fn intersect_bounded_curves(
    a: &dyn Curve,
    range_a: ParamRange,
    b: &dyn Curve,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> IntersectionResult<CurveCurveIntersections> {
    let class_a = CurveDispatch::inspect(a);
    let class_b = CurveDispatch::inspect(b);
    let result = match (class_a, class_b) {
        (Some(CurveDispatch::Line(a)), Some(CurveDispatch::Line(b))) => {
            intersect_bounded_lines(a, range_a, b, range_b, tolerances)
        }
        (Some(CurveDispatch::Line(a)), Some(CurveDispatch::Circle(b))) => {
            intersect_bounded_line_circle(a, range_a, b, range_b, tolerances)
        }
        (Some(CurveDispatch::Circle(a)), Some(CurveDispatch::Line(b))) => {
            intersect_bounded_line_circle(b, range_b, a, range_a, tolerances)
                .map(CurveCurveIntersections::swapped)
        }
        (Some(CurveDispatch::Line(a)), Some(CurveDispatch::Ellipse(b))) => {
            intersect_bounded_line_ellipse(a, range_a, b, range_b, tolerances)
        }
        (Some(CurveDispatch::Ellipse(a)), Some(CurveDispatch::Line(b))) => {
            intersect_bounded_line_ellipse(b, range_b, a, range_a, tolerances)
                .map(CurveCurveIntersections::swapped)
        }
        (Some(CurveDispatch::Line(a)), Some(CurveDispatch::Nurbs(b))) => {
            intersect_bounded_line_nurbs(a, range_a, b, range_b, tolerances)
        }
        (Some(CurveDispatch::Nurbs(a)), Some(CurveDispatch::Line(b))) => {
            intersect_bounded_line_nurbs(b, range_b, a, range_a, tolerances)
                .map(CurveCurveIntersections::swapped)
        }
        (Some(CurveDispatch::Circle(a)), Some(CurveDispatch::Circle(b))) => {
            super::circle_circle::intersect_bounded_circles(a, range_a, b, range_b, tolerances)
        }
        (Some(CurveDispatch::Circle(a)), Some(CurveDispatch::Nurbs(b))) => {
            intersect_bounded_circle_nurbs(a, range_a, b, range_b, tolerances)
        }
        (Some(CurveDispatch::Nurbs(a)), Some(CurveDispatch::Circle(b))) => {
            intersect_bounded_circle_nurbs(b, range_b, a, range_a, tolerances)
                .map(CurveCurveIntersections::swapped)
        }
        (Some(CurveDispatch::Circle(a)), Some(CurveDispatch::Ellipse(b))) => {
            intersect_bounded_circle_ellipse(a, range_a, b, range_b, tolerances)
        }
        (Some(CurveDispatch::Ellipse(a)), Some(CurveDispatch::Circle(b))) => {
            intersect_bounded_circle_ellipse(b, range_b, a, range_a, tolerances)
                .map(CurveCurveIntersections::swapped)
        }
        (Some(CurveDispatch::Ellipse(a)), Some(CurveDispatch::Ellipse(b))) => {
            intersect_bounded_ellipses(a, range_a, b, range_b, tolerances)
        }
        (Some(CurveDispatch::Ellipse(a)), Some(CurveDispatch::Nurbs(b))) => {
            intersect_bounded_ellipse_nurbs(a, range_a, b, range_b, tolerances)
        }
        (Some(CurveDispatch::Nurbs(a)), Some(CurveDispatch::Ellipse(b))) => {
            intersect_bounded_ellipse_nurbs(b, range_b, a, range_a, tolerances)
                .map(CurveCurveIntersections::swapped)
        }
        (Some(CurveDispatch::Nurbs(a)), Some(CurveDispatch::Nurbs(b))) => {
            intersect_bounded_nurbs_nurbs(a, range_a, b, range_b, tolerances)
        }
        _ => {
            return Err(IntersectionError::UnsupportedCurvePair {
                class_a: class_a.map(|class| class.class().key()),
                class_b: class_b.map(|class| class.class().key()),
            });
        }
    };
    result.map_err(IntersectionError::from)
}
