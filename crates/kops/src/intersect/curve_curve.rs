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
    let original_a = CurveDispatch::inspect(a);
    let original_b = CurveDispatch::inspect(b);
    let (Some(mut class_a), Some(mut class_b)) = (original_a, original_b) else {
        return Err(IntersectionError::UnsupportedCurvePair {
            class_a: original_a.map(|class| class.class().key()),
            class_b: original_b.map(|class| class.class().key()),
        });
    };
    let (mut range_a, mut range_b) = (range_a, range_b);
    let swapped = class_a.class() > class_b.class();
    if swapped {
        core::mem::swap(&mut class_a, &mut class_b);
        core::mem::swap(&mut range_a, &mut range_b);
    }

    let result = match (class_a, class_b) {
        (CurveDispatch::Line(a), CurveDispatch::Line(b)) => {
            intersect_bounded_lines(a, range_a, b, range_b, tolerances)
        }
        (CurveDispatch::Line(a), CurveDispatch::Circle(b)) => {
            intersect_bounded_line_circle(a, range_a, b, range_b, tolerances)
        }
        (CurveDispatch::Line(a), CurveDispatch::Ellipse(b)) => {
            intersect_bounded_line_ellipse(a, range_a, b, range_b, tolerances)
        }
        (CurveDispatch::Line(a), CurveDispatch::Nurbs(b)) => {
            intersect_bounded_line_nurbs(a, range_a, b, range_b, tolerances)
        }
        (CurveDispatch::Circle(a), CurveDispatch::Circle(b)) => {
            super::circle_circle::intersect_bounded_circles(a, range_a, b, range_b, tolerances)
        }
        (CurveDispatch::Circle(a), CurveDispatch::Ellipse(b)) => {
            intersect_bounded_circle_ellipse(a, range_a, b, range_b, tolerances)
        }
        (CurveDispatch::Circle(a), CurveDispatch::Nurbs(b)) => {
            intersect_bounded_circle_nurbs(a, range_a, b, range_b, tolerances)
        }
        (CurveDispatch::Ellipse(a), CurveDispatch::Ellipse(b)) => {
            intersect_bounded_ellipses(a, range_a, b, range_b, tolerances)
        }
        (CurveDispatch::Ellipse(a), CurveDispatch::Nurbs(b)) => {
            intersect_bounded_ellipse_nurbs(a, range_a, b, range_b, tolerances)
        }
        (CurveDispatch::Nurbs(a), CurveDispatch::Nurbs(b)) => {
            intersect_bounded_nurbs_nurbs(a, range_a, b, range_b, tolerances)
        }
        _ => unreachable!("curve classes are normalized into canonical order"),
    };
    result
        .map(|result| if swapped { result.swapped() } else { result })
        .map_err(IntersectionError::from)
}
