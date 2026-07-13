use super::conic::{ConicNurbsConfig, intersect_bounded_conic_nurbs};
use super::result::CurveCurveIntersections;
use kcore::error::Result;
use kcore::tolerance::Tolerances;
use kgeom::curve::Ellipse;
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;

/// Intersect a finite ellipse arc with a clamped NURBS curve restricted to a
/// finite range.
///
/// This fixed-grid bridge samples the point-to-ellipse distance along the
/// NURBS curve, polishes local minima, and clips all-on-ellipse spans to the
/// finite periodic ellipse interval.
pub fn intersect_bounded_ellipse_nurbs(
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    curve: &NurbsCurve,
    curve_range: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    intersect_bounded_conic_nurbs(ConicNurbsConfig::ellipse(
        ellipse,
        ellipse_range,
        curve,
        curve_range,
        tolerances,
    ))
}
