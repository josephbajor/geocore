use super::conic::{ConicNurbsConfig, intersect_bounded_conic_nurbs};
use super::result::CurveCurveIntersections;
use kcore::error::Result;
use kcore::tolerance::Tolerances;
use kgeom::curve::Circle;
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;

/// Intersect a finite circle arc with a clamped NURBS curve restricted to a
/// finite range.
///
/// This fixed-grid bridge samples the point-to-circle distance along the
/// NURBS curve, polishes local minima, and clips all-on-circle spans to the
/// finite periodic circle interval.
pub fn intersect_bounded_circle_nurbs(
    circle: &Circle,
    circle_range: ParamRange,
    curve: &NurbsCurve,
    curve_range: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    intersect_bounded_conic_nurbs(ConicNurbsConfig::circle(
        circle,
        circle_range,
        curve,
        curve_range,
        tolerances,
    ))
}
