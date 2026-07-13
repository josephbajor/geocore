use super::conic::{ConicConeConfig, intersect_bounded_conic_cone};
use super::result::CurveSurfaceIntersections;
use kcore::error::Result;
use kcore::tolerance::Tolerances;
use kgeom::curve::Ellipse;
use kgeom::param::ParamRange;
use kgeom::surface::Cone;

/// Intersect an ellipse restricted to a finite range with a finite cone
/// parameter window.
pub fn intersect_bounded_ellipse_cone(
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    cone: &Cone,
    cone_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    intersect_bounded_conic_cone(ConicConeConfig::ellipse(
        ellipse,
        ellipse_range,
        cone,
        cone_range,
        tolerances,
    ))
}
