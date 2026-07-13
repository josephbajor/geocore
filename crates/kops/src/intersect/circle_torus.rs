use super::conic::{ConicTorusConfig, intersect_bounded_conic_torus};
use super::result::CurveSurfaceIntersections;
use kcore::error::Result;
use kcore::tolerance::Tolerances;
use kgeom::curve::Circle;
use kgeom::param::ParamRange;
use kgeom::surface::Torus;

/// Intersect a circle restricted to a finite range with a finite torus
/// parameter window.
pub fn intersect_bounded_circle_torus(
    circle: &Circle,
    circle_range: ParamRange,
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    intersect_bounded_conic_torus(ConicTorusConfig::circle(
        circle,
        circle_range,
        torus,
        torus_range,
        tolerances,
    ))
}
