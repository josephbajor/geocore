use super::conic::{ConicSphereConfig, intersect_bounded_conic_sphere};
use super::result::CurveSurfaceIntersections;
use kcore::error::Result;
use kcore::tolerance::Tolerances;
use kgeom::curve::Circle;
use kgeom::param::ParamRange;
use kgeom::surface::Sphere;

/// Intersect a circle restricted to a finite range with a finite sphere
/// parameter window.
pub fn intersect_bounded_circle_sphere(
    circle: &Circle,
    circle_range: ParamRange,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    intersect_bounded_conic_sphere(ConicSphereConfig::circle(
        circle,
        circle_range,
        sphere,
        sphere_range,
        tolerances,
    ))
}
