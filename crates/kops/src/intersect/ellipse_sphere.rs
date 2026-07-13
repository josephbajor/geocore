use super::conic::{ConicSphereConfig, intersect_bounded_conic_sphere};
use super::result::CurveSurfaceIntersections;
use kcore::error::Result;
use kcore::tolerance::Tolerances;
use kgeom::curve::Ellipse;
use kgeom::param::ParamRange;
use kgeom::surface::Sphere;

/// Intersect an ellipse restricted to a finite range with a finite sphere
/// parameter window.
pub fn intersect_bounded_ellipse_sphere(
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    intersect_bounded_conic_sphere(ConicSphereConfig::ellipse(
        ellipse,
        ellipse_range,
        sphere,
        sphere_range,
        tolerances,
    ))
}
