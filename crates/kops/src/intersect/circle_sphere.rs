use super::conic::{
    ConicSphereConfig, clip_constructed_circle_on_sphere, intersect_bounded_conic_sphere,
};
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

/// Clip a circle that its caller has already constructed as lying on `sphere`.
///
/// This internal seam preserves that construction proof instead of asking the
/// rounded derived radius to re-establish an exact squared-distance identity.
pub(super) fn clip_bounded_circle_on_sphere(
    circle: &Circle,
    circle_range: ParamRange,
    sphere: &Sphere,
    sphere_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    clip_constructed_circle_on_sphere(circle, circle_range, sphere, sphere_range, tolerances)
}
