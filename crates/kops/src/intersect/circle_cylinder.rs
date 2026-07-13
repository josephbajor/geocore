use super::conic::{ConicCylinderConfig, intersect_bounded_conic_cylinder};
use super::result::CurveSurfaceIntersections;
use kcore::error::Result;
use kcore::tolerance::Tolerances;
use kgeom::curve::Circle;
use kgeom::param::ParamRange;
use kgeom::surface::Cylinder;

/// Intersect a circle restricted to a finite range with a finite cylinder
/// parameter window.
pub fn intersect_bounded_circle_cylinder(
    circle: &Circle,
    circle_range: ParamRange,
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    intersect_bounded_conic_cylinder(ConicCylinderConfig::circle(
        circle,
        circle_range,
        cylinder,
        cylinder_range,
        tolerances,
    ))
}
