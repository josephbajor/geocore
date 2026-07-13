use super::conic::{ConicCylinderConfig, intersect_bounded_conic_cylinder};
use super::result::CurveSurfaceIntersections;
use kcore::error::Result;
use kcore::tolerance::Tolerances;
use kgeom::curve::Ellipse;
use kgeom::param::ParamRange;
use kgeom::surface::Cylinder;

/// Intersect an ellipse restricted to a finite range with a finite cylinder
/// parameter window.
pub fn intersect_bounded_ellipse_cylinder(
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    intersect_bounded_conic_cylinder(ConicCylinderConfig::ellipse(
        ellipse,
        ellipse_range,
        cylinder,
        cylinder_range,
        tolerances,
    ))
}
