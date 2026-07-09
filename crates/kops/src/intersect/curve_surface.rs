use super::line_cone::intersect_bounded_line_cone;
use super::line_cylinder::intersect_bounded_line_cylinder;
use super::line_plane::intersect_bounded_line_plane;
use super::line_sphere::intersect_bounded_line_sphere;
use super::result::CurveSurfaceIntersections;
use kcore::error::{Error, Result};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Curve, Line};
use kgeom::param::ParamRange;
use kgeom::surface::{Cone, Cylinder, Plane, Sphere, Surface};

/// Intersect a curve with a surface over finite curve and surface windows.
///
/// This currently dispatches line/plane, line/cylinder, line/cone, and line/sphere
/// analytic cases.
/// Unsupported curve or surface classes fail explicitly; broader analytic
/// cases and the general subdivision/Newton curve/surface solver remain later
/// M4 work.
pub fn intersect_bounded_curve_surface(
    curve: &dyn Curve,
    curve_range: ParamRange,
    surface: &dyn Surface,
    surface_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    if let Some(line) = as_line(curve) {
        if let Some(plane) = as_plane(surface) {
            return intersect_bounded_line_plane(
                line,
                curve_range,
                plane,
                surface_range,
                tolerances,
            );
        }
        if let Some(cylinder) = as_cylinder(surface) {
            return intersect_bounded_line_cylinder(
                line,
                curve_range,
                cylinder,
                surface_range,
                tolerances,
            );
        }
        if let Some(cone) = as_cone(surface) {
            return intersect_bounded_line_cone(line, curve_range, cone, surface_range, tolerances);
        }
        if let Some(sphere) = as_sphere(surface) {
            return intersect_bounded_line_sphere(
                line,
                curve_range,
                sphere,
                surface_range,
                tolerances,
            );
        }
    }

    Err(Error::InvalidGeometry {
        reason: "unsupported curve/surface intersection class",
    })
}

fn as_line(curve: &dyn Curve) -> Option<&Line> {
    curve.as_any().downcast_ref()
}

fn as_plane(surface: &dyn Surface) -> Option<&Plane> {
    surface.as_any().downcast_ref()
}

fn as_cylinder(surface: &dyn Surface) -> Option<&Cylinder> {
    surface.as_any().downcast_ref()
}

fn as_cone(surface: &dyn Surface) -> Option<&Cone> {
    surface.as_any().downcast_ref()
}

fn as_sphere(surface: &dyn Surface) -> Option<&Sphere> {
    surface.as_any().downcast_ref()
}
