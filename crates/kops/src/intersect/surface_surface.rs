use super::plane_sphere::intersect_bounded_plane_sphere;
use super::result::SurfaceSurfaceIntersections;
use kcore::error::{Error, Result};
use kcore::tolerance::Tolerances;
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Sphere, Surface};

/// Intersect two surfaces over finite parameter windows.
///
/// This is the first SSI dispatcher layer and currently routes the bounded
/// plane/sphere analytic case. Unsupported classes fail explicitly; broader
/// closed forms and marching/subdivision SSI remain later M4 work.
pub fn intersect_bounded_surfaces(
    a: &dyn Surface,
    a_range: [ParamRange; 2],
    b: &dyn Surface,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    if let Some(plane) = as_plane(a)
        && let Some(sphere) = as_sphere(b)
    {
        return intersect_bounded_plane_sphere(plane, a_range, sphere, b_range, tolerances);
    }
    if let Some(sphere) = as_sphere(a)
        && let Some(plane) = as_plane(b)
    {
        return intersect_bounded_plane_sphere(plane, b_range, sphere, a_range, tolerances)
            .map(SurfaceSurfaceIntersections::swapped);
    }

    Err(Error::InvalidGeometry {
        reason: "unsupported surface/surface intersection class",
    })
}

fn as_plane(surface: &dyn Surface) -> Option<&Plane> {
    surface.as_any().downcast_ref()
}

fn as_sphere(surface: &dyn Surface) -> Option<&Sphere> {
    surface.as_any().downcast_ref()
}
