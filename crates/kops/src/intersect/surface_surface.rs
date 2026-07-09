use super::cone_cone::intersect_bounded_cones;
use super::cone_cylinder::intersect_bounded_cone_cylinder;
use super::cone_nurbs_surface::intersect_bounded_cone_nurbs_surface;
use super::cone_sphere::intersect_bounded_cone_sphere;
use super::cone_torus::intersect_bounded_cone_torus;
use super::cylinder_cylinder::intersect_bounded_cylinders;
use super::cylinder_nurbs_surface::intersect_bounded_cylinder_nurbs_surface;
use super::cylinder_sphere::intersect_bounded_cylinder_sphere;
use super::cylinder_torus::intersect_bounded_cylinder_torus;
use super::plane_cone::intersect_bounded_plane_cone;
use super::plane_cylinder::intersect_bounded_plane_cylinder;
use super::plane_nurbs_surface::intersect_bounded_plane_nurbs_surface;
use super::plane_plane::intersect_bounded_planes;
use super::plane_sphere::intersect_bounded_plane_sphere;
use super::plane_torus::intersect_bounded_plane_torus;
use super::result::SurfaceSurfaceIntersections;
use super::sphere_nurbs_surface::intersect_bounded_sphere_nurbs_surface;
use super::sphere_sphere::intersect_bounded_spheres;
use super::sphere_torus::intersect_bounded_sphere_torus;
use super::torus_torus::intersect_bounded_tori;
use kcore::error::{Error, Result};
use kcore::tolerance::Tolerances;
use kgeom::nurbs::NurbsSurface;
use kgeom::param::ParamRange;
use kgeom::surface::{Cone, Cylinder, Plane, Sphere, Surface, Torus};

/// Intersect two surfaces over finite parameter windows.
///
/// This SSI dispatcher routes supported bounded analytic pairs. Unsupported
/// classes fail explicitly; broader closed forms and adaptive
/// marching/subdivision SSI remain later M4 work.
pub fn intersect_bounded_surfaces(
    a: &dyn Surface,
    a_range: [ParamRange; 2],
    b: &dyn Surface,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    if let Some(sphere_a) = as_sphere(a)
        && let Some(sphere_b) = as_sphere(b)
    {
        return intersect_bounded_spheres(sphere_a, a_range, sphere_b, b_range, tolerances);
    }
    if let Some(sphere) = as_sphere(a)
        && let Some(torus) = as_torus(b)
    {
        return intersect_bounded_sphere_torus(sphere, a_range, torus, b_range, tolerances);
    }
    if let Some(torus) = as_torus(a)
        && let Some(sphere) = as_sphere(b)
    {
        return intersect_bounded_sphere_torus(sphere, b_range, torus, a_range, tolerances)
            .map(SurfaceSurfaceIntersections::swapped);
    }
    if let Some(torus_a) = as_torus(a)
        && let Some(torus_b) = as_torus(b)
    {
        return intersect_bounded_tori(torus_a, a_range, torus_b, b_range, tolerances);
    }
    if let Some(cylinder_a) = as_cylinder(a)
        && let Some(cylinder_b) = as_cylinder(b)
    {
        return intersect_bounded_cylinders(cylinder_a, a_range, cylinder_b, b_range, tolerances);
    }
    if let Some(cylinder) = as_cylinder(a)
        && let Some(torus) = as_torus(b)
    {
        return intersect_bounded_cylinder_torus(cylinder, a_range, torus, b_range, tolerances);
    }
    if let Some(torus) = as_torus(a)
        && let Some(cylinder) = as_cylinder(b)
    {
        return intersect_bounded_cylinder_torus(cylinder, b_range, torus, a_range, tolerances)
            .map(SurfaceSurfaceIntersections::swapped);
    }
    if let Some(cone_a) = as_cone(a)
        && let Some(cone_b) = as_cone(b)
    {
        return intersect_bounded_cones(cone_a, a_range, cone_b, b_range, tolerances);
    }
    if let Some(cone) = as_cone(a)
        && let Some(torus) = as_torus(b)
    {
        return intersect_bounded_cone_torus(cone, a_range, torus, b_range, tolerances);
    }
    if let Some(torus) = as_torus(a)
        && let Some(cone) = as_cone(b)
    {
        return intersect_bounded_cone_torus(cone, b_range, torus, a_range, tolerances)
            .map(SurfaceSurfaceIntersections::swapped);
    }
    if let Some(cone) = as_cone(a)
        && let Some(cylinder) = as_cylinder(b)
    {
        return intersect_bounded_cone_cylinder(cone, a_range, cylinder, b_range, tolerances);
    }
    if let Some(cylinder) = as_cylinder(a)
        && let Some(cone) = as_cone(b)
    {
        return intersect_bounded_cone_cylinder(cone, b_range, cylinder, a_range, tolerances)
            .map(SurfaceSurfaceIntersections::swapped);
    }
    if let Some(cylinder) = as_cylinder(a)
        && let Some(sphere) = as_sphere(b)
    {
        return intersect_bounded_cylinder_sphere(cylinder, a_range, sphere, b_range, tolerances);
    }
    if let Some(sphere) = as_sphere(a)
        && let Some(cylinder) = as_cylinder(b)
    {
        return intersect_bounded_cylinder_sphere(cylinder, b_range, sphere, a_range, tolerances)
            .map(SurfaceSurfaceIntersections::swapped);
    }
    if let Some(cone) = as_cone(a)
        && let Some(sphere) = as_sphere(b)
    {
        return intersect_bounded_cone_sphere(cone, a_range, sphere, b_range, tolerances);
    }
    if let Some(sphere) = as_sphere(a)
        && let Some(cone) = as_cone(b)
    {
        return intersect_bounded_cone_sphere(cone, b_range, sphere, a_range, tolerances)
            .map(SurfaceSurfaceIntersections::swapped);
    }
    if let Some(plane_a) = as_plane(a)
        && let Some(plane_b) = as_plane(b)
    {
        return intersect_bounded_planes(plane_a, a_range, plane_b, b_range, tolerances);
    }
    if let Some(plane) = as_plane(a)
        && let Some(cylinder) = as_cylinder(b)
    {
        return intersect_bounded_plane_cylinder(plane, a_range, cylinder, b_range, tolerances);
    }
    if let Some(cylinder) = as_cylinder(a)
        && let Some(plane) = as_plane(b)
    {
        return intersect_bounded_plane_cylinder(plane, b_range, cylinder, a_range, tolerances)
            .map(SurfaceSurfaceIntersections::swapped);
    }
    if let Some(plane) = as_plane(a)
        && let Some(cone) = as_cone(b)
    {
        return intersect_bounded_plane_cone(plane, a_range, cone, b_range, tolerances);
    }
    if let Some(cone) = as_cone(a)
        && let Some(plane) = as_plane(b)
    {
        return intersect_bounded_plane_cone(plane, b_range, cone, a_range, tolerances)
            .map(SurfaceSurfaceIntersections::swapped);
    }
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
    if let Some(plane) = as_plane(a)
        && let Some(torus) = as_torus(b)
    {
        return intersect_bounded_plane_torus(plane, a_range, torus, b_range, tolerances);
    }
    if let Some(torus) = as_torus(a)
        && let Some(plane) = as_plane(b)
    {
        return intersect_bounded_plane_torus(plane, b_range, torus, a_range, tolerances)
            .map(SurfaceSurfaceIntersections::swapped);
    }
    if let Some(plane) = as_plane(a)
        && let Some(nurbs) = as_nurbs_surface(b)
    {
        return intersect_bounded_plane_nurbs_surface(plane, a_range, nurbs, b_range, tolerances);
    }
    if let Some(nurbs) = as_nurbs_surface(a)
        && let Some(plane) = as_plane(b)
    {
        return intersect_bounded_plane_nurbs_surface(plane, b_range, nurbs, a_range, tolerances)
            .map(SurfaceSurfaceIntersections::swapped);
    }
    if let Some(sphere) = as_sphere(a)
        && let Some(nurbs) = as_nurbs_surface(b)
    {
        return intersect_bounded_sphere_nurbs_surface(sphere, a_range, nurbs, b_range, tolerances);
    }
    if let Some(nurbs) = as_nurbs_surface(a)
        && let Some(sphere) = as_sphere(b)
    {
        return intersect_bounded_sphere_nurbs_surface(sphere, b_range, nurbs, a_range, tolerances)
            .map(SurfaceSurfaceIntersections::swapped);
    }
    if let Some(cylinder) = as_cylinder(a)
        && let Some(nurbs) = as_nurbs_surface(b)
    {
        return intersect_bounded_cylinder_nurbs_surface(
            cylinder, a_range, nurbs, b_range, tolerances,
        );
    }
    if let Some(nurbs) = as_nurbs_surface(a)
        && let Some(cylinder) = as_cylinder(b)
    {
        return intersect_bounded_cylinder_nurbs_surface(
            cylinder, b_range, nurbs, a_range, tolerances,
        )
        .map(SurfaceSurfaceIntersections::swapped);
    }
    if let Some(cone) = as_cone(a)
        && let Some(nurbs) = as_nurbs_surface(b)
    {
        return intersect_bounded_cone_nurbs_surface(cone, a_range, nurbs, b_range, tolerances);
    }
    if let Some(nurbs) = as_nurbs_surface(a)
        && let Some(cone) = as_cone(b)
    {
        return intersect_bounded_cone_nurbs_surface(cone, b_range, nurbs, a_range, tolerances)
            .map(SurfaceSurfaceIntersections::swapped);
    }

    Err(Error::InvalidGeometry {
        reason: "unsupported surface/surface intersection class",
    })
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

fn as_torus(surface: &dyn Surface) -> Option<&Torus> {
    surface.as_any().downcast_ref()
}

fn as_nurbs_surface(surface: &dyn Surface) -> Option<&NurbsSurface> {
    surface.as_any().downcast_ref()
}
