use super::cone_cone::intersect_bounded_cones;
use super::cone_cylinder::intersect_bounded_cone_cylinder;
use super::cone_nurbs_surface::intersect_bounded_cone_nurbs_surface;
use super::cone_sphere::intersect_bounded_cone_sphere;
use super::cone_torus::intersect_bounded_cone_torus;
use super::cylinder_cylinder::intersect_bounded_cylinders;
use super::cylinder_nurbs_surface::intersect_bounded_cylinder_nurbs_surface;
use super::cylinder_sphere::intersect_bounded_cylinder_sphere;
use super::cylinder_torus::intersect_bounded_cylinder_torus;
use super::error::{IntersectionError, IntersectionResult};
use super::geometry_class::SurfaceDispatch;
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
use super::torus_nurbs_surface::intersect_bounded_torus_nurbs_surface;
use super::torus_torus::intersect_bounded_tori;
use kcore::tolerance::Tolerances;
use kgeom::param::ParamRange;
use kgeom::surface::Surface;

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
) -> IntersectionResult<SurfaceSurfaceIntersections> {
    let class_a = SurfaceDispatch::inspect(a);
    let class_b = SurfaceDispatch::inspect(b);
    let result = match (class_a, class_b) {
        (Some(SurfaceDispatch::Sphere(a)), Some(SurfaceDispatch::Sphere(b))) => {
            intersect_bounded_spheres(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Sphere(a)), Some(SurfaceDispatch::Torus(b))) => {
            intersect_bounded_sphere_torus(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Torus(a)), Some(SurfaceDispatch::Sphere(b))) => {
            intersect_bounded_sphere_torus(b, b_range, a, a_range, tolerances)
                .map(SurfaceSurfaceIntersections::swapped)
        }
        (Some(SurfaceDispatch::Torus(a)), Some(SurfaceDispatch::Torus(b))) => {
            intersect_bounded_tori(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Cylinder(a)), Some(SurfaceDispatch::Cylinder(b))) => {
            intersect_bounded_cylinders(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Cylinder(a)), Some(SurfaceDispatch::Torus(b))) => {
            intersect_bounded_cylinder_torus(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Torus(a)), Some(SurfaceDispatch::Cylinder(b))) => {
            intersect_bounded_cylinder_torus(b, b_range, a, a_range, tolerances)
                .map(SurfaceSurfaceIntersections::swapped)
        }
        (Some(SurfaceDispatch::Cone(a)), Some(SurfaceDispatch::Cone(b))) => {
            intersect_bounded_cones(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Cone(a)), Some(SurfaceDispatch::Torus(b))) => {
            intersect_bounded_cone_torus(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Torus(a)), Some(SurfaceDispatch::Cone(b))) => {
            intersect_bounded_cone_torus(b, b_range, a, a_range, tolerances)
                .map(SurfaceSurfaceIntersections::swapped)
        }
        (Some(SurfaceDispatch::Cone(a)), Some(SurfaceDispatch::Cylinder(b))) => {
            intersect_bounded_cone_cylinder(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Cylinder(a)), Some(SurfaceDispatch::Cone(b))) => {
            intersect_bounded_cone_cylinder(b, b_range, a, a_range, tolerances)
                .map(SurfaceSurfaceIntersections::swapped)
        }
        (Some(SurfaceDispatch::Cylinder(a)), Some(SurfaceDispatch::Sphere(b))) => {
            intersect_bounded_cylinder_sphere(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Sphere(a)), Some(SurfaceDispatch::Cylinder(b))) => {
            intersect_bounded_cylinder_sphere(b, b_range, a, a_range, tolerances)
                .map(SurfaceSurfaceIntersections::swapped)
        }
        (Some(SurfaceDispatch::Cone(a)), Some(SurfaceDispatch::Sphere(b))) => {
            intersect_bounded_cone_sphere(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Sphere(a)), Some(SurfaceDispatch::Cone(b))) => {
            intersect_bounded_cone_sphere(b, b_range, a, a_range, tolerances)
                .map(SurfaceSurfaceIntersections::swapped)
        }
        (Some(SurfaceDispatch::Plane(a)), Some(SurfaceDispatch::Plane(b))) => {
            intersect_bounded_planes(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Plane(a)), Some(SurfaceDispatch::Cylinder(b))) => {
            intersect_bounded_plane_cylinder(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Cylinder(a)), Some(SurfaceDispatch::Plane(b))) => {
            intersect_bounded_plane_cylinder(b, b_range, a, a_range, tolerances)
                .map(SurfaceSurfaceIntersections::swapped)
        }
        (Some(SurfaceDispatch::Plane(a)), Some(SurfaceDispatch::Cone(b))) => {
            intersect_bounded_plane_cone(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Cone(a)), Some(SurfaceDispatch::Plane(b))) => {
            intersect_bounded_plane_cone(b, b_range, a, a_range, tolerances)
                .map(SurfaceSurfaceIntersections::swapped)
        }
        (Some(SurfaceDispatch::Plane(a)), Some(SurfaceDispatch::Sphere(b))) => {
            intersect_bounded_plane_sphere(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Sphere(a)), Some(SurfaceDispatch::Plane(b))) => {
            intersect_bounded_plane_sphere(b, b_range, a, a_range, tolerances)
                .map(SurfaceSurfaceIntersections::swapped)
        }
        (Some(SurfaceDispatch::Plane(a)), Some(SurfaceDispatch::Torus(b))) => {
            intersect_bounded_plane_torus(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Torus(a)), Some(SurfaceDispatch::Plane(b))) => {
            intersect_bounded_plane_torus(b, b_range, a, a_range, tolerances)
                .map(SurfaceSurfaceIntersections::swapped)
        }
        (Some(SurfaceDispatch::Plane(a)), Some(SurfaceDispatch::Nurbs(b))) => {
            intersect_bounded_plane_nurbs_surface(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Nurbs(a)), Some(SurfaceDispatch::Plane(b))) => {
            intersect_bounded_plane_nurbs_surface(b, b_range, a, a_range, tolerances)
                .map(SurfaceSurfaceIntersections::swapped)
        }
        (Some(SurfaceDispatch::Sphere(a)), Some(SurfaceDispatch::Nurbs(b))) => {
            intersect_bounded_sphere_nurbs_surface(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Nurbs(a)), Some(SurfaceDispatch::Sphere(b))) => {
            intersect_bounded_sphere_nurbs_surface(b, b_range, a, a_range, tolerances)
                .map(SurfaceSurfaceIntersections::swapped)
        }
        (Some(SurfaceDispatch::Cylinder(a)), Some(SurfaceDispatch::Nurbs(b))) => {
            intersect_bounded_cylinder_nurbs_surface(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Nurbs(a)), Some(SurfaceDispatch::Cylinder(b))) => {
            intersect_bounded_cylinder_nurbs_surface(b, b_range, a, a_range, tolerances)
                .map(SurfaceSurfaceIntersections::swapped)
        }
        (Some(SurfaceDispatch::Cone(a)), Some(SurfaceDispatch::Nurbs(b))) => {
            intersect_bounded_cone_nurbs_surface(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Nurbs(a)), Some(SurfaceDispatch::Cone(b))) => {
            intersect_bounded_cone_nurbs_surface(b, b_range, a, a_range, tolerances)
                .map(SurfaceSurfaceIntersections::swapped)
        }
        (Some(SurfaceDispatch::Torus(a)), Some(SurfaceDispatch::Nurbs(b))) => {
            intersect_bounded_torus_nurbs_surface(a, a_range, b, b_range, tolerances)
        }
        (Some(SurfaceDispatch::Nurbs(a)), Some(SurfaceDispatch::Torus(b))) => {
            intersect_bounded_torus_nurbs_surface(b, b_range, a, a_range, tolerances)
                .map(SurfaceSurfaceIntersections::swapped)
        }
        _ => {
            return Err(IntersectionError::UnsupportedSurfacePair {
                class_a: class_a.map(|class| class.class().key()),
                class_b: class_b.map(|class| class.class().key()),
            });
        }
    };
    result.map_err(IntersectionError::from)
}
