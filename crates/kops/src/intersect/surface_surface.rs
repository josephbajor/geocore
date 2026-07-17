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
use super::nurbs_nurbs_surface::intersect_bounded_nurbs_nurbs_surfaces;
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
/// Inputs normalize to one canonical arm per unordered class pair. Unsupported
/// classes fail explicitly; adaptive certified fallback remains later M4 work.
pub fn intersect_bounded_surfaces(
    a: &dyn Surface,
    a_range: [ParamRange; 2],
    b: &dyn Surface,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> IntersectionResult<SurfaceSurfaceIntersections> {
    let original_a = SurfaceDispatch::inspect(a);
    let original_b = SurfaceDispatch::inspect(b);
    let (Some(mut a), Some(mut b)) = (original_a, original_b) else {
        return unsupported(original_a, original_b);
    };
    let (mut a_range, mut b_range) = (a_range, b_range);
    let swapped = rank(a) > rank(b);
    if swapped {
        core::mem::swap(&mut a, &mut b);
        core::mem::swap(&mut a_range, &mut b_range);
    }

    let result = match (a, b) {
        (SurfaceDispatch::Plane(a), SurfaceDispatch::Plane(b)) => {
            intersect_bounded_planes(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Plane(a), SurfaceDispatch::Cone(b)) => {
            intersect_bounded_plane_cone(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Plane(a), SurfaceDispatch::Cylinder(b)) => {
            intersect_bounded_plane_cylinder(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Plane(a), SurfaceDispatch::Sphere(b)) => {
            intersect_bounded_plane_sphere(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Plane(a), SurfaceDispatch::Torus(b)) => {
            intersect_bounded_plane_torus(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Plane(a), SurfaceDispatch::Nurbs(b)) => {
            intersect_bounded_plane_nurbs_surface(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Cone(a), SurfaceDispatch::Cone(b)) => {
            intersect_bounded_cones(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Cone(a), SurfaceDispatch::Cylinder(b)) => {
            intersect_bounded_cone_cylinder(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Cone(a), SurfaceDispatch::Sphere(b)) => {
            intersect_bounded_cone_sphere(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Cone(a), SurfaceDispatch::Torus(b)) => {
            intersect_bounded_cone_torus(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Cone(a), SurfaceDispatch::Nurbs(b)) => {
            intersect_bounded_cone_nurbs_surface(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Cylinder(a), SurfaceDispatch::Cylinder(b)) => {
            intersect_bounded_cylinders(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Cylinder(a), SurfaceDispatch::Sphere(b)) => {
            intersect_bounded_cylinder_sphere(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Cylinder(a), SurfaceDispatch::Torus(b)) => {
            intersect_bounded_cylinder_torus(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Cylinder(a), SurfaceDispatch::Nurbs(b)) => {
            intersect_bounded_cylinder_nurbs_surface(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Sphere(a), SurfaceDispatch::Sphere(b)) => {
            intersect_bounded_spheres(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Sphere(a), SurfaceDispatch::Torus(b)) => {
            intersect_bounded_sphere_torus(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Sphere(a), SurfaceDispatch::Nurbs(b)) => {
            intersect_bounded_sphere_nurbs_surface(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Torus(a), SurfaceDispatch::Torus(b)) => {
            intersect_bounded_tori(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Torus(a), SurfaceDispatch::Nurbs(b)) => {
            intersect_bounded_torus_nurbs_surface(a, a_range, b, b_range, tolerances)
        }
        (SurfaceDispatch::Nurbs(a), SurfaceDispatch::Nurbs(b)) => {
            intersect_bounded_nurbs_nurbs_surfaces(a, a_range, b, b_range, tolerances)
        }
        _ => return unsupported(original_a, original_b),
    };
    result
        .map(|result| if swapped { result.swapped() } else { result })
        .map_err(IntersectionError::from)
}

fn rank(surface: SurfaceDispatch<'_>) -> u8 {
    match surface {
        SurfaceDispatch::Plane(_) => 0,
        SurfaceDispatch::Cone(_) => 1,
        SurfaceDispatch::Cylinder(_) => 2,
        SurfaceDispatch::Sphere(_) => 3,
        SurfaceDispatch::Torus(_) => 4,
        SurfaceDispatch::Nurbs(_) => 5,
    }
}

fn unsupported<T>(
    a: Option<SurfaceDispatch<'_>>,
    b: Option<SurfaceDispatch<'_>>,
) -> IntersectionResult<T> {
    Err(IntersectionError::UnsupportedSurfacePair {
        class_a: a.map(|class| class.class().key()),
        class_b: b.map(|class| class.class().key()),
    })
}
