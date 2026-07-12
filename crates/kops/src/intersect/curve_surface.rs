use super::circle_cone::intersect_bounded_circle_cone;
use super::circle_cylinder::intersect_bounded_circle_cylinder;
use super::circle_sphere::intersect_bounded_circle_sphere;
use super::circle_torus::intersect_bounded_circle_torus;
use super::ellipse_cone::intersect_bounded_ellipse_cone;
use super::ellipse_cylinder::intersect_bounded_ellipse_cylinder;
use super::ellipse_sphere::intersect_bounded_ellipse_sphere;
use super::ellipse_torus::intersect_bounded_ellipse_torus;
use super::error::{IntersectionError, IntersectionResult};
use super::geometry_class::{CurveDispatch, SurfaceDispatch};
use super::line_cone::intersect_bounded_line_cone;
use super::line_cylinder::intersect_bounded_line_cylinder;
use super::line_plane::intersect_bounded_line_plane;
use super::line_sphere::intersect_bounded_line_sphere;
use super::line_torus::intersect_bounded_line_torus;
use super::nurbs_cone::intersect_bounded_nurbs_cone;
use super::nurbs_cylinder::intersect_bounded_nurbs_cylinder;
use super::nurbs_plane::intersect_bounded_nurbs_plane;
use super::nurbs_sphere::intersect_bounded_nurbs_sphere;
use super::nurbs_torus::intersect_bounded_nurbs_torus;
use super::planar_curve_plane::{intersect_bounded_circle_plane, intersect_bounded_ellipse_plane};
use super::result::CurveSurfaceIntersections;
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::param::ParamRange;
use kgeom::surface::Surface;

/// Intersect a curve with a surface over finite curve and surface windows.
///
/// Inputs are inspected once and routed through one typed arm per supported
/// class pair. Unsupported curve or surface classes fail explicitly; broader
/// analytic cases and the certified subdivision/Newton curve/surface solver
/// remain later M4 work.
pub fn intersect_bounded_curve_surface(
    curve: &dyn Curve,
    curve_range: ParamRange,
    surface: &dyn Surface,
    surface_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> IntersectionResult<CurveSurfaceIntersections> {
    let curve = CurveDispatch::inspect(curve);
    let surface = SurfaceDispatch::inspect(surface);
    let (Some(curve), Some(surface)) = (curve, surface) else {
        return unsupported(curve, surface);
    };

    let result = match (curve, surface) {
        (CurveDispatch::Line(curve), SurfaceDispatch::Plane(surface)) => {
            intersect_bounded_line_plane(curve, curve_range, surface, surface_range, tolerances)
        }
        (CurveDispatch::Line(curve), SurfaceDispatch::Cylinder(surface)) => {
            intersect_bounded_line_cylinder(curve, curve_range, surface, surface_range, tolerances)
        }
        (CurveDispatch::Line(curve), SurfaceDispatch::Cone(surface)) => {
            intersect_bounded_line_cone(curve, curve_range, surface, surface_range, tolerances)
        }
        (CurveDispatch::Line(curve), SurfaceDispatch::Sphere(surface)) => {
            intersect_bounded_line_sphere(curve, curve_range, surface, surface_range, tolerances)
        }
        (CurveDispatch::Line(curve), SurfaceDispatch::Torus(surface)) => {
            intersect_bounded_line_torus(curve, curve_range, surface, surface_range, tolerances)
        }
        (CurveDispatch::Circle(curve), SurfaceDispatch::Plane(surface)) => {
            intersect_bounded_circle_plane(curve, curve_range, surface, surface_range, tolerances)
        }
        (CurveDispatch::Circle(curve), SurfaceDispatch::Cylinder(surface)) => {
            intersect_bounded_circle_cylinder(
                curve,
                curve_range,
                surface,
                surface_range,
                tolerances,
            )
        }
        (CurveDispatch::Circle(curve), SurfaceDispatch::Cone(surface)) => {
            intersect_bounded_circle_cone(curve, curve_range, surface, surface_range, tolerances)
        }
        (CurveDispatch::Circle(curve), SurfaceDispatch::Sphere(surface)) => {
            intersect_bounded_circle_sphere(curve, curve_range, surface, surface_range, tolerances)
        }
        (CurveDispatch::Circle(curve), SurfaceDispatch::Torus(surface)) => {
            intersect_bounded_circle_torus(curve, curve_range, surface, surface_range, tolerances)
        }
        (CurveDispatch::Ellipse(curve), SurfaceDispatch::Plane(surface)) => {
            intersect_bounded_ellipse_plane(curve, curve_range, surface, surface_range, tolerances)
        }
        (CurveDispatch::Ellipse(curve), SurfaceDispatch::Cylinder(surface)) => {
            intersect_bounded_ellipse_cylinder(
                curve,
                curve_range,
                surface,
                surface_range,
                tolerances,
            )
        }
        (CurveDispatch::Ellipse(curve), SurfaceDispatch::Cone(surface)) => {
            intersect_bounded_ellipse_cone(curve, curve_range, surface, surface_range, tolerances)
        }
        (CurveDispatch::Ellipse(curve), SurfaceDispatch::Sphere(surface)) => {
            intersect_bounded_ellipse_sphere(curve, curve_range, surface, surface_range, tolerances)
        }
        (CurveDispatch::Ellipse(curve), SurfaceDispatch::Torus(surface)) => {
            intersect_bounded_ellipse_torus(curve, curve_range, surface, surface_range, tolerances)
        }
        (CurveDispatch::Nurbs(curve), SurfaceDispatch::Plane(surface)) => {
            intersect_bounded_nurbs_plane(curve, curve_range, surface, surface_range, tolerances)
        }
        (CurveDispatch::Nurbs(curve), SurfaceDispatch::Cylinder(surface)) => {
            intersect_bounded_nurbs_cylinder(curve, curve_range, surface, surface_range, tolerances)
        }
        (CurveDispatch::Nurbs(curve), SurfaceDispatch::Cone(surface)) => {
            intersect_bounded_nurbs_cone(curve, curve_range, surface, surface_range, tolerances)
        }
        (CurveDispatch::Nurbs(curve), SurfaceDispatch::Sphere(surface)) => {
            intersect_bounded_nurbs_sphere(curve, curve_range, surface, surface_range, tolerances)
        }
        (CurveDispatch::Nurbs(curve), SurfaceDispatch::Torus(surface)) => {
            intersect_bounded_nurbs_torus(curve, curve_range, surface, surface_range, tolerances)
        }
        _ => return unsupported(Some(curve), Some(surface)),
    };
    result.map_err(IntersectionError::from)
}

fn unsupported<T>(
    curve: Option<CurveDispatch<'_>>,
    surface: Option<SurfaceDispatch<'_>>,
) -> IntersectionResult<T> {
    Err(IntersectionError::UnsupportedCurveSurfacePair {
        curve_class: curve.map(|class| class.class().key()),
        surface_class: surface.map(|class| class.class().key()),
    })
}
