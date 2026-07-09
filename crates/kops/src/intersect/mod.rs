//! Geometric intersection algorithms and parameter-rich result contracts.

mod circle_circle;
mod circle_cone;
mod circle_cylinder;
mod circle_ellipse;
mod circle_sphere;
mod circle_torus;
mod cone_cone;
mod cone_cylinder;
mod cone_sphere;
mod cone_torus;
mod conic;
mod curve_curve;
mod curve_surface;
mod cylinder_cylinder;
mod cylinder_sphere;
mod cylinder_torus;
mod ellipse_cone;
mod ellipse_cylinder;
mod ellipse_ellipse;
mod ellipse_sphere;
mod ellipse_torus;
mod line_circle;
mod line_cone;
mod line_cylinder;
mod line_ellipse;
mod line_line;
mod line_plane;
mod line_sphere;
mod line_torus;
mod nurbs_plane;
mod planar_curve_plane;
mod plane_cone;
mod plane_cylinder;
mod plane_plane;
mod plane_sphere;
mod plane_torus;
mod result;
mod sphere_sphere;
mod sphere_torus;
mod surface_surface;
mod torus_torus;

pub use circle_circle::intersect_bounded_circles;
pub use circle_cone::intersect_bounded_circle_cone;
pub use circle_cylinder::intersect_bounded_circle_cylinder;
pub use circle_ellipse::intersect_bounded_circle_ellipse;
pub use circle_sphere::intersect_bounded_circle_sphere;
pub use circle_torus::intersect_bounded_circle_torus;
pub use cone_cone::intersect_bounded_cones;
pub use cone_cylinder::intersect_bounded_cone_cylinder;
pub use cone_sphere::intersect_bounded_cone_sphere;
pub use cone_torus::intersect_bounded_cone_torus;
pub use curve_curve::intersect_bounded_curves;
pub use curve_surface::intersect_bounded_curve_surface;
pub use cylinder_cylinder::intersect_bounded_cylinders;
pub use cylinder_sphere::intersect_bounded_cylinder_sphere;
pub use cylinder_torus::intersect_bounded_cylinder_torus;
pub use ellipse_cone::intersect_bounded_ellipse_cone;
pub use ellipse_cylinder::intersect_bounded_ellipse_cylinder;
pub use ellipse_ellipse::intersect_bounded_ellipses;
pub use ellipse_sphere::intersect_bounded_ellipse_sphere;
pub use ellipse_torus::intersect_bounded_ellipse_torus;
pub use line_circle::intersect_bounded_line_circle;
pub use line_cone::intersect_bounded_line_cone;
pub use line_cylinder::intersect_bounded_line_cylinder;
pub use line_ellipse::intersect_bounded_line_ellipse;
pub use line_line::intersect_bounded_lines;
pub use line_plane::intersect_bounded_line_plane;
pub use line_sphere::intersect_bounded_line_sphere;
pub use line_torus::intersect_bounded_line_torus;
pub use nurbs_plane::intersect_bounded_nurbs_plane;
pub use planar_curve_plane::{intersect_bounded_circle_plane, intersect_bounded_ellipse_plane};
pub use plane_cone::intersect_bounded_plane_cone;
pub use plane_cylinder::intersect_bounded_plane_cylinder;
pub use plane_plane::intersect_bounded_planes;
pub use plane_sphere::intersect_bounded_plane_sphere;
pub use plane_torus::intersect_bounded_plane_torus;
pub use result::{
    ContactKind, CurveCurveIntersections, CurveCurveOverlap, CurveCurvePoint,
    CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint, ParamOrientation,
    SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
    SurfaceSurfacePoint, accept_curve_curve_candidate, accept_curve_surface_candidate,
    accept_surface_surface_candidate,
};
pub use sphere_sphere::intersect_bounded_spheres;
pub use sphere_torus::intersect_bounded_sphere_torus;
pub use surface_surface::intersect_bounded_surfaces;
pub use torus_torus::intersect_bounded_tori;
