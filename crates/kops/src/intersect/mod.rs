//! Geometric intersection algorithms and parameter-rich result contracts.

mod circle_circle;
mod circle_ellipse;
mod conic;
mod curve_curve;
mod curve_surface;
mod ellipse_ellipse;
mod line_circle;
mod line_cone;
mod line_cylinder;
mod line_ellipse;
mod line_line;
mod line_plane;
mod line_sphere;
mod result;

pub use circle_circle::intersect_bounded_circles;
pub use circle_ellipse::intersect_bounded_circle_ellipse;
pub use curve_curve::intersect_bounded_curves;
pub use curve_surface::intersect_bounded_curve_surface;
pub use ellipse_ellipse::intersect_bounded_ellipses;
pub use line_circle::intersect_bounded_line_circle;
pub use line_cone::intersect_bounded_line_cone;
pub use line_cylinder::intersect_bounded_line_cylinder;
pub use line_ellipse::intersect_bounded_line_ellipse;
pub use line_line::intersect_bounded_lines;
pub use line_plane::intersect_bounded_line_plane;
pub use line_sphere::intersect_bounded_line_sphere;
pub use result::{
    ContactKind, CurveCurveIntersections, CurveCurveOverlap, CurveCurvePoint,
    CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint, ParamOrientation,
    accept_curve_curve_candidate, accept_curve_surface_candidate,
};
