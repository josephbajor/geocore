//! Geometric intersection algorithms and parameter-rich result contracts.

mod circle_circle;
mod circle_ellipse;
mod conic;
mod ellipse_ellipse;
mod line_circle;
mod line_ellipse;
mod line_line;
mod result;

pub use circle_circle::intersect_bounded_circles;
pub use circle_ellipse::intersect_bounded_circle_ellipse;
pub use ellipse_ellipse::intersect_bounded_ellipses;
pub use line_circle::intersect_bounded_line_circle;
pub use line_ellipse::intersect_bounded_line_ellipse;
pub use line_line::intersect_bounded_lines;
pub use result::{
    ContactKind, CurveCurveIntersections, CurveCurveOverlap, CurveCurvePoint, ParamOrientation,
    accept_curve_curve_candidate,
};
