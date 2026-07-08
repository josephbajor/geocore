//! Geometric intersection algorithms and parameter-rich result contracts.

mod line_circle;
mod line_line;
mod result;

pub use line_circle::intersect_bounded_line_circle;
pub use line_line::intersect_bounded_lines;
pub use result::{
    ContactKind, CurveCurveIntersections, CurveCurveOverlap, CurveCurvePoint, ParamOrientation,
    accept_curve_curve_candidate,
};
