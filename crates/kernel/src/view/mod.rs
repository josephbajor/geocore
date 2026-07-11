//! Read-only semantic topology views over one immutable part borrow.

mod body;
mod boundary;
mod edge;
mod geometry;
mod part;

pub use body::{BodyView, RegionView, ShellView};
pub use boundary::{FaceView, FinView, LoopView};
pub use edge::{EdgeView, VertexView};
pub use geometry::{CurveView, PcurveView, SurfaceView};
