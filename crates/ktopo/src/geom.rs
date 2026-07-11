//! Compatibility names for geometry graph descriptors.
//!
//! Geometry values are owned exactly once by [`kgraph::GeometryGraph`]. These
//! historical names remain aliases so callers can migrate independently while
//! preserving exact enum class inspection and leaf evaluator access.

pub use kgraph::{
    Curve2dDescriptor as Curve2dGeom, CurveDescriptor as CurveGeom,
    SurfaceDescriptor as SurfaceGeom,
};
