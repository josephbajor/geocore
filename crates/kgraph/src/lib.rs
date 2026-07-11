//! `kgraph` — immutable geometry identity and bounded graph evaluation.
//!
//! Pure leaf mathematics remains in `kgeom`. This crate gives those values
//! stable, typed identity and provides the dependency/evaluation boundary used
//! by future procedural geometry without depending on topology or operations.

mod class;
mod descriptor;
mod error;
mod eval;
mod graph;

pub use class::{Curve2dClass, CurveClass, GeometryClassKey, SurfaceClass};
pub use descriptor::{
    Curve2dDescriptor, CurveDescriptor, GeometryDependencies, OffsetSurfaceDescriptor,
    SurfaceDescriptor,
};
pub use error::{EvalError, EvalResult, GeometryGraphError, GeometryGraphResult};
pub use eval::{EvalContext, EvalLimits, SurfaceDerivativeOrder, SurfaceValidity, ValidityGap};
pub use graph::{
    Curve2dHandle, Curve2dNode, CurveHandle, CurveNode, GeometryChanges, GeometryGraph,
    GeometryRef, SurfaceHandle, SurfaceNode,
};
