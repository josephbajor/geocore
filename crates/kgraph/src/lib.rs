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
pub use error::{
    EvalError, EvalResult, GeometryGraphError, GeometryGraphResult, capability as eval_capability,
    code as eval_error_code, stage as eval_stage,
};
pub use eval::{
    EvalBudgetProfile, EvalContext, EvalLimits, EvalUsage, SurfaceDerivativeOrder, SurfaceValidity,
    ValidityGap,
};
#[cfg(feature = "benchmark-internals")]
#[doc(hidden)]
pub use graph::GraphBuildObservation;
pub use graph::{
    Curve2dHandle, Curve2dNode, CurveHandle, CurveNode, GeometryChanges, GeometryGraph,
    GeometryRef, SurfaceHandle, SurfaceNode,
};
