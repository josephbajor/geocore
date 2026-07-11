//! Graph-backed sampling and the certified planar-offset tessellation slice.

use super::{MeshAcc, Result, TessOptions, UvChain, face_case_a};
use crate::entity::SurfaceId;
use crate::geom::SurfaceGeom;
use crate::store::Store;
use kcore::tolerance::Tolerances;
use kgeom::frame::Frame;
use kgeom::surface::Plane;
use kgeom::vec::{Point3, Vec2};
use kgraph::{EvalLimits, SurfaceDerivativeOrder};

pub(super) fn surface_periodicity(store: &Store, surface: SurfaceId) -> Result<[Option<f64>; 2]> {
    Ok(store
        .eval_context(EvalLimits::default(), Tolerances::default())
        .surface_periodicity(surface)?)
}

pub(super) fn eval_surface_point(store: &Store, surface: SurfaceId, uv: Vec2) -> Result<Point3> {
    Ok(store
        .eval_context(EvalLimits::default(), Tolerances::default())
        .eval_surface(surface, [uv.x, uv.y], SurfaceDerivativeOrder::Position)?
        .p)
}

fn offset_chain_has_planar_leaf(store: &Store, mut surface: SurfaceId) -> bool {
    for _ in 0..=EvalLimits::default().max_dependency_depth {
        match store.geometry().surface(surface) {
            Some(SurfaceGeom::Offset(offset)) => surface = offset.basis(),
            Some(SurfaceGeom::Plane(_)) => return true,
            Some(_) | None => return false,
        }
    }
    false
}

/// Tessellate an offset chain whose ultimate basis is a plane.
///
/// Every cumulative offset is exactly a translated plane, which certifies
/// regularity over the complete trim. A temporary exact plane evaluator lets
/// the existing tessellator retain its chord and maximum-edge guarantees while
/// the topology and graph keep the original procedural identity.
pub(super) fn face_case_planar_offset(
    store: &Store,
    surface: SurfaceId,
    chains: Vec<UvChain>,
    flip: bool,
    acc: &mut MeshAcc,
    opts: &TessOptions,
) -> Result<Vec<[u32; 3]>> {
    if !offset_chain_has_planar_leaf(store, surface) {
        return Err(super::TessellationError::Indeterminate {
            surface,
            source: None,
        });
    }
    let derivatives = store
        .eval_context(EvalLimits::default(), Tolerances::default())
        .eval_surface(surface, [0.0, 0.0], SurfaceDerivativeOrder::First)?;
    let frame = Frame::new(
        derivatives.p,
        derivatives.du.cross(derivatives.dv),
        derivatives.du,
    )?;
    let evaluator = Plane::new(frame);
    face_case_a(&evaluator, chains, flip, acc, opts)
}
