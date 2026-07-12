//! Graph-backed sampling and the certified planar-offset tessellation slice.

use super::{BodyTessellationWork, MeshAcc, Result, TessOptions, UvChain, face_case_a};
use crate::entity::SurfaceId;
use crate::store::Store;
use kgeom::frame::Frame;
use kgeom::surface::Plane;
use kgeom::vec::{Point3, Vec2};
use kgraph::{SurfaceClass, SurfaceDerivativeOrder};

pub(super) fn surface_periodicity(
    store: &Store,
    surface: SurfaceId,
    work: &mut BodyTessellationWork<'_, '_, '_>,
) -> Result<[Option<f64>; 2]> {
    work.graph_query(store, |evaluator| evaluator.surface_periodicity(surface))
}

pub(super) fn eval_surface_point(
    store: &Store,
    surface: SurfaceId,
    uv: Vec2,
    work: &mut BodyTessellationWork<'_, '_, '_>,
) -> Result<Point3> {
    Ok(work
        .graph_query(store, |evaluator| {
            evaluator.eval_surface(surface, [uv.x, uv.y], SurfaceDerivativeOrder::Position)
        })?
        .p)
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
    work: &mut BodyTessellationWork<'_, '_, '_>,
) -> Result<Vec<[u32; 3]>> {
    if work.graph_query(store, |evaluator| evaluator.surface_leaf_class(surface))?
        != SurfaceClass::Plane
    {
        return Err(super::TessellationError::Indeterminate {
            surface,
            source: None,
        });
    }
    let derivatives = work.graph_query(store, |evaluator| {
        evaluator.eval_surface(surface, [0.0, 0.0], SurfaceDerivativeOrder::First)
    })?;
    let frame = Frame::new(
        derivatives.p,
        derivatives.du.cross(derivatives.dv),
        derivatives.du,
    )?;
    let evaluator = Plane::new(frame);
    face_case_a(&evaluator, chains, flip, acc, opts, work)
}
