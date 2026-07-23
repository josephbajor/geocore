//! Circular-cap source authority for rounded Plane/Cylinder rulings.
//!
//! This module proves only the source-family relation omitted by the ordinary
//! exact-dyadic graph certificate. The caller independently retains strict
//! secancy, signed carrier-axis, and complete-range residual obligations.

use crate::entity::{EdgeId, FaceId, FinId, PcurveEndpointKind};
use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use crate::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};
use crate::store::Store;
use kcore::predicates::{Orientation, affine_dot3};
use kgeom::curve::Curve;
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::Vec3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct CircularCapSourceWitness {
    pub(super) cap_edge: EdgeId,
    pub(super) cap_fin: FinId,
    pub(super) source_cylinder_fin: FinId,
    pub(super) source_cylinder_face: FaceId,
}

/// Certify an ordinary complete circular cap and its paired source Cylinder.
///
/// Every topology mismatch and unsupported representation fails closed. The
/// exact semantic perpendicular relation is either an exact dyadic dot zero
/// or exact identity with one of the source Cylinder frame's radial axes.
pub(super) fn certify_circular_cap_source(
    store: &Store,
    face_id: FaceId,
    result_plane: Plane,
    opposing_axis: Vec3,
    tolerance: f64,
) -> Option<CircularCapSourceWitness> {
    let face = store.get(face_id).ok()?;
    let [loop_id] = face.loops.as_slice() else {
        return None;
    };
    let loop_ = store.get(*loop_id).ok()?;
    let [cap_fin_id] = loop_.fins.as_slice() else {
        return None;
    };
    let cap_fin = store.get(*cap_fin_id).ok()?;
    let edge = store.get(cap_fin.edge).ok()?;
    let curve_id = edge.curve?;
    let CurveGeom::Circle(circle) = store.get(curve_id).ok()? else {
        return None;
    };
    let cap_use = cap_fin.pcurve?;
    let Curve2dGeom::Circle(_) = store.get(cap_use.curve()).ok()? else {
        return None;
    };

    let [first_fin_id, second_fin_id] = edge.fins.as_slice() else {
        return None;
    };
    if first_fin_id == second_fin_id {
        return None;
    }
    let source_cylinder_fin = if first_fin_id == cap_fin_id {
        *second_fin_id
    } else if second_fin_id == cap_fin_id {
        *first_fin_id
    } else {
        return None;
    };
    let side_fin = store.get(source_cylinder_fin).ok()?;
    let side_loop = store.get(side_fin.parent).ok()?;
    let source_cylinder_face = side_loop.face;
    let side_face = store.get(source_cylinder_face).ok()?;
    let SurfaceGeom::Cylinder(source_cylinder) = store.get(side_face.surface).ok()? else {
        return None;
    };

    let shell = store.get(face.shell).ok()?;
    let stored_plane_is_exact = matches!(
        store.get(face.surface).ok(),
        Some(SurfaceGeom::Plane(stored)) if *stored == result_plane
    );
    let exact_ownership = face.tolerance.is_none()
        && stored_plane_is_exact
        && loop_.face == face_id
        && cap_fin.parent == *loop_id
        && side_fin.edge == cap_fin.edge
        && side_fin.parent != *loop_id
        && side_loop.face == source_cylinder_face
        && source_cylinder_face != face_id
        && side_face.shell == face.shell
        && exactly_once(&side_loop.fins, source_cylinder_fin)
        && exactly_once(&side_face.loops, side_fin.parent)
        && exactly_once(&shell.faces, face_id)
        && exactly_once(&shell.faces, source_cylinder_face)
        && edge.vertices == [None, None]
        && edge.bounds.is_none()
        && edge.tolerance.is_none();
    if !exact_ownership {
        return None;
    }

    let active = cap_use.range();
    let map = cap_use.edge_to_pcurve();
    let edge_parameters = [map.inverse(active.lo), map.inverse(active.hi)];
    let circle_range = circle.param_range();
    let cap_use_is_complete = cap_use.chart().is_identity()
        && cap_use.endpoint_kinds() == [PcurveEndpointKind::Regular; 2]
        && cap_use.closure_winding() == Some([0, 0])
        && cap_use.seam().is_none()
        && map.scale().abs() == 1.0
        && active.width() == circle_range.width()
        && edge_parameters[0].min(edge_parameters[1]) == circle_range.lo
        && edge_parameters[0].max(edge_parameters[1]) == circle_range.hi
        && cap_fin.sense.times(cap_use.sense()) == face.sense;
    if !cap_use_is_complete
        || !source_axis_is_perpendicular(*source_cylinder, opposing_axis)
        || certify_whole_fin_incidence(store, face_id, *loop_id, *cap_fin_id, tolerance)
            != WholeFinIncidence::Certified
        || certify_whole_fin_incidence(
            store,
            source_cylinder_face,
            side_fin.parent,
            source_cylinder_fin,
            tolerance,
        ) != WholeFinIncidence::Certified
    {
        return None;
    }

    Some(CircularCapSourceWitness {
        cap_edge: cap_fin.edge,
        cap_fin: *cap_fin_id,
        source_cylinder_fin,
        source_cylinder_face,
    })
}

fn source_axis_is_perpendicular(source: Cylinder, opposing_axis: Vec3) -> bool {
    let frame = source.frame();
    exactly_perpendicular(frame.z(), opposing_axis)
        || [frame.x(), frame.y()]
            .into_iter()
            .any(|radial| opposing_axis == radial || opposing_axis == -radial)
}

fn exactly_perpendicular(first: Vec3, second: Vec3) -> bool {
    affine_dot3(first.to_array(), second.to_array(), [0.0; 3], 0.0)
        .is_some_and(|dot| dot.sign() == Orientation::Zero)
}

fn exactly_once<T: Copy + PartialEq>(items: &[T], expected: T) -> bool {
    items.iter().filter(|&&item| item == expected).count() == 1
}
