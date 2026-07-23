//! Compact internally tangent radius-transition shell theorem.
//!
//! Two bounded full-period contact circles share one exact tangent vertex and
//! form the two fins of one pinched shoulder loop. The remaining endpoint of
//! each cylinder is an endpoint-free far cap. Exact circle tangency, complete
//! fin incidence, certified local loops, opposite axial sides, and coherent
//! winding identify the regularized union boundary without relying on
//! constructor or face-storage order.

use super::*;
use crate::analytic_tangency::{
    circles_are_exactly_internal_tangent, point_is_within_circle_endpoint_envelope,
};
use crate::loop_proof::is_one_vertex_full_period_circle_edge;

#[derive(Debug)]
struct TangentBoundary {
    ring: RingBoundary,
    vertex: VertexId,
}

#[derive(Debug)]
struct TangentBand {
    face: FaceId,
    cylinder: Cylinder,
    far: RingBoundary,
    contact: TangentBoundary,
}

pub(super) fn certify_internal_tangent_contact(
    store: &Store,
    shell_id: ShellId,
    cylinders: [(FaceId, Cylinder); 2],
) -> Result<Option<ShellCertification>> {
    let [(first_face, first), (second_face, second)] = cylinders;
    if !vectors_are_exactly_parallel(first.frame().z(), second.frame().z()) {
        return Ok(None);
    }
    let (Some(first), Some(second)) = (
        prepare_tangent_band(store, shell_id, first_face, first)?,
        prepare_tangent_band(store, shell_id, second_face, second)?,
    ) else {
        return Ok(None);
    };
    if first.contact.ring.cap_face != second.contact.ring.cap_face
        || first.far.cap_face == second.far.cap_face
        || first.contact.ring.edge == second.contact.ring.edge
        || first.contact.vertex != second.contact.vertex
    {
        return Ok(None);
    }
    let shoulder_face_id = first.contact.ring.cap_face;
    let shoulder = store.get(shoulder_face_id)?;
    let [shoulder_loop_id] = shoulder.loops.as_slice() else {
        return Ok(None);
    };
    let shoulder_loop = store.get(*shoulder_loop_id)?;
    if shoulder_loop.face != shoulder_face_id
        || shoulder_loop.fins.len() != 2
        || !shoulder_loop.fins.iter().all(|fin_id| {
            store.get(*fin_id).is_ok_and(|fin| {
                [first.contact.ring.edge, second.contact.ring.edge].contains(&fin.edge)
            })
        })
        || certify_loop_simplicity(store, *shoulder_loop_id)? != LoopSimplicity::Certified
        || certify_loop_orientation(store, shoulder_face_id, *shoulder_loop_id)?.is_none()
    {
        return Ok(None);
    }
    let SurfaceGeom::Plane(shoulder_plane) = store.get(shoulder.surface)? else {
        return Ok(None);
    };

    let (outer, inner) = if first.contact.ring.circle.radius() > second.contact.ring.circle.radius()
    {
        (&first, &second)
    } else if second.contact.ring.circle.radius() > first.contact.ring.circle.radius() {
        (&second, &first)
    } else {
        return Ok(None);
    };
    let tangent_point = store.vertex_position(first.contact.vertex)?;
    if outer.cylinder.radius().to_bits() != outer.contact.ring.circle.radius().to_bits()
        || inner.cylinder.radius().to_bits() != inner.contact.ring.circle.radius().to_bits()
        || !circles_are_exactly_internal_tangent(
            outer.contact.ring.circle,
            inner.contact.ring.circle,
        )
        || !contact_vertex_incidence(store, outer, tangent_point)?
        || !contact_vertex_incidence(store, inner, tangent_point)?
    {
        return Ok(None);
    }
    if !all_faces_consumed(
        store,
        shell_id,
        &[
            first.face,
            second.face,
            first.far.cap_face,
            second.far.cap_face,
            shoulder_face_id,
        ],
    )? {
        return Ok(None);
    }

    let raw_outer_far_side = exact_affine_sign(
        shoulder_plane.frame().z(),
        outer.far.circle.frame().origin(),
        shoulder_plane.frame().origin(),
    );
    let raw_inner_far_side = exact_affine_sign(
        shoulder_plane.frame().z(),
        inner.far.circle.frame().origin(),
        shoulder_plane.frame().origin(),
    );
    if !matches!(
        (raw_outer_far_side, raw_inner_far_side),
        (
            Some(PredicateOrientation::Negative),
            Some(PredicateOrientation::Positive)
        ) | (
            Some(PredicateOrientation::Positive),
            Some(PredicateOrientation::Negative)
        )
    ) {
        return Ok(None);
    }

    let shoulder_outward = shoulder_plane.frame().z() * sense_factor(shoulder.sense);
    let shoulder_orientation_valid = exact_affine_sign(
        shoulder_outward,
        outer.far.circle.frame().origin(),
        shoulder_plane.frame().origin(),
    ) == Some(PredicateOrientation::Negative)
        && exact_affine_sign(
            shoulder_outward,
            inner.far.circle.frame().origin(),
            shoulder_plane.frame().origin(),
        ) == Some(PredicateOrientation::Positive);
    let outer_band = ContactBand {
        face: outer.face,
        cylinder: outer.cylinder,
        far: outer.far,
        contact: outer.contact.ring,
    };
    let inner_band = ContactBand {
        face: inner.face,
        cylinder: inner.cylinder,
        far: inner.far,
        contact: inner.contact.ring,
    };
    let orientation_valid = shoulder_orientation_valid
        && band_orientation_valid(store, &outer_band, BandRole::Outer)?
        && band_orientation_valid(store, &inner_band, BandRole::Inner)?;
    Ok(Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: if orientation_valid {
            ShellOrientation::Positive
        } else {
            ShellOrientation::Invalid
        },
    }))
}

fn contact_vertex_incidence(store: &Store, band: &TangentBand, point: Point3) -> Result<bool> {
    let edge = store.get(band.contact.ring.edge)?;
    let Some((lo, hi)) = edge.bounds else {
        return Ok(false);
    };
    Ok([lo, hi].into_iter().all(|parameter| {
        point_is_within_circle_endpoint_envelope(
            point,
            band.contact.ring.circle,
            parameter,
            LINEAR_RESOLUTION,
        )
    }))
}

fn prepare_tangent_band(
    store: &Store,
    shell_id: ShellId,
    face_id: FaceId,
    cylinder: Cylinder,
) -> Result<Option<TangentBand>> {
    let face = store.get(face_id)?;
    if face.loops.len() != 2
        || certify_face_loop_layout(store, face_id)? != LoopContainment::Certified
    {
        return Ok(None);
    }
    let mut far = None;
    let mut contact = None;
    for &loop_id in &face.loops {
        let loop_ = store.get(loop_id)?;
        let [fin_id] = loop_.fins.as_slice() else {
            return Ok(None);
        };
        let edge = store.get(store.get(*fin_id)?.edge)?;
        if edge.bounds.is_none() {
            let Some(candidate) = prepare_boundary(store, shell_id, face_id, cylinder, loop_id)?
            else {
                return Ok(None);
            };
            if store.get(candidate.cap_face)?.loops.len() != 1 || far.replace(candidate).is_some() {
                return Ok(None);
            }
        } else {
            let Some(candidate) =
                prepare_tangent_boundary(store, shell_id, face_id, cylinder, loop_id)?
            else {
                return Ok(None);
            };
            if contact.replace(candidate).is_some() {
                return Ok(None);
            }
        }
    }
    let (Some(far), Some(contact)) = (far, contact) else {
        return Ok(None);
    };
    if far.edge == contact.ring.edge || far.cap_face == contact.ring.cap_face {
        return Ok(None);
    }
    Ok(Some(TangentBand {
        face: face_id,
        cylinder,
        far,
        contact,
    }))
}

fn prepare_tangent_boundary(
    store: &Store,
    shell_id: ShellId,
    side_face: FaceId,
    cylinder: Cylinder,
    side_loop_id: LoopId,
) -> Result<Option<TangentBoundary>> {
    let side_loop = store.get(side_loop_id)?;
    let [side_fin_id] = side_loop.fins.as_slice() else {
        return Ok(None);
    };
    if side_loop.face != side_face
        || certify_loop_simplicity(store, side_loop_id)? != LoopSimplicity::Certified
    {
        return Ok(None);
    }
    let side_fin = store.get(*side_fin_id)?;
    let edge = store.get(side_fin.edge)?;
    let [Some(vertex), Some(other)] = edge.vertices else {
        return Ok(None);
    };
    let [first, second] = edge.fins.as_slice() else {
        return Ok(None);
    };
    let cap_fin_id = if first == side_fin_id {
        *second
    } else if second == side_fin_id {
        *first
    } else {
        return Ok(None);
    };
    let cap_fin = store.get(cap_fin_id)?;
    let cap_loop_id = cap_fin.parent;
    let cap_loop = store.get(cap_loop_id)?;
    let cap_face_id = cap_loop.face;
    let cap_face = store.get(cap_face_id)?;
    if vertex != other
        || !is_one_vertex_full_period_circle_edge(store, edge)?
        || side_fin.parent != side_loop_id
        || cap_fin.edge != side_fin.edge
        || cap_fin.sense == side_fin.sense
        || cap_loop.fins.len() != 2
        || cap_face.shell != shell_id
        || cap_face.loops.as_slice() != [cap_loop_id]
        || certify_loop_simplicity(store, cap_loop_id)? != LoopSimplicity::Certified
        || certify_whole_fin_incidence(
            store,
            side_face,
            side_loop_id,
            *side_fin_id,
            LINEAR_RESOLUTION,
        ) != WholeFinIncidence::Certified
        || certify_whole_fin_incidence(
            store,
            cap_face_id,
            cap_loop_id,
            cap_fin_id,
            LINEAR_RESOLUTION,
        ) != WholeFinIncidence::Certified
    {
        return Ok(None);
    }
    let Some(curve_id) = edge.curve else {
        return Ok(None);
    };
    let CurveGeom::Circle(circle) = store.get(curve_id)? else {
        return Ok(None);
    };
    let SurfaceGeom::Plane(cap_plane) = store.get(cap_face.surface)? else {
        return Ok(None);
    };
    if !circle_on_cylinder(*circle, cylinder)
        || !vectors_are_exactly_parallel(cap_plane.frame().z(), cylinder.frame().z())
        || !point_is_within_plane_envelope(
            circle.frame().origin(),
            cap_plane.frame().origin(),
            cap_plane.frame().z(),
            LINEAR_RESOLUTION,
        )
    {
        return Ok(None);
    }
    let (Some(side_use), Some(cap_use)) = (side_fin.pcurve, cap_fin.pcurve) else {
        return Ok(None);
    };
    let Curve2dGeom::Line(side_line) = store.get(side_use.curve())? else {
        return Ok(None);
    };
    let Curve2dGeom::Circle(cap_circle) = store.get(cap_use.curve())? else {
        return Ok(None);
    };
    if side_line.dir().y != 0.0
        || side_line.dir().x == 0.0
        || side_use.closure_winding().is_some()
        || cap_use.closure_winding().is_some()
        || cap_circle.radius().to_bits() != circle.radius().to_bits()
    {
        return Ok(None);
    }
    let Some(edge_positive_side) = traversal_is_positive(
        [side_line.dir().x, side_use.edge_to_pcurve().scale()],
        Sense::Forward,
    ) else {
        return Ok(None);
    };
    let Some(side_traverses_positive_u) = traversal_is_positive(
        [side_line.dir().x, side_use.edge_to_pcurve().scale()],
        side_fin.sense,
    ) else {
        return Ok(None);
    };
    let Some(edge_positive_cap) =
        traversal_is_positive([cap_use.edge_to_pcurve().scale()], Sense::Forward)
    else {
        return Ok(None);
    };
    let cap_axis_alignment = if edge_positive_side == edge_positive_cap {
        PredicateOrientation::Positive
    } else {
        PredicateOrientation::Negative
    };
    Ok(Some(TangentBoundary {
        ring: RingBoundary {
            side_loop: side_loop_id,
            cap_face: cap_face_id,
            edge: side_fin.edge,
            circle: *circle,
            axial_parameter: side_line.origin().y,
            cap_axis_alignment,
            side_traverses_positive_u,
        },
        vertex,
    }))
}
