//! Internally tangent radius-transition shell theorem.
//!
//! Every cylinder band has exactly two complete ring boundaries. Bounded
//! contact rings form a connected path of one or two exact tangent shoulders;
//! the two degree-one bands own the only endpoint-free far caps. Each shoulder
//! is one two-fin pinched loop whose complete-period edges share one tangent
//! vertex. This incidence graph covers both the compact two-band shoulder and
//! the three-band/two-shoulder chain without relying on face storage order,
//! cylinder order, or authored axis direction.

use super::*;
use crate::analytic_tangency::{
    circles_are_exactly_internal_tangent, point_is_within_circle_endpoint_envelope,
};
use crate::loop_proof::is_one_vertex_full_period_circle_edge;

#[derive(Debug, Clone, Copy)]
struct TangentBoundary {
    ring: RingBoundary,
    vertex: VertexId,
}

#[derive(Debug, Clone, Copy)]
enum TangentBandBoundary {
    Far(RingBoundary),
    Contact(TangentBoundary),
}

impl TangentBandBoundary {
    const fn ring(self) -> RingBoundary {
        match self {
            Self::Far(ring) => ring,
            Self::Contact(contact) => contact.ring,
        }
    }

    const fn contact(self) -> Option<TangentBoundary> {
        match self {
            Self::Far(_) => None,
            Self::Contact(contact) => Some(contact),
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct TangentBand {
    face: FaceId,
    cylinder: Cylinder,
    boundaries: [TangentBandBoundary; 2],
}

impl TangentBand {
    fn contacts(&self) -> impl Iterator<Item = TangentBoundary> + '_ {
        self.boundaries
            .iter()
            .filter_map(|boundary| boundary.contact())
    }

    fn contact_count(&self) -> usize {
        self.contacts().count()
    }

    fn other_ring(&self, edge: EdgeId) -> Option<RingBoundary> {
        let mut matching = 0;
        let mut other = None;
        for boundary in self.boundaries {
            let ring = boundary.ring();
            if ring.edge == edge {
                matching += 1;
            } else if other.replace(ring).is_some() {
                return None;
            }
        }
        if matching == 1 { other } else { None }
    }
}

#[derive(Debug, Clone, Copy)]
struct ShoulderSide {
    band: usize,
    contact: TangentBoundary,
}

#[derive(Debug, Clone, Copy)]
struct TangentShoulder {
    face: FaceId,
    plane: kgeom::surface::Plane,
    sides: [ShoulderSide; 2],
    outer: usize,
}

impl TangentShoulder {
    const fn vertex(self) -> VertexId {
        self.sides[0].contact.vertex
    }
}

pub(super) fn certify_internal_tangent_contact(
    store: &Store,
    shell_id: ShellId,
    cylinders: &[(FaceId, Cylinder)],
) -> Result<Option<ShellCertification>> {
    if !matches!(cylinders.len(), 2 | 3) {
        return Ok(None);
    }
    let reference_axis = cylinders[0].1.frame().z();
    if !cylinders
        .iter()
        .all(|(_, cylinder)| vectors_are_exactly_parallel(reference_axis, cylinder.frame().z()))
    {
        return Ok(None);
    }

    let mut bands = Vec::with_capacity(3);
    for &(face, cylinder) in cylinders {
        let Some(band) = prepare_tangent_band(store, shell_id, face, cylinder)? else {
            return Ok(None);
        };
        bands.push(band);
    }
    let Some(shoulders) = prepare_tangent_shoulders(store, shell_id, &bands)? else {
        return Ok(None);
    };
    if !tangent_chain_structure_valid(&bands, &shoulders)
        || !all_tangent_faces_consumed(store, shell_id, &bands, &shoulders)?
        || !shoulders_straddle_adjacent_band_interiors(&bands, &shoulders)
    {
        return Ok(None);
    }

    let mut orientation_valid = true;
    for shoulder in &shoulders {
        orientation_valid &= tangent_shoulder_orientation_valid(store, &bands, shoulder)?;
    }
    for (band_index, band) in bands.iter().enumerate() {
        orientation_valid &= tangent_band_orientation_valid(store, band_index, band, &shoulders)?;
    }
    Ok(Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: if orientation_valid {
            ShellOrientation::Positive
        } else {
            ShellOrientation::Invalid
        },
    }))
}

fn prepare_tangent_shoulders(
    store: &Store,
    shell_id: ShellId,
    bands: &[TangentBand],
) -> Result<Option<Vec<TangentShoulder>>> {
    let mut shoulder_faces = Vec::with_capacity(2);
    for band in bands {
        for contact in band.contacts() {
            if !shoulder_faces.contains(&contact.ring.cap_face) {
                shoulder_faces.push(contact.ring.cap_face);
            }
        }
    }
    if shoulder_faces.len() + 1 != bands.len() {
        return Ok(None);
    }

    let mut shoulders = Vec::with_capacity(2);
    for face in shoulder_faces {
        let mut sides = Vec::with_capacity(2);
        for (band_index, band) in bands.iter().enumerate() {
            for contact in band.contacts() {
                if contact.ring.cap_face == face {
                    sides.push(ShoulderSide {
                        band: band_index,
                        contact,
                    });
                }
            }
        }
        let [first, second] = sides.as_slice() else {
            return Ok(None);
        };
        let Some(shoulder) =
            prepare_tangent_shoulder(store, shell_id, bands, face, [*first, *second])?
        else {
            return Ok(None);
        };
        shoulders.push(shoulder);
    }
    if shoulders.len() == 2 && shoulders[0].vertex() == shoulders[1].vertex() {
        return Ok(None);
    }
    Ok(Some(shoulders))
}

fn prepare_tangent_shoulder(
    store: &Store,
    shell_id: ShellId,
    bands: &[TangentBand],
    face_id: FaceId,
    sides: [ShoulderSide; 2],
) -> Result<Option<TangentShoulder>> {
    let [first, second] = sides;
    if first.band == second.band
        || first.contact.ring.edge == second.contact.ring.edge
        || first.contact.vertex != second.contact.vertex
        || first.contact.ring.cap_face != face_id
        || second.contact.ring.cap_face != face_id
    {
        return Ok(None);
    }

    let face = store.get(face_id)?;
    let [loop_id] = face.loops.as_slice() else {
        return Ok(None);
    };
    let loop_ = store.get(*loop_id)?;
    let [first_fin, second_fin] = loop_.fins.as_slice() else {
        return Ok(None);
    };
    let expected_edges = [first.contact.ring.edge, second.contact.ring.edge];
    let actual_edges = [store.get(*first_fin)?.edge, store.get(*second_fin)?.edge];
    if face.shell != shell_id
        || loop_.face != face_id
        || actual_edges[0] == actual_edges[1]
        || !actual_edges
            .iter()
            .all(|edge| expected_edges.contains(edge))
        || certify_loop_simplicity(store, *loop_id)? != LoopSimplicity::Certified
        || certify_loop_orientation(store, face_id, *loop_id)?.is_none()
    {
        return Ok(None);
    }
    let SurfaceGeom::Plane(plane) = store.get(face.surface)? else {
        return Ok(None);
    };

    let first_band = bands[first.band];
    let second_band = bands[second.band];
    let tangent_point = store.vertex_position(first.contact.vertex)?;
    if exact_affine_sign(
        plane.frame().z(),
        first.contact.ring.circle.frame().origin(),
        plane.frame().origin(),
    ) != Some(PredicateOrientation::Zero)
        || exact_affine_sign(
            plane.frame().z(),
            second.contact.ring.circle.frame().origin(),
            plane.frame().origin(),
        ) != Some(PredicateOrientation::Zero)
        || first_band.cylinder.radius().to_bits() != first.contact.ring.circle.radius().to_bits()
        || second_band.cylinder.radius().to_bits() != second.contact.ring.circle.radius().to_bits()
        || !circles_are_exactly_internal_tangent(
            first.contact.ring.circle,
            second.contact.ring.circle,
        )
        || !contact_vertex_incidence(store, first.contact, tangent_point)?
        || !contact_vertex_incidence(store, second.contact, tangent_point)?
    {
        return Ok(None);
    }
    let outer = if first.contact.ring.circle.radius() > second.contact.ring.circle.radius() {
        0
    } else if second.contact.ring.circle.radius() > first.contact.ring.circle.radius() {
        1
    } else {
        return Ok(None);
    };
    Ok(Some(TangentShoulder {
        face: face_id,
        plane: *plane,
        sides,
        outer,
    }))
}

fn tangent_chain_structure_valid(bands: &[TangentBand], shoulders: &[TangentShoulder]) -> bool {
    if shoulders.len() + 1 != bands.len() {
        return false;
    }
    let mut far_count = 0;
    for (band_index, band) in bands.iter().enumerate() {
        let degree = shoulders
            .iter()
            .flat_map(|shoulder| shoulder.sides)
            .filter(|side| side.band == band_index)
            .count();
        if degree == 0 || degree != band.contact_count() {
            return false;
        }
        far_count += band
            .boundaries
            .iter()
            .filter(|boundary| matches!(boundary, TangentBandBoundary::Far(_)))
            .count();
    }
    if far_count != 2 {
        return false;
    }

    let mut reached = [false; 3];
    reached[0] = true;
    for _ in 0..bands.len() {
        for shoulder in shoulders {
            let [first, second] = shoulder.sides;
            if reached[first.band] || reached[second.band] {
                reached[first.band] = true;
                reached[second.band] = true;
            }
        }
    }
    reached[..bands.len()].iter().all(|value| *value)
}

fn all_tangent_faces_consumed(
    store: &Store,
    shell_id: ShellId,
    bands: &[TangentBand],
    shoulders: &[TangentShoulder],
) -> Result<bool> {
    let mut roles = Vec::with_capacity(7);
    roles.extend(bands.iter().map(|band| band.face));
    for band in bands {
        for boundary in band.boundaries {
            if let TangentBandBoundary::Far(far) = boundary {
                roles.push(far.cap_face);
            }
        }
    }
    roles.extend(shoulders.iter().map(|shoulder| shoulder.face));
    all_faces_consumed(store, shell_id, &roles)
}

fn shoulders_straddle_adjacent_band_interiors(
    bands: &[TangentBand],
    shoulders: &[TangentShoulder],
) -> bool {
    shoulders.iter().all(|shoulder| {
        let [first, second] = shoulder.sides;
        let contact_origin = first.contact.ring.circle.frame().origin();
        let Some(first_other) = bands[first.band].other_ring(first.contact.ring.edge) else {
            return false;
        };
        let Some(second_other) = bands[second.band].other_ring(second.contact.ring.edge) else {
            return false;
        };
        let first_side = exact_affine_sign(
            shoulder.plane.frame().z(),
            first_other.circle.frame().origin(),
            contact_origin,
        );
        let second_side = exact_affine_sign(
            shoulder.plane.frame().z(),
            second_other.circle.frame().origin(),
            contact_origin,
        );
        matches!(
            (first_side, second_side),
            (
                Some(PredicateOrientation::Negative),
                Some(PredicateOrientation::Positive)
            ) | (
                Some(PredicateOrientation::Positive),
                Some(PredicateOrientation::Negative)
            )
        )
    })
}

fn tangent_shoulder_orientation_valid(
    store: &Store,
    bands: &[TangentBand],
    shoulder: &TangentShoulder,
) -> Result<bool> {
    let face = store.get(shoulder.face)?;
    let outward = shoulder.plane.frame().z() * sense_factor(face.sense);
    let contact_origin = shoulder.sides[0].contact.ring.circle.frame().origin();
    for (side_index, side) in shoulder.sides.iter().enumerate() {
        let Some(other) = bands[side.band].other_ring(side.contact.ring.edge) else {
            return Ok(false);
        };
        let expected = if side_index == shoulder.outer {
            PredicateOrientation::Negative
        } else {
            PredicateOrientation::Positive
        };
        if exact_affine_sign(outward, other.circle.frame().origin(), contact_origin)
            != Some(expected)
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn tangent_band_orientation_valid(
    store: &Store,
    band_index: usize,
    band: &TangentBand,
    shoulders: &[TangentShoulder],
) -> Result<bool> {
    if store.get(band.face)?.sense != Sense::Forward {
        return Ok(false);
    }
    let [first, second] = band.boundaries;
    let (low, high) = if first.ring().axial_parameter < second.ring().axial_parameter {
        (first, second)
    } else if second.ring().axial_parameter < first.ring().axial_parameter {
        (second, first)
    } else {
        return Ok(false);
    };
    if !low.ring().side_traverses_positive_u || high.ring().side_traverses_positive_u {
        return Ok(false);
    }
    for (boundary, base_expected) in [(low, -1), (high, 1)] {
        let expected = match boundary.contact() {
            Some(contact) => {
                let Some(inner) = contact_is_inner(shoulders, band_index, contact.ring.edge) else {
                    return Ok(false);
                };
                if inner { -base_expected } else { base_expected }
            }
            None => base_expected,
        };
        let ring = boundary.ring();
        let cap = store.get(ring.cap_face)?;
        if oriented_axis_alignment(ring.cap_axis_alignment, cap.sense) != Some(expected) {
            return Ok(false);
        }
    }
    Ok(true)
}

fn contact_is_inner(shoulders: &[TangentShoulder], band: usize, edge: EdgeId) -> Option<bool> {
    let mut found = None;
    for shoulder in shoulders {
        for (side_index, side) in shoulder.sides.iter().enumerate() {
            if side.band == band
                && side.contact.ring.edge == edge
                && found.replace(side_index != shoulder.outer).is_some()
            {
                return None;
            }
        }
    }
    found
}

fn contact_vertex_incidence(
    store: &Store,
    contact: TangentBoundary,
    point: Point3,
) -> Result<bool> {
    let edge = store.get(contact.ring.edge)?;
    let Some((lo, hi)) = edge.bounds else {
        return Ok(false);
    };
    Ok([lo, hi].into_iter().all(|parameter| {
        point_is_within_circle_endpoint_envelope(
            point,
            contact.ring.circle,
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
    let mut boundaries = Vec::with_capacity(2);
    for &loop_id in &face.loops {
        let loop_ = store.get(loop_id)?;
        let [fin_id] = loop_.fins.as_slice() else {
            return Ok(None);
        };
        let edge = store.get(store.get(*fin_id)?.edge)?;
        let boundary = if edge.bounds.is_none() {
            let Some(candidate) = prepare_boundary(store, shell_id, face_id, cylinder, loop_id)?
            else {
                return Ok(None);
            };
            if store.get(candidate.cap_face)?.loops.len() != 1 {
                return Ok(None);
            }
            TangentBandBoundary::Far(candidate)
        } else {
            let Some(candidate) =
                prepare_tangent_boundary(store, shell_id, face_id, cylinder, loop_id)?
            else {
                return Ok(None);
            };
            TangentBandBoundary::Contact(candidate)
        };
        boundaries.push(boundary);
    }
    let [first, second] = boundaries.as_slice() else {
        return Ok(None);
    };
    if first.ring().edge == second.ring().edge
        || first.ring().cap_face == second.ring().cap_face
        || first.contact().is_none() && second.contact().is_none()
    {
        return Ok(None);
    }
    Ok(Some(TangentBand {
        face: face_id,
        cylinder,
        boundaries: [*first, *second],
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
        || !axis_parameter_identity_is_exact(
            circle.frame().origin(),
            *cylinder.frame(),
            side_line.origin().y,
        )
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

/// Prove the pcurve height against the authored cylinder axis without making
/// a rounded frame evaluation into topology authority.
fn axis_parameter_identity_is_exact(point: Point3, frame: Frame, parameter: f64) -> bool {
    let point = point.to_array();
    let origin = frame.origin().to_array();
    let axis = frame.z().to_array();
    (0..3).all(|component| {
        affine_dot3(
            [1.0, axis[component], -1.0],
            [origin[component], parameter, point[component]],
            [0.0; 3],
            0.0,
        )
        .is_some_and(|value| value.sign() == PredicateOrientation::Zero)
    })
}
