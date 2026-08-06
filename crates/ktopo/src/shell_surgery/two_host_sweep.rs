//! Shared theorem for two-host axial chains and exact contact subfamilies.
//!
//! The admitted unsplit representation is the boundary of
//! `(D0 x [a0,a1]) union (D1 x [b0,b1])` for geometrically parallel cylinder
//! disks with a strict axial chain `a0 < b0 < a1 < b1` (or the same chain in
//! the reversed common-axis direction). The authored cylinder frames may be
//! parallel or antiparallel: the translated transition roots establish the
//! proof-local common axial direction instead of either surface chart. Each
//! cylinder face owns one endpoint-free outer
//! ring and one simple noncontractible boundary made from its two translated
//! strict-secant arcs and the common rulings. The transition planes own the
//! complementary exposed disk differences. Incidence, not face storage or
//! constructor provenance, discovers every role. Opposite radial side proofs
//! plus the same two mapped topology roots establish arc complementarity;
//! parameter-span widths are deliberately irrelevant.

use super::*;

use super::super::shell_lemmas::{
    Cap, CapUse, ProfileCarrier, RadialSide, Translation, all_shell_faces_consumed,
    axis_distance_squared, certified_close, certified_nonzero, certified_parallel,
    circle_secant_span_side, mapped_vertex, oriented_dot_sign, peer_face, peer_face_from_fin,
    peer_fin, prepare_cap, ruling_connects, translated_vertices, two_host_circle_on_cylinder,
};
use crate::cylindrical_band::CylindricalBandSolidInput;
use kgeom::param::ParamRange;

#[path = "two_host_sweep/internal_tangent.rs"]
mod internal_tangent;

#[derive(Debug, Clone, Copy)]
struct WholeEnd {
    face: FaceId,
    center: Point3,
    plane: kgeom::surface::Plane,
    edge: EdgeId,
    circle: kgeom::curve::Circle,
    axial_parameter: f64,
    cap_axis_alignment: PredicateOrientation,
    side_traverses_positive_u: bool,
    local_orientation_valid: bool,
    host_loop_orientation: PredicateOrientation,
}

#[derive(Debug, Clone, Copy)]
struct HostArc {
    edge: EdgeId,
    cap: FaceId,
}

#[derive(Debug)]
struct Boundary {
    loop_orientation: PredicateOrientation,
    arcs: Vec<HostArc>,
    rulings: Vec<EdgeId>,
}

#[derive(Debug)]
struct HostBand {
    face: FaceId,
    cylinder: Cylinder,
    whole: WholeEnd,
    boundary: Boundary,
}

#[derive(Debug)]
struct WholeBand {
    face: FaceId,
    cylinder: Cylinder,
    ends: [WholeEnd; 2],
}

#[derive(Debug)]
struct Transition {
    cap: Cap,
    first: CapUse,
    second: CapUse,
}

#[derive(Debug)]
struct Chain {
    transitions: Vec<Transition>,
    lower: usize,
    upper: usize,
    translation: Translation,
}

#[derive(Debug, Clone, Copy)]
pub(super) struct VerifiedTwoHost {
    pub(super) certification: ShellCertification,
    pub(super) contact: bool,
}

/// Verify one untrusted two-host proposal from live topology and geometry.
pub(super) fn certify_two_host_evidence(
    store: &Store,
    shell_id: ShellId,
    evidence: &TwoHostSurgeryEvidence,
) -> Result<Option<VerifiedTwoHost>> {
    if evidence.shell != shell_id {
        return Ok(None);
    }
    let shell = store.get(shell_id)?;
    if !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(None);
    }
    let mut cylinders = Vec::new();
    let mut planar_faces = Vec::new();
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        if face.shell != shell_id {
            return Ok(None);
        }
        match store.get(face.surface)? {
            SurfaceGeom::Cylinder(cylinder) => cylinders.push((face_id, *cylinder)),
            SurfaceGeom::Plane(_) => planar_faces.push(face_id),
            _ => return Ok(None),
        }
    }
    if cylinders.len() < 2
        || cylinders != evidence.cylinders
        || planar_faces != evidence.planar_faces
    {
        return Ok(None);
    }
    for &(first_face, first) in &cylinders {
        for &(second_face, second) in &cylinders {
            if first_face == second_face {
                continue;
            }
            if let Some((certification, contact)) =
                certify_host_pair(store, shell_id, first_face, first, second_face, second)?
            {
                return Ok(Some(VerifiedTwoHost {
                    certification,
                    contact,
                }));
            }
        }
    }
    if let Some(certification) = certify_nested_axial_contact(store, shell_id, &cylinders)? {
        return Ok(Some(VerifiedTwoHost {
            certification,
            contact: true,
        }));
    }
    let certification =
        internal_tangent::certify_internal_tangent_contact(store, shell_id, &cylinders)?;
    Ok(certification.map(|certification| VerifiedTwoHost {
        certification,
        contact: true,
    }))
}

pub(super) fn contact_work(store: &Store, shell_id: ShellId) -> Result<Option<u64>> {
    let shell = store.get(shell_id)?;
    let mut loops = 0_u64;
    let mut fins = 0_u64;
    for &face in &shell.faces {
        for &loop_id in &store.get(face)?.loops {
            loops = match loops.checked_add(1) {
                Some(value) => value,
                None => return Ok(None),
            };
            fins = match fins.checked_add(store.get(loop_id)?.fins.len() as u64) {
                Some(value) => value,
                None => return Ok(None),
            };
        }
    }
    let Some(faces) = u64::try_from(shell.faces.len()).ok() else {
        return Ok(None);
    };
    faces
        .checked_mul(32)
        .and_then(|work| work.checked_add(loops.checked_mul(16)?))
        .and_then(|work| work.checked_add(fins.checked_mul(16)?))
        .map(Some)
        .ok_or(kcore::error::Error::InvalidGeometry {
            reason: "axial contact shell proof work overflow",
        })
}

/// No-scratch bound for every ordered host-pair scan. With `U` fin uses,
/// unique edges are at most `U` and unique vertices at most `2U`, so
/// `N = 1 + F + L + 4U` bounds all role sets. The structural scans take
/// `N^2 + 48N`. Loop certification is charged separately from the existing
/// complete face-layout formula: four times the sum over every face covers
/// one host layout plus the boundary's explicit simplicity, orientation-side
/// simplicity, and periodic fallback. It also dominates the two bounded-loop
/// passes for every possible transition cap. This retains the full quadratic
/// three-layer periodic-pair term instead of trying to absorb it in `N^2`.
pub(super) fn proof_work(
    store: &Store,
    shell_id: ShellId,
    cylinder_count: usize,
) -> Result<Option<u64>> {
    let shell = store.get(shell_id)?;
    let mut loops = 0_u64;
    let mut fins = 0_u64;
    let mut loop_work = 0_u64;
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        let mut face_fins = 0_usize;
        for &loop_id in &face.loops {
            loops = match loops.checked_add(1) {
                Some(value) => value,
                None => return Ok(None),
            };
            let loop_ = store.get(loop_id)?;
            face_fins = match face_fins.checked_add(loop_.fins.len()) {
                Some(value) => value,
                None => return Ok(None),
            };
            for &fin_id in &loop_.fins {
                fins = match fins.checked_add(1) {
                    Some(value) => value,
                    None => return Ok(None),
                };
                let _ = store.get(fin_id)?;
            }
        }
        let Some(work) = crate::loop_proof::face_loop_containment_work(face.loops.len(), face_fins)
        else {
            return Ok(None);
        };
        loop_work = match loop_work.checked_add(work) {
            Some(value) => value,
            None => return Ok(None),
        };
    }
    let (Some(faces), Some(cylinders)) = (
        u64::try_from(shell.faces.len()).ok(),
        u64::try_from(cylinder_count).ok(),
    ) else {
        return Ok(None);
    };
    let Some(size) = 1_u64
        .checked_add(faces)
        .and_then(|value| value.checked_add(loops))
        .and_then(|value| value.checked_add(fins.checked_mul(4)?))
    else {
        return Ok(None);
    };
    let Some(candidates) = cylinders
        .checked_sub(1)
        .and_then(|less| cylinders.checked_mul(less))
    else {
        return Ok(None);
    };
    let Some(additional) = loop_work.checked_mul(4) else {
        return Ok(None);
    };
    Ok(quadratic_proof_work(size, 48, additional, candidates))
}
fn certify_host_pair(
    store: &Store,
    shell_id: ShellId,
    first_face: FaceId,
    first: Cylinder,
    second_face: FaceId,
    second: Cylinder,
) -> Result<Option<(ShellCertification, bool)>> {
    if !certified_parallel(first.frame().z(), second.frame().z()) {
        return Ok(None);
    }
    let Some(first) = prepare_host_band(store, shell_id, first_face, first, second_face)? else {
        return Ok(None);
    };
    let Some(second) = prepare_host_band(store, shell_id, second_face, second, first_face)? else {
        return Ok(None);
    };
    if first.whole.face == second.whole.face
        || !same_unique_edges(&first.boundary.rulings, &second.boundary.rulings)
    {
        return Ok(None);
    }
    let Some(transitions) = prepare_transitions(store, &first, &second)? else {
        return Ok(None);
    };
    if !certify_host_band_base(&first, &transitions, true)?
        || !certify_host_band_base(&second, &transitions, false)?
    {
        return Ok(None);
    }
    if let Some(chain) = prepare_chain(store, &first, &second, transitions)? {
        let lower = &chain.transitions[chain.lower];
        let upper = &chain.transitions[chain.upper];
        if !certify_chain_geometry(store, &first, &second, lower, upper, &chain.translation)? {
            return Ok(None);
        }
        let mut role_faces = vec![first.face, second.face, first.whole.face, second.whole.face];
        role_faces.extend(chain.transitions.iter().map(|end| end.cap.face));
        if !all_shell_faces_consumed(store, shell_id, &role_faces)? {
            return Ok(None);
        }
        return Ok(Some((
            certification_from_orientation(
                store,
                &first,
                &second,
                lower,
                upper,
                chain.translation.vector,
            )?,
            false,
        )));
    }
    Ok(
        certify_zero_transition_chain(store, shell_id, &first, &second)?
            .map(|certification| (certification, true)),
    )
}

fn certify_zero_transition_chain(
    store: &Store,
    shell_id: ShellId,
    first: &HostBand,
    second: &HostBand,
) -> Result<Option<ShellCertification>> {
    if !first.boundary.rulings.is_empty() || !second.boundary.rulings.is_empty() {
        return Ok(None);
    }
    let Some(transitions) = prepare_transitions(store, first, second)? else {
        return Ok(None);
    };
    let axis = circle_center(transitions[0].first)? - first.whole.center;
    if transitions.len() != 2
        || !certified_nonzero(axis)
        || !certified_parallel(axis, first.cylinder.frame().z())
        || !certified_parallel(axis, second.cylinder.frame().z())
    {
        return Ok(None);
    }
    let mut candidates = Vec::new();
    for lower in 0..2 {
        let upper = 1 - lower;
        let low = &transitions[lower];
        let high = &transitions[upper];
        let Some(translation) = zero_translation(&low.cap, &high.cap) else {
            continue;
        };
        let contact = circle_center(low.first)?;
        if strictly_precedes(first.whole.center, contact, axis)
            && strictly_precedes(contact, second.whole.center, axis)
            && complementary_arcs(low.first, high.first, &translation)
            && complementary_arcs(low.second, high.second, &translation)
            && certify_radial_roles(first.cylinder, second.cylinder, low, high)
        {
            candidates.push((lower, upper));
        }
    }
    let [(lower, upper)] = candidates.as_slice() else {
        return Ok(None);
    };
    let lower = &transitions[*lower];
    let upper = &transitions[*upper];
    let mut roles = vec![first.face, second.face, first.whole.face, second.whole.face];
    roles.extend(transitions.iter().map(|transition| transition.cap.face));
    if !all_shell_faces_consumed(store, shell_id, &roles)? {
        return Ok(None);
    }
    Ok(Some(certification_from_orientation(
        store, first, second, lower, upper, axis,
    )?))
}

fn zero_translation(first: &Cap, second: &Cap) -> Option<Translation> {
    if first.vertices.len() != second.vertices.len()
        || first
            .vertices
            .iter()
            .any(|vertex| !second.vertices.contains(vertex))
    {
        return None;
    }
    Some(Translation {
        vector: Vec3::new(0.0, 0.0, 0.0),
        vertices: first
            .vertices
            .iter()
            .map(|vertex| (*vertex, *vertex))
            .collect(),
    })
}

fn prepare_host_band(
    store: &Store,
    shell_id: ShellId,
    face_id: FaceId,
    cylinder: Cylinder,
    other_face: FaceId,
) -> Result<Option<HostBand>> {
    if certify_face_loop_layout(store, face_id)? != LoopContainment::Certified {
        return Ok(None);
    }
    let face = store.get(face_id)?;
    let mut whole = None;
    let mut boundary = None;
    for &loop_id in &face.loops {
        if let Some(candidate) = prepare_whole_end(store, shell_id, face_id, cylinder, loop_id)? {
            if whole.replace(candidate).is_some() {
                return Ok(None);
            }
            continue;
        }
        let Some(candidate) = prepare_boundary(store, face_id, cylinder, other_face, loop_id)?
        else {
            return Ok(None);
        };
        if boundary.replace(candidate).is_some() {
            return Ok(None);
        }
    }
    let (Some(whole), Some(boundary)) = (whole, boundary) else {
        return Ok(None);
    };
    Ok(Some(HostBand {
        face: face_id,
        cylinder,
        whole,
        boundary,
    }))
}

fn certify_nested_axial_contact(
    store: &Store,
    shell_id: ShellId,
    cylinders: &[(FaceId, Cylinder)],
) -> Result<Option<ShellCertification>> {
    let [(first_face, first), (second_face, second)] = cylinders else {
        return Ok(None);
    };
    let (Some(first), Some(second)) = (
        prepare_whole_band(store, shell_id, *first_face, *first)?,
        prepare_whole_band(store, shell_id, *second_face, *second)?,
    ) else {
        return Ok(None);
    };
    if !certify_cylindrical_base_embedding(
        first.cylinder,
        first.ends[0].center,
        first.ends[1].center,
    )? || !certify_cylindrical_base_embedding(
        second.cylinder,
        second.ends[0].center,
        second.ends[1].center,
    )? {
        return Ok(None);
    }
    let shared = first
        .ends
        .iter()
        .enumerate()
        .flat_map(|(a, end)| {
            second
                .ends
                .iter()
                .enumerate()
                .filter_map(move |(b, peer)| (end.face == peer.face).then_some((a, b)))
        })
        .collect::<Vec<_>>();
    let [(first_contact, second_contact)] = shared.as_slice() else {
        return Ok(None);
    };
    let (outer, outer_contact, inner, inner_contact) =
        if strictly_contains_cylinder_support(first.cylinder, second.cylinder) {
            (&first, *first_contact, &second, *second_contact)
        } else if strictly_contains_cylinder_support(second.cylinder, first.cylinder) {
            (&second, *second_contact, &first, *first_contact)
        } else {
            return Ok(None);
        };
    let annulus = outer.ends[outer_contact];
    if store.get(annulus.face)?.loops.len() != 2
        || certify_face_loop_layout(store, annulus.face)? != LoopContainment::Certified
    {
        return Ok(None);
    }
    let outer_far = outer.ends[1 - outer_contact];
    let inner_far = inner.ends[1 - inner_contact];
    let roles = [
        outer.face,
        inner.face,
        annulus.face,
        outer_far.face,
        inner_far.face,
    ];
    if outer_far.face == inner_far.face
        || !all_shell_faces_consumed(store, shell_id, &roles)?
        || !matches!(
            (
                exact_affine_sign(annulus.plane.frame().z(), outer_far.center, annulus.center,),
                exact_affine_sign(annulus.plane.frame().z(), inner_far.center, annulus.center,),
            ),
            (
                Some(PredicateOrientation::Negative),
                Some(PredicateOrientation::Positive)
            ) | (
                Some(PredicateOrientation::Positive),
                Some(PredicateOrientation::Negative)
            )
        )
    {
        return Ok(None);
    }
    let annulus_face = store.get(annulus.face)?;
    let annulus_outward = annulus.plane.frame().z() * sense_factor(annulus_face.sense);
    let outer_side = exact_affine_sign(annulus_outward, outer_far.center, annulus.center);
    let inner_side = exact_affine_sign(annulus_outward, inner_far.center, annulus.center);
    let outer_orientation = nested_band_orientation_valid(store, outer, outer_contact, true)?;
    let inner_orientation = nested_band_orientation_valid(store, inner, inner_contact, false)?;
    let coherent = outer_side == Some(PredicateOrientation::Negative)
        && inner_side == Some(PredicateOrientation::Positive)
        && outer_orientation
        && inner_orientation;
    Ok(Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: if coherent {
            ShellOrientation::Positive
        } else {
            ShellOrientation::Invalid
        },
    }))
}

fn prepare_whole_band(
    store: &Store,
    shell_id: ShellId,
    face: FaceId,
    cylinder: Cylinder,
) -> Result<Option<WholeBand>> {
    let entity = store.get(face)?;
    let mut ends = Vec::with_capacity(2);
    for &loop_id in &entity.loops {
        let Some(end) = prepare_whole_end(store, shell_id, face, cylinder, loop_id)? else {
            return Ok(None);
        };
        ends.push(end);
    }
    let [first, second] = ends.as_slice() else {
        return Ok(None);
    };
    Ok((first.face != second.face).then_some(WholeBand {
        face,
        cylinder,
        ends: [*first, *second],
    }))
}

fn nested_band_orientation_valid(
    store: &Store,
    band: &WholeBand,
    contact: usize,
    outer: bool,
) -> Result<bool> {
    if store.get(band.face)?.sense != Sense::Forward {
        return Ok(false);
    }
    let far = band.ends[1 - contact];
    let contact = band.ends[contact];
    let (low, high) = if far.axial_parameter < contact.axial_parameter {
        (far, contact)
    } else if contact.axial_parameter < far.axial_parameter {
        (contact, far)
    } else {
        return Ok(false);
    };
    if !low.side_traverses_positive_u || high.side_traverses_positive_u {
        return Ok(false);
    }
    let far_is_low = far.axial_parameter < contact.axial_parameter;
    let expected_far = if far_is_low { -1 } else { 1 };
    let expected_contact = if outer { -expected_far } else { expected_far };
    Ok(
        oriented_axis_alignment(far.cap_axis_alignment, store.get(far.face)?.sense)
            == Some(expected_far)
            && oriented_axis_alignment(contact.cap_axis_alignment, store.get(contact.face)?.sense)
                == Some(expected_contact),
    )
}

fn prepare_whole_end(
    store: &Store,
    shell_id: ShellId,
    host_face: FaceId,
    cylinder: Cylinder,
    loop_id: LoopId,
) -> Result<Option<WholeEnd>> {
    let loop_ = store.get(loop_id)?;
    let [host_fin_id] = loop_.fins.as_slice() else {
        return Ok(None);
    };
    let host_fin = store.get(*host_fin_id)?;
    let edge = store.get(host_fin.edge)?;
    if loop_.face != host_face
        || edge.tolerance.is_some()
        || edge.bounds.is_some()
        || edge.vertices != [None, None]
        || certify_whole_fin_incidence(store, host_face, loop_id, *host_fin_id, LINEAR_RESOLUTION)
            != WholeFinIncidence::Certified
    {
        return Ok(None);
    }
    let Some(peer) = peer_fin(store, *host_fin_id)? else {
        return Ok(None);
    };
    let cap_loop_id = store.get(peer)?.parent;
    let cap_loop = store.get(cap_loop_id)?;
    let cap_face = cap_loop.face;
    let cap = store.get(cap_face)?;
    let [cap_fin_id] = cap_loop.fins.as_slice() else {
        return Ok(None);
    };
    let SurfaceGeom::Plane(plane) = store.get(cap.surface)? else {
        return Ok(None);
    };
    if *cap_fin_id != peer
        || cap.shell != shell_id
        || !matches!(cap.loops.len(), 1 | 2)
        || !cap.loops.contains(&cap_loop_id)
        || certify_whole_fin_incidence(store, cap_face, cap_loop_id, peer, LINEAR_RESOLUTION)
            != WholeFinIncidence::Certified
    {
        return Ok(None);
    }
    let Some(curve_id) = edge.curve else {
        return Ok(None);
    };
    let CurveGeom::Circle(circle) = store.get(curve_id)? else {
        return Ok(None);
    };
    if !two_host_circle_on_cylinder(*circle, cylinder)
        || !certified_parallel(plane.frame().z(), cylinder.frame().z())
    {
        return Ok(None);
    }
    let (Some(host_use), Some(cap_use)) = (host_fin.pcurve, store.get(peer)?.pcurve) else {
        return Ok(None);
    };
    let (Curve2dGeom::Line(host_line), Curve2dGeom::Circle(cap_circle)) =
        (store.get(host_use.curve())?, store.get(cap_use.curve())?)
    else {
        return Ok(None);
    };
    if host_line.dir().y != 0.0
        || host_line.dir().x == 0.0
        || host_use.closure_winding().is_none()
        || cap_use.closure_winding().is_none()
        || cap_circle.radius().to_bits() != circle.radius().to_bits()
    {
        return Ok(None);
    }
    let Some(edge_positive_host) = traversal_is_positive(
        [host_line.dir().x, host_use.edge_to_pcurve().scale()],
        Sense::Forward,
    ) else {
        return Ok(None);
    };
    let Some(side_traverses_positive_u) = traversal_is_positive(
        [host_line.dir().x, host_use.edge_to_pcurve().scale()],
        host_fin.sense,
    ) else {
        return Ok(None);
    };
    let Some(edge_positive_cap) =
        traversal_is_positive([cap_use.edge_to_pcurve().scale()], Sense::Forward)
    else {
        return Ok(None);
    };
    let (Some(host_orientation), Some(cap_orientation)) = (
        certify_loop_orientation(store, host_face, loop_id)?,
        certify_loop_orientation(store, cap_face, cap_loop_id)?,
    ) else {
        return Ok(None);
    };
    Ok(Some(WholeEnd {
        face: cap_face,
        center: circle.frame().origin(),
        plane: *plane,
        edge: host_fin.edge,
        circle: *circle,
        axial_parameter: host_line.origin().y,
        cap_axis_alignment: if edge_positive_host == edge_positive_cap {
            PredicateOrientation::Positive
        } else {
            PredicateOrientation::Negative
        },
        side_traverses_positive_u,
        local_orientation_valid: (cap_orientation == PredicateOrientation::Positive)
            == cap.sense.is_forward(),
        host_loop_orientation: host_orientation,
    }))
}

fn prepare_boundary(
    store: &Store,
    host_face: FaceId,
    cylinder: Cylinder,
    other_face: FaceId,
    loop_id: LoopId,
) -> Result<Option<Boundary>> {
    let loop_ = store.get(loop_id)?;
    if loop_.face != host_face
        || certify_loop_simplicity(store, loop_id)? != LoopSimplicity::Certified
    {
        return Ok(None);
    }
    let Some(loop_orientation) = certify_loop_orientation(store, host_face, loop_id)? else {
        return Ok(None);
    };
    let mut arcs = Vec::new();
    let mut rulings = Vec::new();
    for &fin_id in &loop_.fins {
        if certify_whole_fin_incidence(store, host_face, loop_id, fin_id, LINEAR_RESOLUTION)
            != WholeFinIncidence::Certified
        {
            return Ok(None);
        }
        let fin = store.get(fin_id)?;
        let edge = store.get(fin.edge)?;
        let (Some(curve_id), Some((lo, hi)), [Some(_), Some(_)], Some(peer)) = (
            edge.curve,
            edge.bounds,
            edge.vertices,
            peer_face_from_fin(store, fin_id)?,
        ) else {
            return Ok(None);
        };
        if edge.tolerance.is_some() || !lo.is_finite() || !hi.is_finite() || lo >= hi {
            return Ok(None);
        }
        match store.get(curve_id)? {
            CurveGeom::Circle(circle)
                if two_host_circle_on_cylinder(*circle, cylinder)
                    && matches!(store.get(store.get(peer)?.surface)?, SurfaceGeom::Plane(_)) =>
            {
                if arcs
                    .iter()
                    .any(|arc: &HostArc| arc.edge == fin.edge || arc.cap == peer)
                {
                    return Ok(None);
                }
                arcs.push(HostArc {
                    edge: fin.edge,
                    cap: peer,
                });
            }
            curve
                if peer == other_face
                    && exact_line_carrier(curve).is_some_and(|line| {
                        certified_parallel(line.dir(), cylinder.frame().z())
                    }) =>
            {
                if rulings.contains(&fin.edge) {
                    return Ok(None);
                }
                rulings.push(fin.edge);
            }
            _ => return Ok(None),
        }
    }
    Ok((!arcs.is_empty()).then_some(Boundary {
        loop_orientation,
        arcs,
        rulings,
    }))
}

fn prepare_transitions(
    store: &Store,
    first: &HostBand,
    second: &HostBand,
) -> Result<Option<Vec<Transition>>> {
    let mut transitions = Vec::new();
    let mut used_second = Vec::new();
    for first_arc in &first.boundary.arcs {
        let matching = second
            .boundary
            .arcs
            .iter()
            .copied()
            .filter(|arc| arc.cap == first_arc.cap && !used_second.contains(&arc.edge))
            .collect::<Vec<_>>();
        let [second_arc] = matching.as_slice() else {
            return Ok(None);
        };
        let Some(cap) = prepare_cap(store, first_arc.cap)? else {
            return Ok(None);
        };
        let mut first_use = None;
        let mut second_use = None;
        for &use_ in &cap.uses {
            match peer_face(store, use_)? {
                Some(peer) if peer == first.face && use_.edge == first_arc.edge => {
                    if first_use.replace(use_).is_some() {
                        return Ok(None);
                    }
                }
                Some(peer) if peer == second.face && use_.edge == second_arc.edge => {
                    if second_use.replace(use_).is_some() {
                        return Ok(None);
                    }
                }
                _ => return Ok(None),
            }
        }
        let (Some(first_use), Some(second_use)) = (first_use, second_use) else {
            return Ok(None);
        };
        used_second.push(second_arc.edge);
        transitions.push(Transition {
            cap,
            first: first_use,
            second: second_use,
        });
    }
    Ok((used_second.len() == second.boundary.arcs.len()).then_some(transitions))
}

fn prepare_chain(
    store: &Store,
    first: &HostBand,
    second: &HostBand,
    transitions: Vec<Transition>,
) -> Result<Option<Chain>> {
    let mut candidates = Vec::new();
    for lower in 0..transitions.len() {
        for upper in 0..transitions.len() {
            if lower == upper {
                continue;
            }
            let Some(translation) =
                translated_vertices(store, &transitions[lower].cap, &transitions[upper].cap)?
            else {
                continue;
            };
            let vector = translation.vector;
            let lower_center = circle_center(transitions[lower].first)?;
            let upper_center = circle_center(transitions[upper].first)?;
            if certified_parallel(vector, first.cylinder.frame().z())
                && certified_parallel(vector, second.cylinder.frame().z())
                && strictly_precedes(first.whole.center, lower_center, vector)
                && strictly_precedes(lower_center, upper_center, vector)
                && strictly_precedes(upper_center, second.whole.center, vector)
            {
                candidates.push((lower, upper, translation));
            }
        }
    }
    let [(lower, upper, translation)] = candidates.as_slice() else {
        return Ok(None);
    };
    if transitions
        .iter()
        .enumerate()
        .any(|(index, _)| index != *lower && index != *upper)
    {
        return Ok(None);
    }
    let lower = *lower;
    let upper = *upper;
    let translation = Translation {
        vector: translation.vector,
        vertices: translation.vertices.clone(),
    };
    Ok(Some(Chain {
        transitions,
        lower,
        upper,
        translation,
    }))
}

fn certify_chain_geometry(
    store: &Store,
    first: &HostBand,
    second: &HostBand,
    lower: &Transition,
    upper: &Transition,
    translation: &Translation,
) -> Result<bool> {
    if !certified_nonzero(translation.vector)
        || !complementary_arcs(lower.first, upper.first, translation)
        || !complementary_arcs(lower.second, upper.second, translation)
        || !certify_radial_roles(first.cylinder, second.cylinder, lower, upper)
        || !rulings_biject_vertices(store, &first.boundary.rulings, translation)?
    {
        return Ok(false);
    }
    Ok(true)
}

fn certify_radial_roles(
    first: Cylinder,
    second: Cylinder,
    lower: &Transition,
    upper: &Transition,
) -> bool {
    classify_arc(second, lower.first, lower.second) == Some(RadialSide::Inside)
        && classify_arc(first, lower.second, lower.first) == Some(RadialSide::Outside)
        && classify_arc(second, upper.first, upper.second) == Some(RadialSide::Outside)
        && classify_arc(first, upper.second, upper.first) == Some(RadialSide::Inside)
}

fn classify_arc(cylinder: Cylinder, arc: CapUse, portal: CapUse) -> Option<RadialSide> {
    let (ProfileCarrier::Circle(circle), ProfileCarrier::Circle(portal_circle)) =
        (arc.carrier, portal.carrier)
    else {
        return None;
    };
    circle_secant_span_side(
        cylinder,
        circle,
        arc.range,
        portal_circle,
        arc.tail != arc.head,
    )
}

fn complementary_arcs(first: CapUse, second: CapUse, translation: &Translation) -> bool {
    let (ProfileCarrier::Circle(first_circle), ProfileCarrier::Circle(second_circle)) =
        (first.carrier, second.carrier)
    else {
        return false;
    };
    if first_circle.radius().to_bits() != second_circle.radius().to_bits()
        || !certified_parallel(first_circle.frame().z(), second_circle.frame().z())
        || !certified_close(
            first_circle.frame().origin() + translation.vector,
            second_circle.frame().origin(),
        )
    {
        return false;
    }
    let (Some(mapped_tail), Some(mapped_head)) = (
        mapped_vertex(&translation.vertices, first.tail),
        mapped_vertex(&translation.vertices, first.head),
    ) else {
        return false;
    };
    (mapped_tail == second.tail && mapped_head == second.head)
        || (mapped_tail == second.head && mapped_head == second.tail)
}

fn rulings_biject_vertices(
    store: &Store,
    rulings: &[EdgeId],
    translation: &Translation,
) -> Result<bool> {
    let mut used = Vec::new();
    for &(source, target) in &translation.vertices {
        let mut matches = Vec::new();
        for &ruling in rulings {
            if !used.contains(&ruling)
                && ruling_connects(store, ruling, source, target, translation.vector)?
            {
                matches.push(ruling);
            }
        }
        let [ruling] = matches.as_slice() else {
            return Ok(false);
        };
        used.push(*ruling);
    }
    Ok(used.len() == rulings.len())
}

fn certification_from_orientation(
    store: &Store,
    first: &HostBand,
    second: &HostBand,
    lower: &Transition,
    upper: &Transition,
    outward_axis: Vec3,
) -> Result<ShellCertification> {
    // The strict radial roles already prove that each retained cylinder patch
    // is either the sole disk side or lies outside the peer disk. For the
    // right-handed authored frame, Cylinder `du x dv` is positive-radius
    // radial, so the live face sense is exactly the support sign of that union
    // boundary patch in every frame and common-axis direction.
    let first_support = sense_factor(store.get(first.face)?.sense) as i8;
    let second_support = sense_factor(store.get(second.face)?.sense) as i8;
    let cap_sign = |face: FaceId, plane: kgeom::surface::Plane, expected: i8| -> Result<bool> {
        let entity = store.get(face)?;
        Ok(
            oriented_dot_sign(plane.frame().z() * sense_factor(entity.sense), outward_axis)
                == Some(expected),
        )
    };
    let coherent = first.whole.local_orientation_valid
        && second.whole.local_orientation_valid
        && lower.cap.local_orientation_valid
        && upper.cap.local_orientation_valid
        && first.whole.host_loop_orientation != first.boundary.loop_orientation
        && second.whole.host_loop_orientation != second.boundary.loop_orientation
        && second_support == first_support
        && cap_sign(first.whole.face, first.whole.plane, -first_support)?
        && cap_sign(lower.cap.face, lower.cap.plane, -first_support)?
        && cap_sign(upper.cap.face, upper.cap.plane, first_support)?
        && cap_sign(second.whole.face, second.whole.plane, first_support)?;
    Ok(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: if coherent {
            if first_support > 0 {
                ShellOrientation::Positive
            } else {
                ShellOrientation::Negative
            }
        } else {
            ShellOrientation::Invalid
        },
    })
}

fn same_unique_edges(first: &[EdgeId], second: &[EdgeId]) -> bool {
    let unique = |edges: &[EdgeId]| {
        !edges
            .iter()
            .enumerate()
            .any(|(index, edge)| edges[index + 1..].contains(edge))
    };
    unique(first)
        && unique(second)
        && first.len() == second.len()
        && first.iter().all(|edge| second.contains(edge))
}

fn circle_center(use_: CapUse) -> Result<Point3> {
    match use_.carrier {
        ProfileCarrier::Circle(circle) => Ok(circle.frame().origin()),
        ProfileCarrier::Line(_) => Err(kcore::error::Error::InvalidGeometry {
            reason: "two-host axial-chain transition lost its circle carrier",
        }),
    }
}

fn certify_host_band_base(
    band: &HostBand,
    transitions: &[Transition],
    first_use: bool,
) -> Result<bool> {
    let mut terminal = None;
    let mut greatest = -1.0_f64;
    for transition in transitions {
        let use_ = if first_use {
            transition.first
        } else {
            transition.second
        };
        let center = circle_center(use_)?;
        let distance = (center - band.whole.center)
            .dot(band.cylinder.frame().z())
            .abs();
        if distance > greatest {
            greatest = distance;
            terminal = Some(center);
        }
    }
    let Some(terminal) = terminal else {
        return Ok(false);
    };
    certify_cylindrical_base_embedding(band.cylinder, band.whole.center, terminal)
}

/// Delegate every independently reconstructed finite-cylinder base to the
/// existing generalized convex-cylinder theorem. Surgery incidence is bound
/// separately; this proves the analytic support and axial extent without
/// trusting any family verdict.
pub(super) fn certify_cylindrical_base_embedding(
    cylinder: Cylinder,
    first_center: Point3,
    second_center: Point3,
) -> Result<bool> {
    let coordinate = |point: Point3| (point - cylinder.frame().origin()).dot(cylinder.frame().z());
    let first = coordinate(first_center);
    let second = coordinate(second_center);
    if !first.is_finite() || !second.is_finite() || first == second {
        return Ok(false);
    }
    let input = CylindricalBandSolidInput::new(
        *cylinder.frame(),
        cylinder.radius(),
        ParamRange::new(first.min(second), first.max(second)),
    );
    let mut base = Store::new();
    let mut transaction = base.transaction()?;
    let output = transaction.assemble_cylindrical_band_solid(&input)?;
    Ok(matches!(
        super::super::convex_cylindrical_shell_proof::certify_convex_cylindrical_shell(
            transaction.store(),
            output.shell(),
            None,
        )?,
        Some(ShellCertification {
            embedding: ShellEmbedding::Certified,
            ..
        })
    ))
}

fn strictly_precedes(first: Point3, second: Point3, direction: Vec3) -> bool {
    let offset = second - first;
    certified_nonzero(offset) && oriented_dot_sign(offset, direction) == Some(1)
}

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

fn strictly_contains_cylinder_support(outer: Cylinder, inner: Cylinder) -> bool {
    if outer.radius() <= inner.radius() || !certified_parallel(outer.frame().z(), inner.frame().z())
    {
        return false;
    }
    let Some(radial) = axis_distance_squared(
        inner.frame().origin(),
        outer.frame().origin(),
        outer.frame().z(),
    ) else {
        return false;
    };
    let clearance = Interval::point(outer.radius())
        - Interval::point(inner.radius())
        - Interval::point(2.0 * LINEAR_RESOLUTION);
    radial.lo().is_finite()
        && radial.hi().is_finite()
        && clearance.lo().is_finite()
        && clearance.lo() > 0.0
        && radial.hi() < clearance.square().lo()
}
