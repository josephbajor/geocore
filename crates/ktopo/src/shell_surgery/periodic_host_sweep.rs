//! Shared theorem verification for product sweeps replacing patches of a periodic host.

use super::super::shell_lemmas::{
    Cap, CapUse, IntervalBounds2, ProfileCarrier, RadialSide, Side, certified_nonzero,
    certified_parallel, certify_sweep_support, coordinate_interval, edge_has_vertices,
    mapped_vertex, oriented_dot_sign, peer_face, prepare_cap, prepare_side, ruling_connects,
    translated_carrier, translated_vertices,
};
use super::*;
use crate::entity::FinId;
use kgeom::curve2d::Curve2d;
use kgeom::param::ParamRange;
use kgeom::vec::Vec2;

#[path = "profile_radial_proof.rs"]
mod profile_radial_proof;
use profile_radial_proof::{profile_radial_bounds, profile_radial_side};

pub(super) fn certify_periodic_host_sweep_evidence(
    store: &Store,
    shell_id: ShellId,
    evidence: &PeriodicHostSurgeryEvidence,
) -> Result<Option<ShellCertification>> {
    if evidence.shell != shell_id {
        return Ok(None);
    }
    let shell = store.get(shell_id)?;
    if shell.faces.len() < 6 || !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(None);
    }

    let mut planar_faces = Vec::new();
    let mut host_matches = 0_usize;
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        if face.shell != shell_id {
            return Ok(None);
        }
        match store.get(face.surface)? {
            SurfaceGeom::Cylinder(cylinder) => {
                if face_id == evidence.host_face && same_cylinder(*cylinder, evidence.cylinder) {
                    host_matches += 1;
                }
            }
            SurfaceGeom::Plane(_) => planar_faces.push(face_id),
            _ => return Ok(None),
        }
    }
    if host_matches != 1 || planar_faces != evidence.planar_faces {
        return Ok(None);
    }

    certify_host_candidate(
        store,
        shell_id,
        evidence.host_face,
        evidence.cylinder,
        &planar_faces,
    )
}

pub(super) fn proof_work(store: &Store, shell_id: ShellId) -> Result<Option<u64>> {
    let shell = store.get(shell_id)?;
    let mut host_count = 0_usize;
    let mut plane_count = 0_usize;
    for &face_id in &shell.faces {
        match store.get(store.get(face_id)?.surface)? {
            SurfaceGeom::Cylinder(_) => host_count += 1,
            SurfaceGeom::Plane(_) => plane_count += 1,
            _ => return Ok(None),
        }
    }
    let Some(size) = shell_proof_size(store, shell_id)? else {
        return Ok(None);
    };
    let (Some(hosts), Some(planes)) = (
        u64::try_from(host_count).ok(),
        u64::try_from(plane_count).ok(),
    ) else {
        return Ok(None);
    };
    let Some(pairs) = planes
        .checked_mul(planes.saturating_sub(1))
        .map(|ordered| ordered / 2)
    else {
        return Ok(None);
    };
    let Some(pair_groups) = pairs.checked_add(1) else {
        return Ok(None);
    };
    Ok(quadratic_proof_work(size, 64, 0, pair_groups)
        .and_then(|per_host| per_host.checked_mul(hosts)))
}

struct Portal {
    fins: Vec<(FinId, EdgeId)>,
    arc_edges: [EdgeId; 2],
    ruling_edges: [EdgeId; 2],
}

#[derive(Debug, Clone, Copy)]
struct OuterBoundary {
    cap_face: FaceId,
    edge: EdgeId,
    center: Point3,
    host_fin: FinId,
    cap_fin: FinId,
    positive_sense: Sense,
    side_traverses_positive_u: bool,
}

#[derive(Debug)]
struct Attachment {
    faces: Vec<FaceId>,
    portals: Vec<usize>,
    orientation: i8,
    side: RadialSide,
    radial_bounds: IntervalBounds2,
    axial: Interval,
}

fn certify_host_candidate(
    store: &Store,
    shell_id: ShellId,
    host_face: FaceId,
    cylinder: Cylinder,
    planar: &[FaceId],
) -> Result<Option<ShellCertification>> {
    if certify_face_loop_layout(store, host_face)? != LoopContainment::Certified {
        return Ok(None);
    }
    let host_entity = store.get(host_face)?;
    let mut ring_loops = Vec::new();
    let mut portal_loops = Vec::new();
    for &loop_id in &host_entity.loops {
        let loop_ = store.get(loop_id)?;
        if loop_.face != host_face {
            return Ok(None);
        }
        let is_ring = match loop_.fins.as_slice() {
            [fin_id] => {
                let edge = store.get(store.get(*fin_id)?.edge)?;
                edge.bounds.is_none() && edge.vertices == [None, None]
            }
            _ => false,
        };
        if is_ring {
            ring_loops.push(loop_id);
        } else {
            portal_loops.push(loop_id);
        }
    }
    let [ring_a, ring_b] = ring_loops.as_slice() else {
        return Ok(None);
    };
    if portal_loops.is_empty() {
        return Ok(None);
    }
    let Some(cap_a) = single_fin_peer_face(store, *ring_a)? else {
        return Ok(None);
    };
    let Some(cap_b) = single_fin_peer_face(store, *ring_b)? else {
        return Ok(None);
    };
    if cap_a == cap_b || !planar.contains(&cap_a) || !planar.contains(&cap_b) {
        return Ok(None);
    }
    let cap_faces = [cap_a, cap_b];
    let Some(first_ring) =
        prepare_outer_boundary(store, shell_id, host_face, cylinder, *ring_a, &cap_faces)?
    else {
        return Ok(None);
    };
    let Some(second_ring) =
        prepare_outer_boundary(store, shell_id, host_face, cylinder, *ring_b, &cap_faces)?
    else {
        return Ok(None);
    };
    if first_ring.edge == second_ring.edge || first_ring.cap_face == second_ring.cap_face {
        return Ok(None);
    }
    let ((low_loop, low), (high_loop, high)) =
        match exact_affine_sign(cylinder.frame().z(), second_ring.center, first_ring.center) {
            Some(PredicateOrientation::Positive) => ((*ring_a, first_ring), (*ring_b, second_ring)),
            Some(PredicateOrientation::Negative) => ((*ring_b, second_ring), (*ring_a, first_ring)),
            _ => return Ok(None),
        };
    let host_sense = store.get(host_face)?.sense;
    let base_boundary_orientation_valid = low.side_traverses_positive_u
        == (host_sense == Sense::Forward)
        && high.side_traverses_positive_u == (host_sense == Sense::Reversed);
    let Some(base_certification) = certify_cylindrical_base(
        store,
        shell_id,
        host_face,
        [(low_loop, low), (high_loop, high)],
        cap_faces,
    )?
    else {
        return Ok(None);
    };
    if base_certification.embedding != ShellEmbedding::Certified {
        return Ok(None);
    }

    let mut portals = Vec::with_capacity(portal_loops.len());
    for loop_id in portal_loops {
        let Some(portal) = prepare_portal(store, host_face, cylinder, loop_id)? else {
            return Ok(None);
        };
        portals.push(portal);
    }

    let base_orientation = base_certification.orientation;
    let target_faces = store
        .get(shell_id)?
        .faces
        .iter()
        .copied()
        .filter(|face| *face != host_face && *face != cap_a && *face != cap_b)
        .collect::<Vec<_>>();
    if target_faces.is_empty() {
        return Ok(None);
    }
    let cap_candidates = target_faces
        .iter()
        .copied()
        .filter(|face| planar.contains(face))
        .collect::<Vec<_>>();

    let mut candidates = Vec::new();
    for (index, &first) in cap_candidates.iter().enumerate() {
        for &second in &cap_candidates[index + 1..] {
            if let Some(candidate) = prepare_attachment(
                store,
                host_face,
                cylinder,
                &portals,
                &target_faces,
                first,
                second,
            )? {
                candidates.push(candidate);
            }
        }
    }
    let Some(attachments) = unique_component_cover(&target_faces, portals.len(), candidates) else {
        return Ok(None);
    };
    for (index, first) in attachments.iter().enumerate() {
        for second in &attachments[index + 1..] {
            if !attachments_separated(first, second) {
                return Ok(None);
            }
        }
    }

    let base_sign = match base_orientation {
        ShellOrientation::Positive => 1,
        ShellOrientation::Negative => -1,
        ShellOrientation::Invalid | ShellOrientation::Indeterminate => 0,
    };
    let coherent = base_sign != 0
        && base_boundary_orientation_valid
        && attachments.iter().all(|attachment| {
            attachment.orientation == base_sign * attachment.side.orientation_factor()
        });
    Ok(Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: if coherent {
            base_orientation
        } else {
            ShellOrientation::Invalid
        },
    }))
}

fn certify_cylindrical_base(
    store: &Store,
    shell_id: ShellId,
    host_face: FaceId,
    boundaries: [(LoopId, OuterBoundary); 2],
    cap_faces: [FaceId; 2],
) -> Result<Option<ShellCertification>> {
    let mut base = store.clone();
    base.get_mut(shell_id)?.faces = vec![host_face, cap_faces[0], cap_faces[1]];
    base.get_mut(host_face)?.loops = boundaries.map(|(loop_id, _)| loop_id).to_vec();
    let host_sense = base.get(host_face)?.sense;
    for (index, (_, boundary)) in boundaries.into_iter().enumerate() {
        let expected_positive = matches!(
            (index, host_sense),
            (0, Sense::Forward) | (1, Sense::Reversed)
        );
        let host_fin_sense = if expected_positive {
            boundary.positive_sense
        } else {
            opposite_sense(boundary.positive_sense)
        };
        base.get_mut(boundary.host_fin)?.sense = host_fin_sense;
        base.get_mut(boundary.cap_fin)?.sense = opposite_sense(host_fin_sense);
    }
    super::super::convex_cylindrical_shell_proof::certify_convex_cylindrical_shell(
        &base, shell_id, None,
    )
}

fn opposite_sense(sense: Sense) -> Sense {
    match sense {
        Sense::Forward => Sense::Reversed,
        Sense::Reversed => Sense::Forward,
    }
}

fn single_fin_peer_face(store: &Store, loop_id: LoopId) -> Result<Option<FaceId>> {
    let loop_ = store.get(loop_id)?;
    let [fin_id] = loop_.fins.as_slice() else {
        return Ok(None);
    };
    let fin = store.get(*fin_id)?;
    let edge = store.get(fin.edge)?;
    let [first, second] = edge.fins.as_slice() else {
        return Ok(None);
    };
    let peer = if first == fin_id {
        *second
    } else if second == fin_id {
        *first
    } else {
        return Ok(None);
    };
    if store.get(peer)?.sense == fin.sense {
        return Ok(None);
    }
    Ok(Some(store.get(store.get(peer)?.parent)?.face))
}

#[allow(clippy::too_many_arguments)]
fn prepare_outer_boundary(
    store: &Store,
    shell_id: ShellId,
    host_face: FaceId,
    cylinder: Cylinder,
    loop_id: LoopId,
    cap_faces: &[FaceId; 2],
) -> Result<Option<OuterBoundary>> {
    let loop_ = store.get(loop_id)?;
    let [host_fin_id] = loop_.fins.as_slice() else {
        return Ok(None);
    };
    if loop_.face != host_face
        || certify_loop_simplicity(store, loop_id)? != LoopSimplicity::Certified
    {
        return Ok(None);
    }
    let host_fin = store.get(*host_fin_id)?;
    let edge = store.get(host_fin.edge)?;
    let [first, second] = edge.fins.as_slice() else {
        return Ok(None);
    };
    if edge.tolerance.is_some()
        || edge.bounds.is_some()
        || edge.vertices != [None, None]
        || !edge.fins.contains(host_fin_id)
    {
        return Ok(None);
    }
    let cap_fin_id = if first == host_fin_id {
        *second
    } else if second == host_fin_id {
        *first
    } else {
        return Ok(None);
    };
    let cap_fin = store.get(cap_fin_id)?;
    if cap_fin.edge != host_fin.edge || cap_fin.sense == host_fin.sense {
        return Ok(None);
    }
    let cap_loop_id = cap_fin.parent;
    let cap_loop = store.get(cap_loop_id)?;
    let cap_face = cap_loop.face;
    if !cap_faces.contains(&cap_face)
        || cap_loop.fins.as_slice() != [cap_fin_id]
        || certify_loop_simplicity(store, cap_loop_id)? != LoopSimplicity::Certified
    {
        return Ok(None);
    }
    let cap = store.get(cap_face)?;
    if cap.shell != shell_id || cap.loops.as_slice() != [cap_loop_id] {
        return Ok(None);
    }
    if certify_whole_fin_incidence(store, host_face, loop_id, *host_fin_id, LINEAR_RESOLUTION)
        != WholeFinIncidence::Certified
        || certify_whole_fin_incidence(store, cap_face, cap_loop_id, cap_fin_id, LINEAR_RESOLUTION)
            != WholeFinIncidence::Certified
    {
        return Ok(None);
    }
    let (Some(curve_id), Some(host_use), Some(cap_use)) =
        (edge.curve, host_fin.pcurve, cap_fin.pcurve)
    else {
        return Ok(None);
    };
    let CurveGeom::Circle(circle) = store.get(curve_id)? else {
        return Ok(None);
    };
    let SurfaceGeom::Plane(plane) = store.get(cap.surface)? else {
        return Ok(None);
    };
    if !certified_parallel(cylinder.frame().z(), plane.frame().z()) {
        return Ok(None);
    }
    if !matches!(
        oriented_dot_sign(plane.frame().z(), cylinder.frame().z()),
        Some(1 | -1)
    ) {
        return Ok(None);
    }
    let geometry = (
        circle.radius().to_bits() == cylinder.radius().to_bits(),
        certified_parallel(circle.frame().z(), cylinder.frame().z()),
        certified_point_on_axis(cylinder.frame(), circle.frame().origin()),
        certified_point_on_plane(plane.frame(), circle.frame().origin()),
    );
    if !geometry.0 || !geometry.1 || !geometry.2 || !geometry.3 {
        return Ok(None);
    }
    let Curve2dGeom::Line(host_line) = store.get(host_use.curve())? else {
        return Ok(None);
    };
    let Curve2dGeom::Circle(cap_circle) = store.get(cap_use.curve())? else {
        return Ok(None);
    };
    if host_line.dir().y != 0.0
        || host_line.dir().x == 0.0
        || cap_circle.radius().to_bits() != cylinder.radius().to_bits()
        || !matches!(host_use.closure_winding(), Some([1 | -1, 0]))
        || cap_use.closure_winding() != Some([0, 0])
    {
        return Ok(None);
    }
    let traversal = [host_line.dir().x, host_use.edge_to_pcurve().scale()];
    let Some(side_traverses_positive_u) = traversal_is_positive(traversal, host_fin.sense) else {
        return Ok(None);
    };
    let positive_sense = if traversal_is_positive(traversal, Sense::Forward) == Some(true) {
        Sense::Forward
    } else if traversal_is_positive(traversal, Sense::Reversed) == Some(true) {
        Sense::Reversed
    } else {
        return Ok(None);
    };
    Ok(Some(OuterBoundary {
        cap_face,
        edge: host_fin.edge,
        center: circle.frame().origin(),
        host_fin: *host_fin_id,
        cap_fin: cap_fin_id,
        positive_sense,
        side_traverses_positive_u,
    }))
}

fn prepare_portal(
    store: &Store,
    host_face: FaceId,
    cylinder: Cylinder,
    loop_id: LoopId,
) -> Result<Option<Portal>> {
    let loop_ = store.get(loop_id)?;
    if loop_.face != host_face
        || loop_.fins.len() != 4
        || certify_loop_simplicity(store, loop_id)? != LoopSimplicity::Certified
    {
        return Ok(None);
    }
    let mut horizontal = Vec::new();
    let mut vertical = Vec::new();
    let mut fins = Vec::new();
    for &fin_id in &loop_.fins {
        if certify_whole_fin_incidence(store, host_face, loop_id, fin_id, LINEAR_RESOLUTION)
            != WholeFinIncidence::Certified
        {
            return Ok(None);
        }
        let fin = store.get(fin_id)?;
        let edge = store.get(fin.edge)?;
        let (Some(curve_id), Some((lo, hi)), Some(use_)) = (edge.curve, edge.bounds, fin.pcurve)
        else {
            return Ok(None);
        };
        if edge.tolerance.is_some()
            || edge.fins.len() != 2
            || !lo.is_finite()
            || !hi.is_finite()
            || lo >= hi
            || use_.closure_winding().is_some()
            || use_.seam().is_some()
        {
            return Ok(None);
        }
        let Curve2dGeom::Line(line2d) = store.get(use_.curve())? else {
            return Ok(None);
        };
        let mapped = ParamRange::new(use_.edge_to_pcurve().map(lo), use_.edge_to_pcurve().map(hi));
        let chart_u = f64::from(use_.chart().period_shifts()[0]) * core::f64::consts::TAU;
        let first = line2d.eval(mapped.lo) + Vec2::new(chart_u, 0.0);
        let second = line2d.eval(mapped.hi) + Vec2::new(chart_u, 0.0);
        match store.get(curve_id)? {
            CurveGeom::Circle(circle)
                if line2d.dir().x != 0.0
                    && line2d.dir().y == 0.0
                    && circle.radius().to_bits() == cylinder.radius().to_bits()
                    && certified_parallel(circle.frame().z(), cylinder.frame().z())
                    && certified_point_on_axis(cylinder.frame(), circle.frame().origin()) =>
            {
                horizontal.push((
                    fin.edge,
                    ParamRange::new(first.x.min(second.x), first.x.max(second.x)),
                ));
            }
            curve if line2d.dir().x == 0.0 && line2d.dir().y != 0.0 => {
                let Some(line) = exact_line_carrier(curve) else {
                    return Ok(None);
                };
                if !certified_parallel(line.dir(), cylinder.frame().z()) {
                    return Ok(None);
                }
                vertical.push(fin.edge);
            }
            _ => return Ok(None),
        }
        fins.push((fin_id, fin.edge));
    }
    let [low_arc, high_arc] = horizontal.as_slice() else {
        return Ok(None);
    };
    let [first_ruling, second_ruling] = vertical.as_slice() else {
        return Ok(None);
    };
    let u = ParamRange::new(
        low_arc.1.lo.min(high_arc.1.lo),
        low_arc.1.hi.max(high_arc.1.hi),
    );
    if u.width() <= ANGULAR_RESOLUTION || u.width() >= core::f64::consts::TAU - ANGULAR_RESOLUTION {
        return Ok(None);
    }
    Ok(Some(Portal {
        fins,
        arc_edges: [low_arc.0, high_arc.0],
        ruling_edges: [*first_ruling, *second_ruling],
    }))
}

#[allow(clippy::too_many_arguments)]
fn prepare_attachment(
    store: &Store,
    host_face: FaceId,
    cylinder: Cylinder,
    portals: &[Portal],
    target_faces: &[FaceId],
    first_face: FaceId,
    second_face: FaceId,
) -> Result<Option<Attachment>> {
    let Some(first) = prepare_cap(store, first_face)? else {
        return Ok(None);
    };
    let Some(second) = prepare_cap(store, second_face)? else {
        return Ok(None);
    };
    if first.uses.len() != second.uses.len() || first.vertices.len() != second.vertices.len() {
        return Ok(None);
    }
    let Some(translation) = translated_vertices(store, &first, &second)? else {
        return Ok(None);
    };
    if !certified_parallel(translation.vector, cylinder.frame().z())
        || !certified_nonzero(translation.vector)
        || !certified_parallel(translation.vector, first.plane.frame().z())
        || !certified_parallel(translation.vector, second.plane.frame().z())
    {
        return Ok(None);
    }
    let first_sign = oriented_dot_sign(
        first.plane.frame().z() * sense_factor(store.get(first.face)?.sense),
        -translation.vector,
    );
    let second_sign = oriented_dot_sign(
        second.plane.frame().z() * sense_factor(store.get(second.face)?.sense),
        translation.vector,
    );
    let (Some(first_sign), Some(second_sign)) = (first_sign, second_sign) else {
        return Ok(None);
    };
    let mut support_signs = Vec::new();
    let mut used_second = Vec::new();
    let mut used_sides = Vec::new();
    let mut used_portals = Vec::new();

    for boundary in &first.uses {
        let Some(mapped_tail) = mapped_vertex(&translation.vertices, boundary.tail) else {
            return Ok(None);
        };
        let Some(mapped_head) = mapped_vertex(&translation.vertices, boundary.head) else {
            return Ok(None);
        };
        let mut matching = Vec::new();
        for candidate in &second.uses {
            if !used_second.contains(&candidate.edge)
                && edge_has_vertices(store, candidate.edge, mapped_tail, mapped_head)?
                && translated_carrier(*boundary, *candidate, translation.vector)
            {
                matching.push(candidate);
            }
        }
        let [mapped_top] = matching.as_slice() else {
            return Ok(None);
        };
        let Some(first_peer) = peer_face(store, *boundary)? else {
            return Ok(None);
        };
        let Some(second_peer) = peer_face(store, **mapped_top)? else {
            return Ok(None);
        };
        if first_peer == host_face || second_peer == host_face {
            if first_peer != host_face || second_peer != host_face {
                return Ok(None);
            }
            let matching_portals = portals
                .iter()
                .enumerate()
                .filter(|(index, portal)| {
                    !used_portals.contains(index)
                        && portal.arc_edges.contains(&boundary.edge)
                        && portal.arc_edges.contains(&mapped_top.edge)
                })
                .collect::<Vec<_>>();
            let [(portal_index, portal)] = matching_portals.as_slice() else {
                return Ok(None);
            };
            let valid_rulings = (ruling_connects(
                store,
                portal.ruling_edges[0],
                boundary.tail,
                mapped_tail,
                translation.vector,
            )? && ruling_connects(
                store,
                portal.ruling_edges[1],
                boundary.head,
                mapped_head,
                translation.vector,
            )?) || (ruling_connects(
                store,
                portal.ruling_edges[0],
                boundary.head,
                mapped_head,
                translation.vector,
            )? && ruling_connects(
                store,
                portal.ruling_edges[1],
                boundary.tail,
                mapped_tail,
                translation.vector,
            )?);
            if !valid_rulings {
                return Ok(None);
            }
            let virtual_side = Side {
                face: host_face,
                fins: portal.fins.clone(),
            };
            let Some(host_sign) = certify_sweep_support(
                store,
                &virtual_side,
                *boundary,
                **mapped_top,
                translation.vector,
            )?
            else {
                return Ok(None);
            };
            support_signs.push((host_sign, true));
            used_portals.push(*portal_index);
        } else {
            if first_peer != second_peer
                || !target_faces.contains(&first_peer)
                || first_peer == first.face
                || first_peer == second.face
                || used_sides.contains(&first_peer)
            {
                return Ok(None);
            }
            let Some(side) = prepare_side(store, first_peer)? else {
                return Ok(None);
            };
            if !side.fins.iter().any(|(_, edge)| *edge == boundary.edge)
                || !side.fins.iter().any(|(_, edge)| *edge == mapped_top.edge)
            {
                return Ok(None);
            }
            let rulings = side
                .fins
                .iter()
                .copied()
                .filter(|(_, edge)| *edge != boundary.edge && *edge != mapped_top.edge)
                .collect::<Vec<_>>();
            let [first_ruling, second_ruling] = rulings.as_slice() else {
                return Ok(None);
            };
            let valid_rulings = (ruling_connects(
                store,
                first_ruling.1,
                boundary.tail,
                mapped_tail,
                translation.vector,
            )? && ruling_connects(
                store,
                second_ruling.1,
                boundary.head,
                mapped_head,
                translation.vector,
            )?) || (ruling_connects(
                store,
                first_ruling.1,
                boundary.head,
                mapped_head,
                translation.vector,
            )? && ruling_connects(
                store,
                second_ruling.1,
                boundary.tail,
                mapped_tail,
                translation.vector,
            )?);
            if !valid_rulings {
                return Ok(None);
            }
            let Some(side_sign) =
                certify_sweep_support(store, &side, *boundary, **mapped_top, translation.vector)?
            else {
                return Ok(None);
            };
            support_signs.push((side_sign, false));
            used_sides.push(first_peer);
        }
        used_second.push(mapped_top.edge);
    }
    if used_second.len() != second.uses.len() || used_portals.is_empty() {
        return Ok(None);
    }
    let mut portal_vertices = Vec::new();
    for &index in &used_portals {
        for edge in portals[index].arc_edges {
            portal_vertices.extend(store.get(edge)?.vertices.into_iter().flatten());
        }
    }
    let Some(side) = profile_radial_side(store, cylinder, &first, host_face, &portal_vertices)?
    else {
        return Ok(None);
    };
    let orientation_valid = first.local_orientation_valid
        && second.local_orientation_valid
        && first_sign == second_sign
        && support_signs.iter().all(|(sign, virtual_portal)| {
            let expected = match (side, virtual_portal) {
                (RadialSide::Outside, false) => first_sign,
                (RadialSide::Outside, true) => -first_sign,
                (RadialSide::Inside, false) => -first_sign,
                (RadialSide::Inside, true) => first_sign,
            };
            *sign == expected
        });
    let Some(radial_bounds) = profile_radial_bounds(store, cylinder, &first)? else {
        return Ok(None);
    };
    let first_axial = coordinate_interval(
        cylinder.frame(),
        cylinder.frame().z(),
        first.plane.frame().origin(),
    );
    let second_axial = coordinate_interval(
        cylinder.frame(),
        cylinder.frame().z(),
        second.plane.frame().origin(),
    );
    let axial = Interval::new(
        first_axial.lo().min(second_axial.lo()),
        first_axial.hi().max(second_axial.hi()),
    );

    let mut faces = vec![first.face, second.face];
    faces.extend(used_sides);
    used_portals.sort_unstable();
    Ok(Some(Attachment {
        faces,
        portals: used_portals,
        orientation: if orientation_valid { first_sign } else { 0 },
        side,
        radial_bounds,
        axial,
    }))
}

fn unique_component_cover(
    target_faces: &[FaceId],
    portal_count: usize,
    candidates: Vec<Attachment>,
) -> Option<Vec<Attachment>> {
    if candidates.is_empty() {
        return None;
    }
    for &face in target_faces {
        if candidates
            .iter()
            .filter(|candidate| candidate.faces.contains(&face))
            .count()
            != 1
        {
            return None;
        }
    }
    for portal in 0..portal_count {
        if candidates
            .iter()
            .filter(|candidate| candidate.portals.contains(&portal))
            .count()
            != 1
        {
            return None;
        }
    }
    let mut selected = Vec::new();
    for candidate in candidates {
        if !selected.iter().any(|prior: &Attachment| {
            prior
                .faces
                .iter()
                .any(|face| candidate.faces.contains(face))
        }) {
            selected.push(candidate);
        }
    }
    let covered_faces = selected
        .iter()
        .flat_map(|candidate| &candidate.faces)
        .count();
    let covered_portals = selected
        .iter()
        .flat_map(|candidate| &candidate.portals)
        .count();
    (covered_faces == target_faces.len() && covered_portals == portal_count).then_some(selected)
}

fn attachments_separated(first: &Attachment, second: &Attachment) -> bool {
    first.axial.hi() < second.axial.lo()
        || second.axial.hi() < first.axial.lo()
        || first.radial_bounds.x.hi() < second.radial_bounds.x.lo()
        || second.radial_bounds.x.hi() < first.radial_bounds.x.lo()
        || first.radial_bounds.y.hi() < second.radial_bounds.y.lo()
        || second.radial_bounds.y.hi() < first.radial_bounds.y.lo()
}

fn certified_point_on_plane(frame: &Frame, point: Point3) -> bool {
    let coordinate = coordinate_interval(frame, frame.z(), point);
    coordinate.lo().is_finite()
        && coordinate.lo() >= -LINEAR_RESOLUTION
        && coordinate.hi() <= LINEAR_RESOLUTION
}
