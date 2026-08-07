//! Shared verification for translated mixed-carrier profile sweeps.

use super::super::shell_lemmas::{
    Cap, CapUse, ProfileCarrier, Side, certified_nonzero, certified_parallel,
    certify_sweep_support, circle_affine_range, mapped_vertex, oriented_dot_sign, peer_face,
    prepare_cap, prepare_side, ruling_connects, translated_carrier, translated_vertices,
};
use super::*;
use crate::entity::FinId;

#[derive(Debug)]
struct HostFacet {
    face: FaceId,
    vertices: Vec<VertexId>,
    outward: Vec3,
    origin: Point3,
}

#[derive(Debug)]
struct Portal {
    face: FaceId,
    loop_id: LoopId,
    side: Side,
}

#[derive(Debug)]
struct Feature {
    caps: [Cap; 2],
    cylinder_side: Side,
    translation: Vec3,
    orientation: Option<i8>,
    sweep_signs: Vec<i8>,
}

#[derive(Debug)]
struct Patch {
    feature: Feature,
    radial_side: i8,
    sweep_orientation_valid: bool,
}

type HostClassification = (Vec<HostFacet>, Vec<(Portal, Vec<FaceId>)>);

pub(super) fn certify_chord_portal_evidence(
    store: &Store,
    shell_id: ShellId,
    evidence: &ChordPortalSurgeryEvidence,
) -> Result<Option<ShellCertification>> {
    if evidence.shell != shell_id || evidence.features.is_empty() {
        return Ok(None);
    }
    let Some((host, portal_features)) = verify_host(store, shell_id, evidence)? else {
        return Ok(None);
    };
    let host_certification = certify_convex_planar_facets(
        store,
        host.iter()
            .map(|facet| (facet.face, facet.vertices.clone()))
            .collect(),
        None,
    )?;
    if host_certification.embedding != ShellEmbedding::Certified {
        return Ok(Some(indeterminate()));
    }
    let mut patches = Vec::with_capacity(portal_features.len());
    for (portal, feature_faces) in portal_features {
        let Some(feature) = prepare_feature(store, &portal, &feature_faces)? else {
            return Ok(None);
        };
        let Some(radial_side) = certify_feature_supports(store, &host, &portal, &feature)? else {
            return Ok(Some(indeterminate()));
        };
        let sweep_orientation_valid = feature.orientation.is_some_and(|orientation| {
            feature
                .sweep_signs
                .iter()
                .all(|sign| *sign == orientation * radial_side)
        });
        patches.push(Patch {
            feature,
            radial_side,
            sweep_orientation_valid,
        });
    }
    if !certify_patch_separation(store, &host, &patches)? {
        return Ok(Some(indeterminate()));
    }
    let host_sign = match host_certification.orientation {
        ShellOrientation::Positive => 1,
        ShellOrientation::Negative => -1,
        ShellOrientation::Invalid => 0,
        ShellOrientation::Indeterminate => return Ok(Some(indeterminate())),
    };
    let coherent = host_sign != 0
        && patches.iter().all(|patch| {
            patch.sweep_orientation_valid
                && patch.feature.orientation == Some(host_sign * patch.radial_side)
        });
    Ok(Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: if coherent {
            host_certification.orientation
        } else {
            ShellOrientation::Invalid
        },
    }))
}

fn verify_host(
    store: &Store,
    shell_id: ShellId,
    evidence: &ChordPortalSurgeryEvidence,
) -> Result<Option<HostClassification>> {
    let shell = store.get(shell_id)?;
    let mut host = Vec::new();
    let mut portals = Vec::new();
    let mut feature_faces = Vec::new();
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        if face.shell != shell_id {
            return Ok(None);
        }
        let SurfaceGeom::Plane(plane) = store.get(face.surface)? else {
            feature_faces.push(face_id);
            continue;
        };
        let mut outer = Vec::new();
        for &loop_id in &face.loops {
            if let Some(vertices) = host_outer_loop(store, face_id, loop_id)? {
                outer.push((loop_id, vertices));
            }
        }
        let [(outer_loop, vertices)] = outer.as_slice() else {
            feature_faces.push(face_id);
            continue;
        };
        if certify_face_loop_layout(store, face_id)? != LoopContainment::Certified {
            return Ok(None);
        }
        let matching = evidence
            .base
            .facets
            .iter()
            .filter(|facet| facet.face == face_id)
            .collect::<Vec<_>>();
        let [claimed] = matching.as_slice() else {
            return Ok(None);
        };
        if claimed.outer_loop != *outer_loop || claimed.vertices != *vertices {
            return Ok(None);
        }
        host.push(HostFacet {
            face: face_id,
            vertices: vertices.clone(),
            outward: plane.frame().z() * sense_factor(face.sense),
            origin: plane.frame().origin(),
        });
        for &loop_id in &face.loops {
            if loop_id != *outer_loop {
                let Some(side) = portal_side(store, face_id, loop_id)? else {
                    return Ok(None);
                };
                portals.push(Portal {
                    face: face_id,
                    loop_id,
                    side,
                });
            }
        }
    }
    if host.len() != evidence.base.facets.len() || portals.is_empty() {
        return Ok(None);
    }
    let host_faces = host.iter().map(|facet| facet.face).collect::<Vec<_>>();
    let mut claimed_features = Vec::new();
    let mut portal_features = Vec::with_capacity(portals.len());
    for portal in &portals {
        let mut local = Vec::new();
        for &(fin_id, _) in &portal.side.fins {
            let Some(peer) = peer_face_for_fin(store, fin_id)? else {
                return Ok(None);
            };
            if peer == portal.face || host_faces.contains(&peer) {
                return Ok(None);
            }
            if !local.contains(&peer) {
                local.push(peer);
            }
        }
        if local.len() != 3 || local.iter().any(|face| claimed_features.contains(face)) {
            return Ok(None);
        }
        let matching = evidence
            .features
            .iter()
            .filter(|feature| {
                feature.portal_face == portal.face && feature.portal_loop == portal.loop_id
            })
            .collect::<Vec<_>>();
        let [claimed] = matching.as_slice() else {
            return Ok(None);
        };
        if claimed.feature_faces != local {
            return Ok(None);
        }
        claimed_features.extend(local.iter().copied());
        portal_features.push((clone_portal(portal), local));
    }
    if portals.len() != evidence.features.len()
        || feature_faces.len() != claimed_features.len()
        || feature_faces
            .iter()
            .any(|face| !claimed_features.contains(face))
    {
        return Ok(None);
    }
    Ok(Some((host, portal_features)))
}

fn clone_portal(portal: &Portal) -> Portal {
    Portal {
        face: portal.face,
        loop_id: portal.loop_id,
        side: Side {
            face: portal.side.face,
            fins: portal.side.fins.clone(),
        },
    }
}

fn host_outer_loop(
    store: &Store,
    face_id: FaceId,
    loop_id: LoopId,
) -> Result<Option<Vec<VertexId>>> {
    let loop_ = store.get(loop_id)?;
    if loop_.face != face_id
        || loop_.fins.len() < 3
        || certify_loop_simplicity(store, loop_id)? != LoopSimplicity::Certified
    {
        return Ok(None);
    }
    let mut vertices = Vec::with_capacity(loop_.fins.len());
    for &fin_id in &loop_.fins {
        if certify_whole_fin_incidence(store, face_id, loop_id, fin_id, LINEAR_RESOLUTION)
            != WholeFinIncidence::Certified
        {
            return Ok(None);
        }
        let fin = store.get(fin_id)?;
        let edge = store.get(fin.edge)?;
        let (Some(curve), Some(_), Some(tail)) = (edge.curve, edge.bounds, store.fin_tail(fin_id)?)
        else {
            return Ok(None);
        };
        if edge.tolerance.is_some()
            || exact_line_carrier(store.get(curve)?).is_none()
            || !peer_surface_is_planar(store, fin_id)?
            || vertices.contains(&tail)
        {
            return Ok(None);
        }
        vertices.push(tail);
    }
    Ok(Some(vertices))
}

fn peer_surface_is_planar(store: &Store, fin_id: FinId) -> Result<bool> {
    let Some(peer) = peer_face_for_fin(store, fin_id)? else {
        return Ok(false);
    };
    Ok(matches!(
        store.get(store.get(peer)?.surface)?,
        SurfaceGeom::Plane(_)
    ))
}

fn peer_face_for_fin(store: &Store, fin_id: FinId) -> Result<Option<FaceId>> {
    let fin = store.get(fin_id)?;
    let edge = store.get(fin.edge)?;
    let [first, second] = edge.fins.as_slice() else {
        return Ok(None);
    };
    let peer = if *first == fin_id {
        *second
    } else if *second == fin_id {
        *first
    } else {
        return Ok(None);
    };
    if store.get(peer)?.sense == fin.sense {
        return Ok(None);
    }
    Ok(Some(store.get(store.get(peer)?.parent)?.face))
}

fn portal_side(store: &Store, face: FaceId, loop_id: LoopId) -> Result<Option<Side>> {
    let loop_ = store.get(loop_id)?;
    if loop_.face != face
        || loop_.fins.len() != 4
        || certify_loop_simplicity(store, loop_id)? != LoopSimplicity::Certified
    {
        return Ok(None);
    }
    let mut fins = Vec::with_capacity(4);
    for &fin_id in &loop_.fins {
        if certify_whole_fin_incidence(store, face, loop_id, fin_id, LINEAR_RESOLUTION)
            != WholeFinIncidence::Certified
        {
            return Ok(None);
        }
        let fin = store.get(fin_id)?;
        let edge = store.get(fin.edge)?;
        let Some(curve) = edge.curve else {
            return Ok(None);
        };
        if edge.tolerance.is_some()
            || edge.bounds.is_none()
            || exact_line_carrier(store.get(curve)?).is_none()
        {
            return Ok(None);
        }
        fins.push((fin_id, fin.edge));
    }
    Ok(Some(Side { face, fins }))
}

fn prepare_feature(
    store: &Store,
    portal: &Portal,
    feature_faces: &[FaceId],
) -> Result<Option<Feature>> {
    let mut caps = Vec::new();
    let mut cylinder = Vec::new();
    for &face in feature_faces {
        match store.get(store.get(face)?.surface)? {
            SurfaceGeom::Plane(_) => {
                let Some(cap) = prepare_cap(store, face)? else {
                    return Ok(None);
                };
                caps.push(cap);
            }
            SurfaceGeom::Cylinder(_) => cylinder.push(face),
            _ => return Ok(None),
        }
    }
    let Ok([first, second]) = <Vec<Cap> as TryInto<[Cap; 2]>>::try_into(caps) else {
        return Ok(None);
    };
    let [cylinder_face] = cylinder.as_slice() else {
        return Ok(None);
    };
    let Some(cylinder_side) = prepare_side(store, *cylinder_face)? else {
        return Ok(None);
    };
    let Some(translation) = translated_vertices(store, &first, &second)? else {
        return Ok(None);
    };
    if !certified_nonzero(translation.vector)
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
    let mut sweep_signs = Vec::new();
    let mut matched_second = Vec::new();
    let mut used_sides = Vec::new();
    for boundary in &first.uses {
        let Some(side_face) = peer_face(store, *boundary)? else {
            return Ok(None);
        };
        let expected_side = match boundary.carrier {
            ProfileCarrier::Line(_) if side_face == portal.face => &portal.side,
            ProfileCarrier::Circle(_) if side_face == *cylinder_face => &cylinder_side,
            _ => return Ok(None),
        };
        let matching = second
            .uses
            .iter()
            .filter(|candidate| {
                !matched_second.contains(&candidate.edge)
                    && translated_carrier(*boundary, **candidate, translation.vector)
                    && peer_face(store, **candidate).ok().flatten() == Some(side_face)
            })
            .collect::<Vec<_>>();
        let [mapped] = matching.as_slice() else {
            return Ok(None);
        };
        let Some(mapped_tail) = mapped_vertex(&translation.vertices, boundary.tail) else {
            return Ok(None);
        };
        let Some(mapped_head) = mapped_vertex(&translation.vertices, boundary.head) else {
            return Ok(None);
        };
        let rulings = expected_side
            .fins
            .iter()
            .copied()
            .filter(|(_, edge)| *edge != boundary.edge && *edge != mapped.edge)
            .collect::<Vec<_>>();
        let [first_ruling, second_ruling] = rulings.as_slice() else {
            return Ok(None);
        };
        let connects = (ruling_connects(
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
        if !connects {
            return Ok(None);
        }
        let Some(mut sign) = certify_sweep_support(
            store,
            expected_side,
            *boundary,
            **mapped,
            translation.vector,
        )?
        else {
            return Ok(None);
        };
        if side_face == portal.face {
            sign = -sign;
        }
        sweep_signs.push(sign);
        matched_second.push(mapped.edge);
        used_sides.push(side_face);
    }
    if matched_second.len() != second.uses.len()
        || !used_sides.contains(&portal.face)
        || !used_sides.contains(cylinder_face)
    {
        return Ok(None);
    }
    let coherent_caps = first.local_orientation_valid
        && second.local_orientation_valid
        && first_sign == second_sign;
    Ok(Some(Feature {
        caps: [first, second],
        cylinder_side,
        translation: translation.vector,
        orientation: coherent_caps.then_some(first_sign),
        sweep_signs,
    }))
}

fn certify_feature_supports(
    _store: &Store,
    host: &[HostFacet],
    portal: &Portal,
    feature: &Feature,
) -> Result<Option<i8>> {
    let mut portal_side = None;
    for support in host {
        let is_portal = support.face == portal.face;
        for cap in &feature.caps {
            for use_ in &cap.uses {
                let Some(range) = carrier_affine_range(*use_, support.outward, support.origin)
                else {
                    return Ok(None);
                };
                if is_portal {
                    if matches!(use_.carrier, ProfileCarrier::Circle(_)) {
                        let Some(midpoint) =
                            carrier_midpoint_affine(*use_, support.outward, support.origin)
                        else {
                            return Ok(None);
                        };
                        let side = if range.lo() >= -LINEAR_RESOLUTION
                            && midpoint.lo() > LINEAR_RESOLUTION
                        {
                            1
                        } else if range.hi() <= LINEAR_RESOLUTION
                            && midpoint.hi() < -LINEAR_RESOLUTION
                        {
                            -1
                        } else {
                            return Ok(None);
                        };
                        if portal_side.replace(side).is_some_and(|prior| prior != side) {
                            return Ok(None);
                        }
                    } else if range.lo() < -LINEAR_RESOLUTION || range.hi() > LINEAR_RESOLUTION {
                        return Ok(None);
                    }
                } else if range.hi() >= -LINEAR_RESOLUTION {
                    return Ok(None);
                }
            }
        }
    }
    if !certified_nonzero(feature.translation)
        || feature.cylinder_side.fins.len() != 4
        || portal.side.fins.len() != 4
    {
        return Ok(None);
    }
    Ok(portal_side)
}

fn certify_patch_separation(_store: &Store, host: &[HostFacet], patches: &[Patch]) -> Result<bool> {
    for first_index in 0..patches.len() {
        for second_index in first_index + 1..patches.len() {
            let first = &patches[first_index].feature;
            let second = &patches[second_index].feature;
            let mut directions = host.iter().map(|facet| facet.outward).collect::<Vec<_>>();
            append_feature_directions(first, &mut directions);
            append_feature_directions(second, &mut directions);
            let origin = host
                .first()
                .map(|facet| facet.origin)
                .unwrap_or_else(|| first.caps[0].plane.frame().origin());
            let mut separated = false;
            for direction in directions {
                if !certified_nonzero(direction) {
                    continue;
                }
                let Some(first_range) = feature_affine_range(first, direction, origin) else {
                    return Ok(false);
                };
                let Some(second_range) = feature_affine_range(second, direction, origin) else {
                    return Ok(false);
                };
                if first_range.hi() < second_range.lo() - LINEAR_RESOLUTION
                    || second_range.hi() < first_range.lo() - LINEAR_RESOLUTION
                {
                    separated = true;
                    break;
                }
            }
            if !separated {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

fn append_feature_directions(feature: &Feature, directions: &mut Vec<Vec3>) {
    directions.push(feature.caps[0].plane.frame().z());
    for cap in &feature.caps {
        for use_ in &cap.uses {
            match use_.carrier {
                ProfileCarrier::Line(line) => directions.push(line.dir()),
                ProfileCarrier::Circle(circle) => {
                    directions.push(circle.frame().x());
                    directions.push(circle.frame().y());
                }
            }
        }
    }
}

fn feature_affine_range(feature: &Feature, normal: Vec3, origin: Point3) -> Option<Interval> {
    let mut result: Option<Interval> = None;
    for cap in &feature.caps {
        for use_ in &cap.uses {
            let range = carrier_affine_range(*use_, normal, origin)?;
            result = Some(match result {
                Some(prior) => {
                    Interval::new(prior.lo().min(range.lo()), prior.hi().max(range.hi()))
                }
                None => range,
            });
        }
    }
    result
}

fn carrier_affine_range(use_: CapUse, normal: Vec3, origin: Point3) -> Option<Interval> {
    match use_.carrier {
        ProfileCarrier::Line(line) => {
            let first = affine_interval(normal, line.eval(use_.range.lo), origin);
            let second = affine_interval(normal, line.eval(use_.range.hi), origin);
            Some(Interval::new(
                first.lo().min(second.lo()),
                first.hi().max(second.hi()),
            ))
        }
        ProfileCarrier::Circle(circle) => {
            circle_affine_range(circle, use_.range.lo, use_.range.hi, normal, origin)
        }
    }
}

fn carrier_midpoint_affine(use_: CapUse, normal: Vec3, origin: Point3) -> Option<Interval> {
    let midpoint = 0.5 * (use_.range.lo + use_.range.hi);
    if !midpoint.is_finite() {
        return None;
    }
    let point = match use_.carrier {
        ProfileCarrier::Line(line) => line.eval(midpoint),
        ProfileCarrier::Circle(circle) => circle.eval(midpoint),
    };
    Some(affine_interval(normal, point, origin))
}

fn affine_interval(normal: Vec3, point: Point3, origin: Point3) -> Interval {
    let offset = point - origin;
    Interval::point(normal.x) * Interval::point(offset.x)
        + Interval::point(normal.y) * Interval::point(offset.y)
        + Interval::point(normal.z) * Interval::point(offset.z)
}
