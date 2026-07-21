//! Shell theorem for convex planar hosts with translated chord portals.
//!
//! The recognizer reconstructs the convex host from the outer loops of its
//! planar facets. Each inner four-line loop must be paired, through peer-fin
//! incidence, with its own three-face translated profile patch: two caps
//! bounded by a chord and circular arc, and one cylindrical sweep side. The
//! complete patch lies strictly inside every non-portal host support and its
//! arc lies wholly on one side of its portal support. Distinct patches must
//! also have a certified affine separating direction. Coherent outward patch
//! orientation on the outside certifies an attachment; coherent inward patch
//! orientation on the inside certifies an open pocket. No Boolean-operation
//! tag, face ordering, coordinate axis, or numeric sample chooses between
//! those consequences.

use super::convex_cylindrical_shell_proof::circle_affine_range;
use super::mixed_profile_prism_proof::{
    Cap, CapUse, ProfileCarrier, Side, certified_nonzero, certified_parallel,
    certify_sweep_support, mapped_vertex, oriented_dot_sign, peer_face, prepare_cap, prepare_side,
    ruling_connects, translated_carrier, translated_vertices,
};
use super::*;
use crate::entity::FinId;

/// Cumulative deterministic work for chord-portal shell proofs.
pub(crate) const CHORD_PORTAL_SHELL_WORK: StageId =
    match StageId::new("ktopo.check.chord-portal-shell-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid chord-portal shell work stage"),
    };

const DEFAULT_CHORD_PORTAL_SHELL_WORK: u64 = 1_048_576;

pub(super) fn chord_portal_shell_proof_budget() -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(
        CHORD_PORTAL_SHELL_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        DEFAULT_CHORD_PORTAL_SHELL_WORK,
    )])
    .expect("built-in chord-portal proof budget is valid")
}

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

/// Attempt the representation theorem described in the module contract.
pub(super) fn certify_chord_portal_shell(
    store: &Store,
    shell_id: ShellId,
    scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellCertification>> {
    let shell = store.get(shell_id)?;
    if !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(None);
    }
    let mut has_cylinder = false;
    let mut has_planar_portal_candidate = false;
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        match store.get(face.surface)? {
            SurfaceGeom::Cylinder(_) => has_cylinder = true,
            SurfaceGeom::Plane(_) if face.loops.len() >= 2 => {
                has_planar_portal_candidate = true;
            }
            _ => {}
        }
    }
    if !has_cylinder || !has_planar_portal_candidate {
        return Ok(None);
    }
    if let Some(scope) = scope {
        scope.ledger().require_limit(
            CHORD_PORTAL_SHELL_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )?;
        let Some(work) = proof_work(store, shell_id)? else {
            return Ok(Some(indeterminate()));
        };
        scope.ledger_mut().charge(CHORD_PORTAL_SHELL_WORK, work)?;
    }

    let Some((host, portal_features)) = classify_host(store, shell_id)? else {
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
    let orientation = if coherent {
        host_certification.orientation
    } else {
        ShellOrientation::Invalid
    };
    Ok(Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation,
    }))
}

/// `N² + 32N` owns every structural scan, host-support/feature-edge pair,
/// translated-use match, and stable deduplication performed by this theorem.
fn proof_work(store: &Store, shell_id: ShellId) -> Result<Option<u64>> {
    let shell = store.get(shell_id)?;
    let mut loops = 0_u64;
    let mut fins = 0_u64;
    let mut edges = Vec::new();
    let mut vertices = Vec::new();
    for &face in &shell.faces {
        for &loop_id in &store.get(face)?.loops {
            loops = match loops.checked_add(1) {
                Some(value) => value,
                None => return Ok(None),
            };
            for &fin_id in &store.get(loop_id)?.fins {
                fins = match fins.checked_add(1) {
                    Some(value) => value,
                    None => return Ok(None),
                };
                let edge = store.get(fin_id)?.edge;
                if !edges.contains(&edge) {
                    edges.push(edge);
                    for vertex in store.get(edge)?.vertices.into_iter().flatten() {
                        if !vertices.contains(&vertex) {
                            vertices.push(vertex);
                        }
                    }
                }
            }
        }
    }
    let (Some(faces), Some(edges), Some(vertices)) = (
        u64::try_from(shell.faces.len()).ok(),
        u64::try_from(edges.len()).ok(),
        u64::try_from(vertices.len()).ok(),
    ) else {
        return Ok(None);
    };
    let Some(size) = 1_u64
        .checked_add(faces)
        .and_then(|value| value.checked_add(loops))
        .and_then(|value| value.checked_add(fins))
        .and_then(|value| value.checked_add(edges))
        .and_then(|value| value.checked_add(vertices))
    else {
        return Ok(None);
    };
    Ok(size
        .checked_mul(size)
        .and_then(|value| value.checked_add(size.checked_mul(32)?)))
}

fn classify_host(
    store: &Store,
    shell_id: ShellId,
) -> Result<Option<(Vec<HostFacet>, Vec<(Portal, Vec<FaceId>)>)>> {
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
        let outward = plane.frame().z() * sense_factor(face.sense);
        host.push(HostFacet {
            face: face_id,
            vertices: vertices.clone(),
            outward,
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
    if portals.is_empty() {
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
        claimed_features.extend(local.iter().copied());
        portal_features.push((clone_portal(portal), local));
    }
    if feature_faces.len() != claimed_features.len()
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
        if edge.tolerance.is_some()
            || edge.bounds.is_none()
            || edge.curve.is_none()
            || exact_line_carrier(store.get(edge.curve.unwrap())?).is_none()
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
            // An inner loop's effective closure normal opposes its owning
            // host face.  This is topology, not an operation-specific flip.
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
    store: &Store,
    host: &[HostFacet],
    portal: &Portal,
    feature: &Feature,
) -> Result<Option<i8>> {
    let mut portal_side = None;
    for support in host {
        let is_portal = support.face == portal.face;
        for cap in &feature.caps {
            for use_ in &cap.uses {
                let Some(range) =
                    carrier_affine_range(store, *use_, support.outward, support.origin)?
                else {
                    return Ok(None);
                };
                if is_portal {
                    if matches!(use_.carrier, ProfileCarrier::Circle(_)) {
                        let midpoint =
                            carrier_midpoint_affine(*use_, support.outward, support.origin);
                        let Some(midpoint) = midpoint else {
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
    // Retain reads of the full sweep witnesses in this support proof: all
    // ruling values are affine combinations of their cap endpoints.
    if !certified_nonzero(feature.translation)
        || feature.cylinder_side.fins.len() != 4
        || portal.side.fins.len() != 4
    {
        return Ok(None);
    }
    Ok(portal_side)
}

/// Prove every pair of complete swept patches disjoint by finding an exact
/// affine projection gap. Boundary carriers own all extrema of each compact
/// profile domain, and the translation rulings are their cap-endpoint hulls.
fn certify_patch_separation(store: &Store, host: &[HostFacet], patches: &[Patch]) -> Result<bool> {
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
                let Some(first_range) = feature_affine_range(store, first, direction, origin)?
                else {
                    return Ok(false);
                };
                let Some(second_range) = feature_affine_range(store, second, direction, origin)?
                else {
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

fn feature_affine_range(
    store: &Store,
    feature: &Feature,
    normal: Vec3,
    origin: Point3,
) -> Result<Option<Interval>> {
    let mut result: Option<Interval> = None;
    for cap in &feature.caps {
        for use_ in &cap.uses {
            let Some(range) = carrier_affine_range(store, *use_, normal, origin)? else {
                return Ok(None);
            };
            result = Some(match result {
                Some(prior) => {
                    Interval::new(prior.lo().min(range.lo()), prior.hi().max(range.hi()))
                }
                None => range,
            });
        }
    }
    Ok(result)
}

fn carrier_affine_range(
    _store: &Store,
    use_: CapUse,
    normal: Vec3,
    origin: Point3,
) -> Result<Option<Interval>> {
    Ok(match use_.carrier {
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
    })
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

fn indeterminate() -> ShellCertification {
    ShellCertification {
        embedding: ShellEmbedding::Indeterminate,
        orientation: ShellOrientation::Indeterminate,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analytic_shell::{
        AnalyticEdgeKey, AnalyticFaceKey, AnalyticPcurveUse, AnalyticShellCurve, AnalyticShellEdge,
        AnalyticShellFace, AnalyticShellFin, AnalyticShellInput, AnalyticShellLoop,
        AnalyticShellPcurve, AnalyticShellSurface, AnalyticShellVertex, AnalyticVertexKey,
    };
    use crate::check::{CheckLevel, CheckOutcome, check_body_report};
    use crate::entity::FaceDomain;
    use crate::transaction::FullCommitRequirement;
    use kgeom::curve::{Circle, Curve, Line};
    use kgeom::curve2d::{Circle2d, Line2d};
    use kgeom::param::ParamRange;
    use kgeom::surface::Plane;
    use kgeom::vec::Vec2;
    use kgraph::AffineParamMap1d;

    fn parameter_map(scale: f64) -> AffineParamMap1d {
        AffineParamMap1d::new(scale, 0.0).unwrap()
    }

    fn plane_line_use(edge: u64, sense: Sense, plane: Plane, line: Line) -> AnalyticShellFin {
        let origin = plane.frame().to_local(line.origin());
        let direction = line.dir();
        AnalyticShellFin::new(
            AnalyticEdgeKey::new(edge),
            sense,
            AnalyticPcurveUse::new(
                AnalyticShellPcurve::Line(
                    Line2d::new(
                        Point2::new(origin.x, origin.y),
                        Vec2::new(
                            direction.dot(plane.frame().x()),
                            direction.dot(plane.frame().y()),
                        ),
                    )
                    .unwrap(),
                ),
                parameter_map(1.0),
            ),
        )
    }

    fn plane_circle_use(edge: u64, sense: Sense, plane: Plane, circle: Circle) -> AnalyticShellFin {
        let center = plane.frame().to_local(circle.frame().origin());
        let local_x = Vec2::new(
            circle.frame().x().dot(plane.frame().x()),
            circle.frame().x().dot(plane.frame().y()),
        );
        let local_y = Vec2::new(
            circle.frame().y().dot(plane.frame().x()),
            circle.frame().y().dot(plane.frame().y()),
        );
        let scale = if local_x.perp().dot(local_y) > 0.0 {
            1.0
        } else {
            -1.0
        };
        AnalyticShellFin::new(
            AnalyticEdgeKey::new(edge),
            sense,
            AnalyticPcurveUse::new(
                AnalyticShellPcurve::Circle(
                    Circle2d::new(Point2::new(center.x, center.y), circle.radius(), local_x)
                        .unwrap(),
                ),
                parameter_map(scale),
            ),
        )
    }

    fn cylinder_ruling_use(edge: u64, sense: Sense, longitude: f64) -> AnalyticShellFin {
        AnalyticShellFin::new(
            AnalyticEdgeKey::new(edge),
            sense,
            AnalyticPcurveUse::new(
                AnalyticShellPcurve::Line(
                    Line2d::new(Point2::new(longitude, 0.0), Vec2::new(0.0, 1.0)).unwrap(),
                ),
                parameter_map(1.0),
            ),
        )
    }

    fn cylinder_arc_use(edge: u64, sense: Sense, height: f64) -> AnalyticShellFin {
        AnalyticShellFin::new(
            AnalyticEdgeKey::new(edge),
            sense,
            AnalyticPcurveUse::new(
                AnalyticShellPcurve::Line(
                    Line2d::new(Point2::new(0.0, height), Vec2::new(1.0, 0.0)).unwrap(),
                ),
                parameter_map(1.0),
            ),
        )
    }

    fn reverse_loop(loop_: AnalyticShellLoop) -> AnalyticShellLoop {
        AnalyticShellLoop::new(
            loop_
                .fins()
                .iter()
                .rev()
                .map(|fin| AnalyticShellFin::new(fin.edge(), fin.sense().flipped(), fin.pcurve()))
                .collect(),
        )
    }

    fn line_edge(
        key: u64,
        vertices: [u64; 2],
        start: Point3,
        end: Point3,
    ) -> (AnalyticShellEdge, Line) {
        let displacement = end - start;
        let line = Line::new(start, displacement).unwrap();
        (
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(key),
                vertices.map(AnalyticVertexKey::new),
                AnalyticShellCurve::Line(line),
                ParamRange::new(0.0, displacement.norm()),
            ),
            line,
        )
    }

    /// Fixture B/C cap crossing without retaining Boolean provenance. `pocket`
    /// chooses the reversed minor segment used by B-C; otherwise the major
    /// exterior segment is the attachment used by B union C.
    fn cap_crossing_input(pocket: bool) -> AnalyticShellInput {
        let x_low = 0.5;
        let x_high = 2.5;
        let y_low = -3.0;
        let y_high = 3.0;
        let z_low = -1.0;
        let z_high = 3.0;
        let host_points = [
            Point3::new(x_low, y_low, z_low),
            Point3::new(x_high, y_low, z_low),
            Point3::new(x_high, y_high, z_low),
            Point3::new(x_low, y_high, z_low),
            Point3::new(x_low, y_low, z_high),
            Point3::new(x_high, y_low, z_high),
            Point3::new(x_high, y_high, z_high),
            Point3::new(x_low, y_high, z_high),
        ];
        let cylinder_frame = Frame::world();
        let radius = 1.5;
        let cylinder = Cylinder::new(cylinder_frame, radius).unwrap();
        let bottom_circle = Circle::new(cylinder_frame, radius).unwrap();
        let top_frame = cylinder_frame.with_origin(Point3::new(0.0, 0.0, 2.0));
        let top_circle = Circle::new(top_frame, radius).unwrap();
        let alpha = (2.0_f64.sqrt()).atan2(0.5);
        let arc = if pocket {
            ParamRange::new(-alpha, alpha)
        } else {
            ParamRange::new(alpha, 2.0 * core::f64::consts::PI - alpha)
        };
        let feature_points = [
            bottom_circle.eval(arc.lo),
            bottom_circle.eval(arc.hi),
            top_circle.eval(arc.lo),
            top_circle.eval(arc.hi),
        ];
        let vertices = host_points
            .into_iter()
            .chain(feature_points)
            .enumerate()
            .map(|(index, position)| {
                AnalyticShellVertex::new(AnalyticVertexKey::new(index as u64), position)
            })
            .collect::<Vec<_>>();

        let host_edge_vertices = [
            [0, 1],
            [1, 2],
            [2, 3],
            [3, 0],
            [4, 5],
            [5, 6],
            [6, 7],
            [7, 4],
            [0, 4],
            [1, 5],
            [2, 6],
            [3, 7],
        ];
        let mut edges = Vec::new();
        let mut lines = Vec::new();
        for (index, endpoints) in host_edge_vertices.into_iter().enumerate() {
            let (edge, line) = line_edge(
                index as u64,
                endpoints,
                host_points[endpoints[0] as usize],
                host_points[endpoints[1] as usize],
            );
            edges.push(edge);
            lines.push(line);
        }
        edges.push(AnalyticShellEdge::new(
            AnalyticEdgeKey::new(12),
            [AnalyticVertexKey::new(8), AnalyticVertexKey::new(9)],
            AnalyticShellCurve::Circle(bottom_circle),
            arc,
        ));
        edges.push(AnalyticShellEdge::new(
            AnalyticEdgeKey::new(13),
            [AnalyticVertexKey::new(10), AnalyticVertexKey::new(11)],
            AnalyticShellCurve::Circle(top_circle),
            arc,
        ));
        let (ruling_first, line_14) = line_edge(14, [8, 10], feature_points[0], feature_points[2]);
        let (ruling_second, line_15) = line_edge(15, [9, 11], feature_points[1], feature_points[3]);
        let (bottom_chord, line_16) = line_edge(16, [8, 9], feature_points[0], feature_points[1]);
        let (top_chord, line_17) = line_edge(17, [10, 11], feature_points[2], feature_points[3]);
        edges.extend([ruling_first, ruling_second, bottom_chord, top_chord]);

        let bottom_host = Plane::new(
            Frame::new(
                host_points[0],
                Vec3::new(0.0, 0.0, -1.0),
                Vec3::new(0.0, 1.0, 0.0),
            )
            .unwrap(),
        );
        let top_host = Plane::new(
            Frame::new(
                host_points[4],
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
        );
        let x_low_plane = Plane::new(
            Frame::new(
                host_points[0],
                Vec3::new(-1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            )
            .unwrap(),
        );
        let x_high_plane = Plane::new(
            Frame::new(
                host_points[1],
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            )
            .unwrap(),
        );
        let y_low_plane = Plane::new(
            Frame::new(
                host_points[0],
                Vec3::new(0.0, -1.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
        );
        let y_high_plane = Plane::new(
            Frame::new(
                host_points[3],
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            )
            .unwrap(),
        );
        let bottom_cap = Plane::new(
            Frame::new(
                feature_points[0],
                Vec3::new(0.0, 0.0, -1.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
        );
        let top_cap = Plane::new(top_frame);

        let host_loop = |plane: Plane, uses: &[(u64, Sense)]| {
            AnalyticShellLoop::new(
                uses.iter()
                    .map(|&(edge, sense)| plane_line_use(edge, sense, plane, lines[edge as usize]))
                    .collect(),
            )
        };
        let bottom_outer = host_loop(
            bottom_host,
            &[
                (3, Sense::Reversed),
                (2, Sense::Reversed),
                (1, Sense::Reversed),
                (0, Sense::Reversed),
            ],
        );
        let top_outer = host_loop(
            top_host,
            &[
                (4, Sense::Forward),
                (5, Sense::Forward),
                (6, Sense::Forward),
                (7, Sense::Forward),
            ],
        );
        let x_low_outer = host_loop(
            x_low_plane,
            &[
                (8, Sense::Forward),
                (7, Sense::Reversed),
                (11, Sense::Reversed),
                (3, Sense::Forward),
            ],
        );
        let x_high_outer = host_loop(
            x_high_plane,
            &[
                (1, Sense::Forward),
                (10, Sense::Forward),
                (5, Sense::Reversed),
                (9, Sense::Reversed),
            ],
        );
        let y_low_outer = host_loop(
            y_low_plane,
            &[
                (0, Sense::Forward),
                (9, Sense::Forward),
                (4, Sense::Reversed),
                (8, Sense::Reversed),
            ],
        );
        let y_high_outer = host_loop(
            y_high_plane,
            &[
                (11, Sense::Forward),
                (6, Sense::Reversed),
                (10, Sense::Reversed),
                (2, Sense::Forward),
            ],
        );

        let cylinder_loop = AnalyticShellLoop::new(vec![
            cylinder_ruling_use(14, Sense::Reversed, arc.lo),
            cylinder_arc_use(12, Sense::Forward, 0.0),
            cylinder_ruling_use(15, Sense::Forward, arc.hi),
            cylinder_arc_use(13, Sense::Reversed, 2.0),
        ]);
        let bottom_cap_loop = AnalyticShellLoop::new(vec![
            plane_circle_use(12, Sense::Reversed, bottom_cap, bottom_circle),
            plane_line_use(16, Sense::Forward, bottom_cap, line_16),
        ]);
        let top_cap_loop = AnalyticShellLoop::new(vec![
            plane_circle_use(13, Sense::Forward, top_cap, top_circle),
            plane_line_use(17, Sense::Reversed, top_cap, line_17),
        ]);
        let portal_loop = AnalyticShellLoop::new(vec![
            plane_line_use(16, Sense::Reversed, x_low_plane, line_16),
            plane_line_use(14, Sense::Forward, x_low_plane, line_14),
            plane_line_use(17, Sense::Forward, x_low_plane, line_17),
            plane_line_use(15, Sense::Reversed, x_low_plane, line_15),
        ]);
        let (cylinder_loop, bottom_cap_loop, top_cap_loop, portal_loop, patch_sense) = if pocket {
            (
                reverse_loop(cylinder_loop),
                reverse_loop(bottom_cap_loop),
                reverse_loop(top_cap_loop),
                reverse_loop(portal_loop),
                Sense::Reversed,
            )
        } else {
            (
                cylinder_loop,
                bottom_cap_loop,
                top_cap_loop,
                portal_loop,
                Sense::Forward,
            )
        };
        let wide = || FaceDomain::from_bounds(-10.0, 10.0, -10.0, 10.0).unwrap();
        let faces = vec![
            AnalyticShellFace::new(
                AnalyticFaceKey::new(0),
                AnalyticShellSurface::Plane(bottom_host),
                Sense::Forward,
                wide(),
                vec![bottom_outer],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(1),
                AnalyticShellSurface::Plane(top_host),
                Sense::Forward,
                wide(),
                vec![top_outer],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(2),
                AnalyticShellSurface::Plane(x_low_plane),
                Sense::Forward,
                wide(),
                vec![x_low_outer, portal_loop],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(3),
                AnalyticShellSurface::Plane(x_high_plane),
                Sense::Forward,
                wide(),
                vec![x_high_outer],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(4),
                AnalyticShellSurface::Plane(y_low_plane),
                Sense::Forward,
                wide(),
                vec![y_low_outer],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(5),
                AnalyticShellSurface::Plane(y_high_plane),
                Sense::Forward,
                wide(),
                vec![y_high_outer],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(6),
                AnalyticShellSurface::Cylinder(cylinder),
                patch_sense,
                FaceDomain::from_bounds(arc.lo, arc.hi, 0.0, 2.0).unwrap(),
                vec![cylinder_loop],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(7),
                AnalyticShellSurface::Plane(bottom_cap),
                patch_sense,
                wide(),
                vec![bottom_cap_loop],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(8),
                AnalyticShellSurface::Plane(top_cap),
                patch_sense,
                wide(),
                vec![top_cap_loop],
            ),
        ];
        AnalyticShellInput::new(vertices, edges, faces)
    }

    fn session_with_work(allowed: u64) -> kcore::operation::SessionPolicy {
        let budget = BudgetPlan::new([LimitSpec::new(
            CHORD_PORTAL_SHELL_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            allowed,
        )])
        .unwrap();
        kcore::operation::SessionPolicy::new(
            kcore::operation::SessionPrecision::parasolid(),
            kcore::operation::NumericalPolicy::v1(),
            kcore::operation::ExecutionPolicy::Serial,
            budget,
            kcore::operation::PolicyVersion::V1,
        )
    }

    #[test]
    fn cap_crossing_attachment_and_pocket_are_full_certified() {
        for pocket in [false, true] {
            let mut store = Store::new();
            let mut transaction = store.transaction().unwrap();
            let output = transaction
                .assemble_analytic_shell(&cap_crossing_input(pocket), 1.0e-12)
                .unwrap();
            assert_eq!(
                certify_chord_portal_shell(transaction.store(), output.shell(), None).unwrap(),
                Some(ShellCertification {
                    embedding: ShellEmbedding::Certified,
                    orientation: ShellOrientation::Positive,
                }),
                "pocket={pocket}"
            );
            let report =
                check_body_report(transaction.store(), output.body(), CheckLevel::Full).unwrap();
            assert_eq!(
                report.outcome(),
                CheckOutcome::Valid,
                "pocket={pocket}: {report:#?}"
            );
            transaction
                .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
                .unwrap();
        }
    }

    #[test]
    fn chord_portal_tampering_fails_closed() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&cap_crossing_input(false), 1.0e-12)
            .unwrap();
        let baseline = transaction.store().clone();
        let face = |key: u64| {
            output
                .faces()
                .iter()
                .find_map(|(candidate, face)| (candidate.value() == key).then_some(*face))
                .unwrap()
        };

        let mut sense = baseline.clone();
        sense.get_mut(face(6)).unwrap().sense = Sense::Reversed;
        assert_ne!(
            certify_chord_portal_shell(&sense, output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            })
        );

        let mut geometry = baseline.clone();
        let mut geometry_edit = geometry.transaction().unwrap();
        let cylinder_surface = geometry_edit.store().get(face(6)).unwrap().surface;
        let SurfaceGeom::Cylinder(cylinder) = *geometry_edit.store().get(cylinder_surface).unwrap()
        else {
            unreachable!()
        };
        geometry_edit
            .store_mut()
            .replace_surface(
                cylinder_surface,
                SurfaceGeom::Cylinder(
                    Cylinder::new(*cylinder.frame(), cylinder.radius() + 0.1).unwrap(),
                ),
            )
            .unwrap();
        assert_ne!(
            certify_chord_portal_shell(geometry_edit.store(), output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            })
        );

        let mut topology = baseline;
        let portal_loop = topology.get(face(2)).unwrap().loops[1];
        let duplicate = topology.get(portal_loop).unwrap().fins[0];
        topology.get_mut(portal_loop).unwrap().fins.push(duplicate);
        assert_ne!(
            certify_chord_portal_shell(&topology, output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            })
        );
    }

    #[test]
    fn chord_portal_work_accepts_exact_n_and_rejects_n_minus_one() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&cap_crossing_input(false), 1.0e-12)
            .unwrap();
        let required = proof_work(transaction.store(), output.shell())
            .unwrap()
            .unwrap();
        for allowed in [required, required - 1] {
            let session = session_with_work(allowed);
            let context = kcore::operation::OperationContext::new(
                &session,
                kcore::tolerance::Tolerances::default(),
            )
            .unwrap();
            let mut scope = OperationScope::new(&context);
            let result =
                certify_chord_portal_shell(transaction.store(), output.shell(), Some(&mut scope));
            if allowed == required {
                assert_eq!(
                    result.unwrap().unwrap(),
                    ShellCertification {
                        embedding: ShellEmbedding::Certified,
                        orientation: ShellOrientation::Positive,
                    }
                );
            } else {
                assert_eq!(
                    result.unwrap_err().limit().map(|limit| limit.stage),
                    Some(CHORD_PORTAL_SHELL_WORK)
                );
            }
        }
    }
}
