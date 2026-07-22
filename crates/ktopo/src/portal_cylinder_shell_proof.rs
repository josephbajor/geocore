//! Shell theorem for finite cylinders with disjoint analytic portal surgery.
//!
//! The admitted class starts with an exact finite cylinder band. Any number
//! of pairwise-disjoint rectangular portal patches may be removed from its
//! side. The remaining planar faces must have a unique decomposition into
//! complete normal-translation prisms after every portal is restored as a
//! virtual cylindrical product side. Each prism is proven wholly inside or
//! wholly outside the host cylinder, and distinct prisms require a certified
//! separating local-coordinate slab. Replacing disjoint cylinder patches by
//! those separated product boundaries preserves embedding. No constructor
//! provenance, face ordering, portal count, or Boolean-operation tag enters
//! the proof.

use super::*;
use crate::entity::FinId;
use kgeom::curve2d::Curve2d;
use kgeom::param::ParamRange;
use kgeom::vec::Vec2;

use super::mixed_profile_prism_proof::{
    Cap, CapUse, ProfileCarrier, Side, certified_nonzero, certified_parallel,
    certify_sweep_support, edge_has_vertices, mapped_vertex, oriented_dot_sign, peer_face,
    prepare_cap, prepare_side, ruling_connects, translated_carrier, translated_vertices,
};

#[path = "portal_cylinder_shell_proof/profile_radial_proof.rs"]
mod profile_radial_proof;
pub(super) use profile_radial_proof::circle_secant_span_side;
use profile_radial_proof::{profile_radial_bounds, profile_radial_side};

/// Cumulative deterministic work for portal-cylinder shell proofs.
pub(crate) const PORTAL_CYLINDER_SHELL_WORK: StageId =
    match StageId::new("ktopo.check.portal-cylinder-shell-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid portal-cylinder shell work stage"),
    };

// The first general multi-attachment public layout (five disjoint portal
// patches) charges 14_966_784 units under `proof_work`. Keep the versioned
// default at the smallest power-of-two ceiling that admits that proof while
// retaining a finite fail-closed policy and exact caller overrides.
const DEFAULT_PORTAL_CYLINDER_SHELL_WORK: u64 = 16_777_216;

pub(super) fn portal_cylinder_proof_budget() -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(
        PORTAL_CYLINDER_SHELL_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        DEFAULT_PORTAL_CYLINDER_SHELL_WORK,
    )])
    .expect("built-in portal-cylinder proof budget is valid")
}

#[derive(Debug, Clone)]
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
    cap_axis_alignment: PredicateOrientation,
    side_traverses_positive_u: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum RadialSide {
    Inside,
    Outside,
}

impl RadialSide {
    const fn orientation_factor(self) -> i8 {
        match self {
            Self::Inside => -1,
            Self::Outside => 1,
        }
    }
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

#[derive(Debug, Clone, Copy)]
struct IntervalBounds2 {
    x: Interval,
    y: Interval,
}

/// Attempt the representation-independent disjoint portal-surgery theorem.
pub(super) fn certify_portal_cylinder_shell(
    store: &Store,
    shell_id: ShellId,
    scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellCertification>> {
    let shell = store.get(shell_id)?;
    if shell.faces.len() < 6 || !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(None);
    }

    let mut hosts = Vec::new();
    let mut planar = Vec::new();
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        if face.shell != shell_id {
            return Ok(None);
        }
        match store.get(face.surface)? {
            SurfaceGeom::Cylinder(cylinder) => hosts.push((face_id, *cylinder)),
            SurfaceGeom::Plane(_) => planar.push(face_id),
            _ => return Ok(None),
        }
    }
    if hosts.is_empty() {
        return Ok(None);
    }

    if let Some(scope) = scope {
        scope.ledger().require_limit(
            PORTAL_CYLINDER_SHELL_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )?;
        let Some(work) = proof_work(store, shell_id, hosts.len(), planar.len())? else {
            return Ok(Some(indeterminate()));
        };
        scope
            .ledger_mut()
            .charge(PORTAL_CYLINDER_SHELL_WORK, work)?;
    }

    for (host_face, cylinder) in hosts {
        if let Some(certification) =
            certify_host_candidate(store, shell_id, host_face, cylinder, &planar)?
        {
            return Ok(Some(certification));
        }
    }
    Ok(None)
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
    let (low, high) =
        match exact_affine_sign(cylinder.frame().z(), second_ring.center, first_ring.center) {
            Some(PredicateOrientation::Positive) => (first_ring, second_ring),
            Some(PredicateOrientation::Negative) => (second_ring, first_ring),
            _ => return Ok(None),
        };

    let mut portals = Vec::with_capacity(portal_loops.len());
    for loop_id in portal_loops {
        let Some(portal) = prepare_portal(store, host_face, cylinder, loop_id)? else {
            return Ok(None);
        };
        portals.push(portal);
    }

    let base_orientation = cylinder_band_orientation(store, host_face, low, high);
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

fn proof_work(
    store: &Store,
    shell_id: ShellId,
    host_count: usize,
    plane_count: usize,
) -> Result<Option<u64>> {
    let shell = store.get(shell_id)?;
    let mut loops = 0_u64;
    let mut fins = 0_u64;
    let mut edges = Vec::new();
    let mut vertices = Vec::new();
    for &face_id in &shell.faces {
        for &loop_id in &store.get(face_id)?.loops {
            let Some(next_loops) = loops.checked_add(1) else {
                return Ok(None);
            };
            loops = next_loops;
            for &fin_id in &store.get(loop_id)?.fins {
                let Some(next_fins) = fins.checked_add(1) else {
                    return Ok(None);
                };
                fins = next_fins;
                let edge_id = store.get(fin_id)?.edge;
                if !edges.contains(&edge_id) {
                    edges.push(edge_id);
                    for vertex in store.get(edge_id)?.vertices.into_iter().flatten() {
                        if !vertices.contains(&vertex) {
                            vertices.push(vertex);
                        }
                    }
                }
            }
        }
    }
    let (Some(faces), Some(edges), Some(vertices), Some(hosts), Some(planes)) = (
        u64::try_from(shell.faces.len()).ok(),
        u64::try_from(edges.len()).ok(),
        u64::try_from(vertices.len()).ok(),
        u64::try_from(host_count).ok(),
        u64::try_from(plane_count).ok(),
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
    let Some(pairs) = planes
        .checked_mul(planes.saturating_sub(1))
        .map(|ordered| ordered / 2)
    else {
        return Ok(None);
    };
    Ok(size
        .checked_mul(size)
        .and_then(|quadratic| quadratic.checked_add(size.checked_mul(64)?))
        .and_then(|per_pair| per_pair.checked_mul(pairs.checked_add(1)?))
        .and_then(|per_host| per_host.checked_mul(hosts)))
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
    let cap_axis_alignment = match oriented_dot_sign(plane.frame().z(), cylinder.frame().z()) {
        Some(1) => PredicateOrientation::Positive,
        Some(-1) => PredicateOrientation::Negative,
        _ => return Ok(None),
    };
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
    let Some(side_traverses_positive_u) = traversal_is_positive(
        [host_line.dir().x, host_use.edge_to_pcurve().scale()],
        host_fin.sense,
    ) else {
        return Ok(None);
    };
    Ok(Some(OuterBoundary {
        cap_face,
        edge: host_fin.edge,
        center: circle.frame().origin(),
        cap_axis_alignment,
        side_traverses_positive_u,
    }))
}

fn cylinder_band_orientation(
    store: &Store,
    host_face: FaceId,
    low: OuterBoundary,
    high: OuterBoundary,
) -> ShellOrientation {
    let host = match store.get(host_face) {
        Ok(face) => face,
        Err(_) => return ShellOrientation::Indeterminate,
    };
    let expected_low = host.sense == Sense::Forward;
    let expected_high = host.sense == Sense::Reversed;
    if low.side_traverses_positive_u != expected_low
        || high.side_traverses_positive_u != expected_high
    {
        return ShellOrientation::Invalid;
    }
    let low_outward = store
        .get(low.cap_face)
        .ok()
        .and_then(|face| oriented_axis_alignment(low.cap_axis_alignment, face.sense));
    let high_outward = store
        .get(high.cap_face)
        .ok()
        .and_then(|face| oriented_axis_alignment(high.cap_axis_alignment, face.sense));
    match (host.sense, low_outward, high_outward) {
        (Sense::Forward, Some(-1), Some(1)) => ShellOrientation::Positive,
        (Sense::Reversed, Some(1), Some(-1)) => ShellOrientation::Negative,
        _ => ShellOrientation::Invalid,
    }
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

fn coordinate_interval(frame: &Frame, axis: Vec3, point: Point3) -> Interval {
    let offset = [
        Interval::point(point.x) - Interval::point(frame.origin().x),
        Interval::point(point.y) - Interval::point(frame.origin().y),
        Interval::point(point.z) - Interval::point(frame.origin().z),
    ];
    Interval::point(axis.x) * offset[0]
        + Interval::point(axis.y) * offset[1]
        + Interval::point(axis.z) * offset[2]
}

fn certified_point_on_plane(frame: &Frame, point: Point3) -> bool {
    let coordinate = coordinate_interval(frame, frame.z(), point);
    coordinate.lo().is_finite()
        && coordinate.lo() >= -LINEAR_RESOLUTION
        && coordinate.hi() <= LINEAR_RESOLUTION
}

#[cfg(test)]
#[path = "portal_cylinder_shell_proof/analytic_boss_tests.rs"]
mod analytic_boss_tests;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analytic_shell::{
        AnalyticEdgeKey, AnalyticFaceKey, AnalyticPcurveUse, AnalyticShellClosedEdge,
        AnalyticShellCurve, AnalyticShellEdge, AnalyticShellFace, AnalyticShellFin,
        AnalyticShellInput, AnalyticShellLoop, AnalyticShellPcurve, AnalyticShellSurface,
        AnalyticShellVertex, AnalyticVertexKey,
    };
    use crate::check::{CheckLevel, CheckOutcome, check_body_report};
    use crate::entity::FaceDomain;
    use crate::transaction::FullCommitRequirement;
    use kgeom::curve::{Circle, Line};
    use kgeom::curve2d::{Circle2d, Line2d};
    use kgeom::surface::Plane;
    use kgraph::AffineParamMap1d;

    fn map(scale: f64) -> AffineParamMap1d {
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
                map(1.0),
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
                map(scale),
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
                map(1.0),
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
                map(1.0),
            ),
        )
    }

    fn ring_cylinder_use(edge: u64, sense: Sense, height: f64) -> AnalyticShellFin {
        AnalyticShellFin::new(
            AnalyticEdgeKey::new(edge),
            sense,
            AnalyticPcurveUse::new(
                AnalyticShellPcurve::Line(
                    Line2d::new(Point2::new(0.0, height), Vec2::new(1.0, 0.0)).unwrap(),
                ),
                map(1.0),
            )
            .with_closure_winding([1, 0]),
        )
    }

    fn ring_plane_use(edge: u64, sense: Sense, plane: Plane, circle: Circle) -> AnalyticShellFin {
        let use_ = plane_circle_use(edge, sense, plane, circle);
        AnalyticShellFin::new(
            use_.edge(),
            use_.sense(),
            use_.pcurve().with_closure_winding([0, 0]),
        )
    }

    fn portal_shell_input() -> AnalyticShellInput {
        let radius: f64 = 1.5;
        let low = 0.0;
        let cut_low = 0.5;
        let cut_high = 1.5;
        let high = 2.0;
        let angle = kcore::math::atan2((radius * radius - 1.0).sqrt(), 1.0);
        let opposite = core::f64::consts::PI - angle;
        let lower_left = core::f64::consts::PI + angle;
        let lower_right = core::f64::consts::TAU - angle;
        let frame = Frame::world();
        let cylinder = Cylinder::new(frame, radius).unwrap();
        let circle_at = |height| {
            Circle::new(frame.with_origin(frame.point_at(0.0, 0.0, height)), radius).unwrap()
        };
        let cut_low_circle = circle_at(cut_low);
        let cut_high_circle = circle_at(cut_high);
        let points = [
            cut_low_circle.eval(angle),
            cut_low_circle.eval(opposite),
            cut_low_circle.eval(lower_left),
            cut_low_circle.eval(lower_right),
            cut_high_circle.eval(angle),
            cut_high_circle.eval(opposite),
            cut_high_circle.eval(lower_left),
            cut_high_circle.eval(lower_right),
        ];
        let vertices = points
            .iter()
            .enumerate()
            .map(|(index, point)| {
                AnalyticShellVertex::new(AnalyticVertexKey::new(index as u64), *point)
            })
            .collect::<Vec<_>>();

        let mut edges = Vec::new();
        for (key, vertices, circle, range) in [
            (0, [0, 1], cut_low_circle, ParamRange::new(angle, opposite)),
            (
                1,
                [2, 3],
                cut_low_circle,
                ParamRange::new(lower_left, lower_right),
            ),
            (2, [4, 5], cut_high_circle, ParamRange::new(angle, opposite)),
            (
                3,
                [6, 7],
                cut_high_circle,
                ParamRange::new(lower_left, lower_right),
            ),
        ] {
            edges.push(AnalyticShellEdge::new(
                AnalyticEdgeKey::new(key),
                vertices.map(AnalyticVertexKey::new),
                AnalyticShellCurve::Circle(circle),
                range,
            ));
        }
        for (key, vertices) in [
            (4, [0, 4]),
            (5, [1, 5]),
            (6, [2, 6]),
            (7, [3, 7]),
            (8, [1, 2]),
            (9, [3, 0]),
            (10, [5, 6]),
            (11, [7, 4]),
        ] {
            let start = points[vertices[0] as usize];
            let end = points[vertices[1] as usize];
            let direction = end - start;
            let length = direction.norm();
            let (line, range) = if key < 8 {
                (
                    Line::new(start - frame.z() * cut_low, frame.z()).unwrap(),
                    ParamRange::new(cut_low, cut_high),
                )
            } else {
                (
                    Line::new(start, direction).unwrap(),
                    ParamRange::new(0.0, length),
                )
            };
            edges.push(AnalyticShellEdge::new(
                AnalyticEdgeKey::new(key),
                vertices.map(AnalyticVertexKey::new),
                AnalyticShellCurve::Line(line),
                range,
            ));
        }

        let low_plane =
            Plane::new(Frame::new(frame.point_at(0.0, 0.0, low), -frame.z(), frame.x()).unwrap());
        let high_plane = Plane::new(frame.with_origin(frame.point_at(0.0, 0.0, high)));
        let cut_low_plane = Plane::new(frame.with_origin(frame.point_at(0.0, 0.0, cut_low)));
        let cut_high_plane = Plane::new(
            Frame::new(frame.point_at(0.0, 0.0, cut_high), -frame.z(), frame.x()).unwrap(),
        );
        let right_plane = Plane::new(Frame::new(points[0], -frame.x(), frame.y()).unwrap());
        let left_plane = Plane::new(Frame::new(points[1], frame.x(), -frame.y()).unwrap());
        let line = |edge: usize| match edges[edge].carrier() {
            AnalyticShellCurve::Line(line) => line,
            _ => unreachable!(),
        };

        let host_loops = vec![
            AnalyticShellLoop::new(vec![
                cylinder_arc_use(0, Sense::Reversed, cut_low),
                cylinder_ruling_use(4, Sense::Forward, angle),
                cylinder_arc_use(2, Sense::Forward, cut_high),
                cylinder_ruling_use(5, Sense::Reversed, opposite),
            ]),
            AnalyticShellLoop::new(vec![
                cylinder_arc_use(1, Sense::Reversed, cut_low),
                cylinder_ruling_use(6, Sense::Forward, lower_left),
                cylinder_arc_use(3, Sense::Forward, cut_high),
                cylinder_ruling_use(7, Sense::Reversed, lower_right),
            ]),
            AnalyticShellLoop::new(vec![ring_cylinder_use(100, Sense::Forward, low)]),
            AnalyticShellLoop::new(vec![ring_cylinder_use(101, Sense::Reversed, high)]),
        ];
        let cut_low_loop = AnalyticShellLoop::new(vec![
            plane_circle_use(0, Sense::Forward, cut_low_plane, cut_low_circle),
            plane_line_use(8, Sense::Forward, cut_low_plane, line(8)),
            plane_circle_use(1, Sense::Forward, cut_low_plane, cut_low_circle),
            plane_line_use(9, Sense::Forward, cut_low_plane, line(9)),
        ]);
        let cut_high_loop = AnalyticShellLoop::new(vec![
            plane_circle_use(2, Sense::Reversed, cut_high_plane, cut_high_circle),
            plane_line_use(11, Sense::Reversed, cut_high_plane, line(11)),
            plane_circle_use(3, Sense::Reversed, cut_high_plane, cut_high_circle),
            plane_line_use(10, Sense::Reversed, cut_high_plane, line(10)),
        ]);
        let right_loop = AnalyticShellLoop::new(vec![
            plane_line_use(9, Sense::Reversed, right_plane, line(9)),
            plane_line_use(7, Sense::Forward, right_plane, line(7)),
            plane_line_use(11, Sense::Forward, right_plane, line(11)),
            plane_line_use(4, Sense::Reversed, right_plane, line(4)),
        ]);
        let left_loop = AnalyticShellLoop::new(vec![
            plane_line_use(8, Sense::Reversed, left_plane, line(8)),
            plane_line_use(5, Sense::Forward, left_plane, line(5)),
            plane_line_use(10, Sense::Forward, left_plane, line(10)),
            plane_line_use(6, Sense::Reversed, left_plane, line(6)),
        ]);
        let domain = || FaceDomain::from_bounds(-4.0, 4.0, -4.0, 4.0).unwrap();
        AnalyticShellInput::new(
            vertices,
            edges,
            vec![
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(0),
                    AnalyticShellSurface::Cylinder(cylinder),
                    Sense::Forward,
                    FaceDomain::from_bounds(0.0, core::f64::consts::TAU, low, high).unwrap(),
                    host_loops,
                ),
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(1),
                    AnalyticShellSurface::Plane(low_plane),
                    Sense::Forward,
                    domain(),
                    vec![AnalyticShellLoop::new(vec![ring_plane_use(
                        100,
                        Sense::Reversed,
                        low_plane,
                        circle_at(low),
                    )])],
                ),
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(2),
                    AnalyticShellSurface::Plane(high_plane),
                    Sense::Forward,
                    domain(),
                    vec![AnalyticShellLoop::new(vec![ring_plane_use(
                        101,
                        Sense::Forward,
                        high_plane,
                        circle_at(high),
                    )])],
                ),
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(3),
                    AnalyticShellSurface::Plane(cut_low_plane),
                    Sense::Forward,
                    domain(),
                    vec![cut_low_loop],
                ),
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(4),
                    AnalyticShellSurface::Plane(cut_high_plane),
                    Sense::Forward,
                    domain(),
                    vec![cut_high_loop],
                ),
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(5),
                    AnalyticShellSurface::Plane(right_plane),
                    Sense::Forward,
                    domain(),
                    vec![right_loop],
                ),
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(6),
                    AnalyticShellSurface::Plane(left_plane),
                    Sense::Forward,
                    domain(),
                    vec![left_loop],
                ),
            ],
        )
        .with_closed_edges(vec![
            AnalyticShellClosedEdge::new(
                AnalyticEdgeKey::new(100),
                AnalyticShellCurve::Circle(circle_at(low)),
                ParamRange::new(0.0, core::f64::consts::TAU),
            ),
            AnalyticShellClosedEdge::new(
                AnalyticEdgeKey::new(101),
                AnalyticShellCurve::Circle(circle_at(high)),
                ParamRange::new(0.0, core::f64::consts::TAU),
            ),
        ])
    }

    fn face_for_key(output: &crate::analytic_shell::AnalyticShellOutput, key: u64) -> FaceId {
        output
            .faces()
            .iter()
            .find_map(|(candidate, face)| (candidate.value() == key).then_some(*face))
            .unwrap()
    }

    fn edge_for_key(output: &crate::analytic_shell::AnalyticShellOutput, key: u64) -> EdgeId {
        output
            .edges()
            .iter()
            .find_map(|(candidate, edge)| (candidate.value() == key).then_some(*edge))
            .unwrap()
    }

    #[test]
    fn interior_two_portal_component_is_certified_and_checked() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&portal_shell_input(), 1.0e-12)
            .unwrap();
        assert_eq!(
            certify_portal_cylinder_shell(transaction.store(), output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            })
        );
        let report =
            check_body_report(transaction.store(), output.body(), CheckLevel::Full).unwrap();
        assert_eq!(report.outcome(), CheckOutcome::Valid, "{report:?}");
        transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
    }

    #[test]
    fn portal_shell_face_sense_tamper_is_orientation_invalid() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&portal_shell_input(), 1.0e-12)
            .unwrap();
        let mut tampered = transaction.store().clone();
        tampered.get_mut(face_for_key(&output, 5)).unwrap().sense = Sense::Reversed;
        assert_eq!(
            certify_portal_cylinder_shell(&tampered, output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Invalid,
            })
        );
    }

    #[test]
    fn portal_shell_ring_direction_and_host_geometry_tampering_fail_closed() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&portal_shell_input(), 1.0e-12)
            .unwrap();

        let mut wrong_ring = transaction.store().clone();
        let high_ring = edge_for_key(&output, 101);
        let fins = wrong_ring.get(high_ring).unwrap().fins.clone();
        let host = face_for_key(&output, 0);
        for fin in fins {
            let face = wrong_ring
                .get(wrong_ring.get(fin).unwrap().parent)
                .unwrap()
                .face;
            wrong_ring.get_mut(fin).unwrap().sense = if face == host {
                Sense::Forward
            } else {
                Sense::Reversed
            };
        }
        assert_eq!(
            certify_portal_cylinder_shell(&wrong_ring, output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Invalid,
            })
        );

        let mut wrong_host = transaction.store().clone();
        let surface = wrong_host.get(host).unwrap().surface;
        let SurfaceGeom::Cylinder(cylinder) = *wrong_host.get(surface).unwrap() else {
            unreachable!()
        };
        let changed = Cylinder::new(*cylinder.frame(), 1.6).unwrap();
        let mut edit = wrong_host.transaction().unwrap();
        edit.assembly()
            .replace_surface(surface, SurfaceGeom::Cylinder(changed))
            .unwrap();
        assert_eq!(
            certify_portal_cylinder_shell(edit.store(), output.shell(), None).unwrap(),
            None
        );
    }

    fn session_with_work(allowed: u64) -> kcore::operation::SessionPolicy {
        let budget = BudgetPlan::new([LimitSpec::new(
            PORTAL_CYLINDER_SHELL_WORK,
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
    fn portal_shell_work_accepts_exact_n_and_rejects_n_minus_one() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&portal_shell_input(), 1.0e-12)
            .unwrap();
        let required = proof_work(transaction.store(), output.shell(), 1, 6)
            .unwrap()
            .unwrap();

        let exact_policy = session_with_work(required);
        let exact_context = kcore::operation::OperationContext::new(
            &exact_policy,
            kcore::tolerance::Tolerances::default(),
        )
        .unwrap();
        let mut exact_scope = OperationScope::new(&exact_context);
        assert_eq!(
            certify_portal_cylinder_shell(
                transaction.store(),
                output.shell(),
                Some(&mut exact_scope),
            )
            .unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            })
        );

        let denied_policy = session_with_work(required - 1);
        let denied_context = kcore::operation::OperationContext::new(
            &denied_policy,
            kcore::tolerance::Tolerances::default(),
        )
        .unwrap();
        let mut denied_scope = OperationScope::new(&denied_context);
        let error = certify_portal_cylinder_shell(
            transaction.store(),
            output.shell(),
            Some(&mut denied_scope),
        )
        .unwrap_err();
        assert_eq!(
            error.limit().map(|limit| limit.stage),
            Some(PORTAL_CYLINDER_SHELL_WORK)
        );
    }
}
