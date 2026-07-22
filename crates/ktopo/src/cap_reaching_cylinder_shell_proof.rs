//! Shell theorem for a strict-secant product feature reaching one cylinder cap.
//!
//! The admitted unsplit representation starts with a finite cylinder band and,
//! over one terminal axial interval, either removes its intersection with a
//! second parallel cylinder or adjoins the second disk outside the host. Its
//! incidence roles are one endpoint-free host cap, one piecewise-periodic host
//! side, one feature side, and two planar radial partitions. Constructor
//! provenance and storage order do not matter within this representation.
//! Exact normal translation pairs the feature arcs and rulings; strict radial
//! secancy proves that the complementary host spans have exactly two boundary
//! roots and classifies the feature span as inside (notch) or outside (boss).
//! These witnesses identify either
//! `boundary((D x I) \\ ((D ∩ N) x J))` or
//! `boundary((D x I) ∪ (N x J))` for any authored frame and either axial
//! direction; other decompositions fail closed.

use super::*;
use crate::entity::FinId;

use super::mixed_profile_prism_proof::{
    Cap, CapUse, ProfileCarrier, Side, Translation, certified_close, certified_nonzero,
    certified_parallel, certify_sweep_support, mapped_vertex, oriented_dot_sign, peer_face,
    prepare_cap, ruling_connects, translated_carrier, translated_vertices,
};
use super::portal_cylinder_shell_proof::{RadialSide, circle_secant_span_side};

#[cfg(test)]
#[path = "cap_reaching_cylinder_shell_proof/tests.rs"]
mod tests;

/// Cumulative deterministic work for cap-reaching cylinder-feature proofs.
pub(crate) const CAP_REACHING_CYLINDER_SHELL_WORK: StageId =
    match StageId::new("ktopo.check.cap-reaching-cylinder-shell-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid cap-reaching cylinder-shell work stage"),
    };

const DEFAULT_CAP_REACHING_CYLINDER_SHELL_WORK: u64 = 1_048_576;

pub(super) fn cap_reaching_cylinder_proof_budget() -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(
        CAP_REACHING_CYLINDER_SHELL_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        DEFAULT_CAP_REACHING_CYLINDER_SHELL_WORK,
    )])
    .expect("built-in cap-reaching cylinder-shell proof budget is valid")
}

#[derive(Debug)]
struct WholeCap {
    face: FaceId,
    center: Point3,
    local_orientation_valid: bool,
    host_loop_orientation: PredicateOrientation,
}

#[derive(Debug, Clone, Copy)]
struct HostArc {
    edge: EdgeId,
    cap: FaceId,
}

#[derive(Debug)]
struct HostBoundary {
    loop_orientation: PredicateOrientation,
    arcs: Vec<HostArc>,
    rulings: Vec<EdgeId>,
    feature_face: FaceId,
}

#[derive(Debug)]
struct FeatureBoundary {
    side: Side,
    loop_orientation: PredicateOrientation,
}

#[derive(Debug)]
struct EndCap {
    cap: Cap,
    host: CapUse,
    feature: CapUse,
    axial: Interval,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TerminalFeature {
    Notch,
    Boss,
}

/// Attempt the admitted two-root, two-axial-end cap-reaching feature theorem.
pub(super) fn certify_cap_reaching_cylinder_shell(
    store: &Store,
    shell_id: ShellId,
    scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellCertification>> {
    let shell = store.get(shell_id)?;
    if !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(None);
    }
    let mut cylinder_count = 0_usize;
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        if face.shell != shell_id {
            return Ok(None);
        }
        match store.get(face.surface)? {
            SurfaceGeom::Cylinder(_) => {
                cylinder_count =
                    cylinder_count
                        .checked_add(1)
                        .ok_or(kcore::error::Error::InvalidGeometry {
                            reason: "cap-reaching cylinder count overflow",
                        })?;
            }
            SurfaceGeom::Plane(_) => {}
            _ => return Ok(None),
        }
    }
    if cylinder_count < 2 {
        return Ok(None);
    }
    if let Some(scope) = scope {
        scope.ledger().require_limit(
            CAP_REACHING_CYLINDER_SHELL_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )?;
        let Some(work) = proof_work(store, shell_id, cylinder_count)? else {
            return Ok(Some(indeterminate()));
        };
        scope
            .ledger_mut()
            .charge(CAP_REACHING_CYLINDER_SHELL_WORK, work)?;
    }
    let mut cylinders = Vec::with_capacity(cylinder_count);
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        if let SurfaceGeom::Cylinder(cylinder) = store.get(face.surface)? {
            cylinders.push((face_id, *cylinder));
        }
    }
    for &(host_face, host) in &cylinders {
        if let Some(certification) = certify_host(store, shell_id, host_face, host, &cylinders)? {
            return Ok(Some(certification));
        }
    }
    Ok(None)
}

/// No-scratch structural upper bound for every candidate scan and pairwise
/// comparison. Unique edges are at most the fin count and unique vertices at
/// most twice it, so `1 + F + L + 4U` dominates theorem scratch size.
fn proof_work(store: &Store, shell_id: ShellId, cylinder_count: usize) -> Result<Option<u64>> {
    let shell = store.get(shell_id)?;
    let mut loops = 0_u64;
    let mut fins = 0_u64;
    for &face_id in &shell.faces {
        for &loop_id in &store.get(face_id)?.loops {
            loops = match loops.checked_add(1) {
                Some(value) => value,
                None => return Ok(None),
            };
            for &fin_id in &store.get(loop_id)?.fins {
                fins = match fins.checked_add(1) {
                    Some(value) => value,
                    None => return Ok(None),
                };
                let _ = store.get(fin_id)?;
            }
        }
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
    Ok(size
        .checked_mul(size)
        .and_then(|quadratic| quadratic.checked_add(size.checked_mul(32)?))
        .and_then(|per_candidate| per_candidate.checked_mul(candidates)))
}

fn certify_host(
    store: &Store,
    shell_id: ShellId,
    host_face: FaceId,
    host: Cylinder,
    cylinders: &[(FaceId, Cylinder)],
) -> Result<Option<ShellCertification>> {
    if certify_face_loop_layout(store, host_face)? != LoopContainment::Certified {
        return Ok(None);
    }
    let host_entity = store.get(host_face)?;
    let mut whole = None;
    let mut boundary = None;
    for &loop_id in &host_entity.loops {
        if let Some(candidate) = prepare_whole_cap(store, shell_id, host_face, host, loop_id)? {
            if whole.replace(candidate).is_some() {
                return Ok(None);
            }
            continue;
        }
        let Some(candidate) = prepare_host_boundary(store, host_face, host, loop_id)? else {
            return Ok(None);
        };
        if boundary.replace(candidate).is_some() {
            return Ok(None);
        }
    }
    let (Some(whole), Some(boundary)) = (whole, boundary) else {
        return Ok(None);
    };
    let Some(&(_, feature)) = cylinders
        .iter()
        .find(|(face, _)| *face == boundary.feature_face)
    else {
        return Ok(None);
    };
    let Some(mut ends) = prepare_ends(
        store,
        host_face,
        host,
        boundary.feature_face,
        feature,
        &boundary,
    )?
    else {
        return Ok(None);
    };
    let Some([inner, reached]) = order_ends_from_whole(&whole, host, &mut ends) else {
        return Ok(None);
    };
    let Some(translation) = translated_vertices(store, &inner.cap, &reached.cap)? else {
        return Ok(None);
    };
    let Some(terminal_feature) =
        classify_terminal_feature(host, feature, inner.host, reached.host, inner.feature)
    else {
        return Ok(None);
    };
    let geometry_checks = [
        certified_nonzero(translation.vector),
        certified_parallel(translation.vector, host.frame().z()),
        certified_parallel(translation.vector, feature.frame().z()),
        translated_carrier(inner.feature, reached.feature, translation.vector),
        complementary_host_arcs(inner.host, reached.host, &translation),
        rulings_biject_vertices(store, &boundary.rulings, &translation)?,
    ];
    if !geometry_checks.into_iter().all(|check| check) {
        return Ok(None);
    }
    let Some(feature_side) = prepare_feature_side(
        store,
        boundary.feature_face,
        host_face,
        &boundary.rulings,
        [&inner, &reached],
    )?
    else {
        return Ok(None);
    };
    let Some(feature_support) = certify_sweep_support(
        store,
        &feature_side.side,
        inner.feature,
        reached.feature,
        translation.vector,
    )?
    else {
        return Ok(None);
    };
    let mut role_faces = vec![host_face, boundary.feature_face, whole.face];
    role_faces.extend([inner.cap.face, reached.cap.face]);
    if !all_shell_faces_consumed(store, shell_id, &role_faces)? {
        return Ok(None);
    }
    Ok(Some(certification_from_orientation(
        store,
        host_face,
        &whole,
        &boundary,
        &feature_side,
        [&inner, &reached],
        feature_support,
        translation.vector,
        terminal_feature,
    )?))
}

fn prepare_whole_cap(
    store: &Store,
    shell_id: ShellId,
    host_face: FaceId,
    cylinder: Cylinder,
    loop_id: LoopId,
) -> Result<Option<WholeCap>> {
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
    if *cap_fin_id != peer
        || cap.shell != shell_id
        || cap.loops.as_slice() != [cap_loop_id]
        || !matches!(store.get(cap.surface)?, SurfaceGeom::Plane(_))
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
    if !circle_on_cylinder(*circle, cylinder) {
        return Ok(None);
    }
    let (Some(host_orientation), Some(cap_orientation)) = (
        certify_loop_orientation(store, host_face, loop_id)?,
        certify_loop_orientation(store, cap_face, cap_loop_id)?,
    ) else {
        return Ok(None);
    };
    Ok(Some(WholeCap {
        face: cap_face,
        center: circle.frame().origin(),
        local_orientation_valid: (cap_orientation == PredicateOrientation::Positive)
            == cap.sense.is_forward(),
        host_loop_orientation: host_orientation,
    }))
}

fn prepare_host_boundary(
    store: &Store,
    host_face: FaceId,
    cylinder: Cylinder,
    loop_id: LoopId,
) -> Result<Option<HostBoundary>> {
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
    let mut feature_face = None;
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
                if circle_on_cylinder(*circle, cylinder)
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
                if exact_line_carrier(curve)
                    .is_some_and(|line| certified_parallel(line.dir(), cylinder.frame().z()))
                    && matches!(
                        store.get(store.get(peer)?.surface)?,
                        SurfaceGeom::Cylinder(_)
                    ) =>
            {
                if feature_face
                    .replace(peer)
                    .is_some_and(|prior| prior != peer)
                    || rulings.contains(&fin.edge)
                {
                    return Ok(None);
                }
                rulings.push(fin.edge);
            }
            _ => return Ok(None),
        }
    }
    let Some(feature_face) = feature_face else {
        return Ok(None);
    };
    Ok(Some(HostBoundary {
        loop_orientation,
        arcs,
        rulings,
        feature_face,
    }))
}

fn prepare_ends(
    store: &Store,
    host_face: FaceId,
    host: Cylinder,
    feature_face: FaceId,
    feature: Cylinder,
    boundary: &HostBoundary,
) -> Result<Option<Vec<EndCap>>> {
    let mut ends = Vec::new();
    for arc in &boundary.arcs {
        let Some(cap) = prepare_cap(store, arc.cap)? else {
            return Ok(None);
        };
        let mut host_use = None;
        let mut feature_use = None;
        for &use_ in &cap.uses {
            match peer_face(store, use_)? {
                Some(peer) if peer == host_face && circle_use_on_cylinder(use_, host) => {
                    if use_.edge != arc.edge || host_use.replace(use_).is_some() {
                        return Ok(None);
                    }
                }
                Some(peer) if peer == feature_face && circle_use_on_cylinder(use_, feature) => {
                    if feature_use.replace(use_).is_some() {
                        return Ok(None);
                    }
                }
                _ => return Ok(None),
            }
        }
        let (Some(host_use), Some(feature_use)) = (host_use, feature_use) else {
            return Ok(None);
        };
        let axial = axial_coordinate(host.frame(), cap.plane.frame().origin());
        ends.push(EndCap {
            cap,
            host: host_use,
            feature: feature_use,
            axial,
        });
    }
    Ok(Some(ends))
}

fn order_ends_from_whole(
    whole: &WholeCap,
    host: Cylinder,
    ends: &mut Vec<EndCap>,
) -> Option<[EndCap; 2]> {
    let whole_axial = axial_coordinate(host.frame(), whole.center);
    let mut second = ends.pop()?;
    let mut first = ends.pop()?;
    if !ends.is_empty() {
        return None;
    }
    let first_offset = first.axial - whole_axial;
    let second_offset = second.axial - whole_axial;
    let positive = first_offset.lo() > 0.0 && second_offset.lo() > 0.0;
    let negative = first_offset.hi() < 0.0 && second_offset.hi() < 0.0;
    if positive && first.axial.hi() < second.axial.lo()
        || negative && first.axial.lo() > second.axial.hi()
    {
        Some([first, second])
    } else if positive && second.axial.hi() < first.axial.lo()
        || negative && second.axial.lo() > first.axial.hi()
    {
        core::mem::swap(&mut first, &mut second);
        Some([first, second])
    } else {
        None
    }
}

fn prepare_feature_side(
    store: &Store,
    face_id: FaceId,
    host_face: FaceId,
    rulings: &[EdgeId],
    ends: [&EndCap; 2],
) -> Result<Option<FeatureBoundary>> {
    let face = store.get(face_id)?;
    let [loop_id] = face.loops.as_slice() else {
        return Ok(None);
    };
    if certify_loop_simplicity(store, *loop_id)? != LoopSimplicity::Certified {
        return Ok(None);
    }
    let Some(orientation) = certify_loop_orientation(store, face_id, *loop_id)? else {
        return Ok(None);
    };
    let expected = rulings
        .iter()
        .copied()
        .chain(ends.iter().map(|end| end.feature.edge))
        .collect::<Vec<_>>();
    if expected
        .iter()
        .enumerate()
        .any(|(index, edge)| expected[index + 1..].contains(edge))
    {
        return Ok(None);
    }
    let mut fins = Vec::new();
    let mut actual = Vec::new();
    for &fin_id in &store.get(*loop_id)?.fins {
        if certify_whole_fin_incidence(store, face_id, *loop_id, fin_id, LINEAR_RESOLUTION)
            != WholeFinIncidence::Certified
        {
            return Ok(None);
        }
        let edge = store.get(fin_id)?.edge;
        if actual.contains(&edge) || !expected.contains(&edge) {
            return Ok(None);
        }
        actual.push(edge);
        let peer = peer_face_from_fin(store, fin_id)?;
        let valid_peer = if rulings.contains(&edge) {
            peer == Some(host_face)
        } else {
            ends.iter()
                .any(|end| end.feature.edge == edge && peer == Some(end.cap.face))
        };
        if !valid_peer {
            return Ok(None);
        }
        fins.push((fin_id, edge));
    }
    Ok((actual.len() == expected.len()).then_some(FeatureBoundary {
        side: Side {
            face: face_id,
            fins,
        },
        loop_orientation: orientation,
    }))
}

fn complementary_host_arcs(first: CapUse, second: CapUse, translation: &Translation) -> bool {
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
    let (Some(first_tail), Some(first_head)) = (
        mapped_vertex(&translation.vertices, first.tail),
        mapped_vertex(&translation.vertices, first.head),
    ) else {
        return false;
    };
    (first_tail == second.tail && first_head == second.head)
        || (first_tail == second.head && first_head == second.tail)
}

/// Classify the selected strict-secant radial cells, not merely their carrier
/// widths. The nearer host arc lies inside the feature and its translated
/// complement lies outside. The feature span then distinguishes the two
/// complete terminal products: an inside span removes the lens (`Notch`),
/// while an outside span adjoins the feature disk difference (`Boss`).
/// Together with the two distinct shared topology endpoints, the circle
/// secant theorem derives the complete two-root radial partition.
fn classify_terminal_feature(
    host: Cylinder,
    feature: Cylinder,
    inner_host: CapUse,
    reached_host: CapUse,
    inner_feature: CapUse,
) -> Option<TerminalFeature> {
    let (
        ProfileCarrier::Circle(inner_host_circle),
        ProfileCarrier::Circle(reached_host_circle),
        ProfileCarrier::Circle(feature_circle),
    ) = (
        inner_host.carrier,
        reached_host.carrier,
        inner_feature.carrier,
    )
    else {
        return None;
    };
    let distinct = |use_: CapUse| use_.tail != use_.head;
    let feature_side = circle_secant_span_side(
        host,
        feature_circle,
        inner_feature.range,
        inner_host_circle,
        distinct(inner_feature),
    )?;
    if circle_secant_span_side(
        feature,
        inner_host_circle,
        inner_host.range,
        feature_circle,
        distinct(inner_host),
    ) == Some(RadialSide::Inside)
        && circle_secant_span_side(
            feature,
            reached_host_circle,
            reached_host.range,
            feature_circle,
            distinct(reached_host),
        ) == Some(RadialSide::Outside)
    {
        Some(match feature_side {
            RadialSide::Inside => TerminalFeature::Notch,
            RadialSide::Outside => TerminalFeature::Boss,
        })
    } else {
        None
    }
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
    host_face: FaceId,
    whole: &WholeCap,
    boundary: &HostBoundary,
    feature_boundary: &FeatureBoundary,
    ends: [&EndCap; 2],
    feature_support: i8,
    outward_axis: Vec3,
    terminal_feature: TerminalFeature,
) -> Result<ShellCertification> {
    let host = store.get(host_face)?;
    let feature = store.get(boundary.feature_face)?;
    let host_sign = sense_factor(host.sense) as i8;
    let whole_face = store.get(whole.face)?;
    let whole_plane = match store.get(whole_face.surface)? {
        SurfaceGeom::Plane(plane) => *plane,
        _ => return Ok(indeterminate()),
    };
    let whole_normal = whole_plane.frame().z() * sense_factor(whole_face.sense);
    let whole_sign = oriented_dot_sign(whole_normal, outward_axis);
    let mut end_signs = [None; 2];
    for (index, end) in ends.into_iter().enumerate() {
        let face = store.get(end.cap.face)?;
        end_signs[index] = oriented_dot_sign(
            end.cap.plane.frame().z() * sense_factor(face.sense),
            outward_axis,
        );
    }
    let feature_sign = sense_factor(feature.sense) as i8;
    let terminal_orientation_valid = match terminal_feature {
        TerminalFeature::Notch => feature_sign == -host_sign && end_signs == [Some(host_sign); 2],
        TerminalFeature::Boss => {
            feature_sign == host_sign && end_signs == [Some(-host_sign), Some(host_sign)]
        }
    };
    let coherent = whole.local_orientation_valid
        && whole.host_loop_orientation != boundary.loop_orientation
        && ends.iter().all(|end| end.cap.local_orientation_valid)
        && terminal_orientation_valid
        && (feature_boundary.loop_orientation == PredicateOrientation::Positive)
            == feature.sense.is_forward()
        && whole_sign == Some(-host_sign)
        && feature_support == host_sign;
    Ok(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: if coherent {
            if host_sign > 0 {
                ShellOrientation::Positive
            } else {
                ShellOrientation::Negative
            }
        } else {
            ShellOrientation::Invalid
        },
    })
}

fn all_shell_faces_consumed(store: &Store, shell_id: ShellId, faces: &[FaceId]) -> Result<bool> {
    let expected = &store.get(shell_id)?.faces;
    let unique = !faces
        .iter()
        .enumerate()
        .any(|(index, face)| faces[index + 1..].contains(face));
    Ok(unique && faces.len() == expected.len() && faces.iter().all(|face| expected.contains(face)))
}

fn peer_fin(store: &Store, fin_id: FinId) -> Result<Option<FinId>> {
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
    Ok((store.get(peer)?.sense != fin.sense).then_some(peer))
}

fn peer_face_from_fin(store: &Store, fin_id: FinId) -> Result<Option<FaceId>> {
    let Some(peer) = peer_fin(store, fin_id)? else {
        return Ok(None);
    };
    Ok(Some(store.get(store.get(peer)?.parent)?.face))
}

fn circle_use_on_cylinder(use_: CapUse, cylinder: Cylinder) -> bool {
    matches!(use_.carrier, ProfileCarrier::Circle(circle) if circle_on_cylinder(circle, cylinder))
}

fn circle_on_cylinder(circle: kgeom::curve::Circle, cylinder: Cylinder) -> bool {
    circle.radius().to_bits() == cylinder.radius().to_bits()
        && certified_parallel(circle.frame().z(), cylinder.frame().z())
        && point_on_axis(cylinder.frame(), circle.frame().origin())
}

fn point_on_axis(frame: &Frame, point: Point3) -> bool {
    let offset = point - frame.origin();
    let radial = [frame.x(), frame.y()]
        .into_iter()
        .map(|axis| {
            (Interval::point(axis.x) * Interval::point(offset.x)
                + Interval::point(axis.y) * Interval::point(offset.y)
                + Interval::point(axis.z) * Interval::point(offset.z))
            .square()
        })
        .fold(Interval::point(0.0), |sum, value| sum + value);
    radial.hi().is_finite() && radial.hi() <= Interval::point(LINEAR_RESOLUTION).square().lo()
}

fn axial_coordinate(frame: &Frame, point: Point3) -> Interval {
    let offset = point - frame.origin();
    Interval::point(frame.z().x) * Interval::point(offset.x)
        + Interval::point(frame.z().y) * Interval::point(offset.y)
        + Interval::point(frame.z().z) * Interval::point(offset.z)
}
