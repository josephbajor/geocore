//! Shell theorems for axially touching parallel-cylinder bands.
//!
//! Nested disks retain one shared two-loop annulus. Strict-secant disks retain
//! two complementary coplanar disk differences, four bounded source-circle
//! arcs, and a degree-four link at each of the two exact roots. Exact internal
//! tangency retains one or two pinched shoulder loops whose complete-period
//! edge pairs share exact tangent vertices. In every family the terminal far
//! disks lie beyond the contact planes. Whole-fin incidence, loop layout,
//! exact or outward-interval radial predicates, and coherent winding identify
//! the shell with the regularized boundary after each shared interface is
//! removed.

use super::mixed_profile_prism_proof::{
    Cap, ProfileCarrier, oriented_dot_sign, peer_face, prepare_cap,
};
use super::*;
use crate::entity::VertexId;
use crate::semantic_planar_math::{
    IntervalVec3, cross as interval_cross, dot as interval_dot, point as interval_point,
    sub as interval_sub,
};

#[cfg(test)]
#[path = "parallel_cylinder_contact_shell_proof/tests.rs"]
mod tests;

#[cfg(test)]
#[path = "parallel_cylinder_contact_shell_proof/strict_secant_tests.rs"]
mod strict_secant_tests;

#[cfg(test)]
#[path = "parallel_cylinder_contact_shell_proof/internal_tangent_tests.rs"]
mod internal_tangent_tests;

#[cfg(test)]
#[path = "parallel_cylinder_contact_shell_proof/internal_tangent_chain_tests.rs"]
mod internal_tangent_chain_tests;

#[path = "parallel_cylinder_contact_shell_proof/internal_tangent.rs"]
mod internal_tangent;

/// Cumulative deterministic work for parallel-cylinder contact shell proofs.
pub(crate) const PARALLEL_CYLINDER_CONTACT_SHELL_WORK: StageId =
    match StageId::new("ktopo.check.parallel-cylinder-contact-shell-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid parallel-cylinder contact shell work stage"),
    };

const DEFAULT_PARALLEL_CYLINDER_CONTACT_SHELL_WORK: u64 = 4096;

pub(super) fn parallel_cylinder_contact_proof_budget() -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(
        PARALLEL_CYLINDER_CONTACT_SHELL_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        DEFAULT_PARALLEL_CYLINDER_CONTACT_SHELL_WORK,
    )])
    .expect("built-in parallel-cylinder contact shell proof budget is valid")
}

#[derive(Debug, Clone, Copy)]
struct RingBoundary {
    side_loop: LoopId,
    cap_face: FaceId,
    edge: EdgeId,
    circle: kgeom::curve::Circle,
    axial_parameter: f64,
    cap_axis_alignment: PredicateOrientation,
    side_traverses_positive_u: bool,
}

#[derive(Debug)]
struct ContactBand {
    face: FaceId,
    cylinder: Cylinder,
    far: RingBoundary,
    contact: RingBoundary,
}

/// Attempt an incidence-discovered axial-contact theorem.
pub(super) fn certify_parallel_cylinder_contact_shell(
    store: &Store,
    shell_id: ShellId,
    scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellCertification>> {
    let shell = store.get(shell_id)?;
    if !matches!(shell.faces.len(), 5..=7) || !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(None);
    }
    let mut cylinders = Vec::with_capacity(3);
    let mut planes = Vec::with_capacity(4);
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        if face.shell != shell_id {
            return Ok(None);
        }
        match store.get(face.surface)? {
            SurfaceGeom::Cylinder(cylinder) => cylinders.push((face_id, *cylinder)),
            SurfaceGeom::Plane(plane) => planes.push((face_id, *plane)),
            _ => return Ok(None),
        }
    }
    if !matches!((cylinders.len(), planes.len()), (2, 3) | (2, 4) | (3, 4))
        || planes.len() + cylinders.len() != shell.faces.len()
    {
        return Ok(None);
    }

    if let Some(scope) = scope {
        scope.ledger().require_limit(
            PARALLEL_CYLINDER_CONTACT_SHELL_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )?;
        let Some(work) = proof_work(store, shell_id)? else {
            return Ok(Some(indeterminate()));
        };
        scope
            .ledger_mut()
            .charge(PARALLEL_CYLINDER_CONTACT_SHELL_WORK, work)?;
    }

    if cylinders.len() == 3 {
        return internal_tangent::certify_internal_tangent_contact(store, shell_id, &cylinders);
    }
    let [(first_face, first), (second_face, second)] = cylinders.as_slice() else {
        return Ok(None);
    };
    if planes.len() == 4 {
        return certify_strict_secant_contact(
            store,
            shell_id,
            [(*first_face, *first), (*second_face, *second)],
        );
    }
    let nested = certify_nested_contact(
        store,
        shell_id,
        [(*first_face, *first), (*second_face, *second)],
    )?;
    if nested.is_some() {
        return Ok(nested);
    }
    internal_tangent::certify_internal_tangent_contact(store, shell_id, &cylinders)
}

fn certify_nested_contact(
    store: &Store,
    shell_id: ShellId,
    cylinders: [(FaceId, Cylinder); 2],
) -> Result<Option<ShellCertification>> {
    let [(first_face, first), (second_face, second)] = cylinders;
    let Some(first) = prepare_band(store, shell_id, first_face, first)? else {
        return Ok(None);
    };
    let Some(second) = prepare_band(store, shell_id, second_face, second)? else {
        return Ok(None);
    };
    if first.contact.cap_face != second.contact.cap_face
        || first.far.cap_face == second.far.cap_face
        || !vectors_are_exactly_parallel(first.cylinder.frame().z(), second.cylinder.frame().z())
    {
        return Ok(None);
    }
    let annulus_face = store.get(first.contact.cap_face)?;
    let SurfaceGeom::Plane(annulus_plane) = store.get(annulus_face.surface)? else {
        return Ok(None);
    };
    if annulus_face.loops.len() != 2
        || certify_face_loop_layout(store, first.contact.cap_face)? != LoopContainment::Certified
        || first.contact.edge == second.contact.edge
    {
        return Ok(None);
    }

    let (outer, inner) = if strictly_contains(first.contact.circle, second.contact.circle) {
        (&first, &second)
    } else if strictly_contains(second.contact.circle, first.contact.circle) {
        (&second, &first)
    } else {
        return Ok(None);
    };
    if !strictly_contains_cylinder_support(outer.cylinder, inner.cylinder) {
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
            first.contact.cap_face,
        ],
    )? {
        return Ok(None);
    }

    let raw_outer_far_side = exact_affine_sign(
        annulus_plane.frame().z(),
        outer.far.circle.frame().origin(),
        annulus_plane.frame().origin(),
    );
    let raw_inner_far_side = exact_affine_sign(
        annulus_plane.frame().z(),
        inner.far.circle.frame().origin(),
        annulus_plane.frame().origin(),
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

    let annulus_outward = annulus_plane.frame().z() * sense_factor(annulus_face.sense);
    let annulus_orientation_valid = exact_affine_sign(
        annulus_outward,
        outer.far.circle.frame().origin(),
        annulus_plane.frame().origin(),
    ) == Some(PredicateOrientation::Negative)
        && exact_affine_sign(
            annulus_outward,
            inner.far.circle.frame().origin(),
            annulus_plane.frame().origin(),
        ) == Some(PredicateOrientation::Positive);
    let orientation_valid = annulus_orientation_valid
        && band_orientation_valid(store, outer, BandRole::Outer)?
        && band_orientation_valid(store, inner, BandRole::Inner)?;
    Ok(Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: if orientation_valid {
            ShellOrientation::Positive
        } else {
            ShellOrientation::Invalid
        },
    }))
}

#[derive(Debug, Clone, Copy)]
struct SecantArc {
    edge: EdgeId,
    cap_face: FaceId,
    circle: kgeom::curve::Circle,
    range: kgeom::param::ParamRange,
    vertices: [VertexId; 2],
    axial_parameter: f64,
    side_traverses_positive_u: bool,
}

#[derive(Debug)]
struct SecantBoundary {
    loop_orientation: PredicateOrientation,
    arcs: [SecantArc; 2],
}

#[derive(Debug)]
struct SecantBand {
    face: FaceId,
    cylinder: Cylinder,
    far: RingBoundary,
    far_loop_orientation: PredicateOrientation,
    far_cap_local_orientation_valid: bool,
    contact: SecantBoundary,
}

#[derive(Debug, Clone, Copy)]
struct SecantRoles {
    inside: SecantArc,
    outside: SecantArc,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SecantRadialSide {
    Inside,
    Outside,
}

fn certify_strict_secant_contact(
    store: &Store,
    shell_id: ShellId,
    cylinders: [(FaceId, Cylinder); 2],
) -> Result<Option<ShellCertification>> {
    let [(first_face, first_cylinder), (second_face, second_cylinder)] = cylinders;
    if !vectors_are_exactly_parallel(first_cylinder.frame().z(), second_cylinder.frame().z()) {
        return Ok(None);
    }
    let Some(first) = prepare_secant_band(store, shell_id, first_face, first_cylinder)? else {
        return Ok(None);
    };
    let Some(second) = prepare_secant_band(store, shell_id, second_face, second_cylinder)? else {
        return Ok(None);
    };
    let Some(vertices) = certify_degree_four_link(&first, &second) else {
        return Ok(None);
    };
    if !all_secant_edges_are_unique(&first, &second) {
        return Ok(None);
    }

    let contact_faces = [
        first.contact.arcs[0].cap_face,
        first.contact.arcs[1].cap_face,
    ];
    if contact_faces[0] == contact_faces[1]
        || !second
            .contact
            .arcs
            .iter()
            .all(|arc| contact_faces.contains(&arc.cap_face))
    {
        return Ok(None);
    }
    let Some(caps) = prepare_secant_caps(store, contact_faces, [&first, &second], vertices)? else {
        return Ok(None);
    };
    if !contact_planes_are_common(&caps, [&first, &second]) {
        return Ok(None);
    }

    let Some(first_roles) = classify_secant_roles(&first, &second) else {
        return Ok(None);
    };
    let Some(second_roles) = classify_secant_roles(&second, &first) else {
        return Ok(None);
    };
    if first_roles.outside.cap_face != second_roles.inside.cap_face
        || second_roles.outside.cap_face != first_roles.inside.cap_face
        || first_roles.outside.cap_face == first_roles.inside.cap_face
    {
        return Ok(None);
    }
    if !far_ends_straddle_contact_plane(&caps[0], [&first, &second]) {
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
            contact_faces[0],
            contact_faces[1],
        ],
    )? {
        return Ok(None);
    }

    let orientation =
        strict_secant_orientation(store, [&first, &second], [first_roles, second_roles], &caps)?;
    Ok(Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation,
    }))
}

fn prepare_secant_band(
    store: &Store,
    shell_id: ShellId,
    face_id: FaceId,
    cylinder: Cylinder,
) -> Result<Option<SecantBand>> {
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
        match loop_.fins.len() {
            1 => {
                let Some(candidate) =
                    prepare_boundary(store, shell_id, face_id, cylinder, loop_id)?
                else {
                    return Ok(None);
                };
                if store.get(candidate.cap_face)?.loops.len() != 1
                    || far.replace(candidate).is_some()
                {
                    return Ok(None);
                }
            }
            2 => {
                let Some(candidate) =
                    prepare_secant_boundary(store, shell_id, face_id, cylinder, loop_id)?
                else {
                    return Ok(None);
                };
                if contact.replace(candidate).is_some() {
                    return Ok(None);
                }
            }
            _ => return Ok(None),
        }
    }
    let (Some(far), Some(contact)) = (far, contact) else {
        return Ok(None);
    };
    if contact.arcs.iter().any(|arc| arc.edge == far.edge)
        || contact.arcs[0].axial_parameter == far.axial_parameter
    {
        return Ok(None);
    }
    let Some(far_loop_orientation) = certify_loop_orientation(store, face_id, far.side_loop)?
    else {
        return Ok(None);
    };
    let far_cap = store.get(far.cap_face)?;
    let [far_cap_loop] = far_cap.loops.as_slice() else {
        return Ok(None);
    };
    let Some(far_cap_orientation) = certify_loop_orientation(store, far.cap_face, *far_cap_loop)?
    else {
        return Ok(None);
    };
    Ok(Some(SecantBand {
        face: face_id,
        cylinder,
        far,
        far_loop_orientation,
        far_cap_local_orientation_valid: (far_cap_orientation == PredicateOrientation::Positive)
            == far_cap.sense.is_forward(),
        contact,
    }))
}

fn prepare_secant_boundary(
    store: &Store,
    shell_id: ShellId,
    side_face: FaceId,
    cylinder: Cylinder,
    loop_id: LoopId,
) -> Result<Option<SecantBoundary>> {
    let loop_ = store.get(loop_id)?;
    let [first_fin, second_fin] = loop_.fins.as_slice() else {
        return Ok(None);
    };
    if loop_.face != side_face {
        return Ok(None);
    }
    if certify_loop_simplicity(store, loop_id)? != LoopSimplicity::Certified {
        return Ok(None);
    }
    let Some(loop_orientation) = certify_loop_orientation(store, side_face, loop_id)? else {
        return Ok(None);
    };
    let (Some(first), Some(second)) = (
        prepare_secant_arc(store, shell_id, side_face, cylinder, loop_id, *first_fin)?,
        prepare_secant_arc(store, shell_id, side_face, cylinder, loop_id, *second_fin)?,
    ) else {
        return Ok(None);
    };
    if first.edge == second.edge
        || first.cap_face == second.cap_face
        || first.circle != second.circle
        || !same_vertex_pair(first.vertices, second.vertices)
        || first.axial_parameter.to_bits() != second.axial_parameter.to_bits()
        || first.side_traverses_positive_u != second.side_traverses_positive_u
    {
        return Ok(None);
    }
    let expected_orientation = if first.side_traverses_positive_u {
        PredicateOrientation::Positive
    } else {
        PredicateOrientation::Negative
    };
    if loop_orientation != expected_orientation {
        return Ok(None);
    }
    Ok(Some(SecantBoundary {
        loop_orientation,
        arcs: [first, second],
    }))
}

fn prepare_secant_arc(
    store: &Store,
    shell_id: ShellId,
    side_face: FaceId,
    cylinder: Cylinder,
    side_loop: LoopId,
    side_fin_id: crate::entity::FinId,
) -> Result<Option<SecantArc>> {
    let side_fin = store.get(side_fin_id)?;
    let edge = store.get(side_fin.edge)?;
    let (
        Some(curve_id),
        Some((lo, hi)),
        [Some(first_vertex), Some(second_vertex)],
        [first_fin, second_fin],
    ) = (edge.curve, edge.bounds, edge.vertices, edge.fins.as_slice())
    else {
        return Ok(None);
    };
    if edge.tolerance.is_some()
        || side_fin.parent != side_loop
        || first_vertex == second_vertex
        || !lo.is_finite()
        || !hi.is_finite()
        || lo >= hi
    {
        return Ok(None);
    }
    let peer_fin_id = if *first_fin == side_fin_id {
        *second_fin
    } else if *second_fin == side_fin_id {
        *first_fin
    } else {
        return Ok(None);
    };
    let peer_fin = store.get(peer_fin_id)?;
    let peer_loop = store.get(peer_fin.parent)?;
    let cap_face_id = peer_loop.face;
    let cap_face = store.get(cap_face_id)?;
    let CurveGeom::Circle(circle) = store.get(curve_id)? else {
        return Ok(None);
    };
    let SurfaceGeom::Plane(cap_plane) = store.get(cap_face.surface)? else {
        return Ok(None);
    };
    if peer_fin.edge != side_fin.edge
        || peer_fin.sense == side_fin.sense
        || cap_face.shell != shell_id
        || cap_face.loops.as_slice() != [peer_fin.parent]
        || peer_loop.fins.len() != 2
        || !circle_on_cylinder(*circle, cylinder)
        || !vectors_are_exactly_parallel(cap_plane.frame().z(), cylinder.frame().z())
        || !point_is_within_plane_envelope(
            circle.frame().origin(),
            cap_plane.frame().origin(),
            cap_plane.frame().z(),
            LINEAR_RESOLUTION,
        )
        || certify_whole_fin_incidence(store, side_face, side_loop, side_fin_id, LINEAR_RESOLUTION)
            != WholeFinIncidence::Certified
        || certify_whole_fin_incidence(
            store,
            cap_face_id,
            peer_fin.parent,
            peer_fin_id,
            LINEAR_RESOLUTION,
        ) != WholeFinIncidence::Certified
    {
        return Ok(None);
    }
    let (Some(side_use), Some(cap_use)) = (side_fin.pcurve, peer_fin.pcurve) else {
        return Ok(None);
    };
    let (Curve2dGeom::Line(side_line), Curve2dGeom::Circle(cap_circle)) =
        (store.get(side_use.curve())?, store.get(cap_use.curve())?)
    else {
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
    let Some(side_traverses_positive_u) = traversal_is_positive(
        [side_line.dir().x, side_use.edge_to_pcurve().scale()],
        side_fin.sense,
    ) else {
        return Ok(None);
    };
    Ok(Some(SecantArc {
        edge: side_fin.edge,
        cap_face: cap_face_id,
        circle: *circle,
        range: kgeom::param::ParamRange::new(lo, hi),
        vertices: [first_vertex, second_vertex],
        axial_parameter: side_line.origin().y,
        side_traverses_positive_u,
    }))
}

fn certify_degree_four_link(first: &SecantBand, second: &SecantBand) -> Option<[VertexId; 2]> {
    let arcs = [
        first.contact.arcs[0],
        first.contact.arcs[1],
        second.contact.arcs[0],
        second.contact.arcs[1],
    ];
    if arcs
        .iter()
        .enumerate()
        .any(|(index, arc)| arcs[index + 1..].iter().any(|peer| peer.edge == arc.edge))
    {
        return None;
    }
    let vertices = arcs[0].vertices;
    (vertices[0] != vertices[1]
        && arcs
            .iter()
            .all(|arc| same_vertex_pair(vertices, arc.vertices)))
    .then_some(vertices)
}

fn all_secant_edges_are_unique(first: &SecantBand, second: &SecantBand) -> bool {
    let edges = [
        first.contact.arcs[0].edge,
        first.contact.arcs[1].edge,
        second.contact.arcs[0].edge,
        second.contact.arcs[1].edge,
        first.far.edge,
        second.far.edge,
    ];
    !edges
        .iter()
        .enumerate()
        .any(|(index, edge)| edges[index + 1..].contains(edge))
}

fn same_vertex_pair(first: [VertexId; 2], second: [VertexId; 2]) -> bool {
    first == second || first == [second[1], second[0]]
}

fn prepare_secant_caps(
    store: &Store,
    faces: [FaceId; 2],
    bands: [&SecantBand; 2],
    vertices: [VertexId; 2],
) -> Result<Option<[Cap; 2]>> {
    let Some(first) = prepare_cap(store, faces[0])? else {
        return Ok(None);
    };
    let Some(second) = prepare_cap(store, faces[1])? else {
        return Ok(None);
    };
    for cap in [&first, &second] {
        if cap.uses.len() != 2
            || cap.vertices.len() != 2
            || !cap.vertices.iter().all(|vertex| vertices.contains(vertex))
        {
            return Ok(None);
        }
        for band in bands {
            let mut matching = 0;
            for &use_ in &cap.uses {
                if band.contact.arcs.iter().any(|arc| arc.edge == use_.edge)
                    && matches!(use_.carrier, ProfileCarrier::Circle(_))
                    && peer_face(store, use_)? == Some(band.face)
                {
                    matching += 1;
                }
            }
            if matching != 1 {
                return Ok(None);
            }
        }
    }
    Ok(Some([first, second]))
}

fn contact_planes_are_common(caps: &[Cap; 2], bands: [&SecantBand; 2]) -> bool {
    vectors_are_exactly_parallel(caps[0].plane.frame().z(), caps[1].plane.frame().z())
        && point_is_within_plane_envelope(
            caps[1].plane.frame().origin(),
            caps[0].plane.frame().origin(),
            caps[0].plane.frame().z(),
            LINEAR_RESOLUTION,
        )
        && caps.iter().all(|cap| {
            bands.iter().all(|band| {
                vectors_are_exactly_parallel(cap.plane.frame().z(), band.cylinder.frame().z())
            })
        })
}

fn classify_secant_roles(band: &SecantBand, other: &SecantBand) -> Option<SecantRoles> {
    let portal = other.contact.arcs[0].circle;
    let classified = band.contact.arcs.map(|arc| {
        strict_secant_span_side(
            other.cylinder,
            arc.circle,
            arc.range,
            portal,
            arc.vertices[0] != arc.vertices[1],
        )
        .map(|side| (arc, side))
    });
    let [Some(first), Some(second)] = classified else {
        return None;
    };
    match (first, second) {
        ((inside, SecantRadialSide::Inside), (outside, SecantRadialSide::Outside))
        | ((outside, SecantRadialSide::Outside), (inside, SecantRadialSide::Inside)) => {
            Some(SecantRoles { inside, outside })
        }
        _ => None,
    }
}

fn far_ends_straddle_contact_plane(cap: &Cap, bands: [&SecantBand; 2]) -> bool {
    let sides = bands.map(|band| {
        exact_affine_sign(
            cap.plane.frame().z(),
            band.far.circle.frame().origin(),
            cap.plane.frame().origin(),
        )
    });
    matches!(
        sides,
        [
            Some(PredicateOrientation::Negative),
            Some(PredicateOrientation::Positive)
        ] | [
            Some(PredicateOrientation::Positive),
            Some(PredicateOrientation::Negative)
        ]
    )
}

fn strict_secant_orientation(
    store: &Store,
    bands: [&SecantBand; 2],
    roles: [SecantRoles; 2],
    caps: &[Cap; 2],
) -> Result<ShellOrientation> {
    let support = [
        sense_factor(store.get(bands[0].face)?.sense) as i8,
        sense_factor(store.get(bands[1].face)?.sense) as i8,
    ];
    if support[0] != support[1] {
        return Ok(ShellOrientation::Invalid);
    }
    let mut coherent = true;
    for index in 0..2 {
        let band = bands[index];
        let contact_parameter = band.contact.arcs[0].axial_parameter;
        let vector = if band.far.axial_parameter < contact_parameter {
            band.cylinder.frame().z()
        } else if contact_parameter < band.far.axial_parameter {
            -band.cylinder.frame().z()
        } else {
            return Ok(ShellOrientation::Invalid);
        };
        let Some(contact_cap) = caps
            .iter()
            .find(|cap| cap.face == roles[index].outside.cap_face)
        else {
            return Ok(ShellOrientation::Invalid);
        };
        let far_face = store.get(band.far.cap_face)?;
        let SurfaceGeom::Plane(far_plane) = store.get(far_face.surface)? else {
            return Ok(ShellOrientation::Invalid);
        };
        let far_outward = far_plane.frame().z() * sense_factor(far_face.sense);
        let contact_face = store.get(contact_cap.face)?;
        let contact_outward = contact_cap.plane.frame().z() * sense_factor(contact_face.sense);
        coherent &= band.far_loop_orientation != band.contact.loop_orientation
            && band.far_cap_local_orientation_valid
            && contact_cap.local_orientation_valid
            && oriented_dot_sign(far_outward, vector) == Some(-support[index])
            && oriented_dot_sign(contact_outward, vector) == Some(support[index]);
    }
    Ok(if !coherent {
        ShellOrientation::Invalid
    } else if support[0] > 0 {
        ShellOrientation::Positive
    } else {
        ShellOrientation::Negative
    })
}

/// Strict radial side of one source-circle span against an exact parallel
/// cylinder. Every distance is `|d x a|² / |a|²`; no authored frame x/y
/// projection is trusted as an orthogonal basis.
fn strict_secant_span_side(
    cylinder: Cylinder,
    circle: kgeom::curve::Circle,
    range: kgeom::param::ParamRange,
    portal_circle: kgeom::curve::Circle,
    endpoints_distinct: bool,
) -> Option<SecantRadialSide> {
    if !endpoints_distinct
        || !vectors_are_exactly_parallel(circle.frame().z(), cylinder.frame().z())
        || !vectors_are_exactly_parallel(portal_circle.frame().z(), cylinder.frame().z())
        || portal_circle.radius().to_bits() != cylinder.radius().to_bits()
    {
        return None;
    }
    let portal_distance = axis_distance_squared(
        portal_circle.frame().origin(),
        cylinder.frame().origin(),
        cylinder.frame().z(),
    )?;
    if !finite(portal_distance)
        || portal_distance.hi() > Interval::point(LINEAR_RESOLUTION).square().lo()
    {
        return None;
    }

    let center_distance = axis_distance_squared(
        circle.frame().origin(),
        cylinder.frame().origin(),
        cylinder.frame().z(),
    )?;
    let host_radius = Interval::point(cylinder.radius());
    let profile_radius = Interval::point(circle.radius());
    if !finite(center_distance)
        || center_distance.lo() <= (host_radius - profile_radius).square().hi()
        || center_distance.hi() >= (host_radius + profile_radius).square().lo()
    {
        return None;
    }

    let midpoint = range.lo / 2.0 + range.hi / 2.0;
    if !midpoint.is_finite() || midpoint <= range.lo || midpoint >= range.hi {
        return None;
    }
    let point = interval_circle_point(circle, midpoint)?;
    let radial = interval_axis_distance_squared(
        point,
        interval_point(cylinder.frame().origin().to_array()),
        interval_point(cylinder.frame().z().to_array()),
    )?;
    let radius_squared = host_radius.square();
    if radial.hi() < radius_squared.lo() {
        Some(SecantRadialSide::Inside)
    } else if radial.lo() > radius_squared.hi() {
        Some(SecantRadialSide::Outside)
    } else {
        None
    }
}

fn interval_circle_point(circle: kgeom::curve::Circle, parameter: f64) -> Option<IntervalVec3> {
    let (sine, cosine) = kcore::math::sincos(parameter);
    if !sine.is_finite() || !cosine.is_finite() {
        return None;
    }
    let sine = Interval::new(sine.next_down().next_down(), sine.next_up().next_up());
    let cosine = Interval::new(cosine.next_down().next_down(), cosine.next_up().next_up());
    let center = interval_point(circle.frame().origin().to_array());
    let x = interval_point(circle.frame().x().to_array());
    let y = interval_point(circle.frame().y().to_array());
    let radius = Interval::point(circle.radius());
    let point =
        core::array::from_fn(|axis| center[axis] + radius * (x[axis] * cosine + y[axis] * sine));
    point.into_iter().all(finite).then_some(point)
}

/// `N = 1 + F + L + U` bounds every role scan. `N² + 32N` covers the two
/// host preparations, incidence joins, loop proofs, radial comparisons, and
/// orientation checks without allocating candidate topology.
fn proof_work(store: &Store, shell_id: ShellId) -> Result<Option<u64>> {
    let shell = store.get(shell_id)?;
    let mut loops = 0_u64;
    let mut uses = 0_u64;
    for &face_id in &shell.faces {
        for &loop_id in &store.get(face_id)?.loops {
            loops = match loops.checked_add(1) {
                Some(value) => value,
                None => return Ok(None),
            };
            let loop_ = store.get(loop_id)?;
            let Some(loop_uses) = u64::try_from(loop_.fins.len()).ok() else {
                return Ok(None);
            };
            uses = match uses.checked_add(loop_uses) {
                Some(value) => value,
                None => return Ok(None),
            };
        }
    }
    let Some(faces) = u64::try_from(shell.faces.len()).ok() else {
        return Ok(None);
    };
    let Some(size) = 1_u64
        .checked_add(faces)
        .and_then(|value| value.checked_add(loops))
        .and_then(|value| value.checked_add(uses))
    else {
        return Ok(None);
    };
    Ok(size
        .checked_mul(size)
        .and_then(|quadratic| quadratic.checked_add(size.checked_mul(32)?)))
}

fn prepare_band(
    store: &Store,
    shell_id: ShellId,
    face_id: FaceId,
    cylinder: Cylinder,
) -> Result<Option<ContactBand>> {
    let face = store.get(face_id)?;
    if face.loops.len() != 2
        || certify_face_loop_layout(store, face_id)? != LoopContainment::Certified
    {
        return Ok(None);
    }
    let mut far = None;
    let mut contact = None;
    for &loop_id in &face.loops {
        let Some(boundary) = prepare_boundary(store, shell_id, face_id, cylinder, loop_id)? else {
            return Ok(None);
        };
        let cap = store.get(boundary.cap_face)?;
        let target = match cap.loops.len() {
            1 => &mut far,
            2 => &mut contact,
            _ => return Ok(None),
        };
        if target.replace(boundary).is_some() {
            return Ok(None);
        }
    }
    let (Some(far), Some(contact)) = (far, contact) else {
        return Ok(None);
    };
    if far.edge == contact.edge || far.cap_face == contact.cap_face {
        return Ok(None);
    }
    Ok(Some(ContactBand {
        face: face_id,
        cylinder,
        far,
        contact,
    }))
}

fn prepare_boundary(
    store: &Store,
    shell_id: ShellId,
    side_face: FaceId,
    cylinder: Cylinder,
    side_loop_id: LoopId,
) -> Result<Option<RingBoundary>> {
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
    if edge.tolerance.is_some()
        || edge.bounds.is_some()
        || edge.vertices != [None, None]
        || side_fin.parent != side_loop_id
        || cap_fin.edge != side_fin.edge
        || cap_fin.sense == side_fin.sense
        || cap_loop.fins.as_slice() != [cap_fin_id]
        || cap_face.shell != shell_id
        || !cap_face.loops.contains(&cap_loop_id)
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
        || certify_face_loop_layout(store, cap_face_id)? != LoopContainment::Certified
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
        || side_use
            .closure_winding()
            .is_none_or(|value| value[0] == 0 || value[1] != 0)
        || cap_use.closure_winding() != Some([0, 0])
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
    Ok(Some(RingBoundary {
        side_loop: side_loop_id,
        cap_face: cap_face_id,
        edge: side_fin.edge,
        circle: *circle,
        axial_parameter: side_line.origin().y,
        cap_axis_alignment,
        side_traverses_positive_u,
    }))
}

#[derive(Debug, Clone, Copy)]
enum BandRole {
    Outer,
    Inner,
}

fn band_orientation_valid(store: &Store, band: &ContactBand, role: BandRole) -> Result<bool> {
    let side = store.get(band.face)?;
    if side.sense != Sense::Forward {
        return Ok(false);
    }
    let (low, high) = if band.far.axial_parameter < band.contact.axial_parameter {
        (band.far, band.contact)
    } else if band.contact.axial_parameter < band.far.axial_parameter {
        (band.contact, band.far)
    } else {
        return Ok(false);
    };
    if !low.side_traverses_positive_u || high.side_traverses_positive_u {
        return Ok(false);
    }
    let far_cap = store.get(band.far.cap_face)?;
    let far_outward = oriented_axis_alignment(band.far.cap_axis_alignment, far_cap.sense);
    let contact_cap = store.get(band.contact.cap_face)?;
    let contact_outward =
        oriented_axis_alignment(band.contact.cap_axis_alignment, contact_cap.sense);
    let far_is_low = band.far.axial_parameter < band.contact.axial_parameter;
    let expected_far = if far_is_low { -1 } else { 1 };
    let expected_contact = match role {
        BandRole::Outer => -expected_far,
        BandRole::Inner => expected_far,
    };
    Ok(far_outward == Some(expected_far) && contact_outward == Some(expected_contact))
}

fn strictly_contains(outer: kgeom::curve::Circle, inner: kgeom::curve::Circle) -> bool {
    if outer.radius() <= inner.radius()
        || !vectors_are_exactly_parallel(outer.frame().z(), inner.frame().z())
    {
        return false;
    }
    let distance = interval_distance_squared(inner.frame().origin(), outer.frame().origin());
    let clearance = Interval::point(outer.radius())
        - Interval::point(inner.radius())
        - Interval::point(2.0 * LINEAR_RESOLUTION);
    finite(distance)
        && finite(clearance)
        && clearance.lo() > 0.0
        && distance.hi() < clearance.square().lo()
}

fn interval_distance_squared(point: Point3, origin: Point3) -> Interval {
    point.to_array().into_iter().zip(origin.to_array()).fold(
        Interval::point(0.0),
        |sum, (point, origin)| {
            let point = Interval::point(point);
            let origin = Interval::point(origin);
            sum + point.square() - Interval::point(2.0) * point * origin + origin.square()
        },
    )
}

fn finite(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite()
}

fn circle_on_cylinder(circle: kgeom::curve::Circle, cylinder: Cylinder) -> bool {
    if circle.radius().to_bits() != cylinder.radius().to_bits()
        || !vectors_are_exactly_parallel(circle.frame().z(), cylinder.frame().z())
    {
        return false;
    }
    point_is_within_axis_envelope(
        circle.frame().origin(),
        cylinder.frame().origin(),
        *cylinder.frame(),
        LINEAR_RESOLUTION,
    )
}

fn vectors_are_exactly_parallel(first: Vec3, second: Vec3) -> bool {
    if first == second || first == -second {
        return true;
    }
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        .into_iter()
        .all(|basis| {
            orient3d(first.to_array(), second.to_array(), basis, [0.0; 3])
                == PredicateOrientation::Zero
        })
}

fn strictly_contains_cylinder_support(outer: Cylinder, inner: Cylinder) -> bool {
    if outer.radius() <= inner.radius()
        || !vectors_are_exactly_parallel(outer.frame().z(), inner.frame().z())
    {
        return false;
    }
    let Some(distance) = axis_distance_squared(
        inner.frame().origin(),
        outer.frame().origin(),
        outer.frame().z(),
    ) else {
        return false;
    };
    let clearance = Interval::point(outer.radius())
        - Interval::point(inner.radius())
        - Interval::point(2.0 * LINEAR_RESOLUTION);
    finite(distance)
        && finite(clearance)
        && clearance.lo() > 0.0
        && distance.hi() < clearance.square().lo()
}

fn point_is_within_axis_envelope(
    point: Point3,
    origin: Point3,
    frame: Frame,
    envelope: f64,
) -> bool {
    let Some(distance) = axis_distance_squared(point, origin, frame.z()) else {
        return false;
    };
    let allowed = Interval::point(envelope).square();
    finite(distance) && distance.hi() <= allowed.lo()
}

fn axis_distance_squared(point: Point3, origin: Point3, axis: Vec3) -> Option<Interval> {
    interval_axis_distance_squared(
        interval_point(point.to_array()),
        interval_point(origin.to_array()),
        interval_point(axis.to_array()),
    )
}

fn interval_axis_distance_squared(
    point: IntervalVec3,
    origin: IntervalVec3,
    axis: IntervalVec3,
) -> Option<Interval> {
    let displacement = interval_sub(point, origin);
    let cross = interval_cross(displacement, axis);
    interval_dot(cross, cross).checked_div(interval_dot(axis, axis))
}

fn point_is_within_plane_envelope(
    point: Point3,
    origin: Point3,
    normal: Vec3,
    envelope: f64,
) -> bool {
    let distance = interval_affine_projection(normal, point, origin).square();
    let allowed = Interval::point(envelope).square();
    finite(distance) && distance.hi() <= allowed.lo()
}

fn interval_affine_projection(axis: Vec3, point: Point3, origin: Point3) -> Interval {
    axis.to_array()
        .into_iter()
        .zip(point.to_array())
        .zip(origin.to_array())
        .fold(Interval::point(0.0), |sum, ((axis, point), origin)| {
            let axis = Interval::point(axis);
            sum + axis * Interval::point(point) - axis * Interval::point(origin)
        })
}

fn all_faces_consumed(store: &Store, shell_id: ShellId, roles: &[FaceId]) -> Result<bool> {
    let faces = &store.get(shell_id)?.faces;
    Ok(roles.len() == faces.len()
        && !roles
            .iter()
            .enumerate()
            .any(|(index, role)| roles[index + 1..].contains(role))
        && roles.iter().all(|role| faces.contains(role)))
}
