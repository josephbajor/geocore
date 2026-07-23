//! Shell theorem for one bounded nonparallel-cylinder lobe.
//!
//! The representation class has two parallel analytic cap digons and two
//! Cylinder quadrilaterals. Two persistent skew-cylinder members are the
//! longitudinal boundaries; four exact Line/Circle cap edges close them.
//! Local manifold topology is not omission evidence: the embedding theorem
//! is available only when both persistent edges resolve to one sealed,
//! complete finite-window family and every other member is certified outside
//! both Cylinder face interiors.

use super::mixed_profile_prism_proof::{
    Cap, ProfileCarrier, certified_parallel, oriented_dot_sign, peer_face, prepare_cap,
};
use super::{ShellCertification, ShellEmbedding, ShellOrientation};
use crate::entity::{EdgeId, FaceId, FinId, LoopId, ParamMap1d, Sense, ShellId, VertexId};
use crate::geom::SurfaceGeom;
use crate::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};
use crate::loop_proof::bounded_pcurve_integral::{
    BoundedPcurveSpan, certify_bounded_pcurve_span_integral,
};
use crate::loop_proof::{certify_periodic_aabb2_separation, certify_periodic_range_window_lift};
use crate::store::Store;
use kcore::error::Result;
use kcore::interval::Interval;
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationScope, ResourceKind, StageId,
};
use kcore::predicates::{Orientation as PredicateOrientation, affine_dot3};
use kcore::tolerance::{ANGULAR_RESOLUTION, LINEAR_RESOLUTION};
use kgeom::aabb::Aabb2;
use kgeom::param::ParamRange;
use kgeom::surface::Surface;
use kgeom::vec::{Point3, Vec2, Vec3};
use kgraph::{
    PersistentSkewCylinderAxialBoundTag, PersistentSkewCylinderAxialBoundary,
    PersistentSkewCylinderDirectedChartIntegralCertificate,
    PersistentSkewCylinderFiniteWindowFamilyCertificate,
    PersistentSkewCylinderFiniteWindowSheetOccupancy, PersistentSkewCylinderOpenSpanOrientation,
    PersistentSkewCylinderSpanRangeOrder, PersistentSkewCylinderSpanRelationshipCertificate,
    PersistentSkewCylinderSpanRelationshipRequest, SkewCylinderSheet,
    VerifiedSkewCylinderOpenSpanCurveDescriptor,
    certify_persistent_skew_cylinder_span_relationship,
};

#[path = "bounded_skew_lobe_shell_proof/window_witness.rs"]
mod window_witness;
use window_witness::complete_family_window_witness;
#[path = "bounded_skew_lobe_shell_proof/property_witness.rs"]
mod property_witness;
pub(crate) use property_witness::{
    BoundedSkewLobePropertyWitness, certify_bounded_skew_lobe_property_witness,
};

/// Cumulative deterministic work for the bounded-skew lobe theorem.
pub(crate) const BOUNDED_SKEW_LOBE_SHELL_WORK: StageId =
    match StageId::new("ktopo.check.bounded-skew-lobe-shell-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid bounded-skew lobe shell work stage"),
    };

const DEFAULT_BOUNDED_SKEW_LOBE_SHELL_WORK: u64 = 4_096;

pub(crate) fn bounded_skew_lobe_shell_proof_budget() -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(
        BOUNDED_SKEW_LOBE_SHELL_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        DEFAULT_BOUNDED_SKEW_LOBE_SHELL_WORK,
    )])
    .expect("built-in bounded-skew lobe proof budget is valid")
}

#[derive(Debug, Clone, Copy)]
struct PersistentBoundary {
    edge: EdgeId,
    descriptor: VerifiedSkewCylinderOpenSpanCurveDescriptor,
}

#[derive(Debug)]
struct LobeTopology {
    cylinders: [FaceId; 2],
    caps: [Cap; 2],
    cylinder_loops: [LoopId; 2],
    persistent: [PersistentBoundary; 2],
}

#[derive(Debug, Clone, Copy)]
struct TaggedVertex {
    vertex: VertexId,
    tag: PersistentSkewCylinderAxialBoundTag,
    bound: f64,
}

#[derive(Debug, Clone, Copy)]
struct CompleteFamily {
    ordered: [PersistentBoundary; 2],
    relationship: PersistentSkewCylinderSpanRelationshipCertificate,
    source_faces: [FaceId; 2],
    cap_slab: CapSlab,
}

#[derive(Debug, Clone, Copy)]
struct CapSlab {
    source_slot: usize,
    orientation: ShellOrientation,
}

/// Attempt the bounded-skew lobe theorem.
pub(super) fn certify_bounded_skew_lobe_shell(
    store: &Store,
    shell_id: ShellId,
    scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellCertification>> {
    let Some(topology) = recognize_lobe_topology(store, shell_id)? else {
        return Ok(None);
    };
    if let Some(scope) = scope {
        let Some(work) = proof_work(store, shell_id)? else {
            return Ok(Some(indeterminate()));
        };
        charge_proof_work(scope, work)?;
    }
    let Some(family) = resolve_complete_family(store, &topology)? else {
        return Ok(Some(indeterminate()));
    };
    let Some(orientations) = cylinder_loop_orientations(store, &topology, family)? else {
        return Ok(Some(indeterminate()));
    };
    let locally_coherent = topology.caps.iter().all(|cap| cap.local_orientation_valid)
        && family
            .source_faces
            .iter()
            .copied()
            .zip(orientations)
            .all(|(face, orientation)| {
                (orientation == PredicateOrientation::Positive)
                    == store
                        .get(face)
                        .is_ok_and(|face| face.sense == Sense::Forward)
            });
    Ok(Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: if locally_coherent {
            family.cap_slab.orientation
        } else {
            ShellOrientation::Invalid
        },
    }))
}

/// Specialized loop result used by the Full loop checker.
///
/// Returning `Some` means the whole shell theorem resolved its complete
/// family and this loop is one of the two certified Cylinder quadrilaterals.
pub(crate) fn certify_bounded_skew_cylinder_loop(
    store: &Store,
    loop_id: LoopId,
) -> Result<Option<PredicateOrientation>> {
    let loop_ = store.get(loop_id)?;
    let face = store.get(loop_.face)?;
    if !matches!(store.get(face.surface)?, SurfaceGeom::Cylinder(_)) {
        return Ok(None);
    }
    let Some(topology) = recognize_lobe_topology(store, face.shell)? else {
        return Ok(None);
    };
    let Some(family) = resolve_complete_family(store, &topology)? else {
        return Ok(None);
    };
    let Some(orientations) = cylinder_loop_orientations(store, &topology, family)? else {
        return Ok(None);
    };
    let Some(index) = family
        .source_faces
        .iter()
        .position(|candidate| *candidate == loop_.face)
    else {
        return Ok(None);
    };
    if loop_for_face(&topology, loop_.face)? != loop_id {
        return Ok(None);
    }
    Ok(Some(orientations[index]))
}

/// Detect the exact representation class without assigning geometric roles.
///
/// `None` is not a negative geometric result. It means either that this
/// theorem is inapplicable or that one of its proof obligations is absent.
fn recognize_lobe_topology(store: &Store, shell_id: ShellId) -> Result<Option<LobeTopology>> {
    let shell = store.get(shell_id)?;
    if shell.faces.len() != 4 || !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(None);
    }
    let mut planes = Vec::with_capacity(2);
    let mut cylinders = Vec::with_capacity(2);
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        if face.shell != shell_id {
            return Ok(None);
        }
        match store.get(face.surface)? {
            SurfaceGeom::Plane(_) => planes.push(face_id),
            SurfaceGeom::Cylinder(_) => cylinders.push(face_id),
            _ => return Ok(None),
        }
    }
    let (Ok(planes), Ok(cylinders)) = (
        <[FaceId; 2]>::try_from(planes),
        <[FaceId; 2]>::try_from(cylinders),
    ) else {
        return Ok(None);
    };
    let Some(boundaries) = face_boundaries(store, planes, cylinders)? else {
        return Ok(None);
    };
    let Some((edges, _vertices)) = closed_manifold_signature(store, &boundaries)? else {
        return Ok(None);
    };
    let Some(persistent) = persistent_boundaries(store, edges, cylinders)? else {
        return Ok(None);
    };
    let Some(caps) = prepare_cap_digons(store, planes, cylinders)? else {
        return Ok(None);
    };
    Ok(Some(LobeTopology {
        cylinders,
        caps,
        cylinder_loops: [boundaries[2].0, boundaries[3].0],
        persistent,
    }))
}

type FaceBoundary = (LoopId, Vec<FinId>, Vec<EdgeId>);

fn face_boundaries(
    store: &Store,
    planes: [FaceId; 2],
    cylinders: [FaceId; 2],
) -> Result<Option<[FaceBoundary; 4]>> {
    let faces = [planes[0], planes[1], cylinders[0], cylinders[1]];
    let mut output = Vec::with_capacity(4);
    for (index, face_id) in faces.into_iter().enumerate() {
        let [loop_id] = store.get(face_id)?.loops.as_slice() else {
            return Ok(None);
        };
        let loop_ = store.get(*loop_id)?;
        let expected = if index < 2 { 2 } else { 4 };
        if loop_.face != face_id || loop_.fins.len() != expected {
            return Ok(None);
        }
        let mut edges = Vec::with_capacity(expected);
        for fin_index in 0..loop_.fins.len() {
            let fin_id = loop_.fins[fin_index];
            let next_id = loop_.fins[(fin_index + 1) % loop_.fins.len()];
            let (Some(head), Some(next_tail)) = (store.fin_head(fin_id)?, store.fin_tail(next_id)?)
            else {
                return Ok(None);
            };
            let edge = store.get(fin_id)?.edge;
            let effective = store
                .get(edge)?
                .tolerance
                .map(crate::tolerance::EntityTolerance::value)
                .unwrap_or(0.0)
                .max(LINEAR_RESOLUTION);
            if head != next_tail
                || edges.contains(&edge)
                || certify_whole_fin_incidence(store, face_id, *loop_id, fin_id, effective)
                    != WholeFinIncidence::Certified
            {
                return Ok(None);
            }
            edges.push(edge);
        }
        output.push((*loop_id, loop_.fins.clone(), edges));
    }
    Ok(output.try_into().ok())
}

fn closed_manifold_signature(
    store: &Store,
    boundaries: &[FaceBoundary; 4],
) -> Result<Option<([EdgeId; 6], [VertexId; 4])>> {
    let mut edges = Vec::with_capacity(6);
    let mut vertices = Vec::with_capacity(4);
    for (_, _, face_edges) in boundaries {
        for &edge_id in face_edges {
            if !edges.contains(&edge_id) {
                edges.push(edge_id);
                let edge = store.get(edge_id)?;
                let [Some(first), Some(second)] = edge.vertices else {
                    return Ok(None);
                };
                if first == second
                    || edge.bounds.is_none()
                    || edge.fins.len() != 2
                    || store.get(edge.fins[0])?.sense == store.get(edge.fins[1])?.sense
                {
                    return Ok(None);
                }
                for vertex in [first, second] {
                    if !vertices.contains(&vertex) {
                        vertices.push(vertex);
                    }
                }
            }
        }
    }
    if edges.len() != 6 || vertices.len() != 4 {
        return Ok(None);
    }
    for first in 0..boundaries.len() {
        for second in first + 1..boundaries.len() {
            let actual = boundaries[first]
                .2
                .iter()
                .filter(|edge| boundaries[second].2.contains(edge))
                .count();
            let expected = match (first, second) {
                (0, 1) => 0,
                (2, 3) => 2,
                _ => 1,
            };
            if actual != expected {
                return Ok(None);
            }
        }
    }
    for &vertex in &vertices {
        if edges
            .iter()
            .filter(|edge| {
                store.get(**edge).is_ok_and(|edge| {
                    edge.vertices
                        .into_iter()
                        .flatten()
                        .any(|value| value == vertex)
                })
            })
            .count()
            != 3
        {
            return Ok(None);
        }
    }
    if !vertex_links_are_cycles(store, boundaries, &edges, &vertices)? {
        return Ok(None);
    }
    Ok(Some((
        edges.try_into().expect("length checked"),
        vertices.try_into().expect("length checked"),
    )))
}

fn vertex_links_are_cycles(
    store: &Store,
    boundaries: &[FaceBoundary; 4],
    edges: &[EdgeId],
    vertices: &[VertexId],
) -> Result<bool> {
    let mut shell_faces = Vec::with_capacity(4);
    for (loop_id, _, _) in boundaries {
        let face = store.get(*loop_id)?.face;
        if shell_faces.contains(&face) {
            return Ok(false);
        }
        shell_faces.push(face);
    }
    for &vertex in vertices {
        let incident = edges
            .iter()
            .copied()
            .filter(|edge| {
                store.get(*edge).is_ok_and(|edge| {
                    edge.vertices
                        .into_iter()
                        .flatten()
                        .any(|candidate| candidate == vertex)
                })
            })
            .collect::<Vec<_>>();
        if incident.len() != 3 {
            return Ok(false);
        }
        let mut link_edges = Vec::with_capacity(3);
        for edge_id in incident {
            let edge = store.get(edge_id)?;
            let mut faces = Vec::with_capacity(2);
            for &fin_id in &edge.fins {
                let fin = store.get(fin_id)?;
                let loop_ = store.get(fin.parent)?;
                if fin.edge != edge_id
                    || !loop_.fins.contains(&fin_id)
                    || !shell_faces.contains(&loop_.face)
                    || faces.contains(&loop_.face)
                {
                    return Ok(false);
                }
                faces.push(loop_.face);
            }
            let [first, second] = faces.as_slice() else {
                return Ok(false);
            };
            let pair = (*first, *second);
            if link_edges
                .iter()
                .any(|prior| same_unordered_face_pair(*prior, pair))
            {
                return Ok(false);
            }
            link_edges.push(pair);
        }
        let mut link_vertices = Vec::with_capacity(3);
        for &(first, second) in &link_edges {
            for face in [first, second] {
                if !link_vertices.contains(&face) {
                    link_vertices.push(face);
                }
            }
        }
        if link_vertices.len() != 3
            || link_vertices.iter().any(|face| {
                link_edges
                    .iter()
                    .filter(|(first, second)| first == face || second == face)
                    .count()
                    != 2
            })
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn same_unordered_face_pair(first: (FaceId, FaceId), second: (FaceId, FaceId)) -> bool {
    first == second || (first.0 == second.1 && first.1 == second.0)
}

fn persistent_boundaries(
    store: &Store,
    edges: [EdgeId; 6],
    cylinder_faces: [FaceId; 2],
) -> Result<Option<[PersistentBoundary; 2]>> {
    let cylinder_surfaces = cylinder_faces.map(|face| store.get(face).map(|face| face.surface));
    let [first_surface, second_surface] = cylinder_surfaces;
    let (Ok(first_surface), Ok(second_surface)) = (first_surface, second_surface) else {
        return Ok(None);
    };
    let mut persistent = Vec::with_capacity(2);
    for edge_id in edges {
        let edge = store.get(edge_id)?;
        let Some(curve_id) = edge.curve else { continue };
        let Some(descriptor) = store
            .get(curve_id)?
            .as_persistent_skew_cylinder_open_span()
            .copied()
        else {
            continue;
        };
        let mut adjacent = Vec::with_capacity(2);
        for &fin_id in &edge.fins {
            let loop_id = store.get(fin_id)?.parent;
            adjacent.push(store.get(loop_id)?.face);
        }
        let sources = descriptor.source_surfaces();
        if adjacent.len() != 2
            || adjacent[0] == adjacent[1]
            || !cylinder_faces.iter().all(|face| adjacent.contains(face))
            || !sources.contains(&first_surface)
            || !sources.contains(&second_surface)
            || sources[0] == sources[1]
        {
            return Ok(None);
        }
        persistent.push(PersistentBoundary {
            edge: edge_id,
            descriptor,
        });
    }
    Ok(persistent.try_into().ok())
}

fn prepare_cap_digons(
    store: &Store,
    planes: [FaceId; 2],
    cylinders: [FaceId; 2],
) -> Result<Option<[Cap; 2]>> {
    let mut caps = Vec::with_capacity(2);
    for face in planes {
        let Some(cap) = prepare_cap(store, face)? else {
            return Ok(None);
        };
        let mut line_count = 0;
        let mut circle_count = 0;
        let mut peers = Vec::with_capacity(2);
        for use_ in &cap.uses {
            match use_.carrier {
                ProfileCarrier::Line(_) => line_count += 1,
                ProfileCarrier::Circle(_) => circle_count += 1,
            }
            let Some(peer) = peer_face(store, *use_)? else {
                return Ok(None);
            };
            peers.push(peer);
        }
        if line_count != 1
            || circle_count != 1
            || peers.len() != 2
            || peers[0] == peers[1]
            || !cylinders.iter().all(|face| peers.contains(face))
        {
            return Ok(None);
        }
        caps.push(cap);
    }
    Ok(caps.try_into().ok())
}

fn resolve_complete_family(
    store: &Store,
    topology: &LobeTopology,
) -> Result<Option<CompleteFamily>> {
    let memberships = topology.persistent.map(|boundary| {
        boundary
            .descriptor
            .certificate()
            .finite_window_family_membership()
    });
    let [Some(first_membership), Some(second_membership)] = memberships else {
        return Ok(None);
    };
    let family = first_membership.family();
    if second_membership.family() != family
        || first_membership.ordinal() == second_membership.ordinal()
        || [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper]
            .into_iter()
            .any(|sheet| {
                family.sheet_occupancy(sheet)
                    == PersistentSkewCylinderFiniteWindowSheetOccupancy::Whole
            })
    {
        return Ok(None);
    }
    let (earlier, later, earlier_member, later_member) =
        if first_membership.ordinal() < second_membership.ordinal() {
            (
                topology.persistent[0],
                topology.persistent[1],
                first_membership.member(),
                second_membership.member(),
            )
        } else {
            (
                topology.persistent[1],
                topology.persistent[0],
                second_membership.member(),
                first_membership.member(),
            )
        };
    if !selected_members_are_adjacent(
        family.sheet_occupancy(earlier_member.sheet()),
        earlier_member.ordinal(),
        earlier_member.sheet(),
        later_member.ordinal(),
        later_member.sheet(),
    ) {
        return Ok(None);
    }
    let Ok(relationship) = certify_persistent_skew_cylinder_span_relationship(
        earlier.descriptor,
        later.descriptor,
        PersistentSkewCylinderSpanRelationshipRequest::DisjointRange {
            order: PersistentSkewCylinderSpanRangeOrder::FirstBeforeSecond,
        },
    ) else {
        return Ok(None);
    };
    let Some(source_faces) = source_face_order(store, topology, earlier.descriptor)? else {
        return Ok(None);
    };
    if !descriptor_matches_source_faces(store, later.descriptor, source_faces)?
        || family.source_cylinders()
            != source_faces.map(|face| {
                let surface = store.get(face).expect("resolved face remains live").surface;
                match store.get(surface).expect("resolved surface remains live") {
                    SurfaceGeom::Cylinder(cylinder) => *cylinder,
                    _ => unreachable!("resolved source face is cylindrical"),
                }
            })
    {
        return Ok(None);
    }
    let source_cylinders = family.source_cylinders();
    if !certified_nonparallel(
        source_cylinders[0].frame().z(),
        source_cylinders[1].frame().z(),
    ) {
        return Ok(None);
    }
    let Some(cap_slab) = certify_cap_slab(store, topology, family, [earlier, later], source_faces)?
    else {
        return Ok(None);
    };
    if !complete_family_window_witness(
        store,
        topology,
        family,
        source_faces,
        cap_slab,
        [earlier_member.ordinal(), later_member.ordinal()],
    )? {
        return Ok(None);
    }
    Ok(Some(CompleteFamily {
        ordered: [earlier, later],
        relationship,
        source_faces,
        cap_slab,
    }))
}

fn cylinder_loop_orientations(
    store: &Store,
    topology: &LobeTopology,
    family: CompleteFamily,
) -> Result<Option<[PredicateOrientation; 2]>> {
    let integrals = family.relationship.span_directed_chart_integrals();
    let mut output = Vec::with_capacity(2);
    for (source_slot, face_id) in family.source_faces.into_iter().enumerate() {
        let loop_id = loop_for_face(topology, face_id)?;
        let face = store.get(face_id)?;
        let SurfaceGeom::Cylinder(cylinder) = store.get(face.surface)? else {
            return Ok(None);
        };
        if !certify_cylinder_chart_closure(store, face_id, loop_id, *cylinder)? {
            return Ok(None);
        }
        if source_slot == family.cap_slab.source_slot {
            let Some(orientation) = slab_cylinder_loop_orientation(store, loop_id, &family)? else {
                return Ok(None);
            };
            output.push(orientation);
            continue;
        }
        let Some(radial_orientation) = radial_cylinder_loop_orientation(store, loop_id, &family)?
        else {
            return Ok(None);
        };
        let mut stored = Interval::point(0.0);
        let mut source = Interval::point(0.0);
        for &fin_id in &store.get(loop_id)?.fins {
            let fin = store.get(fin_id)?;
            let edge = store.get(fin.edge)?;
            let persistent_index = family
                .ordered
                .iter()
                .position(|boundary| boundary.edge == fin.edge);
            let terms = if let Some(span_index) = persistent_index {
                let certificate = family.ordered[span_index].descriptor;
                let Some(term) = persistent_fin_integrals(
                    store,
                    fin_id,
                    source_slot,
                    certificate,
                    integrals[span_index][source_slot],
                )?
                else {
                    return Ok(None);
                };
                term
            } else {
                let Some(term) = analytic_fin_integral(store, fin_id, *cylinder)? else {
                    return Ok(None);
                };
                [term, term]
            };
            stored = stored + terms[0];
            source = source + terms[1];
            if !finite_interval(stored) || !finite_interval(source) || edge.fins.len() != 2 {
                return Ok(None);
            }
        }
        if [stored, source]
            .into_iter()
            .filter_map(strict_interval_sign)
            .any(|sign| sign != radial_orientation)
        {
            return Ok(None);
        }
        output.push(radial_orientation);
    }
    Ok(output.try_into().ok())
}

/// Derive the slab Cylinder orientation from its two exact axial boundaries.
///
/// The sealed family and cap theorem already prove that both longitudinal
/// edges bound the same finite source-window slab. On that source Cylinder,
/// the remaining two fins must therefore be exact horizontal `Line2d`
/// boundaries carrying opposite family endpoint tags. Lower-increasing and
/// upper-decreasing traversal is positive in the Cylinder `(u, v)` chart.
fn slab_cylinder_loop_orientation(
    store: &Store,
    loop_id: LoopId,
    family: &CompleteFamily,
) -> Result<Option<PredicateOrientation>> {
    let Some(tagged) = tagged_persistent_vertices(store, family.ordered)? else {
        return Ok(None);
    };
    let mut boundaries = Vec::with_capacity(2);
    let mut persistent_count = 0;
    for &fin_id in &store.get(loop_id)?.fins {
        let fin = store.get(fin_id)?;
        if family
            .ordered
            .iter()
            .any(|boundary| boundary.edge == fin.edge)
        {
            persistent_count += 1;
            continue;
        }
        let Some(tagged_edge) = common_edge_tag(store, fin.edge, &tagged)? else {
            return Ok(None);
        };
        if tagged_edge.tag.source_slot() != family.cap_slab.source_slot {
            return Ok(None);
        }
        let Some(use_) = fin.pcurve else {
            return Ok(None);
        };
        let crate::geom::Curve2dGeom::Line(line) = store.get(use_.curve())? else {
            return Ok(None);
        };
        let direction = line.dir();
        let scale = use_.edge_to_pcurve().scale();
        let shifts = use_.chart().period_shifts();
        if !direction.x.is_finite()
            || direction.x == 0.0
            || direction.y != 0.0
            || !scale.is_finite()
            || scale == 0.0
            || line.origin().y.to_bits() != tagged_edge.bound.to_bits()
            || use_.closure_winding().is_some()
            || use_.seam().is_some()
            || shifts[1] != 0
        {
            return Ok(None);
        }
        let increasing_with_edge = (direction.x > 0.0) == (scale > 0.0);
        let increasing = if fin.sense == Sense::Forward {
            increasing_with_edge
        } else {
            !increasing_with_edge
        };
        boundaries.push((
            tagged_edge.tag.boundary(),
            slab_boundary_orientation(tagged_edge.tag.boundary(), increasing),
        ));
    }
    let [(first_boundary, first), (second_boundary, second)] = boundaries.as_slice() else {
        return Ok(None);
    };
    Ok(
        (persistent_count == 2 && first_boundary != second_boundary && first == second)
            .then_some(*first),
    )
}

fn radial_cylinder_loop_orientation(
    store: &Store,
    loop_id: LoopId,
    family: &CompleteFamily,
) -> Result<Option<PredicateOrientation>> {
    let Some(tagged) = tagged_persistent_vertices(store, family.ordered)? else {
        return Ok(None);
    };
    let mut boundaries = Vec::with_capacity(2);
    let mut persistent_count = 0;
    for &fin_id in &store.get(loop_id)?.fins {
        let fin = store.get(fin_id)?;
        if family
            .ordered
            .iter()
            .any(|boundary| boundary.edge == fin.edge)
        {
            persistent_count += 1;
            continue;
        }
        let Some(tagged_edge) = common_edge_tag(store, fin.edge, &tagged)? else {
            return Ok(None);
        };
        let Some(use_) = fin.pcurve else {
            return Ok(None);
        };
        let crate::geom::Curve2dGeom::Line(line) = store.get(use_.curve())? else {
            return Ok(None);
        };
        let direction = line.dir();
        let scale = use_.edge_to_pcurve().scale();
        let shifts = use_.chart().period_shifts();
        if tagged_edge.tag.source_slot() != family.cap_slab.source_slot
            || direction.x != 0.0
            || !direction.y.is_finite()
            || direction.y == 0.0
            || !scale.is_finite()
            || scale == 0.0
            || use_.closure_winding().is_some()
            || use_.seam().is_some()
            || shifts[1] != 0
        {
            return Ok(None);
        }
        let longitude = Interval::point(line.origin().x)
            + Interval::point(f64::from(shifts[0])) * Interval::point(core::f64::consts::TAU);
        if !finite_interval(longitude) {
            return Ok(None);
        }
        let increasing_with_edge = (direction.y > 0.0) == (scale > 0.0);
        let increasing = if fin.sense == Sense::Forward {
            increasing_with_edge
        } else {
            !increasing_with_edge
        };
        boundaries.push((longitude, tagged_edge.tag.boundary(), increasing));
    }
    let [first, second] = boundaries.as_slice() else {
        return Ok(None);
    };
    if persistent_count != 2 || first.1 == second.1 {
        return Ok(None);
    }
    let (lower, upper) = if first.0.hi() < second.0.lo() {
        (first, second)
    } else if second.0.hi() < first.0.lo() {
        (second, first)
    } else {
        return Ok(None);
    };
    let first_orientation = radial_boundary_orientation(true, lower.2);
    let second_orientation = radial_boundary_orientation(false, upper.2);
    Ok((first_orientation == second_orientation).then_some(first_orientation))
}

fn common_edge_tag(
    store: &Store,
    edge_id: EdgeId,
    tagged: &[TaggedVertex; 4],
) -> Result<Option<TaggedVertex>> {
    let [Some(first_vertex), Some(second_vertex)] = store.get(edge_id)?.vertices else {
        return Ok(None);
    };
    let Some(first) = tagged
        .iter()
        .copied()
        .find(|value| value.vertex == first_vertex)
    else {
        return Ok(None);
    };
    let Some(second) = tagged
        .iter()
        .copied()
        .find(|value| value.vertex == second_vertex)
    else {
        return Ok(None);
    };
    Ok(
        (first.tag == second.tag && first.bound.to_bits() == second.bound.to_bits())
            .then_some(first),
    )
}

fn slab_boundary_orientation(
    boundary: PersistentSkewCylinderAxialBoundary,
    increasing: bool,
) -> PredicateOrientation {
    if increasing == (boundary == PersistentSkewCylinderAxialBoundary::Lower) {
        PredicateOrientation::Positive
    } else {
        PredicateOrientation::Negative
    }
}

fn radial_boundary_orientation(lower_longitude: bool, increasing: bool) -> PredicateOrientation {
    if lower_longitude != increasing {
        PredicateOrientation::Positive
    } else {
        PredicateOrientation::Negative
    }
}

fn persistent_fin_integrals(
    store: &Store,
    fin_id: FinId,
    source_slot: usize,
    descriptor: VerifiedSkewCylinderOpenSpanCurveDescriptor,
    integral: PersistentSkewCylinderDirectedChartIntegralCertificate,
) -> Result<Option<[Interval; 2]>> {
    let fin = store.get(fin_id)?;
    let edge = store.get(fin.edge)?;
    let Some(use_) = fin.pcurve else {
        return Ok(None);
    };
    let shifts = use_.chart().period_shifts();
    if source_slot >= 2
        || edge.curve.is_none()
        || edge.bounds != Some((0.0, 1.0))
        || descriptor.pcurves()[source_slot] != use_.curve()
        || use_.edge_to_pcurve() != ParamMap1d::identity()
        || use_.closure_winding().is_some()
        || use_.seam().is_some()
        || shifts[1] != 0
    {
        return Ok(None);
    }
    let chart_shift = shifts[0] as f64 * core::f64::consts::TAU;
    if !chart_shift.is_finite() {
        return Ok(None);
    }
    let shift = Interval::point(chart_shift);
    let mut stored =
        integral.stored_enclosure() + shift * integral.stored_ordinate_delta_enclosure();
    let mut source =
        integral.source_enclosure() + shift * integral.source_ordinate_delta_enclosure();
    if fin.sense == Sense::Reversed {
        let negative = Interval::point(-1.0);
        stored = negative * stored;
        source = negative * source;
    }
    Ok((finite_interval(stored) && finite_interval(source)).then_some([stored, source]))
}

fn analytic_fin_integral(
    store: &Store,
    fin_id: FinId,
    cylinder: kgeom::surface::Cylinder,
) -> Result<Option<Interval>> {
    let fin = store.get(fin_id)?;
    let edge = store.get(fin.edge)?;
    let (Some((lo, hi)), Some(use_)) = (edge.bounds, fin.pcurve) else {
        return Ok(None);
    };
    let curve = store.get(use_.curve())?;
    if !matches!(
        curve,
        crate::geom::Curve2dGeom::Line(_) | crate::geom::Curve2dGeom::Circle(_)
    ) || use_.closure_winding().is_some()
        || use_.seam().is_some()
    {
        return Ok(None);
    }
    let [edge_start, edge_end] = if fin.sense == Sense::Forward {
        [lo, hi]
    } else {
        [hi, lo]
    };
    let start = use_.edge_to_pcurve().map(edge_start);
    let end = use_.edge_to_pcurve().map(edge_end);
    let chart_offset = use_
        .chart()
        .apply(Vec2::default(), cylinder.periodicity())?;
    Ok(certify_bounded_pcurve_span_integral(
        BoundedPcurveSpan::new(curve, start, end, chart_offset),
    ))
}

fn certify_cylinder_chart_closure(
    store: &Store,
    face_id: FaceId,
    loop_id: LoopId,
    cylinder: kgeom::surface::Cylinder,
) -> Result<bool> {
    let loop_ = store.get(loop_id)?;
    if loop_.face != face_id || loop_.fins.len() != 4 {
        return Ok(false);
    }
    let mut spans = Vec::with_capacity(4);
    for &fin_id in &loop_.fins {
        let fin = store.get(fin_id)?;
        let edge = store.get(fin.edge)?;
        let (Some((lo, hi)), Some(use_)) = (edge.bounds, fin.pcurve) else {
            return Ok(false);
        };
        let curve = store.get(use_.curve())?.as_curve();
        let [start, end] = if fin.sense == Sense::Forward {
            [lo, hi]
        } else {
            [hi, lo]
        };
        let periods = cylinder.periodicity();
        let endpoints = [
            use_.evaluate_uv(curve, start, periods)?,
            use_.evaluate_uv(curve, end, periods)?,
        ];
        let tolerance = edge
            .tolerance
            .map(crate::tolerance::EntityTolerance::value)
            .unwrap_or(0.0)
            .max(LINEAR_RESOLUTION);
        spans.push((endpoints, tolerance));
    }
    for index in 0..spans.len() {
        let next = (index + 1) % spans.len();
        let allowance = Interval::point(spans[index].1) + Interval::point(spans[next].1);
        let delta_u = Interval::point(spans[index].0[1].x) - Interval::point(spans[next].0[0].x);
        let delta_v = Interval::point(spans[index].0[1].y) - Interval::point(spans[next].0[0].y);
        let Some(angular_allowance) = allowance.checked_div(Interval::point(cylinder.radius()))
        else {
            return Ok(false);
        };
        if !finite_interval(allowance)
            || !finite_interval(angular_allowance)
            || interval_abs_upper(delta_u) > angular_allowance.lo()
            || interval_abs_upper(delta_v) > allowance.lo()
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn strict_interval_sign(value: Interval) -> Option<PredicateOrientation> {
    if value.lo() > 0.0 {
        Some(PredicateOrientation::Positive)
    } else if value.hi() < 0.0 {
        Some(PredicateOrientation::Negative)
    } else {
        None
    }
}

fn interval_abs_upper(value: Interval) -> f64 {
    value.lo().abs().max(value.hi().abs())
}

fn finite_interval(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite() && value.lo() <= value.hi()
}

fn certified_nonparallel(first: Vec3, second: Vec3) -> bool {
    let cross = interval_norm_squared(first.cross(second));
    let allowed = Interval::point(ANGULAR_RESOLUTION).square()
        * interval_norm_squared(first)
        * interval_norm_squared(second);
    finite_interval(cross) && finite_interval(allowed) && cross.lo() > allowed.hi()
}

fn interval_norm_squared(vector: Vec3) -> Interval {
    [vector.x, vector.y, vector.z]
        .into_iter()
        .map(|value| Interval::point(value).square())
        .fold(Interval::point(0.0), |sum, value| sum + value)
}

fn occupancy_contains_adjacent_pair(
    occupancy: PersistentSkewCylinderFiniteWindowSheetOccupancy,
    first: usize,
    second: usize,
) -> bool {
    let PersistentSkewCylinderFiniteWindowSheetOccupancy::Open {
        first_member_ordinal,
        member_count,
    } = occupancy
    else {
        return false;
    };
    first >= first_member_ordinal
        && second == first + 1
        && second < first_member_ordinal.saturating_add(member_count)
}

fn selected_members_are_adjacent(
    occupancy: PersistentSkewCylinderFiniteWindowSheetOccupancy,
    first_ordinal: usize,
    first_sheet: SkewCylinderSheet,
    second_ordinal: usize,
    second_sheet: SkewCylinderSheet,
) -> bool {
    first_sheet == second_sheet
        && first_ordinal
            .checked_add(1)
            .is_some_and(|expected| expected == second_ordinal)
        && occupancy_contains_adjacent_pair(occupancy, first_ordinal, second_ordinal)
}

fn source_face_order(
    store: &Store,
    topology: &LobeTopology,
    descriptor: VerifiedSkewCylinderOpenSpanCurveDescriptor,
) -> Result<Option<[FaceId; 2]>> {
    let sources = descriptor.source_surfaces();
    let mut output = [topology.cylinders[0]; 2];
    let mut used = [false; 2];
    for (source_slot, source) in sources.into_iter().enumerate() {
        let Some((face_slot, face)) = topology
            .cylinders
            .iter()
            .copied()
            .enumerate()
            .find(|(_, face)| store.get(*face).is_ok_and(|face| face.surface == source))
        else {
            return Ok(None);
        };
        if used[face_slot] {
            return Ok(None);
        }
        used[face_slot] = true;
        output[source_slot] = face;
    }
    Ok(used.into_iter().all(|value| value).then_some(output))
}

fn descriptor_matches_source_faces(
    store: &Store,
    descriptor: VerifiedSkewCylinderOpenSpanCurveDescriptor,
    source_faces: [FaceId; 2],
) -> Result<bool> {
    let expected = [
        store.get(source_faces[0])?.surface,
        store.get(source_faces[1])?.surface,
    ];
    Ok(descriptor.source_surfaces() == expected)
}

fn loop_for_face(topology: &LobeTopology, face: FaceId) -> Result<LoopId> {
    topology
        .cylinders
        .iter()
        .position(|candidate| *candidate == face)
        .map(|index| topology.cylinder_loops[index])
        .ok_or(kcore::error::Error::InvalidGeometry {
            reason: "bounded-skew lobe lost a resolved Cylinder face",
        })
}

fn certify_cap_slab(
    store: &Store,
    topology: &LobeTopology,
    family: PersistentSkewCylinderFiniteWindowFamilyCertificate,
    ordered: [PersistentBoundary; 2],
    source_faces: [FaceId; 2],
) -> Result<Option<CapSlab>> {
    let Some(tagged) = tagged_persistent_vertices(store, ordered)? else {
        return Ok(None);
    };
    let mut cap_tags = Vec::with_capacity(2);
    for cap in &topology.caps {
        let mut matched = Vec::with_capacity(2);
        for &vertex in &cap.vertices {
            let Some(value) = tagged.iter().copied().find(|value| value.vertex == vertex) else {
                return Ok(None);
            };
            matched.push(value);
        }
        let [first, second] = matched.as_slice() else {
            return Ok(None);
        };
        if first.tag != second.tag || first.bound.to_bits() != second.bound.to_bits() {
            return Ok(None);
        }
        cap_tags.push((cap, first.tag, first.bound));
    }
    let [(_, first_tag, _), (_, second_tag, _)] = cap_tags.as_slice() else {
        return Ok(None);
    };
    let Some(source_slot) = common_slab_source_slot(*first_tag, *second_tag) else {
        return Ok(None);
    };
    if source_slot >= source_faces.len() {
        return Ok(None);
    }
    let source_windows = family.source_windows();
    let cylinder = family.source_cylinders()[source_slot];
    let mut signs = Vec::with_capacity(2);
    for &(cap, tag, bound) in &cap_tags {
        if !tagged_bound_matches_window(tag, bound, source_slot, source_windows[source_slot][1])
            || !certified_parallel(cap.plane.frame().z(), cylinder.frame().z())
            || !certified_axial_plane_alignment(cylinder, cap.plane, bound)
        {
            return Ok(None);
        }
        let boundary_direction = match tag.boundary() {
            PersistentSkewCylinderAxialBoundary::Lower => -cylinder.frame().z(),
            PersistentSkewCylinderAxialBoundary::Upper => cylinder.frame().z(),
        };
        let face = store.get(cap.face)?;
        let outward = cap.plane.frame().z() * sense_factor(face.sense);
        let Some(sign) = oriented_dot_sign(outward, boundary_direction) else {
            return Ok(None);
        };
        signs.push(sign);
    }
    let [first_sign, second_sign] = signs.as_slice() else {
        return Ok(None);
    };
    let orientation = if first_sign != second_sign {
        ShellOrientation::Invalid
    } else if *first_sign > 0 {
        ShellOrientation::Positive
    } else {
        ShellOrientation::Negative
    };
    Ok(Some(CapSlab {
        source_slot,
        orientation,
    }))
}

fn common_slab_source_slot(
    first: PersistentSkewCylinderAxialBoundTag,
    second: PersistentSkewCylinderAxialBoundTag,
) -> Option<usize> {
    (first.source_slot() == second.source_slot()
        && first.source_slot() < 2
        && first.boundary() != second.boundary())
    .then_some(first.source_slot())
}

fn tagged_bound_matches_window(
    tag: PersistentSkewCylinderAxialBoundTag,
    bound: f64,
    source_slot: usize,
    axial_window: ParamRange,
) -> bool {
    let expected = match tag.boundary() {
        PersistentSkewCylinderAxialBoundary::Lower => axial_window.lo,
        PersistentSkewCylinderAxialBoundary::Upper => axial_window.hi,
    };
    tag.source_slot() == source_slot && bound.to_bits() == expected.to_bits()
}

fn tagged_persistent_vertices(
    store: &Store,
    boundaries: [PersistentBoundary; 2],
) -> Result<Option<[TaggedVertex; 4]>> {
    let mut output = Vec::with_capacity(4);
    for boundary in boundaries {
        let edge = store.get(boundary.edge)?;
        let [Some(first), Some(second)] = edge.vertices else {
            return Ok(None);
        };
        let certificate = boundary.descriptor.certificate();
        let Some(membership) = certificate.finite_window_family_membership() else {
            return Ok(None);
        };
        let mut endpoints = membership.member().endpoints();
        if certificate.orientation() == PersistentSkewCylinderOpenSpanOrientation::Reversed {
            endpoints.swap(0, 1);
        }
        for (index, (vertex, proof)) in [first, second].into_iter().zip(endpoints).enumerate() {
            if output
                .iter()
                .any(|value: &TaggedVertex| value.vertex == vertex)
                || !point_bits_equal(
                    store.vertex_position(vertex)?,
                    certificate.endpoint_points()[index],
                )
            {
                return Ok(None);
            }
            output.push(TaggedVertex {
                vertex,
                tag: proof.tag(),
                bound: proof.bound(),
            });
        }
    }
    Ok(output.try_into().ok())
}

/// Certify the semantic axial-plane relation after rigid-map roundoff.
///
/// Exact equality remains the first path. A rigid point/vector map followed
/// by `Frame` normalization uses a fixed chain of binary64 products, sums,
/// square roots, and divisions. The factor below conservatively encloses that
/// chain plus this replay's point construction; it scales only with the live
/// operands and is independent of session/model tolerance. Endpoint tags,
/// bit-identical live vertices, cap incidence, and parallel support remain
/// separate mandatory authority.
fn certified_axial_plane_alignment(
    cylinder: kgeom::surface::Cylinder,
    plane: kgeom::surface::Plane,
    bound: f64,
) -> bool {
    const RIGID_FRAME_ROUNDOFF_OPERATIONS: f64 = 256.0;

    if !bound.is_finite() {
        return false;
    }
    let axis = cylinder.frame().z();
    let cylinder_origin = cylinder.frame().origin();
    let cap_origin = plane.frame().origin();
    let expected = cylinder.frame().point_at(0.0, 0.0, bound);
    if exact_plane_side(axis, cap_origin, expected) == Some(PredicateOrientation::Zero) {
        return true;
    }
    let residual = [axis.x, axis.y, axis.z]
        .into_iter()
        .zip([
            cap_origin.x - expected.x,
            cap_origin.y - expected.y,
            cap_origin.z - expected.z,
        ])
        .fold(Interval::point(0.0), |sum, (component, delta)| {
            sum + Interval::point(component) * Interval::point(delta)
        });
    let scale = 1.0
        + bound.abs()
        + [axis.x, axis.y, axis.z]
            .into_iter()
            .zip([
                (cap_origin.x, cylinder_origin.x),
                (cap_origin.y, cylinder_origin.y),
                (cap_origin.z, cylinder_origin.z),
            ])
            .map(|(component, (cap, source))| {
                component.abs() * (cap.abs() + source.abs() + component.abs() * bound.abs())
            })
            .sum::<f64>();
    let roundoff = RIGID_FRAME_ROUNDOFF_OPERATIONS * f64::EPSILON * scale.max(f64::MIN_POSITIVE);
    finite_interval(residual)
        && roundoff.is_finite()
        && roundoff <= LINEAR_RESOLUTION
        && interval_abs_upper(residual) <= roundoff
}

fn exact_plane_side(normal: Vec3, point: Point3, origin: Point3) -> Option<PredicateOrientation> {
    affine_dot3(normal.to_array(), point.to_array(), origin.to_array(), 0.0)
        .map(|value| value.sign())
}

fn point_bits_equal(first: Point3, second: Point3) -> bool {
    first.x.to_bits() == second.x.to_bits()
        && first.y.to_bits() == second.y.to_bits()
        && first.z.to_bits() == second.z.to_bits()
}

fn sense_factor(sense: Sense) -> f64 {
    if sense.is_forward() { 1.0 } else { -1.0 }
}

/// `N² + 16N`, with `N = 1 + F + L + U + E + V`, owns every bounded scan
/// and role/adjacency comparison in this theorem. The already-paid family
/// and persistent-span work is never recharged here.
fn proof_work(store: &Store, shell_id: ShellId) -> Result<Option<u64>> {
    let shell = store.get(shell_id)?;
    let mut loops = 0_u64;
    let mut fins = 0_u64;
    let mut edges = Vec::new();
    let mut vertices = Vec::new();
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
    Ok(proof_work_for_size(size))
}

fn proof_work_for_size(size: u64) -> Option<u64> {
    size.checked_mul(size)
        .and_then(|value| value.checked_add(size.checked_mul(16)?))
}

fn charge_proof_work(scope: &mut OperationScope<'_, '_>, work: u64) -> Result<()> {
    scope.ledger().require_limit(
        BOUNDED_SKEW_LOBE_SHELL_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
    )?;
    scope
        .ledger_mut()
        .charge(BOUNDED_SKEW_LOBE_SHELL_WORK, work)?;
    Ok(())
}

fn cylinder_loop_box(store: &Store, face_id: FaceId, loop_id: LoopId) -> Result<Option<Aabb2>> {
    let face = store.get(face_id)?;
    let SurfaceGeom::Cylinder(cylinder) = store.get(face.surface)? else {
        return Ok(None);
    };
    let periods = cylinder.periodicity();
    let loop_ = store.get(loop_id)?;
    if loop_.face != face_id {
        return Ok(None);
    }
    let mut output = Aabb2::empty();
    for &fin_id in &loop_.fins {
        let fin = store.get(fin_id)?;
        let Some(pcurve) = fin.pcurve else {
            return Ok(None);
        };
        let curve = store.get(pcurve.curve())?.as_curve();
        let bounds = curve.bounding_box(pcurve.range());
        let min = pcurve.chart().apply(bounds.min, periods)?;
        let max = pcurve.chart().apply(bounds.max, periods)?;
        if !finite_uv(min) || !finite_uv(max) || min.x > max.x || min.y > max.y {
            return Ok(None);
        }
        output = output.union(Aabb2 { min, max });
    }
    let width = Interval::point(output.max.x) - Interval::point(output.min.x);
    if output.is_empty()
        || !finite_interval(width)
        || width.lo() <= 0.0
        || width.hi() >= core::f64::consts::TAU
    {
        return Ok(None);
    }
    Ok(Some(output))
}

fn cylinder_face_domain_box(store: &Store, face_id: FaceId) -> Result<Option<Aabb2>> {
    let face = store.get(face_id)?;
    if !matches!(store.get(face.surface)?, SurfaceGeom::Cylinder(_)) {
        return Ok(None);
    }
    let Some(domain) = face.domain else {
        return Ok(None);
    };
    let output = Aabb2 {
        min: Vec2::new(domain.u.lo, domain.v.lo),
        max: Vec2::new(domain.u.hi, domain.v.hi),
    };
    Ok((!output.is_empty() && finite_uv(output.min) && finite_uv(output.max)).then_some(output))
}

fn certify_periodic_u_lift(value: Aabb2, window: ParamRange) -> Option<i64> {
    if value.is_empty()
        || !value.min.x.is_finite()
        || !value.max.x.is_finite()
        || value.min.x > value.max.x
    {
        return None;
    }
    certify_periodic_range_window_lift(
        ParamRange::new(value.min.x, value.max.x),
        window,
        core::f64::consts::TAU,
    )
}

/// A complete omitted member is outside one cylindrical face when either its
/// axial enclosure is strictly separated or every nearest periodic copy of
/// its angular enclosure is strictly separated. Touching remains ambiguous.
fn periodic_member_box_outside_face(member: Aabb2, face: Aabb2) -> bool {
    certify_periodic_aabb2_separation(member, face, core::f64::consts::TAU)
}

fn finite_uv(value: Vec2) -> bool {
    value.x.is_finite() && value.y.is_finite()
}

fn indeterminate() -> ShellCertification {
    ShellCertification {
        embedding: ShellEmbedding::Indeterminate,
        orientation: ShellOrientation::Indeterminate,
    }
}

#[cfg(test)]
#[path = "bounded_skew_lobe_shell_proof/tests.rs"]
mod tests;
