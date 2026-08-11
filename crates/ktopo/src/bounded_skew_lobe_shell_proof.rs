//! Shell theorem for one bounded nonparallel-cylinder lobe.
//!
//! The representation class has two parallel analytic cap digons and two
//! Cylinder quadrilaterals. Two persistent skew-cylinder members are the
//! longitudinal boundaries; four exact Line/Circle cap edges close them.
//! Local manifold topology is not omission evidence: the embedding theorem
//! is available only when both persistent edges resolve to one sealed,
//! complete finite-window family and every other member is certified outside
//! both Cylinder face interiors.

use super::shell_lemmas::proof_work_budget;
use super::shell_lemmas::{
    Cap, ProfileCarrier, certified_parallel, indeterminate, oriented_dot_sign, peer_face,
    prepare_cap,
};
use super::shell_lemmas::{proof_work as quadratic_proof_work, shell_proof_size};
use super::{ShellCertification, ShellEmbedding, ShellOrientation};
use crate::entity::{EdgeId, FaceId, FinId, LoopId, ParamMap1d, Sense, ShellId, VertexId};
use crate::geom::SurfaceGeom;
use crate::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};
use crate::loop_proof::bounded_pcurve_integral::{
    BoundedPcurveSpan, certify_bounded_pcurve_span_integral,
};
use crate::loop_proof::{
    LoopSimplicity, certify_loop_orientation, certify_loop_simplicity,
    certify_periodic_aabb2_separation, certify_periodic_range_window_lift,
};
use crate::store::Store;
use kcore::error::Result;
use kcore::interval::Interval;
use kcore::operation::{AccountingMode, BudgetPlan, OperationScope, ResourceKind, StageId};
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
    PersistentSkewCylinderFiniteWindowRootEventKind,
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

const DEFAULT_BOUNDED_SKEW_LOBE_SHELL_WORK: u64 = 8_192;

pub(crate) fn bounded_skew_lobe_shell_proof_budget() -> BudgetPlan {
    proof_work_budget(
        BOUNDED_SKEW_LOBE_SHELL_WORK,
        DEFAULT_BOUNDED_SKEW_LOBE_SHELL_WORK,
        "built-in bounded-skew lobe proof budget is valid",
    )
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

#[derive(Debug, Clone, Copy)]
struct CornerCylinderFace {
    face: FaceId,
    loop_id: LoopId,
    source_slot: usize,
}

#[derive(Debug, Clone, Copy)]
struct CornerPlaneFace {
    face: FaceId,
    loop_id: LoopId,
    source_slot: usize,
    support_sign: i8,
}

#[derive(Debug)]
struct CornerContactTopology {
    cylinders: [CornerCylinderFace; 3],
    planes: [CornerPlaneFace; 5],
    persistent: [PersistentBoundary; 2],
    relationship: PersistentSkewCylinderSpanRelationshipCertificate,
    host_sign: i8,
}

/// Attempt the bounded-skew lobe theorem.
pub(super) fn certify_bounded_skew_lobe_shell(
    store: &Store,
    shell_id: ShellId,
    scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellCertification>> {
    let topology = recognize_lobe_topology(store, shell_id)?;
    if topology.is_none() {
        return certify_bounded_skew_corner_contact(store, shell_id, scope);
    }
    let topology = topology.expect("checked above");
    if let Some(scope) = scope {
        let Some(work) = bounded_skew_lobe_proof_work(store, shell_id)? else {
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
        return certify_bounded_skew_corner_contact_loop(store, loop_id);
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

/// Certify the Parasolid-aligned finite-window corner representation.
///
/// This remains a branch of the existing bounded-skew theorem.  The live
/// shell must reconstruct one complete two-member finite-window family, both
/// exact isolated events, the source-boundary support inventory, and the
/// 5-cycle links at the two contact vertices.  No Boolean result tag or
/// constructor provenance participates in the decision.
fn certify_bounded_skew_corner_contact(
    store: &Store,
    shell_id: ShellId,
    scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellCertification>> {
    let Some(topology) = recognize_corner_contact_topology(store, shell_id)? else {
        return Ok(None);
    };
    if let Some(scope) = scope {
        let Some(work) = bounded_skew_lobe_proof_work(store, shell_id)? else {
            return Ok(Some(indeterminate()));
        };
        charge_proof_work(scope, work)?;
    }
    let mut locally_coherent = true;
    let mut checked_cylinder_faces = Vec::with_capacity(3);
    for cylinder in topology.cylinders {
        if checked_cylinder_faces.contains(&cylinder.face) {
            continue;
        }
        checked_cylinder_faces.push(cylinder.face);
        let on_face = topology
            .cylinders
            .iter()
            .filter(|candidate| candidate.face == cylinder.face)
            .collect::<Vec<_>>();
        let orientations = on_face
            .iter()
            .map(|candidate| corner_cylinder_loop_orientation(store, &topology, candidate.loop_id))
            .collect::<Result<Option<Vec<_>>>>()?;
        let Some(orientations) = orientations else {
            return Ok(Some(indeterminate()));
        };
        locally_coherent &= match orientations.as_slice() {
            [orientation] => {
                (*orientation == PredicateOrientation::Positive)
                    == store.get(cylinder.face)?.sense.is_forward()
            }
            [first, second] => first != second,
            _ => false,
        };
    }
    for plane in topology.planes {
        if certify_loop_simplicity(store, plane.loop_id)? != LoopSimplicity::Certified {
            return Ok(Some(indeterminate()));
        }
        let Some(orientation) = certify_loop_orientation(store, plane.face, plane.loop_id)? else {
            return Ok(Some(indeterminate()));
        };
        locally_coherent &= (orientation == PredicateOrientation::Positive)
            == store.get(plane.face)?.sense.is_forward();
    }
    Ok(Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: if locally_coherent {
            if topology.host_sign > 0 {
                ShellOrientation::Positive
            } else {
                ShellOrientation::Negative
            }
        } else {
            ShellOrientation::Invalid
        },
    }))
}

fn certify_bounded_skew_corner_contact_loop(
    store: &Store,
    loop_id: LoopId,
) -> Result<Option<PredicateOrientation>> {
    let loop_ = store.get(loop_id)?;
    let face = store.get(loop_.face)?;
    if !matches!(store.get(face.surface)?, SurfaceGeom::Cylinder(_)) {
        return Ok(None);
    }
    let Some(topology) = recognize_corner_contact_topology(store, face.shell)? else {
        return Ok(None);
    };
    if !topology
        .cylinders
        .iter()
        .any(|cylinder| cylinder.face == loop_.face && cylinder.loop_id == loop_id)
    {
        return Ok(None);
    }
    corner_cylinder_loop_orientation(store, &topology, loop_id)
}

/// Certify the two-loop periodic face used by the operand-reversed corner.
///
/// The complete corner theorem owns both loops. One is an exact endpoint-free
/// horizontal winding and the other is the family-backed contact boundary;
/// strict axial separation and opposite traversal prove their annular layout
/// without cutting the Cylinder at an artificial seam.
pub(crate) fn certify_bounded_skew_corner_face_layout(
    store: &Store,
    face_id: FaceId,
) -> Result<bool> {
    let face = store.get(face_id)?;
    if face.loops.len() != 2 || !matches!(store.get(face.surface)?, SurfaceGeom::Cylinder(_)) {
        return Ok(false);
    }
    let Some(topology) = recognize_corner_contact_topology(store, face.shell)? else {
        return Ok(false);
    };
    if !face.loops.iter().all(|loop_id| {
        topology
            .cylinders
            .iter()
            .any(|candidate| candidate.face == face_id && candidate.loop_id == *loop_id)
    }) {
        return Ok(false);
    }
    let mut orientations = Vec::with_capacity(2);
    let mut ranges = Vec::with_capacity(2);
    for &loop_id in &face.loops {
        if certify_loop_simplicity(store, loop_id)? != LoopSimplicity::Certified {
            return Ok(false);
        }
        let Some(orientation) = corner_cylinder_loop_orientation(store, &topology, loop_id)? else {
            return Ok(false);
        };
        let Some(range) = corner_loop_axial_range(store, face_id, loop_id)? else {
            return Ok(false);
        };
        orientations.push(orientation);
        ranges.push(range);
    }
    Ok(orientations[0] != orientations[1]
        && (ranges[0].1 < ranges[1].0 || ranges[1].1 < ranges[0].0))
}

fn corner_loop_axial_range(
    store: &Store,
    face_id: FaceId,
    loop_id: LoopId,
) -> Result<Option<(f64, f64)>> {
    let face = store.get(face_id)?;
    let SurfaceGeom::Cylinder(cylinder) = store.get(face.surface)? else {
        return Ok(None);
    };
    let loop_ = store.get(loop_id)?;
    if loop_.face != face_id || loop_.fins.is_empty() {
        return Ok(None);
    }
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for &fin_id in &loop_.fins {
        let fin = store.get(fin_id)?;
        let Some(use_) = fin.pcurve else {
            return Ok(None);
        };
        let curve = store.get(use_.curve())?.as_curve();
        let bounds = curve.bounding_box(use_.range());
        let min = use_.chart().apply(bounds.min, cylinder.periodicity())?;
        let max = use_.chart().apply(bounds.max, cylinder.periodicity())?;
        if !finite_uv(min) || !finite_uv(max) || min.y > max.y {
            return Ok(None);
        }
        lo = lo.min(min.y);
        hi = hi.max(max.y);
    }
    Ok((lo.is_finite() && hi.is_finite() && lo <= hi).then_some((lo, hi)))
}

fn recognize_corner_contact_topology(
    store: &Store,
    shell_id: ShellId,
) -> Result<Option<Box<CornerContactTopology>>> {
    let shell = store.get(shell_id)?;
    if !matches!(shell.faces.len(), 7 | 8) || !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(None);
    }
    let mut raw_cylinders = Vec::with_capacity(3);
    let mut raw_planes = Vec::with_capacity(5);
    let mut edges = Vec::with_capacity(14);
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        if face.loops.is_empty() || face.loops.len() > 2 {
            return Ok(None);
        }
        if face.shell != shell_id
            || face
                .loops
                .iter()
                .any(|loop_id| !store.get(*loop_id).is_ok_and(|loop_| loop_.face == face_id))
        {
            return Ok(None);
        }
        match store.get(face.surface)? {
            SurfaceGeom::Cylinder(cylinder) => {
                for &loop_id in &face.loops {
                    raw_cylinders.push((face_id, loop_id, *cylinder));
                }
            }
            SurfaceGeom::Plane(plane) => {
                let [loop_id] = face.loops.as_slice() else {
                    return Ok(None);
                };
                raw_planes.push((face_id, *loop_id, *plane));
            }
            _ => return Ok(None),
        }
        for &loop_id in &face.loops {
            for &fin_id in &store.get(loop_id)?.fins {
                let fin = store.get(fin_id)?;
                if fin.parent != loop_id {
                    return Ok(None);
                }
                if !edges.contains(&fin.edge) {
                    edges.push(fin.edge);
                }
            }
        }
    }
    let expected_edges = if shell.faces.len() == 8 { 14 } else { 13 };
    if raw_cylinders.len() != 3 || raw_planes.len() != 5 || edges.len() != expected_edges {
        return Ok(None);
    }
    let fin_counts = shell
        .faces
        .iter()
        .map(|face| -> Result<Vec<usize>> {
            let face = store.get(*face)?;
            face.loops
                .iter()
                .map(|loop_id| Ok(store.get(*loop_id)?.fins.len()))
                .collect()
        })
        .collect::<Result<Vec<_>>>()?
        .into_iter()
        .flatten()
        .collect::<Vec<_>>();
    let mut fin_counts = fin_counts;
    fin_counts.sort_unstable();
    let expected_fin_counts = if shell.faces.len() == 8 {
        [2, 2, 2, 4, 4, 4, 4, 6]
    } else {
        [1, 1, 2, 2, 4, 4, 4, 8]
    };
    if fin_counts != expected_fin_counts {
        return Ok(None);
    }
    let Some((_vertices, contacts)) = corner_manifold_signature(store, shell_id, &edges)? else {
        return Ok(None);
    };
    let mut persistent = Vec::with_capacity(2);
    for &edge_id in &edges {
        let edge = store.get(edge_id)?;
        let Some(curve_id) = edge.curve else {
            return Ok(None);
        };
        if edge.fins.len() != 2 {
            return Ok(None);
        }
        let first_fin = store.get(edge.fins[0])?;
        let second_fin = store.get(edge.fins[1])?;
        if first_fin.sense == second_fin.sense {
            return Ok(None);
        }
        if edge.bounds.is_none() || edge.vertices == [None, None] {
            if edge.bounds.is_some()
                || edge.vertices != [None, None]
                || edge.tolerance.is_some()
                || store.get(curve_id)?.as_curve().periodicity().is_none()
            {
                return Ok(None);
            }
            continue;
        }
        let (Some(_), [Some(first), Some(second)]) = (edge.bounds, edge.vertices) else {
            return Ok(None);
        };
        if first == second {
            return Ok(None);
        }
        if let Some(descriptor) = store
            .get(curve_id)?
            .as_persistent_skew_cylinder_open_span()
            .copied()
        {
            if edge.tolerance.is_none() {
                return Ok(None);
            }
            persistent.push(PersistentBoundary {
                edge: edge_id,
                descriptor,
            });
        } else if edge.tolerance.is_some() {
            return Ok(None);
        }
    }
    let Ok(mut persistent) =
        <Vec<PersistentBoundary> as TryInto<[PersistentBoundary; 2]>>::try_into(persistent)
    else {
        return Ok(None);
    };
    let memberships = persistent.map(|boundary| {
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
        || family.member_count() != 2
        || first_membership.ordinal() == second_membership.ordinal()
    {
        return Ok(None);
    }
    if first_membership.ordinal() > second_membership.ordinal() {
        persistent.swap(0, 1);
    }
    let ordered_memberships = persistent.map(|boundary| {
        boundary
            .descriptor
            .certificate()
            .finite_window_family_membership()
            .expect("checked above")
    });
    let sheet = ordered_memberships[0].member().sheet();
    if ordered_memberships[1].member().sheet() != sheet
        || ordered_memberships[0].ordinal() + 1 != ordered_memberships[1].ordinal()
        || family.sheet_occupancy(sheet)
            != (PersistentSkewCylinderFiniteWindowSheetOccupancy::Open {
                first_member_ordinal: ordered_memberships[0].ordinal(),
                member_count: 2,
            })
    {
        return Ok(None);
    }
    let other_sheet = match sheet {
        SkewCylinderSheet::Lower => SkewCylinderSheet::Upper,
        SkewCylinderSheet::Upper => SkewCylinderSheet::Lower,
    };
    if family.sheet_occupancy(other_sheet)
        != PersistentSkewCylinderFiniteWindowSheetOccupancy::Outside
    {
        return Ok(None);
    }
    let mut boundary_events = 0_usize;
    let mut isolated_events = Vec::with_capacity(2);
    for ordinal in 0..family.root_event_count(sheet) {
        let Some(event) = family.root_event(sheet, ordinal) else {
            return Ok(None);
        };
        match event.kind() {
            PersistentSkewCylinderFiniteWindowRootEventKind::Boundary => boundary_events += 1,
            PersistentSkewCylinderFiniteWindowRootEventKind::Isolated => {
                let Some(point) = family.isolated_point_certificate(sheet, ordinal) else {
                    return Ok(None);
                };
                isolated_events.push(point);
            }
            PersistentSkewCylinderFiniteWindowRootEventKind::Contact => return Ok(None),
        }
    }
    if boundary_events != 4
        || isolated_events.len() != 2
        || family.root_event_count(other_sheet) != 0
    {
        return Ok(None);
    }
    let relationship = match certify_persistent_skew_cylinder_span_relationship(
        persistent[0].descriptor,
        persistent[1].descriptor,
        PersistentSkewCylinderSpanRelationshipRequest::DisjointRange {
            order: PersistentSkewCylinderSpanRangeOrder::FirstBeforeSecond,
        },
    ) {
        Ok(relationship) => relationship,
        Err(_) => return Ok(None),
    };
    let source_cylinders = family.source_cylinders();
    if !certified_nonparallel(
        source_cylinders[0].frame().z(),
        source_cylinders[1].frame().z(),
    ) {
        return Ok(None);
    }
    let mut cylinders = Vec::with_capacity(3);
    let mut cylinder_counts = [0_usize; 2];
    for (face, loop_id, cylinder) in raw_cylinders {
        let matches = source_cylinders
            .iter()
            .enumerate()
            .filter_map(|(index, source)| (*source == cylinder).then_some(index))
            .collect::<Vec<_>>();
        let [source_slot] = matches.as_slice() else {
            return Ok(None);
        };
        cylinder_counts[*source_slot] += 1;
        cylinders.push(CornerCylinderFace {
            face,
            loop_id,
            source_slot: *source_slot,
        });
    }
    let host_slot = match (shell.faces.len(), cylinder_counts) {
        (8, [1, 2]) | (7, [2, 1]) => 0,
        (8, [2, 1]) | (7, [1, 2]) => 1,
        _ => return Ok(None),
    };
    let feature_slot = 1 - host_slot;
    for boundary in persistent {
        let mut slots = Vec::with_capacity(2);
        for &fin_id in &store.get(boundary.edge)?.fins {
            let face = store.get(store.get(fin_id)?.parent)?.face;
            let Some(slot) = cylinders
                .iter()
                .find(|candidate| candidate.face == face)
                .map(|candidate| candidate.source_slot)
            else {
                return Ok(None);
            };
            slots.push(slot);
        }
        slots.sort_unstable();
        if slots != [0, 1] {
            return Ok(None);
        }
    }
    let source_windows = family.source_windows();
    let mut planes = Vec::with_capacity(5);
    let mut plane_counts = [[0_usize; 2]; 2];
    for (face, loop_id, plane) in raw_planes {
        let mut roles = Vec::with_capacity(1);
        for source_slot in 0..2 {
            for (boundary_index, boundary) in [
                PersistentSkewCylinderAxialBoundary::Lower,
                PersistentSkewCylinderAxialBoundary::Upper,
            ]
            .into_iter()
            .enumerate()
            {
                let bound = if boundary == PersistentSkewCylinderAxialBoundary::Lower {
                    source_windows[source_slot][1].lo
                } else {
                    source_windows[source_slot][1].hi
                };
                if certified_parallel(plane.frame().z(), source_cylinders[source_slot].frame().z())
                    && certified_axial_plane_alignment(source_cylinders[source_slot], plane, bound)
                {
                    roles.push((source_slot, boundary_index, boundary));
                }
            }
        }
        let [(source_slot, boundary_index, boundary)] = roles.as_slice() else {
            return Ok(None);
        };
        let boundary_direction = if *boundary == PersistentSkewCylinderAxialBoundary::Lower {
            -source_cylinders[*source_slot].frame().z()
        } else {
            source_cylinders[*source_slot].frame().z()
        };
        let Some(support_sign) = oriented_dot_sign(
            plane.frame().z() * sense_factor(store.get(face)?.sense),
            boundary_direction,
        ) else {
            return Ok(None);
        };
        plane_counts[*source_slot][*boundary_index] += 1;
        planes.push(CornerPlaneFace {
            face,
            loop_id,
            source_slot: *source_slot,
            support_sign,
        });
    }
    let mut host_plane_counts = plane_counts[host_slot];
    host_plane_counts.sort_unstable();
    let mut feature_plane_counts = plane_counts[feature_slot];
    feature_plane_counts.sort_unstable();
    if !matches!(
        (host_plane_counts, feature_plane_counts),
        ([1, 3], [0, 1]) | ([1, 2], [1, 1])
    ) {
        return Ok(None);
    }
    let host_face = cylinders
        .iter()
        .find(|face| face.source_slot == host_slot)
        .expect("one host face");
    let host_sign = if store.get(host_face.face)?.sense.is_forward() {
        1
    } else {
        -1
    };
    if cylinders.iter().any(|face| {
        let sign = if store
            .get(face.face)
            .is_ok_and(|face| face.sense.is_forward())
        {
            1
        } else {
            -1
        };
        sign != if face.source_slot == host_slot {
            host_sign
        } else {
            -host_sign
        }
    }) || planes.iter().any(|face| {
        face.support_sign
            != if face.source_slot == host_slot {
                host_sign
            } else {
                -host_sign
            }
    }) {
        return Ok(None);
    }
    let tolerance = family.tolerance().max(LINEAR_RESOLUTION);
    let mut matched_contacts = Vec::with_capacity(2);
    for isolated in isolated_events {
        let point = isolated.point();
        let matching = contacts
            .iter()
            .copied()
            .filter(|vertex| {
                store
                    .vertex_position(*vertex)
                    .is_ok_and(|candidate| point_distance_within(candidate, point, tolerance))
            })
            .collect::<Vec<_>>();
        let [vertex] = matching.as_slice() else {
            return Ok(None);
        };
        let incidence = isolated_contact_incidence(store, *vertex, &cylinders, &planes, host_slot)?;
        if matched_contacts.contains(vertex) || !incidence {
            return Ok(None);
        }
        matched_contacts.push(*vertex);
    }
    if matched_contacts.len() != 2 {
        return Ok(None);
    }
    Ok(Some(Box::new(CornerContactTopology {
        cylinders: cylinders.try_into().expect("length checked"),
        planes: planes.try_into().expect("length checked"),
        persistent,
        relationship,
        host_sign,
    })))
}

fn corner_manifold_signature(
    store: &Store,
    shell_id: ShellId,
    edges: &[EdgeId],
) -> Result<Option<([VertexId; 8], Vec<VertexId>)>> {
    let shell = store.get(shell_id)?;
    let mut vertices = Vec::with_capacity(8);
    for &edge_id in edges {
        let edge = store.get(edge_id)?;
        if edge.fins.len() != 2 {
            return Ok(None);
        }
        match (edge.bounds, edge.vertices) {
            (Some(_), [Some(first), Some(second)]) if first != second => {
                for vertex in [first, second] {
                    if !vertices.contains(&vertex) {
                        vertices.push(vertex);
                    }
                }
            }
            (None, [None, None])
                if edge.tolerance.is_none()
                    && edge
                        .curve
                        .and_then(|curve| store.get(curve).ok())
                        .is_some_and(|curve| curve.as_curve().periodicity().is_some()) => {}
            _ => return Ok(None),
        }
        let faces = edge
            .fins
            .iter()
            .map(|fin| {
                let fin = store.get(*fin)?;
                let loop_ = store.get(fin.parent)?;
                Ok((fin, loop_.face))
            })
            .collect::<Result<Vec<_>>>()?;
        let [(first_fin, first_face), (second_fin, second_face)] = faces.as_slice() else {
            return Ok(None);
        };
        if first_fin.sense == second_fin.sense
            || first_face == second_face
            || !shell.faces.contains(first_face)
            || !shell.faces.contains(second_face)
        {
            return Ok(None);
        }
    }
    if vertices.len() != 8 {
        return Ok(None);
    }
    let mut contacts = Vec::with_capacity(2);
    for &vertex in &vertices {
        let incident = edges
            .iter()
            .copied()
            .filter(|edge| {
                store
                    .get(*edge)
                    .is_ok_and(|edge| edge.vertices.into_iter().flatten().any(|v| v == vertex))
            })
            .collect::<Vec<_>>();
        if !matches!(incident.len(), 3 | 5) {
            return Ok(None);
        }
        if incident.len() == 5 {
            contacts.push(vertex);
        }
        let mut link_edges = Vec::with_capacity(incident.len());
        for edge_id in incident {
            let edge = store.get(edge_id)?;
            let mut faces = Vec::with_capacity(2);
            for &fin_id in &edge.fins {
                let fin = store.get(fin_id)?;
                let loop_ = store.get(fin.parent)?;
                if fin.edge != edge_id
                    || !loop_.fins.contains(&fin_id)
                    || !shell.faces.contains(&loop_.face)
                    || faces.contains(&loop_.face)
                {
                    return Ok(None);
                }
                faces.push(loop_.face);
            }
            let [first, second] = faces.as_slice() else {
                return Ok(None);
            };
            let pair = (*first, *second);
            if link_edges
                .iter()
                .any(|prior| same_unordered_face_pair(*prior, pair))
            {
                return Ok(None);
            }
            link_edges.push(pair);
        }
        let mut link_vertices = Vec::with_capacity(link_edges.len());
        for &(first, second) in &link_edges {
            for face in [first, second] {
                if !link_vertices.contains(&face) {
                    link_vertices.push(face);
                }
            }
        }
        if link_vertices.len() != link_edges.len()
            || link_vertices.iter().any(|face| {
                link_edges
                    .iter()
                    .filter(|(first, second)| first == face || second == face)
                    .count()
                    != 2
            })
        {
            return Ok(None);
        }
    }
    if !matches!(contacts.len(), 0 | 2) {
        return Ok(None);
    }
    if contacts.is_empty() {
        contacts.extend(vertices.iter().copied());
    }
    Ok(Some((
        vertices.try_into().expect("length checked"),
        contacts,
    )))
}

fn isolated_contact_incidence(
    store: &Store,
    vertex: VertexId,
    cylinders: &[CornerCylinderFace],
    planes: &[CornerPlaneFace],
    host_slot: usize,
) -> Result<bool> {
    let mut faces = Vec::with_capacity(5);
    for candidate in cylinders
        .iter()
        .map(|face| face.face)
        .chain(planes.iter().map(|face| face.face))
    {
        let face = store.get(candidate)?;
        let incident = face.loops.iter().any(|loop_id| {
            store.get(*loop_id).is_ok_and(|loop_| {
                loop_.fins.iter().any(|fin| {
                    store
                        .get(*fin)
                        .ok()
                        .and_then(|fin| store.get(fin.edge).ok())
                        .is_some_and(|edge| {
                            edge.vertices
                                .into_iter()
                                .flatten()
                                .any(|candidate| candidate == vertex)
                        })
                })
            })
        });
        if incident && !faces.contains(&candidate) {
            faces.push(candidate);
        }
    }
    if !matches!(faces.len(), 3 | 5) {
        return Ok(false);
    }
    let host_cylinders = faces
        .iter()
        .filter(|face| {
            cylinders
                .iter()
                .any(|candidate| candidate.face == **face && candidate.source_slot == host_slot)
        })
        .count();
    let feature_cylinders = faces
        .iter()
        .filter(|face| {
            cylinders
                .iter()
                .any(|candidate| candidate.face == **face && candidate.source_slot != host_slot)
        })
        .count();
    let host_planes = planes
        .iter()
        .filter(|face| face.source_slot == host_slot && faces.contains(&face.face))
        .count();
    let feature_planes = planes
        .iter()
        .filter(|face| face.source_slot != host_slot && faces.contains(&face.face))
        .count();
    let roles = [
        host_cylinders,
        feature_cylinders,
        host_planes,
        feature_planes,
    ];
    Ok(matches!(roles, [1, 1, 2, 1] | [1, 0, 1, 1]))
}

fn point_distance_within(first: Point3, second: Point3, tolerance: f64) -> bool {
    if !tolerance.is_finite() || tolerance < 0.0 {
        return false;
    }
    let distance_squared = [first.x, first.y, first.z]
        .into_iter()
        .zip([second.x, second.y, second.z])
        .fold(Interval::point(0.0), |sum, (first, second)| {
            sum + (Interval::point(first) - Interval::point(second)).square()
        });
    let allowed = Interval::point(tolerance).square();
    finite_interval(distance_squared)
        && finite_interval(allowed)
        && distance_squared.hi() <= allowed.lo()
}

fn corner_cylinder_loop_orientation(
    store: &Store,
    topology: &CornerContactTopology,
    loop_id: LoopId,
) -> Result<Option<PredicateOrientation>> {
    let Some(cylinder_face) = topology
        .cylinders
        .iter()
        .find(|face| face.loop_id == loop_id)
    else {
        return Ok(None);
    };
    let face = store.get(cylinder_face.face)?;
    let SurfaceGeom::Cylinder(cylinder) = store.get(face.surface)? else {
        return Ok(None);
    };
    if store.get(loop_id)?.fins.len() == 1 {
        return corner_endpoint_free_cylinder_loop_orientation(store, cylinder_face, *cylinder);
    }
    if !certify_cylinder_chart_closure(store, cylinder_face.face, loop_id, *cylinder)? {
        return Ok(None);
    }
    if let Some(orientation) = corner_periodic_loop_winding_orientation(store, loop_id, *cylinder)?
    {
        return Ok(Some(orientation));
    }
    let integrals = topology.relationship.span_directed_chart_integrals();
    let mut stored = Interval::point(0.0);
    let mut source = Interval::point(0.0);
    let loop_ = store.get(loop_id)?;
    let mut tails = Vec::with_capacity(loop_.fins.len());
    for &fin_id in &loop_.fins {
        let Some(tail) = store.fin_tail(fin_id)? else {
            return Ok(None);
        };
        if tails.contains(&tail) {
            return Ok(None);
        }
        tails.push(tail);
        let fin = store.get(fin_id)?;
        let edge = store.get(fin.edge)?;
        let terms = if let Some(span_index) = topology
            .persistent
            .iter()
            .position(|boundary| boundary.edge == fin.edge)
        {
            let certificate = topology.persistent[span_index].descriptor;
            let Some(terms) = persistent_fin_integrals(
                store,
                fin_id,
                cylinder_face.source_slot,
                certificate,
                integrals[span_index][cylinder_face.source_slot],
            )?
            else {
                return Ok(None);
            };
            terms
        } else {
            if edge.tolerance.is_some() || edge.fins.len() != 2 {
                return Ok(None);
            }
            let Some(term) = analytic_fin_integral(store, fin_id, *cylinder)? else {
                return Ok(None);
            };
            [term, term]
        };
        stored = stored + terms[0];
        source = source + terms[1];
        if !finite_interval(stored) || !finite_interval(source) {
            return Ok(None);
        }
    }
    match (strict_interval_sign(stored), strict_interval_sign(source)) {
        (Some(stored_sign), Some(source_sign)) if stored_sign == source_sign => {
            Ok(Some(stored_sign))
        }
        _ => {
            if let Some(orientation) =
                corner_vertical_boundary_orientation(store, topology, cylinder_face)?
            {
                Ok(Some(orientation))
            } else {
                corner_axial_boundary_orientation(store, topology, cylinder_face)
            }
        }
    }
}

fn corner_periodic_loop_winding_orientation(
    store: &Store,
    loop_id: LoopId,
    cylinder: kgeom::surface::Cylinder,
) -> Result<Option<PredicateOrientation>> {
    let mut delta = Interval::point(0.0);
    let mut allowance = Interval::point(0.0);
    for &fin_id in &store.get(loop_id)?.fins {
        let fin = store.get(fin_id)?;
        let edge = store.get(fin.edge)?;
        let (Some((lo, hi)), Some(use_)) = (edge.bounds, fin.pcurve) else {
            return Ok(None);
        };
        let curve = store.get(use_.curve())?.as_curve();
        let [start, end] = if fin.sense == Sense::Forward {
            [lo, hi]
        } else {
            [hi, lo]
        };
        let periods = cylinder.periodicity();
        let start = use_.evaluate_uv(curve, start, periods)?;
        let end = use_.evaluate_uv(curve, end, periods)?;
        delta = delta + Interval::point(end.x) - Interval::point(start.x);
        let tolerance = edge
            .tolerance
            .map(crate::tolerance::EntityTolerance::value)
            .unwrap_or(0.0)
            .max(LINEAR_RESOLUTION);
        let Some(angular) =
            Interval::point(tolerance).checked_div(Interval::point(cylinder.radius()))
        else {
            return Ok(None);
        };
        allowance = allowance + angular;
    }
    if !finite_interval(delta) || !finite_interval(allowance) {
        return Ok(None);
    }
    let period = Interval::point(core::f64::consts::TAU);
    if interval_abs_upper(delta - period) <= allowance.lo() {
        Ok(Some(PredicateOrientation::Positive))
    } else if interval_abs_upper(delta + period) <= allowance.lo() {
        Ok(Some(PredicateOrientation::Negative))
    } else {
        Ok(None)
    }
}

fn corner_endpoint_free_cylinder_loop_orientation(
    store: &Store,
    cylinder_face: &CornerCylinderFace,
    cylinder: kgeom::surface::Cylinder,
) -> Result<Option<PredicateOrientation>> {
    let loop_ = store.get(cylinder_face.loop_id)?;
    let [fin_id] = loop_.fins.as_slice() else {
        return Ok(None);
    };
    let fin = store.get(*fin_id)?;
    let edge = store.get(fin.edge)?;
    let Some(use_) = fin.pcurve else {
        return Ok(None);
    };
    let crate::geom::Curve2dGeom::Line(line) = store.get(use_.curve())? else {
        return Ok(None);
    };
    let Some([winding @ (-1 | 1), 0]) = use_.closure_winding() else {
        return Ok(None);
    };
    let map = use_.edge_to_pcurve();
    let rate = line.dir().x * map.scale();
    if edge.bounds.is_some()
        || edge.vertices != [None, None]
        || edge.tolerance.is_some()
        || line.dir().y != 0.0
        || !rate.is_finite()
        || rate == 0.0
        || rate.is_sign_positive() != (winding > 0)
        || certify_whole_fin_incidence(
            store,
            cylinder_face.face,
            cylinder_face.loop_id,
            *fin_id,
            LINEAR_RESOLUTION,
        ) != WholeFinIncidence::Certified
        || cylinder.periodicity()[0] != Some(core::f64::consts::TAU)
    {
        return Ok(None);
    }
    let positive = rate.is_sign_positive() == (fin.sense == Sense::Forward);
    Ok(Some(if positive {
        PredicateOrientation::Positive
    } else {
        PredicateOrientation::Negative
    }))
}

fn corner_vertical_boundary_orientation(
    store: &Store,
    topology: &CornerContactTopology,
    cylinder_face: &CornerCylinderFace,
) -> Result<Option<PredicateOrientation>> {
    let mut boundaries = Vec::with_capacity(2);
    for &fin_id in &store.get(cylinder_face.loop_id)?.fins {
        let fin = store.get(fin_id)?;
        if topology
            .persistent
            .iter()
            .any(|boundary| boundary.edge == fin.edge)
        {
            continue;
        }
        let Some(use_) = fin.pcurve else {
            return Ok(None);
        };
        let crate::geom::Curve2dGeom::Line(line) = store.get(use_.curve())? else {
            continue;
        };
        let direction = line.dir();
        let scale = use_.edge_to_pcurve().scale();
        let shifts = use_.chart().period_shifts();
        if direction.x != 0.0 {
            continue;
        }
        if !direction.y.is_finite()
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
        boundaries.push((longitude, increasing));
    }
    let [first, second] = boundaries.as_slice() else {
        return Ok(None);
    };
    let (lower, upper) = if first.0.hi() < second.0.lo() {
        (first, second)
    } else if second.0.hi() < first.0.lo() {
        (second, first)
    } else {
        return Ok(None);
    };
    let lower_orientation = radial_boundary_orientation(true, lower.1);
    let upper_orientation = radial_boundary_orientation(false, upper.1);
    Ok((lower_orientation == upper_orientation).then_some(lower_orientation))
}

fn corner_axial_boundary_orientation(
    store: &Store,
    topology: &CornerContactTopology,
    cylinder_face: &CornerCylinderFace,
) -> Result<Option<PredicateOrientation>> {
    let mut boundaries = Vec::with_capacity(2);
    for &fin_id in &store.get(cylinder_face.loop_id)?.fins {
        let fin = store.get(fin_id)?;
        if topology
            .persistent
            .iter()
            .any(|boundary| boundary.edge == fin.edge)
        {
            continue;
        }
        let Some(use_) = fin.pcurve else {
            return Ok(None);
        };
        let crate::geom::Curve2dGeom::Line(line) = store.get(use_.curve())? else {
            continue;
        };
        let direction = line.dir();
        let scale = use_.edge_to_pcurve().scale();
        let shifts = use_.chart().period_shifts();
        if direction.y != 0.0 {
            continue;
        }
        if !direction.x.is_finite()
            || direction.x == 0.0
            || !scale.is_finite()
            || scale == 0.0
            || use_.closure_winding().is_some()
            || use_.seam().is_some()
            || shifts[1] != 0
        {
            return Ok(None);
        }
        let height = Interval::point(line.origin().y);
        let increasing_with_edge = (direction.x > 0.0) == (scale > 0.0);
        let increasing = if fin.sense == Sense::Forward {
            increasing_with_edge
        } else {
            !increasing_with_edge
        };
        boundaries.push((height, increasing));
    }
    let [first, second] = boundaries.as_slice() else {
        return Ok(None);
    };
    let (lower, upper) = if first.0.hi() < second.0.lo() {
        (first, second)
    } else if second.0.hi() < first.0.lo() {
        (second, first)
    } else {
        return Ok(None);
    };
    let lower_orientation =
        slab_boundary_orientation(PersistentSkewCylinderAxialBoundary::Lower, lower.1);
    let upper_orientation =
        slab_boundary_orientation(PersistentSkewCylinderAxialBoundary::Upper, upper.1);
    Ok((lower_orientation == upper_orientation).then_some(lower_orientation))
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
    if loop_.face != face_id || loop_.fins.len() < 2 {
        return Ok(false);
    }
    let domain = store.get(face_id)?.domain;
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
        let direct_u = interval_abs_upper(delta_u) <= angular_allowance.lo();
        let seam_u = domain.is_some_and(|domain| {
            let period = core::f64::consts::TAU;
            let wrapped = if delta_u.hi() < 0.0 {
                delta_u + Interval::point(period)
            } else {
                delta_u - Interval::point(period)
            };
            let end = spans[index].0[1].x;
            let start = spans[next].0[0].x;
            let opposite_boundaries = ((end - domain.u.lo).abs() <= angular_allowance.lo()
                && (start - domain.u.hi).abs() <= angular_allowance.lo())
                || ((end - domain.u.hi).abs() <= angular_allowance.lo()
                    && (start - domain.u.lo).abs() <= angular_allowance.lo());
            finite_interval(wrapped)
                && interval_abs_upper(wrapped) <= angular_allowance.lo()
                && opposite_boundaries
        });
        if !finite_interval(allowance)
            || !finite_interval(angular_allowance)
            || !(direct_u || seam_u)
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
        let mut endpoint_ordinals = [0, 1];
        if certificate.orientation() == PersistentSkewCylinderOpenSpanOrientation::Reversed {
            endpoints.swap(0, 1);
            endpoint_ordinals.swap(0, 1);
        }
        for (index, ((vertex, proof), endpoint_ordinal)) in [first, second]
            .into_iter()
            .zip(endpoints)
            .zip(endpoint_ordinals)
            .enumerate()
        {
            let Some(root) = membership.endpoint_root(endpoint_ordinal, 0) else {
                return Ok(None);
            };
            if output
                .iter()
                .any(|value: &TaggedVertex| value.vertex == vertex)
                || proof.root_count() != 1
                || !point_bits_equal(
                    store.vertex_position(vertex)?,
                    certificate.endpoint_points()[index],
                )
            {
                return Ok(None);
            }
            output.push(TaggedVertex {
                vertex,
                tag: root.tag,
                bound: root.bound,
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
fn bounded_skew_lobe_proof_work(store: &Store, shell_id: ShellId) -> Result<Option<u64>> {
    let Some(size) = shell_proof_size(store, shell_id)? else {
        return Ok(None);
    };
    Ok(quadratic_proof_work(size, 16, 0, 1))
}
#[cfg(test)]
fn proof_work_for_size(size: u64) -> Option<u64> {
    quadratic_proof_work(size, 16, 0, 1)
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

#[cfg(test)]
#[path = "bounded_skew_lobe_shell_proof/tests.rs"]
mod tests;
