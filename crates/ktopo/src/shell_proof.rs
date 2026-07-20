//! Conservative shell embedding and orientation proofs.
//!
//! A closed manifold whose faces are strict convex planar facets, with every
//! facet a supporting plane of the complete vertex set, is the boundary of
//! its convex hull. Coplanar facet partitions are admitted only when exact
//! projected predicates prove their interiors disjoint. Together these give
//! a compact proof of global non-self-intersection and outward orientation. A
//! single planar sheet face is embedded when every polygonal loop is proven
//! simple and the holes have certified strict containment.

use crate::entity::{BodyKind, FaceId, RegionKind, Sense, ShellId, VertexId};
use crate::geom::{CurveGeom, SurfaceGeom};
use crate::incidence::{IncidenceCertification, certify_edge_surface_incidence};
use crate::loop_proof::{
    LoopContainment, LoopSimplicity, certify_loop_containment, certify_loop_simplicity,
};
use crate::store::Store;
use kcore::error::Result;
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationScope, ResourceKind, StageId,
};
use kcore::predicates::{Orientation as PredicateOrientation, orient2d, orient3d};
use kcore::tolerance::{ANGULAR_RESOLUTION, LINEAR_RESOLUTION};
use kgeom::curve::Curve;
use kgeom::vec::{Point2, Vec3};

/// Cumulative exact contact work for coplanar shell-facet partitions.
pub(crate) const SHELL_FACET_PAIR_WORK: StageId =
    match StageId::new("ktopo.check.shell-facet-pair-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid shell facet-pair proof stage"),
    };

const DEFAULT_SHELL_FACET_PAIR_WORK: u64 = 100_000;

/// Version-1 deterministic budget for planar shell partition proofs.
pub(crate) fn shell_proof_budget() -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(
        SHELL_FACET_PAIR_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        DEFAULT_SHELL_FACET_PAIR_WORK,
    )])
    .expect("built-in shell proof budget is valid")
}

/// Proof state for global shell self-intersection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ShellEmbedding {
    /// The shell belongs to a proven embedded representation class.
    Certified,
    /// The current proof slice cannot establish global embedding.
    Indeterminate,
}

/// Proof state for a solid shell's global outward orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ShellOrientation {
    /// Every facet normal points away from the convex interior.
    Certified,
    /// At least one supporting facet normal provably points into the
    /// convex interior.
    Invalid,
    /// The current proof slice cannot establish an interior half-space.
    Indeterminate,
}

/// Independent embedding and orientation results for one shell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ShellCertification {
    pub(crate) embedding: ShellEmbedding,
    pub(crate) orientation: ShellOrientation,
}

/// Attempt to certify one shell in the context of its owning body/region,
/// charging every non-constant proof stage to the caller-owned scope.
pub(crate) fn certify_shell_in_scope(
    store: &Store,
    shell_id: ShellId,
    body_kind: BodyKind,
    region_kind: RegionKind,
    scope: &mut OperationScope<'_, '_>,
) -> Result<ShellCertification> {
    certify_shell_impl(store, shell_id, body_kind, region_kind, Some(scope))
}

#[cfg(test)]
fn certify_shell(
    store: &Store,
    shell_id: ShellId,
    body_kind: BodyKind,
    region_kind: RegionKind,
) -> Result<ShellCertification> {
    certify_shell_impl(store, shell_id, body_kind, region_kind, None)
}

fn certify_shell_impl(
    store: &Store,
    shell_id: ShellId,
    body_kind: BodyKind,
    region_kind: RegionKind,
    mut scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<ShellCertification> {
    let shell = store.get(shell_id)?;
    if body_kind == BodyKind::Sheet && shell.faces.len() == 1 {
        let face = store.get(shell.faces[0])?;
        let planar = matches!(store.get(face.surface)?, SurfaceGeom::Plane(_));
        let mut simple = !face.loops.is_empty();
        for &loop_id in &face.loops {
            simple &= certify_loop_simplicity(store, loop_id)? == LoopSimplicity::Certified;
        }
        let contained = certify_loop_containment(store, &face.loops)? == LoopContainment::Certified;
        return Ok(ShellCertification {
            embedding: if planar && simple && contained {
                ShellEmbedding::Certified
            } else {
                ShellEmbedding::Indeterminate
            },
            orientation: ShellOrientation::Indeterminate,
        });
    }
    if body_kind != BodyKind::Solid || region_kind != RegionKind::Solid {
        return Ok(indeterminate());
    }
    if let Some(certification) = certify_whole_closed_surface(store, shell_id)? {
        return Ok(certification);
    }
    if let Some(certification) = certify_sphere_cap_shell(store, shell_id)? {
        return Ok(certification);
    }
    if let Some(certification) = certify_planar_profile_prism(store, shell_id)? {
        return Ok(certification);
    }
    let convex = certify_convex_planar_shell(store, shell_id, scope.as_deref_mut())?;
    if convex != indeterminate() {
        return Ok(convex);
    }
    let Some(scope) = scope else {
        return Ok(convex);
    };
    crate::planar_shell_proof::certify_general_planar_shell_in_scope(store, shell_id, scope)
}

fn indeterminate() -> ShellCertification {
    ShellCertification {
        embedding: ShellEmbedding::Indeterminate,
        orientation: ShellOrientation::Indeterminate,
    }
}

fn certify_whole_closed_surface(
    store: &Store,
    shell_id: ShellId,
) -> Result<Option<ShellCertification>> {
    let shell = store.get(shell_id)?;
    if shell.faces.len() != 1 {
        return Ok(None);
    }
    let face = store.get(shell.faces[0])?;
    if !face.loops.is_empty()
        || !matches!(
            store.get(face.surface)?,
            SurfaceGeom::Sphere(_) | SurfaceGeom::Torus(_)
        )
    {
        return Ok(None);
    }
    Ok(Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: if face.sense == Sense::Forward {
            ShellOrientation::Certified
        } else {
            ShellOrientation::Invalid
        },
    }))
}

fn certify_sphere_cap_shell(
    store: &Store,
    shell_id: ShellId,
) -> Result<Option<ShellCertification>> {
    let shell = store.get(shell_id)?;
    if shell.faces.len() != 2 {
        return Ok(None);
    }
    let mut sphere_face = None;
    let mut plane_face = None;
    for &face_id in &shell.faces {
        match store.get(store.get(face_id)?.surface)? {
            SurfaceGeom::Sphere(_) => sphere_face = Some(face_id),
            SurfaceGeom::Plane(_) => plane_face = Some(face_id),
            _ => return Ok(None),
        }
    }
    let (Some(sphere_face_id), Some(plane_face_id)) = (sphere_face, plane_face) else {
        return Ok(None);
    };
    let sphere_face = store.get(sphere_face_id)?;
    let plane_face = store.get(plane_face_id)?;
    if sphere_face.loops.len() != 1 || plane_face.loops.len() != 1 {
        return Ok(None);
    }
    let sphere_loop = store.get(sphere_face.loops[0])?;
    let plane_loop = store.get(plane_face.loops[0])?;
    if sphere_loop.fins.len() != 1
        || plane_loop.fins.len() != 1
        || certify_loop_simplicity(store, sphere_face.loops[0])? != LoopSimplicity::Certified
        || certify_loop_simplicity(store, plane_face.loops[0])? != LoopSimplicity::Certified
    {
        return Ok(None);
    }
    let sphere_fin = store.get(sphere_loop.fins[0])?;
    let plane_fin = store.get(plane_loop.fins[0])?;
    if sphere_fin.edge != plane_fin.edge {
        return Ok(None);
    }
    let edge = store.get(sphere_fin.edge)?;
    if edge.tolerance.is_some() {
        return Ok(None);
    }
    let Some(curve_id) = edge.curve else {
        return Ok(None);
    };
    let CurveGeom::Circle(circle) = store.get(curve_id)? else {
        return Ok(None);
    };
    let SurfaceGeom::Sphere(sphere) = store.get(sphere_face.surface)? else {
        unreachable!("classified above");
    };
    let SurfaceGeom::Plane(plane) = store.get(plane_face.surface)? else {
        unreachable!("classified above");
    };
    if certify_edge_surface_incidence(
        store,
        sphere_fin.edge,
        sphere_face.surface,
        LINEAR_RESOLUTION,
    )? != IncidenceCertification::Certified
        || certify_edge_surface_incidence(
            store,
            plane_fin.edge,
            plane_face.surface,
            LINEAR_RESOLUTION,
        )? != IncidenceCertification::Certified
    {
        return Ok(None);
    }

    let plane_normal = plane.frame().z();
    if 1.0 - circle.frame().z().dot(plane_normal).abs() > ANGULAR_RESOLUTION {
        return Ok(None);
    }
    let center_offset = sphere.frame().origin() - plane.frame().origin();
    let signed_height = center_offset.dot(plane_normal);
    if signed_height.abs() >= sphere.radius() {
        return Ok(None);
    }
    let expected_center = sphere.frame().origin() - plane_normal * signed_height;
    let expected_radius =
        (sphere.radius() * sphere.radius() - signed_height * signed_height).sqrt();
    if circle.frame().origin().dist(expected_center) > LINEAR_RESOLUTION
        || (circle.radius() - expected_radius).abs() > LINEAR_RESOLUTION
    {
        return Ok(None);
    }

    let range = match edge.bounds {
        Some((lo, hi)) if lo.is_finite() && hi.is_finite() && lo < hi => {
            kgeom::param::ParamRange::new(lo, hi)
        }
        Some(_) => return Ok(None),
        None => circle.param_range(),
    };
    let parameter = if sphere_fin.sense.is_forward() {
        range.lo
    } else {
        range.hi
    };
    let point = circle.eval(parameter);
    let mut tangent = circle.eval_derivs(parameter, 1).d[1];
    if !sphere_fin.sense.is_forward() {
        tangent = -tangent;
    }
    let sphere_normal =
        (point - sphere.frame().origin()) / sphere.radius() * sense_factor(sphere_face.sense);
    let cap_interior = sphere_normal.cross(tangent);
    let plane_outward = plane_normal * sense_factor(plane_face.sense);
    let alignment = cap_interior.dot(-plane_outward);
    if alignment.abs() <= circle.radius() * ANGULAR_RESOLUTION {
        return Ok(None);
    }
    let orientation_valid = sphere_face.sense == Sense::Forward && alignment > 0.0;
    Ok(Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: if orientation_valid {
            ShellOrientation::Certified
        } else {
            ShellOrientation::Invalid
        },
    }))
}

fn sense_factor(sense: Sense) -> f64 {
    if sense.is_forward() { 1.0 } else { -1.0 }
}

#[derive(Debug)]
struct PrismCap {
    face: FaceId,
    vertices: Vec<VertexId>,
    uses: Vec<PrismBoundaryUse>,
}

#[derive(Debug, Clone, Copy)]
struct PrismBoundaryUse {
    fin: crate::entity::FinId,
    edge: crate::entity::EdgeId,
    tail: VertexId,
    head: VertexId,
}

#[derive(Debug)]
struct PrismTranslation {
    vector: Vec3,
    vertices: Vec<(VertexId, VertexId)>,
}

/// Certify the exact topology product emitted by the polygonal-profile
/// extrusion builder: two translated planar regions and one planar quad for
/// every boundary segment. Profile loop simplicity/containment proves the
/// planar material region; the one-to-one translated edge/quad closure and a
/// nonzero translation component normal to the caps then prove that its affine
/// sweep is embedded.
fn certify_planar_profile_prism(
    store: &Store,
    shell_id: ShellId,
) -> Result<Option<ShellCertification>> {
    let shell = store.get(shell_id)?;
    if shell.faces.len() < 5 || !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(None);
    }
    let Some(bottom) = planar_prism_cap(store, shell.faces[0])? else {
        return Ok(None);
    };
    let Some(top) = planar_prism_cap(store, shell.faces[1])? else {
        return Ok(None);
    };
    if bottom.vertices.len() != top.vertices.len()
        || bottom.uses.len() != top.uses.len()
        || shell.faces.len() != bottom.uses.len() + 2
    {
        return Ok(None);
    }
    let Some(translation) = translated_prism_vertices(store, &bottom, &top)? else {
        return Ok(None);
    };
    let bottom_face = store.get(bottom.face)?;
    let top_face = store.get(top.face)?;
    let SurfaceGeom::Plane(bottom_plane) = store.get(bottom_face.surface)? else {
        unreachable!("prism cap classification retains a plane");
    };
    let SurfaceGeom::Plane(top_plane) = store.get(top_face.surface)? else {
        unreachable!("prism cap classification retains a plane");
    };
    let top_surface_axis = top_plane.frame().z();
    if translation.vector.dot(top_surface_axis) == 0.0
        || 1.0 - bottom_plane.frame().z().dot(top_surface_axis).abs() > ANGULAR_RESOLUTION
    {
        return Ok(None);
    }
    let axis = if translation.vector.dot(top_surface_axis) > 0.0 {
        top_surface_axis
    } else {
        -top_surface_axis
    };

    let bottom_edges: Vec<_> = bottom.uses.iter().map(|use_| use_.edge).collect();
    let top_edges: Vec<_> = top.uses.iter().map(|use_| use_.edge).collect();
    let side_faces = &shell.faces[2..];
    let mut used_sides = Vec::with_capacity(side_faces.len());
    let mut used_top_edges = Vec::with_capacity(top_edges.len());
    let mut orientation_invalid = false;
    let expected_bottom_normal = -axis;
    let bottom_normal = bottom_plane.frame().z() * sense_factor(bottom_face.sense);
    let top_normal = top_plane.frame().z() * sense_factor(top_face.sense);
    orientation_invalid |= bottom_normal.dot(expected_bottom_normal) <= 0.0;
    orientation_invalid |= top_normal.dot(axis) <= 0.0;

    for boundary in &bottom.uses {
        let edge = store.get(boundary.edge)?;
        if edge.fins.len() != 2 {
            return Ok(None);
        }
        let Some(other_fin) = edge.fins.iter().copied().find(|&fin| fin != boundary.fin) else {
            return Ok(None);
        };
        let other_loop = store.get(store.get(other_fin)?.parent)?;
        let side_face_id = other_loop.face;
        if !side_faces.contains(&side_face_id) || used_sides.contains(&side_face_id) {
            return Ok(None);
        }
        let Some(side_edges) = planar_prism_side(store, side_face_id)? else {
            return Ok(None);
        };
        if !side_edges.contains(&boundary.edge) {
            return Ok(None);
        }
        let Some(mapped_tail) = mapped_prism_vertex(&translation.vertices, boundary.tail) else {
            return Ok(None);
        };
        let Some(mapped_head) = mapped_prism_vertex(&translation.vertices, boundary.head) else {
            return Ok(None);
        };
        let mut mapped_top_edge = None;
        for edge in side_edges
            .iter()
            .copied()
            .filter(|edge| top_edges.contains(edge))
        {
            if edge_has_vertices(store, edge, mapped_tail, mapped_head)?
                && mapped_top_edge.replace(edge).is_some()
            {
                return Ok(None);
            }
        }
        let Some(mapped_top_edge) = mapped_top_edge else {
            return Ok(None);
        };
        let mut tail_vertical = false;
        let mut head_vertical = false;
        for edge in side_edges.iter().copied() {
            tail_vertical |= edge_has_vertices(store, edge, boundary.tail, mapped_tail)?;
            head_vertical |= edge_has_vertices(store, edge, boundary.head, mapped_head)?;
        }
        if used_top_edges.contains(&mapped_top_edge) || !tail_vertical || !head_vertical {
            return Ok(None);
        }
        let allowed = [boundary.edge, mapped_top_edge];
        if side_edges
            .iter()
            .filter(|edge| allowed.contains(edge))
            .count()
            != 2
            || side_edges.len() != 4
        {
            return Ok(None);
        }

        let tangent =
            store.vertex_position(boundary.head)? - store.vertex_position(boundary.tail)?;
        let Some(expected_side_normal) = translation.vector.cross(tangent).normalized() else {
            return Ok(None);
        };
        let side_face = store.get(side_face_id)?;
        let SurfaceGeom::Plane(side_plane) = store.get(side_face.surface)? else {
            unreachable!("prism side classification retains a plane");
        };
        let side_surface_normal = side_plane.frame().z();
        if 1.0 - side_surface_normal.dot(expected_side_normal).abs() > ANGULAR_RESOLUTION {
            return Ok(None);
        }
        let side_normal = side_surface_normal * sense_factor(side_face.sense);
        orientation_invalid |= side_normal.dot(expected_side_normal) <= 0.0;
        used_sides.push(side_face_id);
        used_top_edges.push(mapped_top_edge);
    }
    if used_sides.len() != side_faces.len()
        || used_top_edges.len() != top_edges.len()
        || bottom_edges.len() != top_edges.len()
    {
        return Ok(None);
    }

    Ok(Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: if orientation_invalid {
            ShellOrientation::Invalid
        } else {
            ShellOrientation::Certified
        },
    }))
}

fn planar_prism_cap(store: &Store, face_id: FaceId) -> Result<Option<PrismCap>> {
    let face = store.get(face_id)?;
    if face.loops.is_empty() || !matches!(store.get(face.surface)?, SurfaceGeom::Plane(_)) {
        return Ok(None);
    }
    if certify_loop_containment(store, &face.loops)? != LoopContainment::Certified {
        return Ok(None);
    }
    let mut vertices = Vec::new();
    let mut uses = Vec::new();
    for &loop_id in &face.loops {
        if certify_loop_simplicity(store, loop_id)? != LoopSimplicity::Certified {
            return Ok(None);
        }
        let loop_ = store.get(loop_id)?;
        if loop_.fins.len() < 3 {
            return Ok(None);
        }
        for &fin in &loop_.fins {
            let fin_value = store.get(fin)?;
            let edge = store.get(fin_value.edge)?;
            let (Some(curve), Some(tail), Some(head)) =
                (edge.curve, store.fin_tail(fin)?, store.fin_head(fin)?)
            else {
                return Ok(None);
            };
            if edge.tolerance.is_some()
                || edge.bounds.is_none()
                || !matches!(store.get(curve)?, CurveGeom::Line(_))
                || certify_edge_surface_incidence(
                    store,
                    fin_value.edge,
                    face.surface,
                    LINEAR_RESOLUTION,
                )? != IncidenceCertification::Certified
                || uses
                    .iter()
                    .any(|use_: &PrismBoundaryUse| use_.edge == fin_value.edge)
            {
                return Ok(None);
            }
            if !vertices.contains(&tail) {
                vertices.push(tail);
            }
            uses.push(PrismBoundaryUse {
                fin,
                edge: fin_value.edge,
                tail,
                head,
            });
        }
    }
    if vertices.len() != uses.len() {
        return Ok(None);
    }
    Ok(Some(PrismCap {
        face: face_id,
        vertices,
        uses,
    }))
}

fn translated_prism_vertices(
    store: &Store,
    bottom: &PrismCap,
    top: &PrismCap,
) -> Result<Option<PrismTranslation>> {
    let anchor = store.vertex_position(bottom.vertices[0])?;
    for &candidate in &top.vertices {
        let translation = store.vertex_position(candidate)? - anchor;
        if translation.norm() <= LINEAR_RESOLUTION {
            continue;
        }
        let mut map = Vec::with_capacity(bottom.vertices.len());
        let mut used = Vec::with_capacity(top.vertices.len());
        let mut valid = true;
        for &source in &bottom.vertices {
            let expected = store.vertex_position(source)? + translation;
            let matches: Vec<_> = top
                .vertices
                .iter()
                .copied()
                .filter(|target| {
                    !used.contains(target)
                        && store
                            .vertex_position(*target)
                            .is_ok_and(|point| point.dist(expected) <= LINEAR_RESOLUTION)
                })
                .collect();
            if matches.len() != 1 {
                valid = false;
                break;
            }
            used.push(matches[0]);
            map.push((source, matches[0]));
        }
        if valid && used.len() == top.vertices.len() {
            return Ok(Some(PrismTranslation {
                vector: translation,
                vertices: map,
            }));
        }
    }
    Ok(None)
}

fn mapped_prism_vertex(map: &[(VertexId, VertexId)], source: VertexId) -> Option<VertexId> {
    map.iter()
        .find_map(|&(candidate, target)| (candidate == source).then_some(target))
}

fn planar_prism_side(store: &Store, face_id: FaceId) -> Result<Option<Vec<crate::entity::EdgeId>>> {
    let face = store.get(face_id)?;
    if face.loops.len() != 1 || !matches!(store.get(face.surface)?, SurfaceGeom::Plane(_)) {
        return Ok(None);
    }
    let loop_id = face.loops[0];
    if certify_loop_simplicity(store, loop_id)? != LoopSimplicity::Certified {
        return Ok(None);
    }
    let loop_ = store.get(loop_id)?;
    if loop_.fins.len() != 4 {
        return Ok(None);
    }
    let mut edges = Vec::with_capacity(4);
    for &fin in &loop_.fins {
        let fin = store.get(fin)?;
        let edge = store.get(fin.edge)?;
        let Some(curve) = edge.curve else {
            return Ok(None);
        };
        if edge.tolerance.is_some()
            || edge.bounds.is_none()
            || !matches!(store.get(curve)?, CurveGeom::Line(_))
            || certify_edge_surface_incidence(store, fin.edge, face.surface, LINEAR_RESOLUTION)?
                != IncidenceCertification::Certified
            || edges.contains(&fin.edge)
        {
            return Ok(None);
        }
        edges.push(fin.edge);
    }
    Ok(Some(edges))
}

fn edge_has_vertices(
    store: &Store,
    edge: crate::entity::EdgeId,
    first: VertexId,
    second: VertexId,
) -> Result<bool> {
    let vertices = store.get(edge)?.vertices;
    Ok(matches!(
        vertices,
        [Some(a), Some(b)] if (a == first && b == second) || (a == second && b == first)
    ))
}

type CoplanarFacet = (FaceId, Vec<VertexId>);
type CoplanarFacetGroup = (Vec<VertexId>, Vec<CoplanarFacet>);

fn certify_convex_planar_shell(
    store: &Store,
    shell_id: ShellId,
    scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<ShellCertification> {
    let shell = store.get(shell_id)?;
    if shell.faces.len() < 4 {
        return Ok(indeterminate());
    }
    let mut shell_vertices = Vec::new();
    let mut facets = Vec::with_capacity(shell.faces.len());
    for &face_id in &shell.faces {
        let Some(vertices) = convex_planar_face_vertices(store, face_id)? else {
            return Ok(indeterminate());
        };
        for &vertex in &vertices {
            if !shell_vertices.contains(&vertex) {
                shell_vertices.push(vertex);
            }
        }
        facets.push((face_id, vertices));
    }
    if shell_vertices.len() < 4 {
        return Ok(indeterminate());
    }

    let mut orientation_invalid = false;
    let mut coplanar_groups: Vec<CoplanarFacetGroup> = Vec::new();
    for (face_id, loop_vertices) in facets {
        let face = store.get(face_id)?;
        let SurfaceGeom::Plane(plane) = store.get(face.surface)? else {
            return Ok(indeterminate());
        };
        let frame = plane.frame();
        let a = frame.origin();
        let b = a + frame.x();
        let c = a + frame.y();
        let mut positive = false;
        let mut negative = false;
        let mut coplanar = Vec::new();
        for &vertex in &shell_vertices {
            let point = store.vertex_position(vertex)?;
            let signed_distance = (point - a).dot(frame.z());
            let side = if signed_distance.abs() <= LINEAR_RESOLUTION {
                PredicateOrientation::Zero
            } else {
                orient3d(a.to_array(), b.to_array(), c.to_array(), point.to_array())
            };
            match side {
                PredicateOrientation::Positive => positive = true,
                PredicateOrientation::Negative => negative = true,
                PredicateOrientation::Zero => coplanar.push(vertex),
            }
        }
        if positive == negative {
            // Both sides occupied, or the whole shell is coplanar.
            return Ok(indeterminate());
        }
        if loop_vertices
            .iter()
            .any(|vertex| !coplanar.contains(vertex))
        {
            return Ok(indeterminate());
        }
        let expected = if positive {
            // orient3d is positive below the frame's +z plane, so +z points
            // away from vertices on that side.
            Sense::Forward
        } else {
            Sense::Reversed
        };
        orientation_invalid |= face.sense != expected;

        if coplanar.len() > loop_vertices.len() {
            let Some(exact_coplanar) =
                exact_supporting_plane_members(store, &loop_vertices, &shell_vertices)?
            else {
                return Ok(indeterminate());
            };
            if exact_coplanar.len() <= loop_vertices.len() {
                return Ok(indeterminate());
            }
            if let Some((_, group)) = coplanar_groups.iter_mut().find(|(members, _)| {
                members.len() == exact_coplanar.len()
                    && members.iter().all(|vertex| exact_coplanar.contains(vertex))
            }) {
                group.push((face_id, loop_vertices));
            } else {
                coplanar_groups.push((exact_coplanar, vec![(face_id, loop_vertices)]));
            }
        }
    }

    let pair_work = coplanar_pair_work(&coplanar_groups);
    if pair_work > 0
        && let Some(scope) = scope
    {
        scope
            .ledger_mut()
            .charge(SHELL_FACET_PAIR_WORK, pair_work)?;
    }
    for (_, group) in coplanar_groups {
        for left in 0..group.len() {
            for right in left + 1..group.len() {
                if !coplanar_facets_have_disjoint_interiors(
                    store,
                    group[left].0,
                    &group[left].1,
                    group[right].0,
                    &group[right].1,
                )? {
                    return Ok(indeterminate());
                }
            }
        }
    }

    Ok(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: if orientation_invalid {
            ShellOrientation::Invalid
        } else {
            ShellOrientation::Certified
        },
    })
}

/// Exact supporting-plane membership used only to widen one convex hull
/// facet into a partition group. The witness consists solely of stored loop
/// vertices; tolerance-band proximity never becomes coplanar identity.
fn exact_supporting_plane_members(
    store: &Store,
    loop_vertices: &[VertexId],
    shell_vertices: &[VertexId],
) -> Result<Option<Vec<VertexId>>> {
    let witness = [
        store.vertex_position(loop_vertices[0])?.to_array(),
        store.vertex_position(loop_vertices[1])?.to_array(),
        store.vertex_position(loop_vertices[2])?.to_array(),
    ];
    let mut positive = false;
    let mut negative = false;
    let mut coplanar = Vec::new();
    for &vertex in shell_vertices {
        let point = store.vertex_position(vertex)?.to_array();
        match orient3d(witness[0], witness[1], witness[2], point) {
            PredicateOrientation::Positive => positive = true,
            PredicateOrientation::Negative => negative = true,
            PredicateOrientation::Zero => coplanar.push(vertex),
        }
    }
    if positive == negative
        || loop_vertices
            .iter()
            .any(|vertex| !coplanar.contains(vertex))
    {
        Ok(None)
    } else {
        Ok(Some(coplanar))
    }
}

fn coplanar_pair_work(groups: &[CoplanarFacetGroup]) -> u64 {
    let mut total = 0_u64;
    for (_, group) in groups {
        for left in 0..group.len() {
            for right in left + 1..group.len() {
                let left_edges = group[left].1.len() as u64;
                let right_edges = group[right].1.len() as u64;
                let pair = left_edges
                    .saturating_mul(right_edges)
                    .saturating_add(left_edges)
                    .saturating_add(right_edges)
                    .saturating_add(1);
                total = total.saturating_add(pair);
            }
        }
    }
    total
}

/// Prove that two convex facets on one supporting plane meet only through
/// shared topological boundary entities. Projection drops the dominant plane
/// axis, so every orientation and segment decision is made by exact `orient2d`
/// signs over stored vertex coordinates rather than derived UV samples.
fn coplanar_facets_have_disjoint_interiors(
    store: &Store,
    left_face: FaceId,
    left_vertices: &[VertexId],
    right_face: FaceId,
    right_vertices: &[VertexId],
) -> Result<bool> {
    let SurfaceGeom::Plane(plane) = store.get(store.get(left_face)?.surface)? else {
        return Ok(false);
    };
    let normal = plane.frame().z();
    let dropped_axis = if normal.x.abs() >= normal.y.abs() && normal.x.abs() >= normal.z.abs() {
        0
    } else if normal.y.abs() >= normal.z.abs() {
        1
    } else {
        2
    };
    let left = projected_facet(store, left_face, left_vertices, dropped_axis)?;
    let right = projected_facet(store, right_face, right_vertices, dropped_axis)?;
    if left.vertices.len() < 3 || right.vertices.len() < 3 {
        return Ok(false);
    }
    if left.vertices.len() == right.vertices.len()
        && left.vertices.iter().all(|vertex| {
            right
                .vertices
                .iter()
                .any(|candidate| candidate.vertex == vertex.vertex)
        })
    {
        // Equal topological boundary cycles have coincident interiors; shared
        // edge identities authorize a partition seam, never a duplicate face.
        return Ok(false);
    }

    for left_edge in &left.edges {
        for right_edge in &right.edges {
            match segment_contact(left_edge, right_edge) {
                SegmentContact::None => {}
                SegmentContact::Endpoint(left_vertex, right_vertex) => {
                    if left_vertex != right_vertex {
                        return Ok(false);
                    }
                }
                SegmentContact::Overlap => {
                    if left_edge.edge != right_edge.edge {
                        return Ok(false);
                    }
                }
                SegmentContact::Proper => return Ok(false),
            }
        }
    }

    for vertex in &left.vertices {
        match convex_point_location(vertex.point, &right.vertices) {
            ConvexPointLocation::Outside => {}
            ConvexPointLocation::Boundary => {
                if !right
                    .vertices
                    .iter()
                    .any(|candidate| candidate.vertex == vertex.vertex)
                {
                    return Ok(false);
                }
            }
            ConvexPointLocation::Inside => return Ok(false),
        }
    }
    for vertex in &right.vertices {
        match convex_point_location(vertex.point, &left.vertices) {
            ConvexPointLocation::Outside => {}
            ConvexPointLocation::Boundary => {
                if !left
                    .vertices
                    .iter()
                    .any(|candidate| candidate.vertex == vertex.vertex)
                {
                    return Ok(false);
                }
            }
            ConvexPointLocation::Inside => return Ok(false),
        }
    }
    Ok(true)
}

#[derive(Debug, Clone, Copy)]
struct ProjectedVertex {
    vertex: VertexId,
    point: [f64; 2],
}

#[derive(Debug, Clone, Copy)]
struct ProjectedEdge {
    edge: crate::entity::EdgeId,
    start: ProjectedVertex,
    end: ProjectedVertex,
}

struct ProjectedFacet {
    vertices: Vec<ProjectedVertex>,
    edges: Vec<ProjectedEdge>,
}

fn projected_facet(
    store: &Store,
    face_id: FaceId,
    vertices: &[VertexId],
    dropped_axis: usize,
) -> Result<ProjectedFacet> {
    let face = store.get(face_id)?;
    let loop_ = store.get(face.loops[0])?;
    if loop_.fins.len() != vertices.len() {
        return Ok(ProjectedFacet {
            vertices: Vec::new(),
            edges: Vec::new(),
        });
    }
    let mut projected = Vec::with_capacity(vertices.len());
    for &vertex in vertices {
        let point = store.vertex_position(vertex)?.to_array();
        let point = match dropped_axis {
            0 => [point[1], point[2]],
            1 => [point[0], point[2]],
            _ => [point[0], point[1]],
        };
        projected.push(ProjectedVertex { vertex, point });
    }
    let mut edges = Vec::with_capacity(projected.len());
    for (index, &fin_id) in loop_.fins.iter().enumerate() {
        edges.push(ProjectedEdge {
            edge: store.get(fin_id)?.edge,
            start: projected[index],
            end: projected[(index + 1) % projected.len()],
        });
    }
    Ok(ProjectedFacet {
        vertices: projected,
        edges,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SegmentContact {
    None,
    Endpoint(VertexId, VertexId),
    Overlap,
    Proper,
}

fn segment_contact(left: &ProjectedEdge, right: &ProjectedEdge) -> SegmentContact {
    let o1 = orient2d(left.start.point, left.end.point, right.start.point);
    let o2 = orient2d(left.start.point, left.end.point, right.end.point);
    let o3 = orient2d(right.start.point, right.end.point, left.start.point);
    let o4 = orient2d(right.start.point, right.end.point, left.end.point);
    if opposite(o1, o2) && opposite(o3, o4) {
        return SegmentContact::Proper;
    }

    let mut contacts = Vec::new();
    for (orientation, endpoint, endpoint_vertex, segment_start, segment_end) in [
        (o1, right.start, right.start.vertex, left.start, left.end),
        (o2, right.end, right.end.vertex, left.start, left.end),
        (o3, left.start, left.start.vertex, right.start, right.end),
        (o4, left.end, left.end.vertex, right.start, right.end),
    ] {
        if orientation != PredicateOrientation::Zero
            || !point_on_closed_segment(endpoint.point, segment_start.point, segment_end.point)
        {
            continue;
        }
        let Some(segment_vertex) = (endpoint.point == segment_start.point)
            .then_some(segment_start.vertex)
            .or_else(|| (endpoint.point == segment_end.point).then_some(segment_end.vertex))
        else {
            // A T-junction is an improper facet contact even when the
            // touching point happens to carry another topological identity.
            return SegmentContact::Proper;
        };
        let pair = if endpoint.point == right.start.point || endpoint.point == right.end.point {
            (segment_vertex, endpoint_vertex, endpoint.point)
        } else {
            (endpoint_vertex, segment_vertex, endpoint.point)
        };
        contacts.push(pair);
    }
    contacts.sort_by(|left, right| {
        left.2[0]
            .total_cmp(&right.2[0])
            .then_with(|| left.2[1].total_cmp(&right.2[1]))
    });
    contacts.dedup_by(|a, b| a.2 == b.2);
    match contacts.len() {
        0 => SegmentContact::None,
        1 => SegmentContact::Endpoint(contacts[0].0, contacts[0].1),
        _ => SegmentContact::Overlap,
    }
}

fn opposite(left: PredicateOrientation, right: PredicateOrientation) -> bool {
    matches!(
        (left, right),
        (
            PredicateOrientation::Positive,
            PredicateOrientation::Negative
        ) | (
            PredicateOrientation::Negative,
            PredicateOrientation::Positive
        )
    )
}

fn point_on_closed_segment(point: [f64; 2], start: [f64; 2], end: [f64; 2]) -> bool {
    point[0] >= start[0].min(end[0])
        && point[0] <= start[0].max(end[0])
        && point[1] >= start[1].min(end[1])
        && point[1] <= start[1].max(end[1])
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ConvexPointLocation {
    Outside,
    Boundary,
    Inside,
}

fn convex_point_location(point: [f64; 2], polygon: &[ProjectedVertex]) -> ConvexPointLocation {
    if polygon.len() < 3 {
        return ConvexPointLocation::Outside;
    }
    let mut winding = None;
    let mut boundary = false;
    for index in 0..polygon.len() {
        let start = polygon[index].point;
        let end = polygon[(index + 1) % polygon.len()].point;
        let side = orient2d(start, end, point);
        if side == PredicateOrientation::Zero {
            if point_on_closed_segment(point, start, end) {
                boundary = true;
                continue;
            }
            return ConvexPointLocation::Outside;
        }
        if let Some(expected) = winding {
            if side != expected {
                return ConvexPointLocation::Outside;
            }
        } else {
            winding = Some(side);
        }
    }
    if boundary {
        ConvexPointLocation::Boundary
    } else {
        ConvexPointLocation::Inside
    }
}

fn convex_planar_face_vertices(store: &Store, face_id: FaceId) -> Result<Option<Vec<VertexId>>> {
    let face = store.get(face_id)?;
    let SurfaceGeom::Plane(plane) = store.get(face.surface)? else {
        return Ok(None);
    };
    if face.loops.len() != 1 {
        return Ok(None);
    }
    let loop_id = face.loops[0];
    if certify_loop_simplicity(store, loop_id)? != LoopSimplicity::Certified {
        return Ok(None);
    }
    let loop_ = store.get(loop_id)?;
    if loop_.fins.len() < 3 {
        return Ok(None);
    }
    let mut vertices = Vec::with_capacity(loop_.fins.len());
    let mut points = Vec::with_capacity(loop_.fins.len());
    for &fin_id in &loop_.fins {
        let fin = store.get(fin_id)?;
        let edge = store.get(fin.edge)?;
        let Some(curve_id) = edge.curve else {
            return Ok(None);
        };
        if edge.tolerance.is_some() || !matches!(store.get(curve_id)?, CurveGeom::Line(_)) {
            return Ok(None);
        }
        let Some(vertex) = store.fin_tail(fin_id)? else {
            return Ok(None);
        };
        if vertices.contains(&vertex) {
            return Ok(None);
        }
        let local = plane.frame().to_local(store.vertex_position(vertex)?);
        vertices.push(vertex);
        points.push(Point2::new(local.x, local.y));
    }
    if !strictly_convex(&points) {
        return Ok(None);
    }
    Ok(Some(vertices))
}

fn strictly_convex(points: &[Point2]) -> bool {
    let mut winding = None;
    for index in 0..points.len() {
        let a = points[index];
        let b = points[(index + 1) % points.len()];
        let c = points[(index + 2) % points.len()];
        let turn = orient2d([a.x, a.y], [b.x, b.y], [c.x, c.y]);
        if turn == PredicateOrientation::Zero {
            return false;
        }
        if let Some(winding) = winding {
            if turn != winding {
                return false;
            }
        } else {
            winding = Some(turn);
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::make::{block, cylinder, extrude_profile, extrude_profile_along, sphere, torus};
    use crate::planar::{PlanarSolidFace, PlanarSolidInput, PlanarSolidVertex, PlanarVertexKey};
    use crate::profile::PlanarProfile;
    use crate::store::Store;
    use crate::transaction::FullCommitRequirement;
    use kgeom::frame::Frame;
    use kgeom::vec::{Point2, Point3, Vec3};

    fn solid_shell(store: &Store, body: crate::entity::BodyId) -> ShellId {
        let solid = store
            .get(body)
            .unwrap()
            .regions
            .iter()
            .copied()
            .find(|&region| store.get(region).unwrap().kind == RegionKind::Solid)
            .unwrap();
        store.get(solid).unwrap().shells[0]
    }

    #[test]
    fn convex_block_shell_is_embedded_and_orientation_is_decidable() {
        let mut store = Store::new();
        let body = block(&mut store, &Frame::world(), [2.0, 3.0, 4.0]).unwrap();
        let shell = solid_shell(&store, body);
        assert_eq!(
            certify_shell(&store, shell, BodyKind::Solid, RegionKind::Solid).unwrap(),
            ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Certified,
            }
        );

        let face = store.get(shell).unwrap().faces[0];
        store.get_mut(face).unwrap().sense = store.get(face).unwrap().sense.flipped();
        assert_eq!(
            certify_shell(&store, shell, BodyKind::Solid, RegionKind::Solid)
                .unwrap()
                .orientation,
            ShellOrientation::Invalid
        );
    }

    #[test]
    fn convex_shell_with_coplanar_facet_partitions_is_certified() {
        let points = [
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(0.0, -1.0, -1.0),
            Point3::new(1.0, -1.0, -1.0),
            Point3::new(-1.0, 1.0, -1.0),
            Point3::new(0.0, 1.0, -1.0),
            Point3::new(1.0, 1.0, -1.0),
            Point3::new(-1.0, -1.0, 1.0),
            Point3::new(0.0, -1.0, 1.0),
            Point3::new(1.0, -1.0, 1.0),
            Point3::new(-1.0, 1.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
        ];
        let keys: Vec<_> = (0..points.len())
            .map(|index| PlanarVertexKey::new(index as u64 + 1))
            .collect();
        let vertices = keys
            .iter()
            .copied()
            .zip(points)
            .map(|(key, point)| PlanarSolidVertex::new(key, point))
            .collect();
        let rings = [
            [0, 3, 4, 1],
            [1, 4, 5, 2],
            [6, 7, 10, 9],
            [7, 8, 11, 10],
            [0, 1, 7, 6],
            [1, 2, 8, 7],
            [3, 9, 10, 4],
            [4, 10, 11, 5],
            [0, 6, 9, 3],
            [2, 5, 11, 8],
        ];
        let faces = rings
            .into_iter()
            .map(|ring| PlanarSolidFace::new(ring.into_iter().map(|index| keys[index]).collect()))
            .collect();
        let input = PlanarSolidInput::new(vertices, faces);

        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction.assemble_planar_solid(&input).unwrap();
        assert_eq!(
            transaction.store().get(output.shell()).unwrap().faces.len(),
            10
        );
        let mut perturbed = transaction.store().clone();
        let moved = output.vertex(keys[2]).unwrap();
        let point = perturbed.get(moved).unwrap().point;
        perturbed.get_mut(point).unwrap().z += LINEAR_RESOLUTION / 2.0;
        let partition_face = output.faces()[1];
        let partition_vertices = convex_planar_face_vertices(&perturbed, partition_face)
            .unwrap()
            .unwrap();
        let shell_vertices = output
            .vertices()
            .iter()
            .map(|(_, vertex)| *vertex)
            .collect::<Vec<_>>();
        assert!(
            exact_supporting_plane_members(&perturbed, &partition_vertices, &shell_vertices)
                .unwrap()
                .is_none(),
            "subresolution plane tilt must not become exact coplanar identity"
        );

        let check_with_limit = |allowed| {
            let budget = BudgetPlan::new([LimitSpec::new(
                SHELL_FACET_PAIR_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            )])
            .unwrap();
            let session = kcore::operation::SessionPolicy::new(
                kcore::operation::SessionPrecision::parasolid(),
                kcore::operation::NumericalPolicy::v1(),
                kcore::operation::ExecutionPolicy::Serial,
                budget,
                kcore::operation::PolicyVersion::V1,
            );
            let context = kcore::operation::OperationContext::new(
                &session,
                kcore::tolerance::Tolerances::default(),
            )
            .unwrap();
            crate::check::check_body_report_with_context(
                transaction.store(),
                output.body(),
                crate::check::CheckLevel::Full,
                &context,
            )
            .unwrap()
        };
        let denied = check_with_limit(99);
        assert_eq!(
            denied.result().as_ref().unwrap_err().limit(),
            Some(kcore::operation::LimitSnapshot {
                stage: SHELL_FACET_PAIR_WORK,
                resource: ResourceKind::Work,
                consumed: 100,
                allowed: 99,
            })
        );
        let accepted = check_with_limit(100);
        assert_eq!(
            accepted.result().as_ref().unwrap().outcome(),
            crate::check::CheckOutcome::Valid
        );
        assert!(
            accepted
                .report()
                .usage()
                .contains(&kcore::operation::LimitSnapshot {
                    stage: SHELL_FACET_PAIR_WORK,
                    resource: ResourceKind::Work,
                    consumed: 100,
                    allowed: 100,
                })
        );

        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(decision.is_committed());
        assert!(
            decision
                .checks()
                .iter()
                .all(|check| check.report().outcome() == crate::check::CheckOutcome::Valid)
        );
    }

    #[test]
    fn duplicate_coplanar_facet_is_not_a_disjoint_partition() {
        let mut store = Store::new();
        let body = block(&mut store, &Frame::world(), [2.0, 3.0, 4.0]).unwrap();
        let shell = solid_shell(&store, body);
        let face = store.get(shell).unwrap().faces[0];
        let vertices = convex_planar_face_vertices(&store, face)
            .unwrap()
            .expect("block face is a convex planar facet");
        assert!(
            !coplanar_facets_have_disjoint_interiors(&store, face, &vertices, face, &vertices,)
                .unwrap()
        );
    }

    #[test]
    fn whole_sphere_and_torus_shells_are_embedded_and_oriented() {
        let mut store = Store::new();
        for body in [
            sphere(&mut store, &Frame::world(), 1.0).unwrap(),
            torus(&mut store, &Frame::world(), 2.0, 0.5).unwrap(),
        ] {
            let shell = solid_shell(&store, body);
            assert_eq!(
                certify_shell(&store, shell, BodyKind::Solid, RegionKind::Solid).unwrap(),
                ShellCertification {
                    embedding: ShellEmbedding::Certified,
                    orientation: ShellOrientation::Certified,
                }
            );
        }
    }

    #[test]
    fn polygonal_profile_prism_is_embedded_and_orientation_is_decidable() {
        let outer = [
            Point2::new(-2.0, -2.0),
            Point2::new(2.0, -2.0),
            Point2::new(2.0, 2.0),
            Point2::new(-2.0, 2.0),
        ];
        let hole = [
            Point2::new(-1.0, -1.0),
            Point2::new(1.0, -1.0),
            Point2::new(1.0, 1.0),
            Point2::new(-1.0, 1.0),
        ];
        let profile =
            PlanarProfile::from_polygon_with_holes(Frame::world(), &outer, &[&hole]).unwrap();
        let mut store = Store::new();
        let body = extrude_profile(&mut store, &profile, 2.0).unwrap();
        let shell = solid_shell(&store, body);
        assert_eq!(
            certify_shell(&store, shell, BodyKind::Solid, RegionKind::Solid).unwrap(),
            ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Certified,
            }
        );

        let side = store.get(shell).unwrap().faces[2];
        store.get_mut(side).unwrap().sense = store.get(side).unwrap().sense.flipped();
        assert_eq!(
            certify_shell(&store, shell, BodyKind::Solid, RegionKind::Solid).unwrap(),
            ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Invalid,
            }
        );
    }

    #[test]
    fn oblique_polygonal_profile_prism_is_embedded_and_orientation_is_decidable() {
        let polygon = [
            Point2::new(-1.0, -1.0),
            Point2::new(1.0, -1.0),
            Point2::new(1.0, 1.0),
            Point2::new(-1.0, 1.0),
        ];
        let profile = PlanarProfile::from_polygon(Frame::world(), &polygon).unwrap();
        let mut store = Store::new();
        let body = extrude_profile_along(&mut store, &profile, Vec3::new(0.75, -0.5, 2.0)).unwrap();
        let shell = solid_shell(&store, body);
        assert_eq!(
            certify_shell(&store, shell, BodyKind::Solid, RegionKind::Solid).unwrap(),
            ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Certified,
            }
        );

        let side = store.get(shell).unwrap().faces[2];
        store.get_mut(side).unwrap().sense = store.get(side).unwrap().sense.flipped();
        assert_eq!(
            certify_shell(&store, shell, BodyKind::Solid, RegionKind::Solid)
                .unwrap()
                .orientation,
            ShellOrientation::Invalid
        );
    }

    #[test]
    fn unsupported_curved_multiface_shell_remains_indeterminate() {
        let mut store = Store::new();
        let body = cylinder(&mut store, &Frame::world(), 1.0, 2.0).unwrap();
        let shell = solid_shell(&store, body);
        assert_eq!(
            certify_shell(&store, shell, BodyKind::Solid, RegionKind::Solid).unwrap(),
            indeterminate()
        );
    }
}
