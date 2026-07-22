//! Shell theorem for two axially touching, strictly nested cylinder bands.
//!
//! Incidence discovers two complete cylinder bands with one far disk each and
//! one shared two-loop planar annulus. Exact outward interval arithmetic proves
//! strict radial disk containment. The annulus' oriented normal separates the
//! far ends onto opposite open half-spaces: the larger disk's band terminates
//! at the annulus while the smaller disk's band continues from its hole. Whole
//! fin incidence, loop containment, and source pcurves prove the four circular
//! seams. This identifies the shell with the regularized boundary of the two
//! product solids after their positive-area shared disk interface is removed.

use super::*;
use crate::semantic_planar_math::{
    cross as interval_cross, dot as interval_dot, point as interval_point, sub as interval_sub,
};

#[cfg(test)]
#[path = "parallel_cylinder_contact_shell_proof/tests.rs"]
mod tests;

/// Cumulative deterministic work for nested axial-contact shell proofs.
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

/// Attempt the incidence-discovered nested axial-contact theorem.
pub(super) fn certify_parallel_cylinder_contact_shell(
    store: &Store,
    shell_id: ShellId,
    scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellCertification>> {
    let shell = store.get(shell_id)?;
    if shell.faces.len() != 5 || !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(None);
    }
    let mut cylinders = Vec::with_capacity(2);
    let mut planes = Vec::with_capacity(3);
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
    let ([(first_face, first), (second_face, second)], [_, _, _]) =
        (cylinders.as_slice(), planes.as_slice())
    else {
        return Ok(None);
    };

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

    let Some(first) = prepare_band(store, shell_id, *first_face, *first)? else {
        return Ok(None);
    };
    let Some(second) = prepare_band(store, shell_id, *second_face, *second)? else {
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
    let axis = interval_point(axis.to_array());
    let displacement = interval_sub(
        interval_point(point.to_array()),
        interval_point(origin.to_array()),
    );
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
