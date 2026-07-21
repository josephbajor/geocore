//! Full shell theorem for normal translation sweeps of analytic profiles.
//!
//! R2 decomposition: a certified simple Plane loop bounds one Jordan domain
//! `D`; a second cap must be its bijective nonzero normal translation; every
//! remaining face must be exactly one four-edge product strip over one cap
//! edge. Bounded Line edges require Plane strips and bounded Circle edges
//! require Cylinder strips, while the two other strip edges must be complete
//! translation rulings. Whole-fin incidence proves the authored pcurves over
//! every complete edge range. These local witnesses identify the shell with
//! `boundary(D x [0,1])`, so global embedding follows without convexity,
//! layout tags, constructor provenance, or sampled sidedness. Any unsupported
//! carrier, ambiguous correspondence, incomplete strip, or inconclusive
//! interval comparison returns no theorem.

use super::*;
use crate::entity::{EdgeId, FinId};
use kgeom::curve::{Circle, Line};
use kgeom::param::ParamRange;

/// Cumulative deterministic work for mixed analytic profile-prism proofs.
pub(crate) const MIXED_PROFILE_PRISM_WORK: StageId =
    match StageId::new("ktopo.check.mixed-profile-prism-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid mixed profile-prism work stage"),
    };

// The next power of two above the 2,851,200-work ten-support non-convex
// extrusion fixture, leaving deterministic headroom without changing the
// input-size-exact work formula.
const DEFAULT_MIXED_PROFILE_PRISM_WORK: u64 = 4_194_304;

pub(super) fn mixed_profile_prism_proof_budget() -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(
        MIXED_PROFILE_PRISM_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        DEFAULT_MIXED_PROFILE_PRISM_WORK,
    )])
    .expect("built-in mixed profile-prism proof budget is valid")
}

#[derive(Debug, Clone, Copy)]
pub(super) enum ProfileCarrier {
    Line(Line),
    Circle(Circle),
}

#[derive(Debug, Clone, Copy)]
pub(super) struct CapUse {
    pub(super) fin: FinId,
    pub(super) edge: EdgeId,
    pub(super) tail: VertexId,
    pub(super) head: VertexId,
    pub(super) carrier: ProfileCarrier,
    pub(super) range: ParamRange,
}

#[derive(Debug)]
pub(super) struct Cap {
    pub(super) face: FaceId,
    pub(super) plane: kgeom::surface::Plane,
    pub(super) vertices: Vec<VertexId>,
    pub(super) uses: Vec<CapUse>,
    pub(super) local_orientation_valid: bool,
}

#[derive(Debug)]
pub(super) struct Translation {
    pub(super) vector: Vec3,
    pub(super) vertices: Vec<(VertexId, VertexId)>,
}

#[derive(Debug)]
pub(super) struct Side {
    pub(super) face: FaceId,
    pub(super) fins: Vec<(FinId, EdgeId)>,
}

/// Attempt the representation-independent product-shell theorem.
pub(super) fn certify_mixed_profile_prism(
    store: &Store,
    shell_id: ShellId,
    scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellCertification>> {
    let shell = store.get(shell_id)?;
    if shell.faces.len() < 4 || !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(None);
    }
    let mut planar_faces = Vec::new();
    let mut has_cylinder = false;
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        if face.shell != shell_id {
            return Ok(None);
        }
        match store.get(face.surface)? {
            SurfaceGeom::Plane(_) => planar_faces.push(face_id),
            SurfaceGeom::Cylinder(_) => has_cylinder = true,
            _ => return Ok(None),
        }
    }
    if planar_faces.len() < 2 || !has_cylinder {
        return Ok(None);
    }

    if let Some(scope) = scope {
        scope.ledger().require_limit(
            MIXED_PROFILE_PRISM_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )?;
        let Some(work) = proof_work(store, shell_id, planar_faces.len())? else {
            return Ok(Some(indeterminate()));
        };
        scope.ledger_mut().charge(MIXED_PROFILE_PRISM_WORK, work)?;
    }

    for (index, &first) in planar_faces.iter().enumerate() {
        for &second in &planar_faces[index + 1..] {
            if let Some(candidate) = certify_cap_pair(store, shell_id, first, second)? {
                // Existence of one complete D x [0,1] decomposition is the
                // embedding witness. Highly symmetric all-planar prisms can
                // have several equally authoritative sweep axes.
                return Ok(Some(candidate));
            }
        }
    }
    Ok(None)
}

/// Checked upper bound for cap-pair search and all structural comparisons.
///
/// With `N = 1 + F + L + U + E + V`, every planar face pair performs at
/// most `N^2 + 16N` visits/comparisons. Multiplying by the exact unordered
/// planar-pair count bounds vertex matching, edge/side bijections, loop scans,
/// and carrier checks before the search allocates any topology.
fn proof_work(store: &Store, shell_id: ShellId, plane_count: usize) -> Result<Option<u64>> {
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
    let Some(faces) = u64::try_from(shell.faces.len()).ok() else {
        return Ok(None);
    };
    let Some(edges) = u64::try_from(edges.len()).ok() else {
        return Ok(None);
    };
    let Some(vertices) = u64::try_from(vertices.len()).ok() else {
        return Ok(None);
    };
    let Some(planes) = u64::try_from(plane_count).ok() else {
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
    let Some(pair_count) = planes
        .checked_sub(1)
        .and_then(|less| planes.checked_mul(less))
        .map(|ordered| ordered / 2)
    else {
        return Ok(None);
    };
    Ok(size
        .checked_mul(size)
        .and_then(|quadratic| quadratic.checked_add(size.checked_mul(16)?))
        .and_then(|per_pair| per_pair.checked_mul(pair_count)))
}

fn certify_cap_pair(
    store: &Store,
    shell_id: ShellId,
    first: FaceId,
    second: FaceId,
) -> Result<Option<ShellCertification>> {
    let Some(first) = prepare_cap(store, first)? else {
        return Ok(None);
    };
    let Some(second) = prepare_cap(store, second)? else {
        return Ok(None);
    };
    let shell = store.get(shell_id)?;
    if first.uses.len() != second.uses.len()
        || first.vertices.len() != second.vertices.len()
        || shell.faces.len() != first.uses.len() + 2
    {
        return Ok(None);
    }
    let Some(translation) = translated_vertices(store, &first, &second)? else {
        return Ok(None);
    };
    if !certified_parallel(translation.vector, first.plane.frame().z())
        || !certified_parallel(translation.vector, second.plane.frame().z())
        || !certified_parallel(first.plane.frame().z(), second.plane.frame().z())
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
    let mut orientation_signs = vec![first_sign, second_sign];
    let mut orientation_invalid = !first.local_orientation_valid || !second.local_orientation_valid;

    let second_edges = second.uses.iter().map(|use_| use_.edge).collect::<Vec<_>>();
    let side_faces = shell
        .faces
        .iter()
        .copied()
        .filter(|face| *face != first.face && *face != second.face)
        .collect::<Vec<_>>();
    let mut used_sides = Vec::with_capacity(side_faces.len());
    let mut used_second_edges = Vec::with_capacity(second_edges.len());
    for boundary in &first.uses {
        let Some(side_face_id) = peer_face(store, *boundary)? else {
            return Ok(None);
        };
        if !side_faces.contains(&side_face_id) || used_sides.contains(&side_face_id) {
            return Ok(None);
        }
        let Some(side) = prepare_side(store, side_face_id)? else {
            return Ok(None);
        };
        let Some(mapped_tail) = mapped_vertex(&translation.vertices, boundary.tail) else {
            return Ok(None);
        };
        let Some(mapped_head) = mapped_vertex(&translation.vertices, boundary.head) else {
            return Ok(None);
        };
        let mut matching_top = Vec::new();
        for candidate in &second.uses {
            if edge_has_vertices(store, candidate.edge, mapped_tail, mapped_head)?
                && translated_carrier(*boundary, *candidate, translation.vector)
            {
                matching_top.push(candidate);
            }
        }
        let [mapped_top] = matching_top.as_slice() else {
            return Ok(None);
        };
        if used_second_edges.contains(&mapped_top.edge)
            || !side.fins.iter().any(|(_, edge)| *edge == boundary.edge)
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
        orientation_signs.push(side_sign);
        used_sides.push(side.face);
        used_second_edges.push(mapped_top.edge);
    }
    if used_sides.len() != side_faces.len() || used_second_edges.len() != second_edges.len() {
        return Ok(None);
    }
    orientation_invalid |= orientation_signs
        .iter()
        .any(|sign| *sign != orientation_signs[0]);
    let orientation = if orientation_invalid {
        ShellOrientation::Invalid
    } else if orientation_signs[0] > 0 {
        ShellOrientation::Positive
    } else {
        ShellOrientation::Negative
    };
    Ok(Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation,
    }))
}

pub(super) fn prepare_cap(store: &Store, face_id: FaceId) -> Result<Option<Cap>> {
    let face = store.get(face_id)?;
    let SurfaceGeom::Plane(plane) = store.get(face.surface)? else {
        return Ok(None);
    };
    let [loop_id] = face.loops.as_slice() else {
        return Ok(None);
    };
    if certify_loop_simplicity(store, *loop_id)? != LoopSimplicity::Certified {
        return Ok(None);
    }
    let Some(loop_orientation) = certify_loop_orientation(store, face_id, *loop_id)? else {
        return Ok(None);
    };
    let loop_ = store.get(*loop_id)?;
    if loop_.face != face_id || loop_.fins.len() < 2 {
        return Ok(None);
    }
    let mut vertices = Vec::with_capacity(loop_.fins.len());
    let mut uses = Vec::with_capacity(loop_.fins.len());
    for &fin_id in &loop_.fins {
        if certify_whole_fin_incidence(store, face_id, *loop_id, fin_id, LINEAR_RESOLUTION)
            != WholeFinIncidence::Certified
        {
            return Ok(None);
        }
        let fin = store.get(fin_id)?;
        let edge = store.get(fin.edge)?;
        let (Some(curve_id), Some((lo, hi)), Some(tail), Some(head)) = (
            edge.curve,
            edge.bounds,
            store.fin_tail(fin_id)?,
            store.fin_head(fin_id)?,
        ) else {
            return Ok(None);
        };
        if edge.tolerance.is_some()
            || !lo.is_finite()
            || !hi.is_finite()
            || lo >= hi
            || edge.fins.len() != 2
            || uses.iter().any(|use_: &CapUse| use_.edge == fin.edge)
        {
            return Ok(None);
        }
        let curve = store.get(curve_id)?;
        let carrier = match (exact_line_carrier(curve), curve) {
            (Some(line), _) => ProfileCarrier::Line(line),
            (None, CurveGeom::Circle(circle))
                if hi - lo < circle.param_range().width()
                    && certified_parallel(circle.frame().z(), plane.frame().z()) =>
            {
                ProfileCarrier::Circle(*circle)
            }
            _ => return Ok(None),
        };
        if certify_edge_surface_incidence(store, fin.edge, face.surface, LINEAR_RESOLUTION)?
            != IncidenceCertification::Certified
            || vertices.contains(&tail)
        {
            return Ok(None);
        }
        vertices.push(tail);
        uses.push(CapUse {
            fin: fin_id,
            edge: fin.edge,
            tail,
            head,
            carrier,
            range: ParamRange::new(lo, hi),
        });
    }
    if uses.iter().any(|use_| !vertices.contains(&use_.head)) {
        return Ok(None);
    }
    Ok(Some(Cap {
        face: face_id,
        plane: *plane,
        vertices,
        uses,
        local_orientation_valid: (loop_orientation == PredicateOrientation::Positive)
            == face.sense.is_forward(),
    }))
}

pub(super) fn translated_vertices(
    store: &Store,
    first: &Cap,
    second: &Cap,
) -> Result<Option<Translation>> {
    let anchor = store.vertex_position(first.vertices[0])?;
    let mut translations = Vec::new();
    for &candidate in &second.vertices {
        let vector = store.vertex_position(candidate)? - anchor;
        if !certified_nonzero(vector)
            || !certified_parallel(vector, first.plane.frame().z())
            || !certified_parallel(vector, second.plane.frame().z())
        {
            continue;
        }
        let mut map = Vec::with_capacity(first.vertices.len());
        let mut used = Vec::with_capacity(second.vertices.len());
        for &source in &first.vertices {
            let expected = store.vertex_position(source)? + vector;
            let mut matches = Vec::new();
            for &target in &second.vertices {
                if !used.contains(&target)
                    && certified_close(expected, store.vertex_position(target)?)
                {
                    matches.push(target);
                }
            }
            let [target] = matches.as_slice() else {
                map.clear();
                break;
            };
            used.push(*target);
            map.push((source, *target));
        }
        if map.len() == first.vertices.len() && used.len() == second.vertices.len() {
            translations.push(Translation {
                vector,
                vertices: map,
            });
        }
    }
    Ok(match translations.len() {
        1 => translations.pop(),
        _ => None,
    })
}

pub(super) fn peer_face(store: &Store, use_: CapUse) -> Result<Option<FaceId>> {
    let edge = store.get(use_.edge)?;
    let [first, second] = edge.fins.as_slice() else {
        return Ok(None);
    };
    let peer = if *first == use_.fin {
        *second
    } else if *second == use_.fin {
        *first
    } else {
        return Ok(None);
    };
    if store.get(peer)?.sense == store.get(use_.fin)?.sense {
        return Ok(None);
    }
    Ok(Some(store.get(store.get(peer)?.parent)?.face))
}

pub(super) fn prepare_side(store: &Store, face_id: FaceId) -> Result<Option<Side>> {
    let face = store.get(face_id)?;
    if !matches!(
        store.get(face.surface)?,
        SurfaceGeom::Plane(_) | SurfaceGeom::Cylinder(_)
    ) {
        return Ok(None);
    }
    let [loop_id] = face.loops.as_slice() else {
        return Ok(None);
    };
    let loop_ = store.get(*loop_id)?;
    if loop_.face != face_id
        || loop_.fins.len() != 4
        || certify_loop_simplicity(store, *loop_id)? != LoopSimplicity::Certified
    {
        return Ok(None);
    }
    let mut fins = Vec::with_capacity(4);
    for &fin_id in &loop_.fins {
        if certify_whole_fin_incidence(store, face_id, *loop_id, fin_id, LINEAR_RESOLUTION)
            != WholeFinIncidence::Certified
        {
            return Ok(None);
        }
        let fin = store.get(fin_id)?;
        let edge = store.get(fin.edge)?;
        if edge.tolerance.is_some()
            || edge.bounds.is_none()
            || edge.curve.is_none()
            || fins.iter().any(|(_, prior)| *prior == fin.edge)
        {
            return Ok(None);
        }
        fins.push((fin_id, fin.edge));
    }
    Ok(Some(Side {
        face: face_id,
        fins,
    }))
}

pub(super) fn translated_carrier(first_use: CapUse, second_use: CapUse, translation: Vec3) -> bool {
    match (first_use.carrier, second_use.carrier) {
        (ProfileCarrier::Line(first), ProfileCarrier::Line(second)) => {
            certified_parallel(first.dir(), second.dir())
                && translated_interval_matches(
                    first.eval(first_use.range.lo),
                    first.eval(first_use.range.hi),
                    second.eval(second_use.range.lo),
                    second.eval(second_use.range.hi),
                    translation,
                )
        }
        (ProfileCarrier::Circle(first), ProfileCarrier::Circle(second)) => {
            first.radius().to_bits() == second.radius().to_bits()
                && certified_parallel(first.frame().z(), second.frame().z())
                && certified_close(
                    first.frame().origin() + translation,
                    second.frame().origin(),
                )
                && certified_equal_span(first_use.range, second_use.range)
                && translated_arc_matches(
                    first,
                    first_use.range,
                    second,
                    second_use.range,
                    translation,
                )
        }
        _ => false,
    }
}

fn translated_interval_matches(
    first_lo: Point3,
    first_hi: Point3,
    second_lo: Point3,
    second_hi: Point3,
    translation: Vec3,
) -> bool {
    (certified_close(first_lo + translation, second_lo)
        && certified_close(first_hi + translation, second_hi))
        || (certified_close(first_lo + translation, second_hi)
            && certified_close(first_hi + translation, second_lo))
}

fn certified_equal_span(first: ParamRange, second: ParamRange) -> bool {
    let difference = Interval::point(first.width()) - Interval::point(second.width());
    difference.lo().is_finite()
        && difference.lo() >= -ANGULAR_RESOLUTION
        && difference.hi() <= ANGULAR_RESOLUTION
}

fn translated_arc_matches(
    first: Circle,
    first_range: ParamRange,
    second: Circle,
    second_range: ParamRange,
    translation: Vec3,
) -> bool {
    let first_mid = 0.5 * (first_range.lo + first_range.hi);
    let second_mid = 0.5 * (second_range.lo + second_range.hi);
    let first_points = [
        first.eval(first_range.lo),
        first.eval(first_mid),
        first.eval(first_range.hi),
    ];
    let second_points = [
        second.eval(second_range.lo),
        second.eval(second_mid),
        second.eval(second_range.hi),
    ];
    first_points
        .iter()
        .zip(second_points.iter())
        .all(|(first, second)| certified_close(*first + translation, *second))
        || first_points
            .iter()
            .zip(second_points.iter().rev())
            .all(|(first, second)| certified_close(*first + translation, *second))
}

pub(super) fn ruling_connects(
    store: &Store,
    edge_id: EdgeId,
    first: VertexId,
    second: VertexId,
    translation: Vec3,
) -> Result<bool> {
    if !edge_has_vertices(store, edge_id, first, second)? {
        return Ok(false);
    }
    let edge = store.get(edge_id)?;
    let (Some(curve_id), Some((lo, hi)), [Some(low_vertex), Some(high_vertex)]) =
        (edge.curve, edge.bounds, edge.vertices)
    else {
        return Ok(false);
    };
    let Some(line) = exact_line_carrier(store.get(curve_id)?) else {
        return Ok(false);
    };
    if !lo.is_finite()
        || !hi.is_finite()
        || lo >= hi
        || !certified_parallel(line.dir(), translation)
        || !certified_close(line.eval(lo), store.vertex_position(low_vertex)?)
        || !certified_close(line.eval(hi), store.vertex_position(high_vertex)?)
    {
        return Ok(false);
    }
    let first_position = store.vertex_position(first)?;
    let second_position = store.vertex_position(second)?;
    Ok(
        certified_close(first_position + translation, second_position)
            || certified_close(second_position + translation, first_position),
    )
}

pub(super) fn certify_sweep_support(
    store: &Store,
    side: &Side,
    first: CapUse,
    second: CapUse,
    translation: Vec3,
) -> Result<Option<i8>> {
    let face = store.get(side.face)?;
    let midpoint = 0.5 * (first.range.lo + first.range.hi);
    let tangent = match first.carrier {
        ProfileCarrier::Line(line) => line.dir(),
        ProfileCarrier::Circle(circle) => circle.eval_derivs(midpoint, 1).d[1],
    } * if store.get(first.fin)?.sense == Sense::Forward {
        1.0
    } else {
        -1.0
    };
    let expected = translation.cross(tangent);
    if !certified_nonzero(expected) {
        return Ok(None);
    }
    let actual = match (first.carrier, second.carrier, store.get(face.surface)?) {
        (ProfileCarrier::Line(_), ProfileCarrier::Line(_), SurfaceGeom::Plane(plane)) => {
            if !certified_parallel(expected, plane.frame().z()) {
                return Ok(None);
            }
            plane.frame().z()
        }
        (
            ProfileCarrier::Circle(first_circle),
            ProfileCarrier::Circle(second_circle),
            SurfaceGeom::Cylinder(cylinder),
        ) => {
            if cylinder.radius().to_bits() != first_circle.radius().to_bits()
                || cylinder.radius().to_bits() != second_circle.radius().to_bits()
                || !certified_parallel(cylinder.frame().z(), translation)
                || !certified_parallel(first_circle.frame().z(), translation)
                || !certified_parallel(second_circle.frame().z(), translation)
                || !certified_point_on_axis(cylinder.frame(), first_circle.frame().origin())
                || !certified_point_on_axis(cylinder.frame(), second_circle.frame().origin())
            {
                return Ok(None);
            }
            let point = first_circle.eval(midpoint);
            let radial = point - first_circle.frame().origin();
            if !certified_parallel(expected, radial) {
                return Ok(None);
            }
            radial
        }
        _ => return Ok(None),
    } * sense_factor(face.sense);
    Ok(oriented_dot_sign(actual, expected))
}

pub(super) fn mapped_vertex(map: &[(VertexId, VertexId)], source: VertexId) -> Option<VertexId> {
    map.iter()
        .find_map(|&(candidate, target)| (candidate == source).then_some(target))
}

pub(super) fn edge_has_vertices(
    store: &Store,
    edge: EdgeId,
    first: VertexId,
    second: VertexId,
) -> Result<bool> {
    Ok(matches!(
        store.get(edge)?.vertices,
        [Some(a), Some(b)] if (a == first && b == second) || (a == second && b == first)
    ))
}

pub(super) fn certified_close(first: Point3, second: Point3) -> bool {
    let distance = [first.x, first.y, first.z]
        .into_iter()
        .zip([second.x, second.y, second.z])
        .fold(Interval::point(0.0), |sum, (left, right)| {
            sum + (Interval::point(left) - Interval::point(right)).square()
        });
    distance.hi().is_finite() && distance.hi() <= Interval::point(LINEAR_RESOLUTION).square().lo()
}

pub(super) fn certified_nonzero(vector: Vec3) -> bool {
    let norm = interval_norm_squared(vector);
    norm.lo().is_finite() && norm.lo() > Interval::point(LINEAR_RESOLUTION).square().hi()
}

pub(super) fn certified_parallel(first: Vec3, second: Vec3) -> bool {
    let cross = first.cross(second);
    let cross_norm = interval_norm_squared(cross);
    let allowed = Interval::point(ANGULAR_RESOLUTION).square()
        * interval_norm_squared(first)
        * interval_norm_squared(second);
    cross_norm.hi().is_finite() && allowed.lo().is_finite() && cross_norm.hi() <= allowed.lo()
}

fn interval_norm_squared(vector: Vec3) -> Interval {
    [vector.x, vector.y, vector.z]
        .into_iter()
        .map(|value| Interval::point(value).square())
        .fold(Interval::point(0.0), |sum, value| sum + value)
}

pub(super) fn oriented_dot_sign(first: Vec3, second: Vec3) -> Option<i8> {
    let dot = Interval::point(first.x) * Interval::point(second.x)
        + Interval::point(first.y) * Interval::point(second.y)
        + Interval::point(first.z) * Interval::point(second.z);
    if dot.lo() > 0.0 {
        Some(1)
    } else if dot.hi() < 0.0 {
        Some(-1)
    } else {
        None
    }
}

fn certified_point_on_axis(frame: &Frame, point: Point3) -> bool {
    let offset = point - frame.origin();
    let radial = [frame.x(), frame.y()]
        .into_iter()
        .map(|axis| {
            let dot = Interval::point(axis.x) * Interval::point(offset.x)
                + Interval::point(axis.y) * Interval::point(offset.y)
                + Interval::point(axis.z) * Interval::point(offset.z);
            dot.square()
        })
        .fold(Interval::point(0.0), |sum, value| sum + value);
    radial.hi().is_finite() && radial.hi() <= Interval::point(LINEAR_RESOLUTION).square().lo()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analytic_shell::tests::half_cylinder_input;
    use crate::analytic_shell::{
        AnalyticEdgeKey, AnalyticFaceKey, AnalyticPcurveUse, AnalyticShellCurve, AnalyticShellEdge,
        AnalyticShellFace, AnalyticShellFin, AnalyticShellInput, AnalyticShellLoop,
        AnalyticShellPcurve, AnalyticShellSurface, AnalyticShellVertex, AnalyticVertexKey,
    };
    use crate::check::{CheckLevel, CheckOutcome, check_body_report};
    use crate::entity::FaceDomain;
    use crate::transaction::FullCommitRequirement;
    use kgeom::curve2d::{Circle2d, Line2d};
    use kgeom::surface::Plane;
    use kgeom::vec::Vec2;
    use kgraph::AffineParamMap1d;

    fn parameter_map(scale: f64) -> AffineParamMap1d {
        AffineParamMap1d::new(scale, 0.0).unwrap()
    }

    fn plane_line_use(
        edge: AnalyticEdgeKey,
        sense: Sense,
        plane: Plane,
        line: Line,
    ) -> AnalyticShellFin {
        let origin = plane.frame().to_local(line.origin());
        let direction = line.dir();
        let local_direction = Vec2::new(
            direction.dot(plane.frame().x()),
            direction.dot(plane.frame().y()),
        );
        AnalyticShellFin::new(
            edge,
            sense,
            AnalyticPcurveUse::new(
                AnalyticShellPcurve::Line(
                    Line2d::new(Point2::new(origin.x, origin.y), local_direction).unwrap(),
                ),
                parameter_map(1.0),
            ),
        )
    }

    fn plane_circle_use(
        edge: AnalyticEdgeKey,
        sense: Sense,
        plane: Plane,
        circle: Circle,
    ) -> AnalyticShellFin {
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
            edge,
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

    fn cylinder_ruling_use(
        edge: AnalyticEdgeKey,
        sense: Sense,
        longitude: f64,
    ) -> AnalyticShellFin {
        AnalyticShellFin::new(
            edge,
            sense,
            AnalyticPcurveUse::new(
                AnalyticShellPcurve::Line(
                    Line2d::new(Point2::new(longitude, 0.0), Vec2::new(0.0, 1.0)).unwrap(),
                ),
                parameter_map(1.0),
            ),
        )
    }

    fn cylinder_arc_use(edge: AnalyticEdgeKey, sense: Sense, height: f64) -> AnalyticShellFin {
        AnalyticShellFin::new(
            edge,
            sense,
            AnalyticPcurveUse::new(
                AnalyticShellPcurve::Line(
                    Line2d::new(Point2::new(0.0, height), Vec2::new(1.0, 0.0)).unwrap(),
                ),
                parameter_map(1.0),
            ),
        )
    }

    /// A major circular segment is non-convex: its chord removes a strict
    /// circular cap. The translated frame also refuses world-axis shortcuts.
    fn concave_oblique_profile_input() -> AnalyticShellInput {
        let frame = Frame::new(
            Point3::new(3.0, -2.0, 1.25),
            Vec3::new(0.6, 0.0, 0.8),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let height = 1.25;
        let translation = frame.z() * height;
        let top_frame = frame.with_origin(frame.origin() + translation);
        let cylinder = Cylinder::new(frame, 1.0).unwrap();
        let bottom_circle = Circle::new(frame, 1.0).unwrap();
        let top_circle = Circle::new(top_frame, 1.0).unwrap();
        let arc = ParamRange::new(0.25 * core::f64::consts::PI, 1.75 * core::f64::consts::PI);
        let points = [
            bottom_circle.eval(arc.lo),
            bottom_circle.eval(arc.hi),
            top_circle.eval(arc.lo),
            top_circle.eval(arc.hi),
        ];
        let vertices = points
            .into_iter()
            .enumerate()
            .map(|(index, point)| {
                AnalyticShellVertex::new(AnalyticVertexKey::new(index as u64), point)
            })
            .collect::<Vec<_>>();

        let chord_direction = points[1] - points[0];
        let chord_length = chord_direction.norm();
        let chord_line = Line::new(points[0], chord_direction).unwrap();
        let top_chord_line = Line::new(points[2], chord_line.dir()).unwrap();
        let first_ruling = Line::new(points[0], frame.z()).unwrap();
        let second_ruling = Line::new(points[1], frame.z()).unwrap();
        let edges = vec![
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(0),
                [AnalyticVertexKey::new(0), AnalyticVertexKey::new(1)],
                AnalyticShellCurve::Circle(bottom_circle),
                arc,
            ),
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(1),
                [AnalyticVertexKey::new(2), AnalyticVertexKey::new(3)],
                AnalyticShellCurve::Circle(top_circle),
                arc,
            ),
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(2),
                [AnalyticVertexKey::new(0), AnalyticVertexKey::new(2)],
                AnalyticShellCurve::Line(first_ruling),
                ParamRange::new(0.0, height),
            ),
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(3),
                [AnalyticVertexKey::new(1), AnalyticVertexKey::new(3)],
                AnalyticShellCurve::Line(second_ruling),
                ParamRange::new(0.0, height),
            ),
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(4),
                [AnalyticVertexKey::new(0), AnalyticVertexKey::new(1)],
                AnalyticShellCurve::Line(chord_line),
                ParamRange::new(0.0, chord_length),
            ),
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(5),
                [AnalyticVertexKey::new(2), AnalyticVertexKey::new(3)],
                AnalyticShellCurve::Line(top_chord_line),
                ParamRange::new(0.0, chord_length),
            ),
        ];

        let bottom_plane = Plane::new(Frame::new(frame.origin(), -frame.z(), frame.x()).unwrap());
        let top_plane = Plane::new(top_frame);
        let cut_plane = Plane::new(Frame::new(points[0], frame.x(), chord_line.dir()).unwrap());
        let cylinder_loop = AnalyticShellLoop::new(vec![
            cylinder_ruling_use(AnalyticEdgeKey::new(2), Sense::Reversed, arc.lo),
            cylinder_arc_use(AnalyticEdgeKey::new(0), Sense::Forward, 0.0),
            cylinder_ruling_use(AnalyticEdgeKey::new(3), Sense::Forward, arc.hi),
            cylinder_arc_use(AnalyticEdgeKey::new(1), Sense::Reversed, height),
        ]);
        let bottom_loop = AnalyticShellLoop::new(vec![
            plane_circle_use(
                AnalyticEdgeKey::new(0),
                Sense::Reversed,
                bottom_plane,
                bottom_circle,
            ),
            plane_line_use(
                AnalyticEdgeKey::new(4),
                Sense::Forward,
                bottom_plane,
                chord_line,
            ),
        ]);
        let top_loop = AnalyticShellLoop::new(vec![
            plane_circle_use(
                AnalyticEdgeKey::new(1),
                Sense::Forward,
                top_plane,
                top_circle,
            ),
            plane_line_use(
                AnalyticEdgeKey::new(5),
                Sense::Reversed,
                top_plane,
                top_chord_line,
            ),
        ]);
        let cut_loop = AnalyticShellLoop::new(vec![
            plane_line_use(
                AnalyticEdgeKey::new(4),
                Sense::Reversed,
                cut_plane,
                chord_line,
            ),
            plane_line_use(
                AnalyticEdgeKey::new(2),
                Sense::Forward,
                cut_plane,
                first_ruling,
            ),
            plane_line_use(
                AnalyticEdgeKey::new(5),
                Sense::Forward,
                cut_plane,
                top_chord_line,
            ),
            plane_line_use(
                AnalyticEdgeKey::new(3),
                Sense::Reversed,
                cut_plane,
                second_ruling,
            ),
        ]);
        let wide_domain = || FaceDomain::from_bounds(-2.0, 2.0, -2.0, 2.0).unwrap();
        AnalyticShellInput::new(
            vertices,
            edges,
            vec![
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(0),
                    AnalyticShellSurface::Cylinder(cylinder),
                    Sense::Forward,
                    FaceDomain::from_bounds(arc.lo, arc.hi, 0.0, height).unwrap(),
                    vec![cylinder_loop],
                ),
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(1),
                    AnalyticShellSurface::Plane(bottom_plane),
                    Sense::Forward,
                    wide_domain(),
                    vec![bottom_loop],
                ),
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(2),
                    AnalyticShellSurface::Plane(top_plane),
                    Sense::Forward,
                    wide_domain(),
                    vec![top_loop],
                ),
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(3),
                    AnalyticShellSurface::Plane(cut_plane),
                    Sense::Forward,
                    FaceDomain::from_bounds(-1.0, chord_length + 1.0, -height - 1.0, 1.0).unwrap(),
                    vec![cut_loop],
                ),
            ],
        )
    }

    #[test]
    fn half_cylinder_is_a_mixed_profile_prism() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&half_cylinder_input(), 1.0e-12)
            .unwrap();
        assert_eq!(
            certify_mixed_profile_prism(transaction.store(), output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            })
        );
    }

    #[test]
    fn concave_oblique_profile_is_full_certified() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&concave_oblique_profile_input(), 1.0e-12)
            .unwrap();
        assert_eq!(
            certify_mixed_profile_prism(transaction.store(), output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            })
        );
        assert!(matches!(
            check_body_report(transaction.store(), output.body(), CheckLevel::Full)
                .unwrap()
                .outcome(),
            CheckOutcome::Valid
        ));
        transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
    }

    #[test]
    fn mixed_profile_tampering_fails_closed_and_live_senses_remain_decidable() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&concave_oblique_profile_input(), 1.0e-12)
            .unwrap();
        let baseline = transaction.store().clone();
        let edge = |key: u64| {
            output
                .edges()
                .iter()
                .find_map(|(candidate, edge)| (candidate.value() == key).then_some(*edge))
                .unwrap()
        };
        let face = |key: u64| {
            output
                .faces()
                .iter()
                .find_map(|(candidate, face)| (candidate.value() == key).then_some(*face))
                .unwrap()
        };
        let vertex = |key: u64| {
            output
                .vertices()
                .iter()
                .find_map(|(candidate, vertex)| (candidate.value() == key).then_some(*vertex))
                .unwrap()
        };

        for case in [
            "unsupported",
            "mapping",
            "simple",
            "radius",
            "axis",
            "range",
            "pcurve",
            "partial",
        ] {
            let mut copy = baseline.clone();
            let mut edit = copy.transaction().unwrap();
            match case {
                "unsupported" => {
                    let edge_id = edge(0);
                    let curve_id = edit.store().get(edge_id).unwrap().curve.unwrap();
                    let [Some(first), Some(second)] = edit.store().get(edge_id).unwrap().vertices
                    else {
                        unreachable!()
                    };
                    let start = edit.store().vertex_position(first).unwrap();
                    let end = edit.store().vertex_position(second).unwrap();
                    edit.store_mut()
                        .replace_curve(
                            curve_id,
                            CurveGeom::Line(Line::new(start, end - start).unwrap()),
                        )
                        .unwrap();
                }
                "mapping" => {
                    let point = edit.store().get(vertex(2)).unwrap().point;
                    edit.store_mut().get_mut(point).unwrap().y += 0.1;
                }
                "simple" => {
                    let loop_id = edit.store().get(face(1)).unwrap().loops[0];
                    let duplicate = edit.store().get(loop_id).unwrap().fins[0];
                    edit.store_mut()
                        .get_mut(loop_id)
                        .unwrap()
                        .fins
                        .push(duplicate);
                }
                "radius" | "axis" => {
                    let surface_id = edit.store().get(face(0)).unwrap().surface;
                    let SurfaceGeom::Cylinder(cylinder) = *edit.store().get(surface_id).unwrap()
                    else {
                        unreachable!()
                    };
                    let changed = if case == "radius" {
                        Cylinder::new(*cylinder.frame(), cylinder.radius() + 0.1).unwrap()
                    } else {
                        Cylinder::new(
                            Frame::new(
                                cylinder.frame().origin(),
                                cylinder.frame().z() + cylinder.frame().x() * 0.1,
                                cylinder.frame().x(),
                            )
                            .unwrap(),
                            cylinder.radius(),
                        )
                        .unwrap()
                    };
                    edit.store_mut()
                        .replace_surface(surface_id, SurfaceGeom::Cylinder(changed))
                        .unwrap();
                }
                "range" => {
                    let edge = edit.store_mut().get_mut(edge(1)).unwrap();
                    edge.bounds = edge.bounds.map(|(lo, hi)| (lo, hi - 0.1));
                }
                "pcurve" => {
                    let loop_id = edit.store().get(face(1)).unwrap().loops[0];
                    let fin = edit.store().get(loop_id).unwrap().fins[0];
                    edit.store_mut().get_mut(fin).unwrap().pcurve = None;
                }
                "partial" => {
                    let loop_id = edit.store().get(face(0)).unwrap().loops[0];
                    edit.store_mut().get_mut(loop_id).unwrap().fins.pop();
                }
                _ => unreachable!(),
            }
            assert_eq!(
                certify_mixed_profile_prism(edit.store(), output.shell(), None).unwrap(),
                None,
                "{case} tamper must not retain the theorem"
            );
        }

        let mut wrong = baseline.clone();
        wrong.get_mut(face(0)).unwrap().sense = Sense::Reversed;
        assert_eq!(
            certify_mixed_profile_prism(&wrong, output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Invalid,
            })
        );
    }

    fn session_with_work(allowed: u64) -> kcore::operation::SessionPolicy {
        let budget = BudgetPlan::new([LimitSpec::new(
            MIXED_PROFILE_PRISM_WORK,
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
    fn mixed_profile_work_accepts_exact_n_and_rejects_n_minus_one() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&concave_oblique_profile_input(), 1.0e-12)
            .unwrap();
        let required = proof_work(transaction.store(), output.shell(), 3)
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
                certify_mixed_profile_prism(transaction.store(), output.shell(), Some(&mut scope));
            if allowed == required {
                assert_eq!(
                    result.unwrap().unwrap().embedding,
                    ShellEmbedding::Certified
                );
            } else {
                assert_eq!(
                    result.unwrap_err().limit().map(|limit| limit.stage),
                    Some(MIXED_PROFILE_PRISM_WORK)
                );
            }
        }
    }
}
