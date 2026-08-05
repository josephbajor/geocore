//! Shared proof lemmas for analytic shell certifiers.

use super::*;
use crate::entity::{EdgeId, FinId};
use kgeom::curve::{Circle, Line};
use kgeom::param::ParamRange;

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
