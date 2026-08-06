//! Shared proof lemmas for analytic shell certifiers.

use super::*;
use crate::entity::{EdgeId, FinId};
use kcore::math;
use kgeom::curve::{Circle, Line};
use kgeom::param::ParamRange;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum RadialSide {
    Inside,
    Outside,
}

impl RadialSide {
    pub(super) const fn orientation_factor(self) -> i8 {
        match self {
            Self::Inside => -1,
            Self::Outside => 1,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(super) struct IntervalBounds2 {
    pub(super) x: Interval,
    pub(super) y: Interval,
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

pub(super) fn coordinate_interval(frame: &Frame, axis: Vec3, point: Point3) -> Interval {
    let offset = [
        Interval::point(point.x) - Interval::point(frame.origin().x),
        Interval::point(point.y) - Interval::point(frame.origin().y),
        Interval::point(point.z) - Interval::point(frame.origin().z),
    ];
    Interval::point(axis.x) * offset[0]
        + Interval::point(axis.y) * offset[1]
        + Interval::point(axis.z) * offset[2]
}

pub(super) fn radial_coordinates(frame: &Frame, point: Point3) -> IntervalBounds2 {
    IntervalBounds2 {
        x: coordinate_interval(frame, frame.x(), point),
        y: coordinate_interval(frame, frame.y(), point),
    }
}

pub(in crate::shell_proof) fn circle_secant_span_side(
    cylinder: Cylinder,
    circle: kgeom::curve::Circle,
    range: ParamRange,
    portal_circle: kgeom::curve::Circle,
    endpoints_distinct: bool,
) -> Option<RadialSide> {
    if !endpoints_distinct
        || !certified_parallel(circle.frame().z(), cylinder.frame().z())
        || !certified_parallel(portal_circle.frame().z(), cylinder.frame().z())
        || portal_circle.radius().to_bits() != cylinder.radius().to_bits()
    {
        return None;
    }

    let portal_center = radial_coordinates(cylinder.frame(), portal_circle.frame().origin());
    let portal_center_sq = portal_center.x.square() + portal_center.y.square();
    if portal_center_sq.hi() > LINEAR_RESOLUTION * LINEAR_RESOLUTION {
        return None;
    }

    // For transverse center distance d and radii R,r, strict secancy is
    // |R-r| < d < R+r. Squared outward intervals prove both inequalities.
    let center = radial_coordinates(cylinder.frame(), circle.frame().origin());
    let center_sq = center.x.square() + center.y.square();
    let host_radius = Interval::point(cylinder.radius());
    let profile_radius = Interval::point(circle.radius());
    let radius_difference_sq = (host_radius - profile_radius).square();
    let radius_sum_sq = (host_radius + profile_radius).square();
    if center_sq.lo() <= radius_difference_sq.hi() || center_sq.hi() >= radius_sum_sq.lo() {
        return None;
    }

    let midpoint = range.lo / 2.0 + range.hi / 2.0;
    if !midpoint.is_finite() || midpoint <= range.lo || midpoint >= range.hi {
        return None;
    }
    let radial = circle_radial_coordinates(cylinder, circle, midpoint)?;
    let radial_sq = radial.x.square() + radial.y.square();
    let host_radius_sq = host_radius.square();
    if radial_sq.hi() < host_radius_sq.lo() {
        Some(RadialSide::Inside)
    } else if radial_sq.lo() > host_radius_sq.hi() {
        Some(RadialSide::Outside)
    } else {
        None
    }
}

/// Outward radial-coordinate enclosure at one exact `f64` parameter.
///
/// `Circle::eval` would round the center-plus-harmonic point before interval
/// arithmetic sees it, losing the construction error precisely when a radial
/// comparison cancels near the host boundary. Deterministic `sincos` has a
/// documented error below one ulp; two adjacent representable values on each
/// side cover that bound across binade boundaries. All subsequent center
/// subtraction, frame projection, scaling, and addition remain interval
/// operations, so the decision never treats a rounded `Point3` as exact.
fn circle_radial_coordinates(
    cylinder: Cylinder,
    circle: kgeom::curve::Circle,
    parameter: f64,
) -> Option<IntervalBounds2> {
    let (sine, cosine) = kcore::math::sincos(parameter);
    if !sine.is_finite() || !cosine.is_finite() {
        return None;
    }
    let sine = Interval::new(sine.next_down().next_down(), sine.next_up().next_up());
    let cosine = Interval::new(cosine.next_down().next_down(), cosine.next_up().next_up());
    let radius = Interval::point(circle.radius());
    let coordinate = |axis| {
        let center = coordinate_interval(cylinder.frame(), axis, circle.frame().origin());
        let x = vector_dot_interval(axis, circle.frame().x());
        let y = vector_dot_interval(axis, circle.frame().y());
        let value = center + radius * (x * cosine + y * sine);
        (value.lo().is_finite() && value.hi().is_finite()).then_some(value)
    };
    Some(IntervalBounds2 {
        x: coordinate(cylinder.frame().x())?,
        y: coordinate(cylinder.frame().y())?,
    })
}

fn vector_dot_interval(first: Vec3, second: Vec3) -> Interval {
    Interval::point(first.x) * Interval::point(second.x)
        + Interval::point(first.y) * Interval::point(second.y)
        + Interval::point(first.z) * Interval::point(second.z)
}

pub(super) fn circle_affine_range(
    circle: kgeom::curve::Circle,
    lo: f64,
    hi: f64,
    normal: Vec3,
    plane_origin: Point3,
) -> Option<Interval> {
    if !lo.is_finite() || !hi.is_finite() || lo >= hi {
        return None;
    }
    let frame = circle.frame();
    let constant = affine_value(normal, frame.origin(), plane_origin)?;
    let x = dot_interval(normal, frame.x())? * Interval::point(circle.radius());
    let y = dot_interval(normal, frame.y())? * Interval::point(circle.radius());
    finite_interval(constant + harmonic_range(x, y, lo, hi)?)
}

/// Outward range of `a*cos(u) + b*sin(u)` over one finite interval.
///
/// Ranging sine and cosine independently loses their unit-circle correlation
/// and can place a clipped cylinder patch outside a sloped support that owns
/// both of its exact endpoints.  Here endpoint values are evaluated together.
/// The only possible interior extrema are the coefficient direction and its
/// antipode.  An outward phase enclosure includes coefficient-dot rounding;
/// an uncertain direction therefore inserts the full amplitude and fails
/// loose rather than omitting an extremum.
pub(super) fn harmonic_range(a: Interval, b: Interval, lo: f64, hi: f64) -> Option<Interval> {
    if finite_interval(a).is_none()
        || finite_interval(b).is_none()
        || !lo.is_finite()
        || !hi.is_finite()
        || lo > hi
    {
        return None;
    }
    let amplitude = (Interval::point(interval_abs_upper(a)).square()
        + Interval::point(interval_abs_upper(b)).square())
    .sqrt()?
    .hi();
    if !amplitude.is_finite() {
        return None;
    }
    if amplitude == 0.0 {
        return Some(Interval::point(0.0));
    }
    let first = harmonic_value(a, b, lo)?;
    let second = harmonic_value(a, b, hi)?;
    let mut range = union(first, second);
    let Some(phase) = harmonic_maximum_phase(a, b) else {
        return Some(Interval::new(-amplitude, amplitude));
    };
    if periodic_phase_intersects(lo, hi, phase)? {
        range = Interval::new(range.lo(), range.hi().max(amplitude));
    }
    let minimum_phase = phase
        + Interval::new(
            core::f64::consts::PI.next_down(),
            core::f64::consts::PI.next_up(),
        );
    if periodic_phase_intersects(lo, hi, minimum_phase)? {
        range = Interval::new(range.lo().min(-amplitude), range.hi());
    }
    finite_interval(range)
}

fn harmonic_value(a: Interval, b: Interval, parameter: f64) -> Option<Interval> {
    let (sine, cosine) = math::sincos(parameter);
    if !sine.is_finite() || !cosine.is_finite() {
        return None;
    }
    let sine = Interval::new(sine.next_down(), sine.next_up());
    let cosine = Interval::new(cosine.next_down(), cosine.next_up());
    finite_interval(a * cosine + b * sine)
}

/// Enclose the phase of every coefficient vector represented by `a × b`.
fn harmonic_maximum_phase(a: Interval, b: Interval) -> Option<Interval> {
    let a_center = 0.5 * a.lo() + 0.5 * a.hi();
    let b_center = 0.5 * b.lo() + 0.5 * b.hi();
    if !a_center.is_finite()
        || !b_center.is_finite()
        || !a.contains(a_center)
        || !b.contains(b_center)
    {
        return None;
    }
    let a_error = interval_abs_upper(a - Interval::point(a_center));
    let b_error = interval_abs_upper(b - Interval::point(b_center));
    let error = (Interval::point(a_error).square() + Interval::point(b_error).square())
        .sqrt()?
        .hi();
    let center_norm =
        (Interval::point(a_center).square() + Interval::point(b_center).square()).sqrt()?;
    let parallel_lower = (center_norm.lo() - error).next_down();
    if !error.is_finite() || !parallel_lower.is_finite() || parallel_lower <= 0.0 {
        return None;
    }
    let phase = math::atan2(b_center, a_center);
    let uncertainty = math::atan2(error, parallel_lower).next_up().next_up();
    if !phase.is_finite() || !uncertainty.is_finite() {
        return None;
    }
    Some(Interval::new(
        (phase - uncertainty).next_down().next_down(),
        (phase + uncertainty).next_up().next_up(),
    ))
}

fn periodic_phase_intersects(lo: f64, hi: f64, phase: Interval) -> Option<bool> {
    if !lo.is_finite()
        || !hi.is_finite()
        || lo > hi
        || !phase.lo().is_finite()
        || !phase.hi().is_finite()
    {
        return None;
    }
    let period = Interval::new(
        core::f64::consts::TAU.next_down(),
        core::f64::consts::TAU.next_up(),
    );
    let turns = (Interval::new(lo, hi) - phase).checked_div(period)?;
    const EXACT_INTEGER_LIMIT: f64 = (1_u64 << 52) as f64;
    if turns.lo().abs() > EXACT_INTEGER_LIMIT || turns.hi().abs() > EXACT_INTEGER_LIMIT {
        return None;
    }
    Some(turns.lo().ceil() <= turns.hi().floor())
}

pub(super) fn interval_abs_upper(interval: Interval) -> f64 {
    interval.lo().abs().max(interval.hi().abs())
}

pub(super) fn affine_value(normal: Vec3, point: Point3, origin: Point3) -> Option<Interval> {
    if !finite_vec(normal) || !finite_point(point) || !finite_point(origin) {
        return None;
    }
    dot_interval(normal, point - origin)
}

pub(super) fn dot_interval(left: Vec3, right: Vec3) -> Option<Interval> {
    if !finite_vec(left) || !finite_vec(right) {
        return None;
    }
    finite_interval(
        Interval::point(left.x) * Interval::point(right.x)
            + Interval::point(left.y) * Interval::point(right.y)
            + Interval::point(left.z) * Interval::point(right.z),
    )
}

pub(super) fn union(first: Interval, second: Interval) -> Interval {
    Interval::new(first.lo().min(second.lo()), first.hi().max(second.hi()))
}

pub(super) fn finite_interval(interval: Interval) -> Option<Interval> {
    (interval.lo().is_finite() && interval.hi().is_finite()).then_some(interval)
}

pub(super) fn finite_point(point: Point3) -> bool {
    point.x.is_finite() && point.y.is_finite() && point.z.is_finite()
}

pub(super) fn finite_vec(vector: Vec3) -> bool {
    vector.x.is_finite() && vector.y.is_finite() && vector.z.is_finite()
}
