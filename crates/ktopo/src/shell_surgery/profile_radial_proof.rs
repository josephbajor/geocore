//! Strict radial classification for analytic portal-product profiles.
//!
//! Lines use their convex quadratic radial-distance range. A circular span is
//! admitted only when its carrier and the matching host-circle span are in
//! strict two-root secancy: the shared, distinct topological endpoints are
//! therefore the only full-carrier intersections. The active span is shorter
//! than one period, so a strict midpoint test plus continuity proves that its
//! open interior stays wholly on one side of the host cylinder. Tangency,
//! coincidence, a missing endpoint match, or interval ambiguity fails closed.

use super::super::super::shell_lemmas::{
    IntervalBounds2, RadialSide, circle_affine_range, circle_secant_span_side, radial_coordinates,
};
use super::*;

pub(super) fn profile_radial_side(
    store: &Store,
    cylinder: Cylinder,
    cap: &Cap,
    host_face: FaceId,
    portal_vertices: &[VertexId],
) -> Result<Option<RadialSide>> {
    let portal_uses = cap
        .uses
        .iter()
        .copied()
        .filter_map(|use_| match peer_face(store, use_) {
            Ok(Some(peer)) if peer == host_face => Some(Ok(use_)),
            Ok(_) => None,
            Err(error) => Some(Err(error)),
        })
        .collect::<Result<Vec<_>>>()?;
    if portal_uses.is_empty() {
        return Ok(None);
    }

    let mut side = None;
    for use_ in &cap.uses {
        if portal_uses.iter().any(|portal| portal.edge == use_.edge) {
            continue;
        }
        let candidate = match use_.carrier {
            ProfileCarrier::Line(_) => line_radial_side(store, cylinder, *use_, portal_vertices)?,
            ProfileCarrier::Circle(_) => {
                let matching = portal_uses
                    .iter()
                    .copied()
                    .filter(|portal| same_endpoints(*use_, *portal))
                    .collect::<Vec<_>>();
                let [portal] = matching.as_slice() else {
                    return Ok(None);
                };
                circle_radial_side(cylinder, *use_, *portal)
            }
        };
        let Some(candidate) = candidate else {
            return Ok(None);
        };
        if side.is_some_and(|prior| prior != candidate) {
            return Ok(None);
        }
        side = Some(candidate);
    }
    Ok(side)
}

fn same_endpoints(first: CapUse, second: CapUse) -> bool {
    (first.tail == second.tail && first.head == second.head)
        || (first.tail == second.head && first.head == second.tail)
}

fn circle_radial_side(cylinder: Cylinder, use_: CapUse, portal: CapUse) -> Option<RadialSide> {
    let (ProfileCarrier::Circle(circle), ProfileCarrier::Circle(portal_circle)) =
        (use_.carrier, portal.carrier)
    else {
        return None;
    };
    circle_secant_span_side(
        cylinder,
        circle,
        use_.range,
        portal_circle,
        use_.tail != use_.head,
    )
}

fn line_radial_side(
    store: &Store,
    cylinder: Cylinder,
    use_: CapUse,
    portal_vertices: &[VertexId],
) -> Result<Option<RadialSide>> {
    let first = radial_coordinates(cylinder.frame(), store.vertex_position(use_.tail)?);
    let second = radial_coordinates(cylinder.frame(), store.vertex_position(use_.head)?);
    let radius_sq = Interval::point(cylinder.radius()).square();
    let first_sq = first.x.square() + first.y.square();
    let second_sq = second.x.square() + second.y.square();
    let endpoint_inside = |value: Interval, vertex: VertexId| {
        value.hi() < radius_sq.lo() || portal_vertices.contains(&vertex)
    };
    if endpoint_inside(first_sq, use_.tail) && endpoint_inside(second_sq, use_.head) {
        return Ok(Some(RadialSide::Inside));
    }

    let direction = IntervalBounds2 {
        x: second.x - first.x,
        y: second.y - first.y,
    };
    let a = direction.x.square() + direction.y.square();
    let b = first.x * direction.x + first.y * direction.y;
    let end_derivative = a + b;
    let outside = if b.lo() >= 0.0 {
        first_sq.lo() > radius_sq.hi() || (portal_vertices.contains(&use_.tail) && b.lo() > 0.0)
    } else if end_derivative.hi() <= 0.0 {
        second_sq.lo() > radius_sq.hi()
            || (portal_vertices.contains(&use_.head) && end_derivative.hi() < 0.0)
    } else {
        let Some(quotient) = b.square().checked_div(a) else {
            return Ok(None);
        };
        (first_sq - quotient).lo() > radius_sq.hi()
    };
    Ok(outside.then_some(RadialSide::Outside))
}

pub(super) fn profile_radial_bounds(
    store: &Store,
    cylinder: Cylinder,
    cap: &Cap,
) -> Result<Option<IntervalBounds2>> {
    let mut bounds = None;
    for use_ in &cap.uses {
        let next = match use_.carrier {
            ProfileCarrier::Line(_) => {
                let first = radial_coordinates(cylinder.frame(), store.vertex_position(use_.tail)?);
                let second =
                    radial_coordinates(cylinder.frame(), store.vertex_position(use_.head)?);
                union_bounds(Some(first), second)
            }
            ProfileCarrier::Circle(circle) => {
                let Some(x) = circle_affine_range(
                    circle,
                    use_.range.lo,
                    use_.range.hi,
                    cylinder.frame().x(),
                    cylinder.frame().origin(),
                ) else {
                    return Ok(None);
                };
                let Some(y) = circle_affine_range(
                    circle,
                    use_.range.lo,
                    use_.range.hi,
                    cylinder.frame().y(),
                    cylinder.frame().origin(),
                ) else {
                    return Ok(None);
                };
                IntervalBounds2 { x, y }
            }
        };
        bounds = Some(union_bounds(bounds, next));
    }
    Ok(bounds)
}

fn union_bounds(current: Option<IntervalBounds2>, next: IntervalBounds2) -> IntervalBounds2 {
    let Some(current) = current else {
        return next;
    };
    IntervalBounds2 {
        x: Interval::new(
            current.x.lo().min(next.x.lo()),
            current.x.hi().max(next.x.hi()),
        ),
        y: Interval::new(
            current.y.lo().min(next.y.lo()),
            current.y.hi().max(next.y.hi()),
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strict_secancy_classifies_both_spans_and_rejects_degeneracies() {
        let world = Frame::world();
        let host = Cylinder::new(world, 1.0).unwrap();
        let portal = kgeom::curve::Circle::new(world, 1.0).unwrap();
        let shifted =
            kgeom::curve::Circle::new(world.with_origin(Point3::new(1.0, 0.0, 0.0)), 1.0).unwrap();
        let third = core::f64::consts::PI / 3.0;
        assert_eq!(
            circle_secant_span_side(
                host,
                shifted,
                ParamRange::new(-2.0 * third, 2.0 * third),
                portal,
                true,
            ),
            Some(RadialSide::Outside)
        );
        assert_eq!(
            circle_secant_span_side(
                host,
                shifted,
                ParamRange::new(2.0 * third, 4.0 * third),
                portal,
                true,
            ),
            Some(RadialSide::Inside)
        );

        let tangent =
            kgeom::curve::Circle::new(world.with_origin(Point3::new(2.0, 0.0, 0.0)), 1.0).unwrap();
        let coincident = kgeom::curve::Circle::new(world, 1.0).unwrap();
        let off_axis_frame = Frame::new(Point3::new(1.0, 0.0, 0.0), world.x(), world.y()).unwrap();
        let off_axis = kgeom::curve::Circle::new(off_axis_frame, 1.0).unwrap();
        for rejected in [tangent, coincident, off_axis] {
            assert_eq!(
                circle_secant_span_side(host, rejected, ParamRange::new(-1.0, 1.0), portal, true,),
                None
            );
        }
        assert_eq!(
            circle_secant_span_side(
                host,
                shifted,
                ParamRange::new(-2.0 * third, 2.0 * third),
                portal,
                false,
            ),
            None
        );

        let boundary_parameter = 2.0 * third;
        assert_eq!(
            circle_secant_span_side(
                host,
                shifted,
                ParamRange::new(boundary_parameter.next_down(), boundary_parameter.next_up(),),
                portal,
                true,
            ),
            None,
            "a midpoint at a secant root must not acquire a guessed radial side",
        );

        // Regression for center-plus-harmonic cancellation: evaluating the
        // point first at this model scale rounds both absolute coordinates.
        // The rounded Point3 appears strictly outside even though the exact
        // harmonic sample is at the secant root; the proof must refuse it.
        let model_scale = 1.0e15;
        let translated_frame = world.with_origin(Point3::new(model_scale, model_scale, 0.0));
        let translated_host = Cylinder::new(translated_frame, 1.0).unwrap();
        let translated_portal = kgeom::curve::Circle::new(translated_frame, 1.0).unwrap();
        let translated_profile = kgeom::curve::Circle::new(
            translated_frame.with_origin(Point3::new(model_scale + 1.0, model_scale, 0.0)),
            1.0,
        )
        .unwrap();
        let rounded = radial_coordinates(
            translated_host.frame(),
            translated_profile.eval(boundary_parameter),
        );
        let rounded_sq = rounded.x.square() + rounded.y.square();
        let translated_radius_sq = Interval::point(translated_host.radius()).square();
        assert!(
            rounded_sq.lo() > translated_radius_sq.hi(),
            "fixture must exercise the old rounded-point outside guess",
        );
        assert_eq!(
            circle_secant_span_side(
                translated_host,
                translated_profile,
                ParamRange::new(boundary_parameter.next_down(), boundary_parameter.next_up(),),
                translated_portal,
                true,
            ),
            None,
        );
    }
}
