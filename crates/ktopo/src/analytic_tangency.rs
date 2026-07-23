//! Exact predicates for topology-owned analytic tangency.
//!
//! These helpers compare the stored `f64` inputs as exact dyadic values. The
//! expansion path is deliberately bounded and fails closed when Dekker
//! products could overflow, underflow, or create non-normal components.

use kcore::expansion;
use kcore::interval::Interval;
use kcore::predicates::{Orientation, affine_dot3, orient3d};
use kgeom::curve::{Circle, Curve};
use kgeom::vec::{Point3, Vec3};

const EXACT_COMPONENT_MAX: f64 = f64::from_bits(((1023 + 500) as u64) << 52);
const EXACT_PRODUCT_MIN: f64 = f64::from_bits(((1023 - 800) as u64) << 52);
const EXACT_PRODUCT_MAX: f64 = f64::from_bits(((1023 + 400) as u64) << 52);
const MAX_COMPONENTS: usize = 64;

/// Prove that two unequal coplanar circle carriers have exact internal
/// tangency. Parallel carrier normals and exact center distance
/// `|c1-c0| = |r1-r0|` are both required.
pub(crate) fn circles_are_exactly_internal_tangent(first: Circle, second: Circle) -> bool {
    let radii = [first.radius(), second.radius()];
    radii[0].is_finite()
        && radii[1].is_finite()
        && radii[0] > 0.0
        && radii[1] > 0.0
        && radii[0] != radii[1]
        && vectors_are_exactly_parallel(first.frame().z(), second.frame().z())
        && exact_coplanar(
            first.frame().z(),
            second.frame().origin(),
            first.frame().origin(),
        )
        && squared_center_distance_minus_radius_difference_squared(
            second.frame().origin(),
            first.frame().origin(),
            radii[0],
            radii[1],
        ) == Some(0)
}

/// Certify one stored point against a circle endpoint with outward intervals.
/// This is resolution-bounded because a normalized authored frame need not
/// make `circle.eval(parameter)` satisfy an exact dyadic radius identity.
pub(crate) fn point_is_within_circle_endpoint_envelope(
    point: Point3,
    circle: Circle,
    parameter: f64,
    tolerance: f64,
) -> bool {
    if !parameter.is_finite() || !tolerance.is_finite() || tolerance < 0.0 {
        return false;
    }
    let endpoint = circle.eval(parameter);
    let distance_squared = point
        .to_array()
        .into_iter()
        .zip(endpoint.to_array())
        .fold(Interval::point(0.0), |sum, (left, right)| {
            sum + (Interval::point(left) - Interval::point(right)).square()
        });
    let allowed = Interval::point(tolerance).square();
    distance_squared.hi().is_finite()
        && allowed.lo().is_finite()
        && distance_squared.hi() <= allowed.lo()
}

fn exact_coplanar(normal: Vec3, point: Point3, origin: Point3) -> bool {
    affine_dot3(normal.to_array(), point.to_array(), origin.to_array(), 0.0)
        .is_some_and(|value| value.sign() == Orientation::Zero)
}

fn vectors_are_exactly_parallel(first: Vec3, second: Vec3) -> bool {
    if first == second || first == -second {
        return true;
    }
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        .into_iter()
        .all(|basis| {
            orient3d(first.to_array(), second.to_array(), basis, [0.0; 3]) == Orientation::Zero
        })
}

fn squared_center_distance_minus_radius_difference_squared(
    point: Point3,
    origin: Point3,
    first_radius: f64,
    second_radius: f64,
) -> Option<i8> {
    let mut exact = vec![0.0];
    for axis in 0..3 {
        let point = point.to_array()[axis];
        let origin = origin.to_array()[axis];
        add_product(&mut exact, point, point, 1.0)?;
        add_product(&mut exact, point, origin, -2.0)?;
        add_product(&mut exact, origin, origin, 1.0)?;
    }
    add_product(&mut exact, first_radius, first_radius, -1.0)?;
    add_product(&mut exact, first_radius, second_radius, 2.0)?;
    add_product(&mut exact, second_radius, second_radius, -1.0)?;
    Some(expansion::sign(&exact))
}

fn add_product(exact: &mut Vec<f64>, left: f64, right: f64, scale: f64) -> Option<()> {
    let mut product = checked_product(left, right)?;
    if scale != 1.0 {
        product = checked_components(expansion::scale(&product, scale))?;
    }
    *exact = checked_components(expansion::sum(exact, &product))?;
    Some(())
}

fn checked_product(left: f64, right: f64) -> Option<Vec<f64>> {
    if left == 0.0 || right == 0.0 {
        return Some(vec![0.0]);
    }
    if !left.is_normal()
        || !right.is_normal()
        || left.abs() > EXACT_COMPONENT_MAX
        || right.abs() > EXACT_COMPONENT_MAX
    {
        return None;
    }
    let product = left.abs() * right.abs();
    if !product.is_normal() || !(EXACT_PRODUCT_MIN..=EXACT_PRODUCT_MAX).contains(&product) {
        return None;
    }
    let (rounded, residual) = expansion::two_product(left, right);
    checked_components(expansion::from_two(rounded, residual))
}

fn checked_components(components: Vec<f64>) -> Option<Vec<f64>> {
    (!components.is_empty()
        && components.len() <= MAX_COMPONENTS
        && components
            .iter()
            .all(|component| *component == 0.0 || component.is_normal()))
    .then_some(components)
}

#[cfg(test)]
mod tests {
    use super::*;
    use kgeom::frame::Frame;

    #[test]
    fn exact_circle_tangency_rejects_adjacent_center_offsets() {
        let frame = Frame::world();
        let outer = Circle::new(frame, 2.0).unwrap();
        let tangent = Circle::new(frame.with_origin(Point3::new(1.0, 0.0, 0.0)), 1.0).unwrap();
        assert!(circles_are_exactly_internal_tangent(outer, tangent));
        for offset in [1.0_f64.next_down(), 1.0_f64.next_up()] {
            let near = Circle::new(frame.with_origin(Point3::new(offset, 0.0, 0.0)), 1.0).unwrap();
            assert!(!circles_are_exactly_internal_tangent(outer, near));
        }
    }
}
