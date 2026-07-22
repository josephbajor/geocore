//! Exact proof-only classification of parallel Cylinder radial supports.
//! Wall-time budget: less than 5 seconds for the focused arithmetic matrix.

use kgeom::frame::Frame;
use kgeom::surface::Cylinder;
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{ParallelCylinderRadialRelation, classify_parallel_cylinder_radial_relation};

fn cylinder(origin: Point3, axis: Vec3, radius: f64) -> Cylinder {
    let x_hint = if axis.x.abs() < 0.5 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    Cylinder::new(Frame::new(origin, axis, x_hint).unwrap(), radius).unwrap()
}

fn world_cylinder(x: f64, z: f64, radius: f64) -> Cylinder {
    cylinder(Point3::new(x, 0.0, z), Vec3::new(0.0, 0.0, 1.0), radius)
}

fn assert_relation(expected: ParallelCylinderRadialRelation, first: Cylinder, second: Cylinder) {
    assert_eq!(
        classify_parallel_cylinder_radial_relation([first, second]),
        expected
    );
    assert_eq!(
        classify_parallel_cylinder_radial_relation([second, first]),
        expected
    );
}

#[test]
fn exact_external_tangent_is_axial_order_and_axis_norm_independent() {
    let first = cylinder(Point3::new(5.0, -2.0, 7.0), Vec3::new(0.0, 3.0, 4.0), 1.25);
    // The stored 0.6/0.8 direction is not assumed to have exact dyadic unit
    // norm. Scaling the radius term by |axis|^2 makes the x-offset proof exact.
    assert_eq!(first.frame().z(), Vec3::new(0.0, 0.6, 0.8));
    let second = cylinder(Point3::new(7.0, -2.0, 7.0), -first.frame().z(), 0.75);
    assert_relation(
        ParallelCylinderRadialRelation::ExactExternalTangent,
        first,
        second,
    );

    // Axial displacement is absent from the infinite-support relation.
    assert_relation(
        ParallelCylinderRadialRelation::ExactExternalTangent,
        world_cylinder(0.0, 0.0, 1.0),
        world_cylinder(2.0, 4096.0, 1.0),
    );
}

#[test]
fn one_ulp_neighbors_are_not_promoted_to_tangency() {
    let first = world_cylinder(0.0, 0.0, 1.0);
    assert_relation(
        ParallelCylinderRadialRelation::ExactExternalTangent,
        first,
        world_cylinder(2.0, 0.0, 1.0),
    );
    assert_relation(
        ParallelCylinderRadialRelation::StrictExterior,
        first,
        world_cylinder(2.0_f64.next_up(), 0.0, 1.0),
    );
    assert_relation(
        ParallelCylinderRadialRelation::Unresolved,
        first,
        world_cylinder(2.0_f64.next_down(), 0.0, 1.0),
    );
}

#[test]
fn rounded_radius_sum_cannot_supply_equality_evidence() {
    let half_ulp = 2.0_f64.powi(-53);
    assert_eq!(1.0 + half_ulp, 1.0);
    let first = world_cylinder(0.0, 0.0, 1.0);
    let rounded_sum_center = world_cylinder(1.0, 0.0, half_ulp);
    assert_relation(
        ParallelCylinderRadialRelation::Unresolved,
        first,
        rounded_sum_center,
    );

    // The two-origin difference can represent the exact dyadic radius sum as
    // an expansion even when that sum is not one `f64`. Keep this exact-zero
    // clearance conservative too: downstream topology must not depend on a
    // rounded scalar radius sum.
    assert_ne!(kcore::expansion::two_sum(1.0, 0.2).1, 0.0);
    assert_relation(
        ParallelCylinderRadialRelation::Unresolved,
        world_cylinder(-0.2, 0.0, 0.2),
        world_cylinder(1.0, 0.0, 1.0),
    );
}

#[test]
fn internal_tangent_nonparallel_and_unsafe_inputs_fail_closed() {
    assert_relation(
        ParallelCylinderRadialRelation::Unresolved,
        world_cylinder(0.0, 0.0, 2.0),
        world_cylinder(1.0, 0.0, 1.0),
    );

    let first = world_cylinder(0.0, 0.0, 1.0);
    let nonparallel = cylinder(Point3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 1.0);
    assert_relation(
        ParallelCylinderRadialRelation::Unresolved,
        first,
        nonparallel,
    );

    let unsafe_scale = 2.0_f64.powi(600);
    let unsafe_offset = world_cylinder(unsafe_scale, 0.0, 1.0);
    assert_relation(
        ParallelCylinderRadialRelation::Unresolved,
        first,
        unsafe_offset,
    );
}
