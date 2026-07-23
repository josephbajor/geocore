//! Exact proof-only classification of parallel Cylinder radial supports.
//! Wall-time budget: less than 5 seconds for the focused arithmetic matrix.

use kgeom::frame::Frame;
use kgeom::surface::Cylinder;
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    ParallelCylinderInternalTangency, ParallelCylinderRadialRelation,
    classify_parallel_cylinder_radial_relation,
};

fn cylinder(origin: Point3, axis: Vec3, radius: f64) -> Cylinder {
    let x_hint = if axis.x.abs() < 0.5 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    Cylinder::new(Frame::new(origin, axis, x_hint).unwrap(), radius).unwrap()
}

fn cylinder_with_chart(origin: Point3, axis: Vec3, x_hint: Vec3, radius: f64) -> Cylinder {
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

fn assert_internal_tangency(
    expected: ParallelCylinderInternalTangency,
    first: Cylinder,
    second: Cylinder,
) {
    let reversed = match expected {
        ParallelCylinderInternalTangency::FirstContainsSecond => {
            ParallelCylinderInternalTangency::SecondContainsFirst
        }
        ParallelCylinderInternalTangency::SecondContainsFirst => {
            ParallelCylinderInternalTangency::FirstContainsSecond
        }
    };
    assert_eq!(
        classify_parallel_cylinder_radial_relation([first, second]),
        ParallelCylinderRadialRelation::ExactInternalTangent(expected)
    );
    assert_eq!(
        classify_parallel_cylinder_radial_relation([second, first]),
        ParallelCylinderRadialRelation::ExactInternalTangent(reversed)
    );
}

#[test]
fn exact_common_support_is_axial_direction_and_chart_independent() {
    let world_first = cylinder_with_chart(
        Point3::new(5.0, -2.0, -4096.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
        1.25,
    );
    let world_second = cylinder_with_chart(
        Point3::new(5.0, -2.0, 8192.0),
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(0.0, 1.0, 0.0),
        1.25,
    );
    assert_relation(
        ParallelCylinderRadialRelation::ExactCommonSupport,
        world_first,
        world_second,
    );

    let oblique_first = cylinder_with_chart(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 3.0, 4.0),
        Vec3::new(1.0, 0.0, 0.0),
        0.75,
    );
    let axis = oblique_first.frame().z();
    let oblique_second = cylinder_with_chart(
        Point3::new(axis.x, axis.y, axis.z),
        -axis,
        Vec3::new(0.0, 0.8, -0.6),
        0.75,
    );
    assert_relation(
        ParallelCylinderRadialRelation::ExactCommonSupport,
        oblique_first,
        oblique_second,
    );
}

#[test]
fn common_support_rejects_one_ulp_radius_and_radial_differences() {
    let first = world_cylinder(1.0, -4.0, 1.0);
    assert_relation(
        ParallelCylinderRadialRelation::Unresolved,
        first,
        world_cylinder(1.0, 8.0, 1.0_f64.next_up()),
    );
    assert_relation(
        ParallelCylinderRadialRelation::Unresolved,
        first,
        world_cylinder(1.0_f64.next_up(), 8.0, 1.0),
    );
}

#[test]
fn common_support_rejects_near_parallel_unsafe_and_rounded_oblique_lines() {
    let first = world_cylinder(0.0, 0.0, 1.0);
    let near_parallel = cylinder(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(2.0_f64.powi(-40), 0.0, 1.0),
        1.0,
    );
    assert_relation(
        ParallelCylinderRadialRelation::Unresolved,
        first,
        near_parallel,
    );

    let unsafe_scale = 2.0_f64.powi(600);
    assert_relation(
        ParallelCylinderRadialRelation::Unresolved,
        first,
        world_cylinder(unsafe_scale, 0.0, 1.0),
    );

    let translated_origin = Point3::new(1.0, 1.0, 1.0);
    let oblique_first = cylinder(translated_origin, Vec3::new(0.6, 0.0, 0.8), 1.0);
    let rounded_axial_origin = translated_origin + oblique_first.frame().z();
    let rounded_oblique = cylinder(rounded_axial_origin, oblique_first.frame().z(), 1.0);
    assert_relation(
        ParallelCylinderRadialRelation::Unresolved,
        oblique_first,
        rounded_oblique,
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
fn exact_internal_tangency_is_directed_swap_stable_and_frame_independent() {
    let translated_outer = cylinder(Point3::new(5.0, -2.0, 7.0), Vec3::new(0.0, 3.0, 4.0), 1.75);
    assert_eq!(translated_outer.frame().z(), Vec3::new(0.0, 0.6, 0.8));
    let translated_inner = cylinder(
        Point3::new(6.0, -2.0, 7.0),
        -translated_outer.frame().z(),
        0.75,
    );

    let cases = [
        (
            world_cylinder(0.0, -4096.0, 2.0),
            world_cylinder(1.0, 8192.0, 1.0),
        ),
        (translated_outer, translated_inner),
    ];
    for (outer, inner) in cases {
        assert_internal_tangency(
            ParallelCylinderInternalTangency::FirstContainsSecond,
            outer,
            inner,
        );
    }
}

#[test]
fn exact_internal_tangency_sums_pythagorean_transverse_components() {
    let outer = world_cylinder(0.0, -4096.0, 6.0);
    let inner = cylinder(
        Point3::new(3.0, 4.0, 8192.0),
        Vec3::new(0.0, 0.0, -1.0),
        1.0,
    );
    assert_internal_tangency(
        ParallelCylinderInternalTangency::FirstContainsSecond,
        outer,
        inner,
    );

    for near_x in [3.0_f64.next_down(), 3.0_f64.next_up()] {
        let near_inner = cylinder(
            Point3::new(near_x, 4.0, 8192.0),
            Vec3::new(0.0, 0.0, -1.0),
            1.0,
        );
        assert_relation(
            ParallelCylinderRadialRelation::Unresolved,
            outer,
            near_inner,
        );
    }
}

#[test]
fn exact_internal_tangency_preserves_nonrepresentable_radius_difference() {
    let tiny_radius = 2.0_f64.powi(-54);
    assert_eq!(1.0 - tiny_radius, 1.0);
    assert_ne!(kcore::expansion::two_diff(1.0, tiny_radius).1, 0.0);

    assert_internal_tangency(
        ParallelCylinderInternalTangency::FirstContainsSecond,
        world_cylinder(1.0, 0.0, 1.0),
        world_cylinder(tiny_radius, 0.0, tiny_radius),
    );
}

#[test]
fn one_ulp_radial_and_radius_neighbors_are_not_internal_tangency_proofs() {
    let outer = world_cylinder(0.0, 0.0, 2.0);
    let cases = [
        world_cylinder(1.0_f64.next_up(), 0.0, 1.0),
        world_cylinder(1.0_f64.next_down(), 0.0, 1.0),
        world_cylinder(1.0, 0.0, 1.0_f64.next_up()),
        world_cylinder(1.0, 0.0, 1.0_f64.next_down()),
    ];
    for near_inner in cases {
        assert_relation(
            ParallelCylinderRadialRelation::Unresolved,
            outer,
            near_inner,
        );
    }

    for near_outer_radius in [2.0_f64.next_down(), 2.0_f64.next_up()] {
        assert_relation(
            ParallelCylinderRadialRelation::Unresolved,
            world_cylinder(0.0, 0.0, near_outer_radius),
            world_cylinder(1.0, 0.0, 1.0),
        );
    }
}

#[test]
fn internal_tangency_rejects_containment_near_parallel_equal_and_unsafe_inputs() {
    let unresolved_cases = [
        (world_cylinder(0.0, 0.0, 3.0), world_cylinder(1.0, 0.0, 1.0)),
        (world_cylinder(0.0, 0.0, 1.0), world_cylinder(1.0, 0.0, 1.0)),
        (
            world_cylinder(0.0, 0.0, 1.0),
            cylinder(
                Point3::new(1.0, 0.0, 0.0),
                Vec3::new(2.0_f64.powi(-40), 0.0, 1.0),
                0.5,
            ),
        ),
    ];
    for (first, second) in unresolved_cases {
        assert_relation(ParallelCylinderRadialRelation::Unresolved, first, second);
    }

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

    // These are mathematically internally tangent, but squaring their exact
    // dyadic radius difference leaves the classifier's bounded expansion
    // envelope and must not produce a proof.
    let unsafe_product_scale = 2.0_f64.powi(449);
    assert_relation(
        ParallelCylinderRadialRelation::Unresolved,
        world_cylinder(0.0, 0.0, unsafe_product_scale * 2.0),
        world_cylinder(unsafe_product_scale, 0.0, unsafe_product_scale),
    );
}
