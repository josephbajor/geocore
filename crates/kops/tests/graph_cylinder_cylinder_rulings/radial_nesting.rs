//! Complete support exclusion for strict nesting; part of the existing target.

use super::*;

#[test]
fn strict_radial_nesting_is_complete_swap_stable_and_never_exterior_separation() {
    let frames = [
        Frame::world(),
        Frame::world().with_origin(Point3::new(4.0, -3.0, 2.0)),
        Frame::new(
            Point3::new(-2.0, 3.0, 1.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap(),
        Frame::new(
            Point3::new(2.0, -1.0, 3.0),
            Vec3::new(1.0, -2.0, 3.0),
            Vec3::new(2.0, 1.0, 0.5),
        )
        .unwrap(),
    ];
    for frame in frames {
        for (offset, radius) in [(0.0, 0.5), (0.25, 0.75), (0.5, 1.0)] {
            for reversed_axis in [false, true] {
                let first = Cylinder::new(frame, 2.0).unwrap();
                let second_frame = Frame::new(
                    frame.point_at(offset, 0.0, 4.0),
                    if reversed_axis { -frame.z() } else { frame.z() },
                    frame.x(),
                )
                .unwrap();
                let second = Cylinder::new(second_frame, radius).unwrap();
                let (graph, first, second) = graph_pair(first, second);
                let windows = [
                    cylinder_window(range(-2.0, 3.0)),
                    cylinder_window(range(-8.0, 6.0)),
                ];
                for (a, b, wa, wb) in [
                    (first, second, windows[0], windows[1]),
                    (second, first, windows[1], windows[0]),
                ] {
                    let result = intersect_bounded_graph_surfaces(
                        &graph,
                        a,
                        wa,
                        b,
                        wb,
                        Tolerances::default(),
                    )
                    .unwrap();
                    let replay = intersect_bounded_graph_surfaces(
                        &graph,
                        a,
                        wa,
                        b,
                        wb,
                        Tolerances::default(),
                    )
                    .unwrap();
                    assert_eq!(result, replay);
                    assert!(result.raw.is_proven_empty());
                    assert!(
                        result.branch_graph.vertices.is_empty()
                            && result.branch_graph.edges.is_empty()
                    );
                    assert_eq!(result.branch_graph.source_surfaces, [a, b]);
                    assert!(result.parallel_cylinder_strict_radial_nesting().is_some());
                    assert!(
                        result
                            .parallel_cylinder_exterior_radial_separation()
                            .is_none()
                    );
                    assert!(result.skew_cylinder_strict_discriminant_miss().is_none());
                }
            }
        }
    }
}

#[test]
fn radial_nesting_uses_exact_clearance_and_validates_windows() {
    let window = cylinder_window(range(-1.0, 1.0));
    for (offset, nested) in [
        (1.0_f64.next_down(), true),
        (1.0, false),
        (1.0_f64.next_up(), false),
    ] {
        let first = Cylinder::new(Frame::world(), 2.0).unwrap();
        let second = Cylinder::new(
            Frame::world().with_origin(Point3::new(offset, 0.0, 0.0)),
            1.0,
        )
        .unwrap();
        let (graph, first, second) = graph_pair(first, second);
        let result = intersect_bounded_graph_surfaces(
            &graph,
            first,
            window,
            second,
            window,
            Tolerances::default(),
        );
        if nested {
            let result = result.unwrap();
            assert!(result.raw.is_proven_empty());
            assert!(result.parallel_cylinder_strict_radial_nesting().is_some());
            assert!(
                result
                    .parallel_cylinder_exterior_radial_separation()
                    .is_none()
            );
        } else if let Ok(result) = result {
            assert!(!result.raw.is_proven_empty());
            assert!(result.parallel_cylinder_strict_radial_nesting().is_none());
        }
        let malformed = cylinder_window(ParamRange { lo: 1.0, hi: -1.0 });
        assert!(
            intersect_bounded_graph_surfaces(
                &graph,
                first,
                malformed,
                second,
                window,
                Tolerances::default()
            )
            .is_err()
        );
    }
    // Equal supports have no strict radial clearance, even with an axial shift.
    let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
    assert_typed_gap(cylinder, window, cylinder, window);
    let tiny = Cylinder::new(Frame::world(), 1.0e-12).unwrap();
    let outer = Cylinder::new(Frame::world(), 2.0e-12).unwrap();
    let (graph, first, second) = graph_pair(tiny, outer);
    let result = intersect_bounded_graph_surfaces(
        &graph,
        first,
        window,
        second,
        window,
        Tolerances::default(),
    )
    .unwrap();
    assert!(result.raw.is_proven_empty());
    assert!(result.parallel_cylinder_strict_radial_nesting().is_some());
}
