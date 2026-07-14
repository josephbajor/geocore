//! Coincident bounded sphere/sphere region conformance.

use kcore::error::Error;
use kcore::math;
use kcore::proof::Completion;
use kcore::tolerance::Tolerances;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Sphere, Surface};
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    ContactKind, SurfaceIntersectionCurve, SurfaceRegionCorrespondence, SurfaceRegionOrientation,
    SurfaceSurfaceIntersections, intersect_bounded_spheres,
};

fn range(lo: f64, hi: f64) -> ParamRange {
    ParamRange::new(lo, hi)
}

fn window(u_lo: f64, u_hi: f64, v_lo: f64, v_hi: f64) -> [ParamRange; 2] {
    [range(u_lo, u_hi), range(v_lo, v_hi)]
}

fn world_sphere() -> Sphere {
    Sphere::new(Frame::world(), 1.0).unwrap()
}

fn orthogonal_sphere(origin: Point3, radius: f64) -> Sphere {
    // Local (x, y, z) maps to world (z, x, y), a right-handed cyclic
    // permutation whose latitude axis is perpendicular to the world sphere's.
    Sphere::new(
        Frame::new(origin, Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)).unwrap(),
        radius,
    )
    .unwrap()
}

fn y_tilted_sphere(origin: Point3, radius: f64, angle: f64) -> Sphere {
    Sphere::new(
        Frame::new(
            origin,
            Vec3::new(math::sin(angle), 0.0, math::cos(angle)),
            Vec3::new(math::cos(angle), 0.0, -math::sin(angle)),
        )
        .unwrap(),
        radius,
    )
    .unwrap()
}

fn assert_regions_lift(hit: &SurfaceSurfaceIntersections, a: &Sphere, b: &Sphere) {
    assert!(hit.is_complete());
    assert!(hit.points.is_empty());
    assert!(hit.curves.is_empty());
    assert!(!hit.regions.is_empty());
    for region in &hit.regions {
        assert_eq!(
            region.correspondence,
            SurfaceRegionCorrespondence::Polygonal
        );
        assert_eq!(region.boundary.len(), 4);
        assert_eq!(region.orientation, SurfaceRegionOrientation::Same);
        for vertex in &region.boundary {
            let pa = a.eval(vertex.uv_a);
            let pb = b.eval(vertex.uv_b);
            assert_eq!(vertex.point, (pa + pb) / 2.0);
            assert_eq!(vertex.residual, pa.dist(pb));
            assert!(vertex.residual <= region.max_residual);
        }
        for u_weight in [0.0, 0.25, 0.5, 0.75, 1.0] {
            for v_weight in [0.0, 0.25, 0.5, 0.75, 1.0] {
                let uv_a = bilinear_uv(region, true, u_weight, v_weight);
                let uv_b = bilinear_uv(region, false, u_weight, v_weight);
                assert!(a.eval(uv_a).dist(b.eval(uv_b)) <= region.max_residual);
            }
        }
    }
}

fn assert_orthogonal_octant_region(hit: &SurfaceSurfaceIntersections, a: &Sphere, b: &Sphere) {
    assert!(hit.is_complete());
    assert!(hit.points.is_empty());
    assert!(hit.curves.is_empty());
    assert_eq!(hit.regions.len(), 1);
    let region = &hit.regions[0];
    assert_eq!(region.boundary.len(), 3);
    assert_eq!(region.orientation, SurfaceRegionOrientation::Same);
    let SurfaceRegionCorrespondence::OrthogonalSphereOctant(map) = region.correspondence else {
        panic!("expected an exact orthogonal-sphere octant correspondence");
    };

    for vertex in &region.boundary {
        assert!(vertex.residual <= region.max_residual);
    }
    for u_weight in [0.0, 0.125, 0.375, 0.625, 0.875, 1.0] {
        for v_weight in [0.0, 0.2, 0.5, 0.8, 1.0] {
            let a_range = map.first_range();
            let uv_a = [
                a_range[0].lo + u_weight * a_range[0].width(),
                a_range[1].lo + v_weight * a_range[1].width(),
            ];
            let uv_b = map
                .map_first_to_second(uv_a)
                .expect("every first-octant interior sample must map");
            assert!(map.second_range()[0].contains(uv_b[0]));
            assert!(map.second_range()[1].contains(uv_b[1]));
            assert!(a.eval(uv_a).dist(b.eval(uv_b)) <= region.max_residual);

            let b_range = map.second_range();
            let uv_b = [
                b_range[0].lo + u_weight * b_range[0].width(),
                b_range[1].lo + v_weight * b_range[1].width(),
            ];
            let uv_a = map
                .map_second_to_first(uv_b)
                .expect("every second-octant interior sample must map");
            assert!(map.first_range()[0].contains(uv_a[0]));
            assert!(map.first_range()[1].contains(uv_a[1]));
            assert!(a.eval(uv_a).dist(b.eval(uv_b)) <= region.max_residual);
        }
    }
    assert!(
        map.map_first_to_second([map.first_range()[0].lo - 0.25, 0.25])
            .is_none()
    );
}

fn assert_arbitrary_octant_region(hit: &SurfaceSurfaceIntersections, a: &Sphere, b: &Sphere) {
    assert!(hit.is_complete());
    assert!(hit.points.is_empty());
    assert!(hit.curves.is_empty());
    assert_eq!(hit.regions.len(), 1);
    let region = &hit.regions[0];
    assert!(region.boundary.len() >= 3);
    assert_eq!(region.orientation, SurfaceRegionOrientation::Same);
    let SurfaceRegionCorrespondence::ArbitrarySphereOctant(map) = region.correspondence else {
        panic!("expected an arbitrary-frame sphere-octant correspondence");
    };
    for vertex in &region.boundary {
        assert!(vertex.residual <= region.max_residual);
        assert!(map.map_first_to_second(vertex.uv_a).is_some());
        assert!(map.map_second_to_first(vertex.uv_b).is_some());
        assert!(a.eval(vertex.uv_a).dist(b.eval(vertex.uv_b)) <= region.max_residual);
    }
}

fn assert_general_sphere_window_region(hit: &SurfaceSurfaceIntersections, a: &Sphere, b: &Sphere) {
    assert_general_sphere_window_regions(hit, a, b, 1);
}

fn assert_general_sphere_window_regions(
    hit: &SurfaceSurfaceIntersections,
    a: &Sphere,
    b: &Sphere,
    expected_regions: usize,
) {
    assert!(hit.is_complete(), "unexpected incomplete result: {hit:?}");
    assert!(hit.points.is_empty());
    assert!(hit.curves.is_empty());
    assert_eq!(hit.regions.len(), expected_regions);
    for region in &hit.regions {
        assert!(region.boundary.len() >= 3);
        assert_eq!(region.orientation, SurfaceRegionOrientation::Same);
        let SurfaceRegionCorrespondence::GeneralSphereWindow(map) = region.correspondence else {
            panic!("expected a certified general sphere-window correspondence");
        };
        for vertex in &region.boundary {
            assert!(vertex.residual <= region.max_residual);
            let mapped_b = map
                .map_first_to_second(vertex.uv_a)
                .expect("every certified first-chart anchor must map");
            let mapped_a = map
                .map_second_to_first(vertex.uv_b)
                .expect("every certified second-chart anchor must map");
            assert!(a.eval(vertex.uv_a).dist(b.eval(mapped_b)) <= region.max_residual);
            assert!(a.eval(mapped_a).dist(b.eval(vertex.uv_b)) <= region.max_residual);
        }
    }
}

fn assert_indeterminate_sphere_window(hit: &SurfaceSurfaceIntersections, reason: &'static str) {
    assert!(hit.is_empty());
    assert_eq!(hit.completion(), Completion::Indeterminate { reason });
}

#[test]
fn general_non_octant_arbitrary_axis_windows_emit_certified_region_and_swap() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.4);
    let a_window = window(0.15, 1.25, -0.55, 0.65);
    let b_window = window(0.05, 1.15, -0.45, 0.55);
    let hit = intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert_general_sphere_window_region(&hit, &a, &b);

    let swapped =
        intersect_bounded_spheres(&b, b_window, &a, a_window, Tolerances::default()).unwrap();
    assert_eq!(hit.clone().swapped(), swapped);
    assert_general_sphere_window_region(&swapped, &b, &a);

    let mut invalid = hit.regions[0].clone();
    invalid.boundary[0].uv_b = invalid.boundary[1].uv_b;
    assert_eq!(
        SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
            Vec::new(),
            Vec::new(),
            vec![invalid],
        )
        .unwrap_err(),
        Error::InvalidGeometry {
            reason: "general sphere window regions require mutually mapped certified boundary anchors and same orientation"
        }
    );
}

#[test]
fn general_non_octant_fallback_certifies_containment_and_seam_windows() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.2);
    let containing = window(-0.9, 0.9, -0.85, 0.85);
    let contained = window(-0.2, 0.2, -0.2, 0.2);
    let containment =
        intersect_bounded_spheres(&a, containing, &b, contained, Tolerances::default()).unwrap();
    assert_general_sphere_window_region(&containment, &a, &b);
    assert_eq!(containment.regions[0].boundary.len(), 4);

    let tau = core::f64::consts::TAU;
    let seam_a = window(tau - 0.8, tau + 0.6, -0.6, 0.6);
    let seam_b = window(-0.7, 0.7, -0.5, 0.5);
    let seam = intersect_bounded_spheres(&a, seam_a, &b, seam_b, Tolerances::default()).unwrap();
    assert_general_sphere_window_region(&seam, &a, &b);
    assert!(
        seam.regions[0]
            .boundary
            .iter()
            .any(|vertex| { vertex.uv_a[0] > tau && vertex.uv_b[0] < 0.7 })
    );
}

#[test]
fn general_non_octant_fallback_rejects_unsupported_and_uncertified_windows() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.4);
    let supported = window(0.0, 1.0, -0.5, 0.5);
    let just_below_pi = f64::from_bits(core::f64::consts::PI.to_bits() - 1);
    let slice_reason = "general coincident sphere window fallback supports only positive-area pole-clear windows with longitude span below pi";
    assert_indeterminate_sphere_window(
        &intersect_bounded_spheres(
            &a,
            window(0.0, just_below_pi, -0.5, 0.5),
            &b,
            supported,
            Tolerances::default(),
        )
        .unwrap(),
        slice_reason,
    );
    assert_indeterminate_sphere_window(
        &intersect_bounded_spheres(
            &a,
            window(0.0, 1.0, -core::f64::consts::FRAC_PI_2, 0.5),
            &b,
            supported,
            Tolerances::default(),
        )
        .unwrap(),
        "general coincident sphere window proof supports only positive-area pole-clear windows",
    );

    let near_parallel = y_tilted_sphere(
        Point3::new(0.0, 0.0, 0.0),
        1.0,
        0.5 * Tolerances::default().angular(),
    );
    let hit = intersect_bounded_spheres(
        &a,
        supported,
        &near_parallel,
        supported,
        Tolerances::default(),
    )
    .unwrap();
    assert!(matches!(
        hit.completion(),
        Completion::Indeterminate {
            reason: "general coincident sphere window boundary planes exceed the certified angular corridor"
                | "general coincident sphere window proof encountered an unresolved multiple boundary vertex"
        }
    ));

    let tangent = intersect_bounded_spheres(
        &a,
        window(-0.5, 0.5, -0.5, 0.0),
        &b,
        window(-0.5, 0.5, 0.4, 0.9),
        Tolerances::default(),
    )
    .unwrap();
    assert!(tangent.is_empty());
    assert!(matches!(
        tangent.completion(),
        Completion::Indeterminate {
            reason: "general coincident sphere window boundary tangency is not certified by this fallback arm"
                | "general coincident sphere window membership is inside the unresolved proof corridor"
        }
    ));
}

#[test]
fn general_non_octant_disjoint_windows_have_certified_empty_evidence_and_swap() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.4);
    let a_window = window(0.1, 0.7, -0.3, 0.3);
    let b_window = window(2.0, 2.6, -0.3, 0.3);
    let disjoint =
        intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert!(disjoint.is_proven_empty());

    let disjoint_swapped =
        intersect_bounded_spheres(&b, b_window, &a, a_window, Tolerances::default()).unwrap();
    assert_eq!(disjoint.clone().swapped(), disjoint_swapped);
}

#[test]
fn general_non_octant_exact_boundary_lock_emits_tangent_arc_and_swap() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.4);
    let a_window = window(0.0, 0.8, -0.3, 0.5);
    let b_window = window(-0.8, 0.0, -0.2, 0.4);
    let hit = intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert!(hit.is_complete());
    assert!(hit.points.is_empty());
    assert!(hit.regions.is_empty());
    assert_eq!(hit.curves.len(), 1);
    assert_eq!(hit.curves[0].kind, ContactKind::Tangent);
    assert!(matches!(
        hit.curves[0].curve,
        SurfaceIntersectionCurve::Circle(_)
    ));
    assert!(hit.curves[0].curve_range.width() > 0.0);
    assert!(
        a.eval(hit.curves[0].uv_a_start)
            .dist(b.eval(hit.curves[0].uv_b_start))
            <= Tolerances::default().linear()
    );
    assert!(
        a.eval(hit.curves[0].uv_a_end)
            .dist(b.eval(hit.curves[0].uv_b_end))
            <= Tolerances::default().linear()
    );

    let swapped =
        intersect_bounded_spheres(&b, b_window, &a, a_window, Tolerances::default()).unwrap();
    assert_eq!(hit.swapped(), swapped);
}

#[test]
fn general_non_octant_two_exact_boundary_locks_emit_tangent_point_and_swap() {
    let a = world_sphere();
    let b = Sphere::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, -1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let a_window = window(0.0, 0.8, 0.0, 0.5);
    let b_window = window(-0.8, 0.0, 0.0, 0.5);
    let hit = intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert!(hit.is_complete());
    assert_eq!(hit.points.len(), 1);
    assert!(hit.curves.is_empty());
    assert!(hit.regions.is_empty());
    assert_eq!(hit.points[0].kind, ContactKind::Tangent);
    assert!(hit.points[0].point.dist(Point3::new(1.0, 0.0, 0.0)) <= 1.0e-14);

    let swapped =
        intersect_bounded_spheres(&b, b_window, &a, a_window, Tolerances::default()).unwrap();
    assert_eq!(hit.swapped(), swapped);
}

#[test]
fn general_non_octant_near_lock_stays_indeterminate_without_exact_equality() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.4);
    let hit = intersect_bounded_spheres(
        &a,
        window(0.0, 0.8, -0.3, 0.5),
        &b,
        window(-0.8, 1.0e-12, -0.2, 0.4),
        Tolerances::default(),
    )
    .unwrap();
    assert!(hit.is_empty());
    assert!(matches!(hit.completion(), Completion::Indeterminate { .. }));
}

#[test]
fn general_single_wide_window_certifies_contained_region_and_swap() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.2);
    let a_window = window(-0.6, 1.2 * core::f64::consts::PI - 0.6, -0.8, 0.8);
    let b_window = window(-0.25, 0.25, -0.2, 0.2);
    let hit = intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert_general_sphere_window_region(&hit, &a, &b);
    let SurfaceRegionCorrespondence::GeneralSphereWindow(map) = hit.regions[0].correspondence
    else {
        unreachable!()
    };
    assert_eq!(map.first_range(), a_window);
    assert_eq!(map.second_range(), b_window);
    let width = a_window[0].width();
    for seam in [
        a_window[0].lo + width / 3.0,
        a_window[0].lo + 2.0 * width / 3.0,
    ] {
        assert!(
            hit.regions[0]
                .boundary
                .iter()
                .all(|vertex| (vertex.uv_a[0] - seam).abs() > Tolerances::default().angular())
        );
    }

    let swapped =
        intersect_bounded_spheres(&b, b_window, &a, a_window, Tolerances::default()).unwrap();
    assert_eq!(hit.swapped(), swapped);
}

#[test]
fn general_single_wide_window_certifies_empty_union_and_swap() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.2);
    let a_window = window(-0.6, core::f64::consts::PI - 0.6, -0.8, 0.8);
    let b_window = window(-2.6, -2.3, -0.2, 0.2);
    let empty =
        intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert!(empty.is_proven_empty());

    let swapped =
        intersect_bounded_spheres(&b, b_window, &a, a_window, Tolerances::default()).unwrap();
    assert_eq!(empty.swapped(), swapped);
}

#[test]
fn general_both_wide_windows_certify_empty_grid_repeatably_and_swap() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.2);
    let a_window = window(-0.6, -0.6 + 1.01 * core::f64::consts::PI, -1.2, -0.8);
    let b_window = window(-0.4, -0.4 + 1.1 * core::f64::consts::PI, 0.8, 1.2);
    let empty =
        intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert!(empty.is_proven_empty());

    let repeated =
        intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert_eq!(empty, repeated);
    let swapped =
        intersect_bounded_spheres(&b, b_window, &a, a_window, Tolerances::default()).unwrap();
    assert_eq!(empty.swapped(), swapped);

    let near_parallel = y_tilted_sphere(
        Point3::new(0.0, 0.0, 0.0),
        1.0,
        0.5 * Tolerances::default().angular(),
    );
    let uncertified = intersect_bounded_spheres(
        &a,
        a_window,
        &near_parallel,
        b_window,
        Tolerances::default(),
    )
    .unwrap();
    assert!(uncertified.is_empty());
    assert!(matches!(
        uncertified.completion(),
        Completion::Indeterminate {
            reason: "general coincident sphere window boundary planes exceed the certified angular corridor"
                | "general coincident sphere window proof encountered an unresolved multiple boundary vertex"
        }
    ));
}

#[test]
fn general_both_wide_windows_certify_single_cell_region_repeatably_and_swap() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.2);
    let a_window = window(-0.6, -0.6 + 1.01 * core::f64::consts::PI, -0.4, 0.4);
    let b_window = window(2.2, 2.2 + 1.01 * core::f64::consts::PI, -0.35, 0.35);
    let hit = intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert_general_sphere_window_region(&hit, &a, &b);
    let SurfaceRegionCorrespondence::GeneralSphereWindow(map) = hit.regions[0].correspondence
    else {
        unreachable!()
    };
    assert_eq!(map.first_range(), a_window);
    assert_eq!(map.second_range(), b_window);
    for (range, first_operand) in [(a_window[0], true), (b_window[0], false)] {
        for seam in [
            range.lo + range.width() / 3.0,
            range.lo + 2.0 * range.width() / 3.0,
        ] {
            assert!(hit.regions[0].boundary.iter().all(|vertex| {
                let parameter = if first_operand {
                    vertex.uv_a[0]
                } else {
                    vertex.uv_b[0]
                };
                (parameter - seam).abs() > Tolerances::default().angular()
            }));
        }
    }

    let repeated =
        intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert_eq!(hit, repeated);
    let swapped =
        intersect_bounded_spheres(&b, b_window, &a, a_window, Tolerances::default()).unwrap();
    assert_eq!(hit.clone().swapped(), swapped);
    assert_general_sphere_window_region(&swapped, &b, &a);
}

#[test]
fn general_both_wide_windows_certify_two_isolated_regions_repeatably_and_swap() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.2);
    let span = 1.2 * core::f64::consts::PI;
    let a_window = window(-0.6, -0.6 + span, -0.25, 0.25);
    let b_window = window(
        -0.6 + core::f64::consts::PI,
        -0.6 + core::f64::consts::PI + span,
        -0.25,
        0.25,
    );
    let hit = intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert_general_sphere_window_regions(&hit, &a, &b, 2);
    for region in &hit.regions {
        let SurfaceRegionCorrespondence::GeneralSphereWindow(map) = region.correspondence else {
            unreachable!()
        };
        assert_eq!(map.first_range(), a_window);
        assert_eq!(map.second_range(), b_window);
        for (range, first_operand) in [(a_window[0], true), (b_window[0], false)] {
            for seam in [
                range.lo + range.width() / 3.0,
                range.lo + 2.0 * range.width() / 3.0,
            ] {
                assert!(region.boundary.iter().all(|vertex| {
                    let parameter = if first_operand {
                        vertex.uv_a[0]
                    } else {
                        vertex.uv_b[0]
                    };
                    (parameter - seam).abs() > Tolerances::default().angular()
                }));
            }
        }
    }

    let repeated =
        intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert_eq!(hit, repeated);
    let swapped =
        intersect_bounded_spheres(&b, b_window, &a, a_window, Tolerances::default()).unwrap();
    assert_eq!(hit.clone().swapped(), swapped);
    assert_general_sphere_window_regions(&swapped, &b, &a, 2);
}

#[test]
fn general_both_wide_windows_merge_exact_adjacent_regions_and_swap() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.05);
    let a_window = window(-0.6, -0.6 + 1.01 * core::f64::consts::PI, -0.2, 0.2);
    let b_window = window(1.4, 1.4 + 1.3 * core::f64::consts::PI, -0.2, 0.2);
    let hit = intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert_general_sphere_window_region(&hit, &a, &b);
    assert_eq!(hit.regions[0].boundary.len(), 8);
    let SurfaceRegionCorrespondence::GeneralSphereWindow(map) = hit.regions[0].correspondence
    else {
        unreachable!()
    };
    assert_eq!(map.first_range(), a_window);
    assert_eq!(map.second_range(), b_window);

    let shared_seam = a_window[0].lo + 2.0 * a_window[0].width() / 3.0;
    let seam_vertices = hit.regions[0]
        .boundary
        .iter()
        .enumerate()
        .filter_map(|(index, vertex)| {
            (vertex.uv_a[0].to_bits() == shared_seam.to_bits()).then_some(index)
        })
        .collect::<Vec<_>>();
    assert_eq!(seam_vertices.len(), 2);
    assert_ne!(
        (seam_vertices[0] + 1) % hit.regions[0].boundary.len(),
        seam_vertices[1]
    );
    assert_ne!(
        (seam_vertices[1] + 1) % hit.regions[0].boundary.len(),
        seam_vertices[0]
    );
    let other_a_seam = a_window[0].lo + a_window[0].width() / 3.0;
    assert!(
        hit.regions[0]
            .boundary
            .iter()
            .all(|vertex| vertex.uv_a[0].to_bits() != other_a_seam.to_bits())
    );
    for b_seam in [
        b_window[0].lo + b_window[0].width() / 3.0,
        b_window[0].lo + 2.0 * b_window[0].width() / 3.0,
    ] {
        assert!(
            hit.regions[0]
                .boundary
                .iter()
                .all(|vertex| vertex.uv_b[0].to_bits() != b_seam.to_bits())
        );
    }

    let repeated =
        intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert_eq!(hit, repeated);
    let swapped =
        intersect_bounded_spheres(&b, b_window, &a, a_window, Tolerances::default()).unwrap();
    assert_eq!(hit.clone().swapped(), swapped);
    assert_general_sphere_window_region(&swapped, &b, &a);
}

#[test]
fn general_both_wide_windows_merge_exact_bent_three_cell_path_and_swap() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.05);
    let a_window = window(-0.6, -0.6 + 1.01 * core::f64::consts::PI, -0.2, 0.2);
    let b_window = window(1.4, 1.4 + 1.02 * core::f64::consts::PI, -0.2, 0.2);
    let hit = intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert_general_sphere_window_region(&hit, &a, &b);
    assert_eq!(hit.regions[0].boundary.len(), 10);
    let SurfaceRegionCorrespondence::GeneralSphereWindow(map) = hit.regions[0].correspondence
    else {
        unreachable!()
    };
    assert_eq!(map.first_range(), a_window);
    assert_eq!(map.second_range(), b_window);

    for (seam_on_first_operand, seam) in [
        (true, a_window[0].lo + 2.0 * a_window[0].width() / 3.0),
        (false, b_window[0].lo + b_window[0].width() / 3.0),
    ] {
        let seam_vertices = hit.regions[0]
            .boundary
            .iter()
            .enumerate()
            .filter_map(|(index, vertex)| {
                let parameter = if seam_on_first_operand {
                    vertex.uv_a[0]
                } else {
                    vertex.uv_b[0]
                };
                (parameter.to_bits() == seam.to_bits()).then_some(index)
            })
            .collect::<Vec<_>>();
        assert_eq!(seam_vertices.len(), 2);
        assert_ne!(
            (seam_vertices[0] + 1) % hit.regions[0].boundary.len(),
            seam_vertices[1]
        );
        assert_ne!(
            (seam_vertices[1] + 1) % hit.regions[0].boundary.len(),
            seam_vertices[0]
        );
    }

    for (seam_on_first_operand, seam) in [
        (true, a_window[0].lo + a_window[0].width() / 3.0),
        (false, b_window[0].lo + 2.0 * b_window[0].width() / 3.0),
    ] {
        assert!(hit.regions[0].boundary.iter().all(|vertex| {
            let parameter = if seam_on_first_operand {
                vertex.uv_a[0]
            } else {
                vertex.uv_b[0]
            };
            parameter.to_bits() != seam.to_bits()
        }));
    }

    let repeated =
        intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert_eq!(hit, repeated);
    let swapped =
        intersect_bounded_spheres(&b, b_window, &a, a_window, Tolerances::default()).unwrap();
    assert_eq!(hit.clone().swapped(), swapped);
    assert_general_sphere_window_region(&swapped, &b, &a);
}

#[test]
fn general_both_wide_windows_merge_exact_four_cell_path_and_swap() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.05);
    let a_window = window(-0.6, -0.6 + 1.01 * core::f64::consts::PI, -0.2, 0.2);
    let b_window = window(0.3, 0.3 + 1.3 * core::f64::consts::PI, -0.2, 0.2);
    let hit = intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert_general_sphere_window_region(&hit, &a, &b);
    assert_eq!(hit.regions[0].boundary.len(), 12);
    let SurfaceRegionCorrespondence::GeneralSphereWindow(map) = hit.regions[0].correspondence
    else {
        unreachable!()
    };
    assert_eq!(map.first_range(), a_window);
    assert_eq!(map.second_range(), b_window);

    for (seam_on_first_operand, seam) in [
        (true, a_window[0].lo + a_window[0].width() / 3.0),
        (true, a_window[0].lo + 2.0 * a_window[0].width() / 3.0),
        (false, b_window[0].lo + b_window[0].width() / 3.0),
    ] {
        let seam_vertices = hit.regions[0]
            .boundary
            .iter()
            .enumerate()
            .filter_map(|(index, vertex)| {
                let parameter = if seam_on_first_operand {
                    vertex.uv_a[0]
                } else {
                    vertex.uv_b[0]
                };
                (parameter.to_bits() == seam.to_bits()).then_some(index)
            })
            .collect::<Vec<_>>();
        assert_eq!(seam_vertices.len(), 2);
        assert_ne!(
            (seam_vertices[0] + 1) % hit.regions[0].boundary.len(),
            seam_vertices[1]
        );
        assert_ne!(
            (seam_vertices[1] + 1) % hit.regions[0].boundary.len(),
            seam_vertices[0]
        );
    }

    let unused_b_seam = b_window[0].lo + 2.0 * b_window[0].width() / 3.0;
    assert!(
        hit.regions[0]
            .boundary
            .iter()
            .all(|vertex| vertex.uv_b[0].to_bits() != unused_b_seam.to_bits())
    );

    let repeated =
        intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert_eq!(hit, repeated);
    let swapped =
        intersect_bounded_spheres(&b, b_window, &a, a_window, Tolerances::default()).unwrap();
    assert_eq!(hit.clone().swapped(), swapped);
    assert_general_sphere_window_region(&swapped, &b, &a);
}

#[test]
fn general_both_wide_four_cell_path_rejects_approximate_shared_seam() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.05);
    let a_window = window(-0.6, -0.6 + 1.01 * core::f64::consts::PI, -0.2, 0.2);
    // The four positive cells are [0, 0], [1, 0], [2, 0], and [2, 1],
    // but the last shared-seam endpoint parameters recover one ulp away from
    // the decomposition seam and therefore cannot prove a bit-exact splice.
    let b_window = window(0.4, 0.4 + 1.3 * core::f64::consts::PI, -0.2, 0.2);
    let hit = intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert_indeterminate_sphere_window(
        &hit,
        "general coincident sphere both-wide union supports at most four positive cells; three or four cells require an exact shared-seam path",
    );

    let repeated =
        intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert_eq!(hit, repeated);
    let swapped =
        intersect_bounded_spheres(&b, b_window, &a, a_window, Tolerances::default()).unwrap();
    assert_eq!(hit.clone().swapped(), swapped);
}

#[test]
fn general_single_wide_window_preserves_parent_periodic_seam_evidence() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.2);
    let tau = core::f64::consts::TAU;
    let a_window = window(tau - 0.6, tau - 0.6 + core::f64::consts::PI, -0.8, 0.8);
    let b_window = window(-0.25, 0.25, -0.15, 0.15);
    let hit = intersect_bounded_spheres(&a, a_window, &b, b_window, Tolerances::default()).unwrap();
    assert_general_sphere_window_region(&hit, &a, &b);
    let SurfaceRegionCorrespondence::GeneralSphereWindow(map) = hit.regions[0].correspondence
    else {
        unreachable!()
    };
    assert_eq!(map.first_range(), a_window);
    assert!(
        hit.regions[0]
            .boundary
            .iter()
            .all(|vertex| vertex.uv_a[0] >= tau - 0.6)
    );
}

#[test]
fn general_wide_window_union_fails_closed_across_artificial_seams_and_two_wide_inputs() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.2);
    let crossing = intersect_bounded_spheres(
        &a,
        window(-0.6, core::f64::consts::PI - 0.6, -0.8, 0.8),
        &b,
        window(0.35, 0.55, -0.2, 0.2),
        Tolerances::default(),
    )
    .unwrap();
    assert!(crossing.is_empty());
    assert!(
        matches!(
            crossing.completion(),
            Completion::Indeterminate {
                reason: "general coincident sphere wide-window union requires one positive-area cell and certified-empty siblings"
            }
        ),
        "unexpected seam-crossing result: {crossing:?}"
    );

    let both_a_window = window(-0.6, -0.6 + 1.1 * core::f64::consts::PI, -0.8, 0.8);
    let both_b_window = window(-0.4, -0.4 + 1.1 * core::f64::consts::PI, -0.7, 0.7);
    let both_wide =
        intersect_bounded_spheres(&a, both_a_window, &b, both_b_window, Tolerances::default())
            .unwrap();
    assert!(both_wide.is_empty());
    assert!(matches!(
        both_wide.completion(),
        Completion::Indeterminate {
            reason: "general coincident sphere both-wide union supports at most four positive cells; three or four cells require an exact shared-seam path"
        }
    ));
    let repeated_both_wide =
        intersect_bounded_spheres(&a, both_a_window, &b, both_b_window, Tolerances::default())
            .unwrap();
    assert_eq!(both_wide, repeated_both_wide);

    let shared_seam = intersect_bounded_spheres(
        &a,
        window(-0.6, -0.6 + 1.1 * core::f64::consts::PI, -0.4, 0.4),
        &b,
        window(1.35, 1.35 + 1.1 * core::f64::consts::PI, -0.35, 0.35),
        Tolerances::default(),
    )
    .unwrap();
    assert_indeterminate_sphere_window(
        &shared_seam,
        "general coincident sphere both-wide union supports at most four positive cells; three or four cells require an exact shared-seam path",
    );

    let polar = intersect_bounded_spheres(
        &a,
        window(
            -0.6,
            core::f64::consts::PI - 0.6,
            -core::f64::consts::FRAC_PI_2,
            0.8,
        ),
        &b,
        window(-0.25, 0.25, -0.2, 0.2),
        Tolerances::default(),
    )
    .unwrap();
    assert_indeterminate_sphere_window(
        &polar,
        "general coincident sphere window proof supports only positive-area pole-clear windows",
    );
}

#[test]
fn arbitrary_rotated_octants_emit_nonlinear_regions_and_swap_exactly() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.4);
    let half_pi = core::f64::consts::FRAC_PI_2;
    let octant = window(0.0, half_pi, 0.0, half_pi);
    let hit = intersect_bounded_spheres(&a, octant, &b, octant, Tolerances::default()).unwrap();
    assert_arbitrary_octant_region(&hit, &a, &b);
    assert_eq!(hit.regions[0].boundary.len(), 3);

    let SurfaceRegionCorrespondence::ArbitrarySphereOctant(map) = hit.regions[0].correspondence
    else {
        unreachable!()
    };
    let interior = [0.5, 0.3];
    let mapped = map
        .map_first_to_second(interior)
        .expect("chosen strict interior direction lies in both octants");
    assert_ne!(mapped, interior);
    assert!(a.eval(interior).dist(b.eval(mapped)) <= hit.regions[0].max_residual);
    for u in [0.1, 0.5, 1.0] {
        for v in [0.1, 0.3] {
            let uv_a = [u, v];
            let uv_b = map
                .map_first_to_second(uv_a)
                .expect("certified strict polygon sample must map");
            assert!(a.eval(uv_a).dist(b.eval(uv_b)) <= hit.regions[0].max_residual);
        }
    }
    assert!(map.map_first_to_second([1.4, 1.4]).is_none());

    let direct_swapped =
        intersect_bounded_spheres(&b, octant, &a, octant, Tolerances::default()).unwrap();
    assert_eq!(hit.clone().swapped(), direct_swapped);
    assert_arbitrary_octant_region(&direct_swapped, &b, &a);
}

#[test]
fn remote_arbitrary_octant_map_rejects_point_outside_certified_allowance() {
    let a = world_sphere();
    let b = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, 0.4);
    let half_pi = core::f64::consts::FRAC_PI_2;
    let remote_index = 22_504_i64;
    let remote = window(
        (remote_index as f64) * half_pi,
        ((remote_index + 1) as f64) * half_pi,
        0.0,
        half_pi,
    );
    let hit = intersect_bounded_spheres(&a, remote, &b, remote, Tolerances::default()).unwrap();
    assert_arbitrary_octant_region(&hit, &a, &b);
    let SurfaceRegionCorrespondence::ArbitrarySphereOctant(map) = hit.regions[0].correspondence
    else {
        unreachable!()
    };

    let boundary = [remote[0].lo, half_pi - 0.4];
    assert!(
        map.map_first_to_second(boundary).is_some(),
        "the certified remote phase allowance must retain the exact target boundary"
    );
    let outside = [boundary[0], boundary[1] + 1.0e-9];
    assert!(remote[0].contains(outside[0]));
    assert!(remote[1].contains(outside[1]));
    assert!(
        map.map_first_to_second(outside).is_none(),
        "remote parameter magnitude must not widen the certified target boundary"
    );
}

#[test]
fn arbitrary_rotated_octants_collapse_to_arc_point_and_empty() {
    let a = world_sphere();
    let half_pi = core::f64::consts::FRAC_PI_2;
    let positive = window(0.0, half_pi, 0.0, half_pi);

    let angle = 0.4;
    let arc_frame = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(math::cos(angle), math::sin(angle), 0.0),
        Vec3::new(0.0, 0.0, -1.0),
    )
    .unwrap();
    let arc_sphere = Sphere::new(arc_frame, 1.0).unwrap();
    let arc = intersect_bounded_spheres(&a, positive, &arc_sphere, positive, Tolerances::default())
        .unwrap();
    assert!(arc.points.is_empty());
    assert_eq!(arc.curves.len(), 1);
    assert!(arc.regions.is_empty());
    assert!(matches!(
        arc.curves[0].curve,
        SurfaceIntersectionCurve::Circle(_)
    ));

    let tilted = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, angle);
    let point_window = window(half_pi, core::f64::consts::PI, -half_pi, 0.0);
    let point =
        intersect_bounded_spheres(&a, positive, &tilted, point_window, Tolerances::default())
            .unwrap();
    assert_eq!(point.points.len(), 1);
    assert!(point.curves.is_empty());
    assert!(point.regions.is_empty());

    let miss_window = window(core::f64::consts::PI, 3.0 * half_pi, -half_pi, 0.0);
    let miss = intersect_bounded_spheres(&a, positive, &tilted, miss_window, Tolerances::default())
        .unwrap();
    assert!(miss.is_proven_empty());

    for (b_sphere, b_window, result) in [
        (&arc_sphere, positive, arc),
        (&tilted, point_window, point),
        (&tilted, miss_window, miss),
    ] {
        let swapped =
            intersect_bounded_spheres(b_sphere, b_window, &a, positive, Tolerances::default())
                .unwrap();
        assert_eq!(result.swapped(), swapped);
    }
}

#[test]
fn arbitrary_octant_residual_bound_is_outward_at_large_model_scale() {
    let origin = Point3::new(1.0e12, -2.0e12, 3.0e12);
    let radius = 1.0e6;
    let a = Sphere::new(Frame::world().with_origin(origin), radius).unwrap();
    let b = y_tilted_sphere(origin, radius, 0.4);
    let half_pi = core::f64::consts::FRAC_PI_2;
    let octant = window(0.0, half_pi, 0.0, half_pi);
    let hit = intersect_bounded_spheres(&a, octant, &b, octant, Tolerances::default()).unwrap();
    assert_arbitrary_octant_region(&hit, &a, &b);
    assert!(hit.regions[0].max_residual.is_finite());
    assert!(hit.regions[0].max_residual > 0.0);
    let SurfaceRegionCorrespondence::ArbitrarySphereOctant(map) = hit.regions[0].correspondence
    else {
        unreachable!()
    };
    assert!(
        map.map_first_to_second([0.0, half_pi - 0.4 + 1.0e-7])
            .is_none(),
        "large model origin must not widen the origin-independent chart domain"
    );

    let mut invalid = hit.regions[0].clone();
    invalid.orientation = SurfaceRegionOrientation::Reversed;
    let error = SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
        Vec::new(),
        Vec::new(),
        vec![invalid],
    )
    .unwrap_err();
    assert_eq!(
        error,
        Error::InvalidGeometry {
            reason: "arbitrary sphere octant regions require mutually mapped boundary anchors and same orientation"
        }
    );

    let mut mismatched_anchor = hit.regions[0].clone();
    let mismatched_uv_b = map
        .map_first_to_second([0.5, 0.3])
        .expect("strict interior sample must map into both octants");
    assert!(map.map_second_to_first(mismatched_uv_b).is_some());
    mismatched_anchor.boundary[0].uv_b = mismatched_uv_b;
    let error = SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
        Vec::new(),
        Vec::new(),
        vec![mismatched_anchor],
    )
    .unwrap_err();
    assert_eq!(
        error,
        Error::InvalidGeometry {
            reason: "arbitrary sphere octant regions require mutually mapped boundary anchors and same orientation"
        }
    );
}

#[test]
fn ill_conditioned_nonzero_arbitrary_octants_fail_closed_without_dimension_collapse() {
    let a = world_sphere();
    let tolerance = Tolerances::default();
    let half_pi = core::f64::consts::FRAC_PI_2;
    let positive = window(0.0, half_pi, 0.0, half_pi);
    let delta = 0.5 * tolerance.angular();
    let expected = Error::InvalidGeometry {
        reason: "arbitrary sphere octant boundary planes exceed the certified angular corridor",
    };

    let near_parallel = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, delta);
    assert_eq!(
        intersect_bounded_spheres(&a, positive, &near_parallel, positive, tolerance).unwrap_err(),
        expected
    );

    let narrow_region = y_tilted_sphere(Point3::new(0.0, 0.0, 0.0), 1.0, half_pi - delta);
    assert_eq!(
        intersect_bounded_spheres(&a, positive, &narrow_region, positive, tolerance).unwrap_err(),
        expected
    );

    let narrow_arc_frame = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(math::cos(half_pi - delta), math::sin(half_pi - delta), 0.0),
        Vec3::new(0.0, 0.0, -1.0),
    )
    .unwrap();
    let narrow_arc = Sphere::new(narrow_arc_frame, 1.0).unwrap();
    assert_eq!(
        intersect_bounded_spheres(&a, positive, &narrow_arc, positive, tolerance).unwrap_err(),
        expected
    );
}

#[test]
fn subnormal_center_and_axis_deltas_cannot_alias_exact_coincidence() {
    let a = world_sphere();
    let tiny = f64::from_bits(1);
    let half_pi = core::f64::consts::FRAC_PI_2;
    let octant = window(0.0, half_pi, 0.0, half_pi);

    let shifted =
        Sphere::new(Frame::world().with_origin(Point3::new(tiny, 0.0, 0.0)), 1.0).unwrap();
    assert_eq!(
        intersect_bounded_spheres(&a, octant, &shifted, octant, Tolerances::default()).unwrap_err(),
        Error::InvalidGeometry {
            reason: "near-coincident non-identical spheres require the general certified fallback"
        }
    );

    let tilted = Sphere::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(tiny, 0.0, 1.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    assert_eq!(
        intersect_bounded_spheres(&a, octant, &tilted, octant, Tolerances::default()).unwrap_err(),
        Error::InvalidGeometry {
            reason: "arbitrary sphere octant boundary planes exceed the certified angular corridor"
        }
    );
}

#[test]
fn nonparallel_signed_axis_octants_have_exact_bidirectional_region_maps_and_swap() {
    let a = world_sphere();
    let b = orthogonal_sphere(Point3::new(0.0, 0.0, 0.0), 1.0);
    let half_pi = core::f64::consts::FRAC_PI_2;
    let octant = window(0.0, half_pi, 0.0, half_pi);
    let hit = intersect_bounded_spheres(&a, octant, &b, octant, Tolerances::default()).unwrap();
    assert_orthogonal_octant_region(&hit, &a, &b);

    let SurfaceRegionCorrespondence::OrthogonalSphereOctant(map) = hit.regions[0].correspondence
    else {
        unreachable!()
    };
    let interior = [0.31, 0.47];
    let mapped = map.map_first_to_second(interior).unwrap();
    assert_ne!(
        mapped, interior,
        "the accepted chart map is genuinely nonlinear"
    );

    let direct_swapped =
        intersect_bounded_spheres(&b, octant, &a, octant, Tolerances::default()).unwrap();
    assert_eq!(hit.clone().swapped(), direct_swapped);
    assert_orthogonal_octant_region(&direct_swapped, &b, &a);
}

#[test]
fn nonparallel_signed_axis_octants_collapse_to_exact_edge_vertex_or_miss() {
    let a = world_sphere();
    let b = orthogonal_sphere(Point3::new(0.0, 0.0, 0.0), 1.0);
    let half_pi = core::f64::consts::FRAC_PI_2;
    let a_octant = window(0.0, half_pi, 0.0, half_pi);
    let b_windows = [
        window(-half_pi, 0.0, 0.0, half_pi),
        window(-half_pi, 0.0, -half_pi, 0.0),
        window(2.0 * half_pi, 3.0 * half_pi, -half_pi, 0.0),
    ];

    let edge =
        intersect_bounded_spheres(&a, a_octant, &b, b_windows[0], Tolerances::default()).unwrap();
    assert!(edge.is_complete());
    assert!(edge.points.is_empty());
    assert_eq!(edge.curves.len(), 1);
    assert!(edge.regions.is_empty());
    assert!(matches!(
        edge.curves[0].curve,
        SurfaceIntersectionCurve::Circle(_)
    ));
    assert_eq!(
        edge.curves[0].curve_range,
        range(0.0, core::f64::consts::FRAC_PI_2)
    );

    let vertex =
        intersect_bounded_spheres(&a, a_octant, &b, b_windows[1], Tolerances::default()).unwrap();
    assert!(vertex.is_complete());
    assert_eq!(vertex.points.len(), 1);
    assert!(vertex.curves.is_empty());
    assert!(vertex.regions.is_empty());
    assert_eq!(vertex.points[0].kind, ContactKind::Singular);

    let miss =
        intersect_bounded_spheres(&a, a_octant, &b, b_windows[2], Tolerances::default()).unwrap();
    assert!(miss.is_proven_empty());

    for (b_window, hit) in [
        (b_windows[0], edge),
        (b_windows[1], vertex),
        (b_windows[2], miss),
    ] {
        let direct_swapped =
            intersect_bounded_spheres(&b, b_window, &a, a_octant, Tolerances::default()).unwrap();
        assert_eq!(hit.swapped(), direct_swapped);
    }
}

#[test]
fn orthogonal_octant_residual_certificate_is_outward_at_large_model_scale() {
    let origin = Point3::new(1.0e12, -2.0e12, 3.0e12);
    let radius = 1.0e6;
    let a = Sphere::new(Frame::world().with_origin(origin), radius).unwrap();
    let b = orthogonal_sphere(origin, radius);
    let half_pi = core::f64::consts::FRAC_PI_2;
    let octant = window(0.0, half_pi, 0.0, half_pi);
    let hit = intersect_bounded_spheres(&a, octant, &b, octant, Tolerances::default()).unwrap();
    assert_orthogonal_octant_region(&hit, &a, &b);
    assert!(hit.regions[0].max_residual.is_finite());
    assert!(hit.regions[0].max_residual > 0.0);
}

#[test]
fn angular_safe_remote_octant_map_and_bound_cover_both_whole_windows() {
    let a = world_sphere();
    let b = orthogonal_sphere(Point3::new(0.0, 0.0, 0.0), 1.0);
    let half_pi = core::f64::consts::FRAC_PI_2;
    let first_index = 22_504_i64;
    let second_index = first_index + 8;
    let remote_octant = |index: i64| {
        window(
            (index as f64) * half_pi,
            ((index + 1) as f64) * half_pi,
            0.0,
            half_pi,
        )
    };
    let hit = intersect_bounded_spheres(
        &a,
        remote_octant(first_index),
        &b,
        remote_octant(second_index),
        Tolerances::default(),
    )
    .unwrap();
    assert_orthogonal_octant_region(&hit, &a, &b);
    assert!(hit.regions[0].max_residual > Tolerances::default().angular());
}

#[test]
fn remote_octants_outside_angular_phase_corridor_fail_closed() {
    let a = world_sphere();
    let b = orthogonal_sphere(Point3::new(0.0, 0.0, 0.0), 1.0);
    let half_pi = core::f64::consts::FRAC_PI_2;
    let remote_index = 400_000_000_i64;
    let remote = window(
        (remote_index as f64) * half_pi,
        ((remote_index + 1) as f64) * half_pi,
        0.0,
        half_pi,
    );
    let error =
        intersect_bounded_spheres(&a, remote, &b, remote, Tolerances::default()).unwrap_err();
    assert_eq!(
        error,
        Error::InvalidGeometry {
            reason: "coincident sphere charts with nonparallel latitude axes require the general certified fallback"
        }
    );
}

#[test]
fn angular_safe_remote_adjacent_octants_preserve_edge_vertex_and_miss_dimension() {
    let a = world_sphere();
    let b = orthogonal_sphere(Point3::new(0.0, 0.0, 0.0), 1.0);
    let half_pi = core::f64::consts::FRAC_PI_2;
    let remote = |index: i64, v_lo: f64, v_hi: f64| {
        window(
            (index as f64) * half_pi,
            ((index + 1) as f64) * half_pi,
            v_lo,
            v_hi,
        )
    };
    let a_octant = remote(22_504, 0.0, half_pi);

    let edge = intersect_bounded_spheres(
        &a,
        a_octant,
        &b,
        remote(22_511, 0.0, half_pi),
        Tolerances::default(),
    )
    .unwrap();
    assert!(edge.points.is_empty());
    assert_eq!(edge.curves.len(), 1);
    assert!(edge.regions.is_empty());

    let vertex = intersect_bounded_spheres(
        &a,
        a_octant,
        &b,
        remote(22_511, -half_pi, 0.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(vertex.points.len(), 1);
    assert!(vertex.curves.is_empty());
    assert!(vertex.regions.is_empty());

    let miss = intersect_bounded_spheres(
        &a,
        a_octant,
        &b,
        remote(22_510, -half_pi, 0.0),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_proven_empty());
}

#[test]
fn near_integer_exactness_limit_octants_fail_closed_before_phase_aliasing() {
    let a = world_sphere();
    let b = orthogonal_sphere(Point3::new(0.0, 0.0, 0.0), 1.0);
    let half_pi = core::f64::consts::FRAC_PI_2;
    let near_limit = (1_i64 << 52) - 8;
    let adversarial = window(
        (near_limit as f64) * half_pi,
        ((near_limit + 1) as f64) * half_pi,
        0.0,
        half_pi,
    );
    let error = intersect_bounded_spheres(&a, adversarial, &b, adversarial, Tolerances::default())
        .unwrap_err();
    assert_eq!(
        error,
        Error::InvalidGeometry {
            reason: "coincident sphere charts with nonparallel latitude axes require the general certified fallback"
        }
    );
}

#[test]
fn aligned_partial_overlap_and_containment_are_complete_regions() {
    let sphere = world_sphere();
    let partial = intersect_bounded_spheres(
        &sphere,
        window(0.0, 2.0, -0.5, 0.8),
        &sphere,
        window(1.0, 3.0, 0.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&partial, &sphere, &sphere);
    assert_eq!(partial.regions.len(), 1);
    assert_eq!(partial.regions[0].boundary[0].uv_a, [1.0, 0.0]);
    assert_eq!(partial.regions[0].boundary[2].uv_a, [2.0, 0.8]);

    let contained = intersect_bounded_spheres(
        &sphere,
        window(-1.0, 2.0, -1.0, 1.0),
        &sphere,
        window(-0.5, 0.5, -0.25, 0.25),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&contained, &sphere, &sphere);
    assert_eq!(contained.regions[0].boundary[0].uv_a, [-0.5, -0.25]);
}

#[test]
fn rotated_and_antiparallel_common_axis_charts_preserve_paired_regions_and_swap() {
    let a = world_sphere();
    let angle = 0.4_f64;
    let rotated = Sphere::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(math::cos(angle), math::sin(angle), 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let a_range = window(0.4, 1.4, -0.5, 0.75);
    let b_range = window(0.6 - angle, 1.2 - angle, -0.25, 0.5);
    let hit =
        intersect_bounded_spheres(&a, a_range, &rotated, b_range, Tolerances::default()).unwrap();
    assert_regions_lift(&hit, &a, &rotated);
    for vertex in &hit.regions[0].boundary {
        assert!((vertex.uv_b[0] - (vertex.uv_a[0] - angle)).abs() < 1.0e-15);
        assert_eq!(vertex.uv_b[1], vertex.uv_a[1]);
    }
    let direct_swapped =
        intersect_bounded_spheres(&rotated, b_range, &a, a_range, Tolerances::default()).unwrap();
    assert_eq!(hit.clone().swapped(), direct_swapped);

    let antiparallel = Sphere::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let anti = intersect_bounded_spheres(
        &a,
        window(0.2, 1.0, -0.75, 0.5),
        &antiparallel,
        window(-0.8, -0.4, -0.25, 0.5),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&anti, &a, &antiparallel);
    for vertex in &anti.regions[0].boundary {
        assert_eq!(vertex.uv_b[0], -vertex.uv_a[0]);
        assert_eq!(vertex.uv_b[1], -vertex.uv_a[1]);
    }
}

#[test]
fn full_turn_overlap_splits_deterministically_at_the_other_chart_seam() {
    let sphere = world_sphere();
    let tau = core::f64::consts::TAU;
    let first = intersect_bounded_spheres(
        &sphere,
        window(0.0, tau, -0.5, 0.5),
        &sphere,
        window(1.0, 1.0 + tau, -0.5, 0.5),
        Tolerances::default(),
    )
    .unwrap();
    let second = intersect_bounded_spheres(
        &sphere,
        window(0.0, tau, -0.5, 0.5),
        &sphere,
        window(1.0, 1.0 + tau, -0.5, 0.5),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(first, second);
    assert_regions_lift(&first, &sphere, &sphere);
    assert_eq!(first.regions.len(), 2);
    assert_eq!(first.regions[0].boundary[0].uv_a[0], 0.0);
    assert_eq!(first.regions[0].boundary[1].uv_a[0], 1.0);
    assert_eq!(first.regions[1].boundary[0].uv_a[0], 1.0);
    assert_eq!(first.regions[1].boundary[1].uv_a[0], tau);
}

#[test]
fn collapsed_windows_and_chart_disjoint_poles_are_dimensionally_truthful() {
    let sphere = world_sphere();
    let latitude = intersect_bounded_spheres(
        &sphere,
        window(0.0, 1.0, 0.25, 0.25),
        &sphere,
        window(0.2, 0.8, 0.25, 0.25),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(latitude.curves.len(), 1);
    assert!(matches!(
        latitude.curves[0].curve,
        SurfaceIntersectionCurve::Circle(_)
    ));
    assert_eq!(latitude.curves[0].curve_range, range(0.2, 0.8));

    let meridian = intersect_bounded_spheres(
        &sphere,
        window(0.5, 0.5, -0.75, 0.75),
        &sphere,
        window(0.5, 0.5, -0.5, 0.5),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(meridian.curves.len(), 1);
    assert!(matches!(
        meridian.curves[0].curve,
        SurfaceIntersectionCurve::Circle(_)
    ));
    assert_eq!(meridian.curves[0].curve_range, range(-0.5, 0.5));

    let point = intersect_bounded_spheres(
        &sphere,
        window(0.5, 0.5, 0.25, 0.25),
        &sphere,
        window(0.5, 0.5, 0.25, 0.25),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(point.points.len(), 1);
    assert_eq!(point.points[0].kind, ContactKind::Tangent);

    let half_pi = core::f64::consts::FRAC_PI_2;
    let pole = intersect_bounded_spheres(
        &sphere,
        window(0.0, 0.5, 1.0, half_pi),
        &sphere,
        window(1.0, 1.5, 1.0, half_pi),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(pole.points.len(), 1);
    assert_eq!(pole.points[0].kind, ContactKind::Singular);

    let two_poles = intersect_bounded_spheres(
        &sphere,
        window(0.0, 0.5, -half_pi, half_pi),
        &sphere,
        window(1.0, 1.5, -half_pi, half_pi),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(two_poles.points.len(), 2);
    assert!(
        two_poles
            .points
            .iter()
            .all(|point| point.kind == ContactKind::Singular)
    );

    let miss = intersect_bounded_spheres(
        &sphere,
        window(0.0, 0.5, -0.5, 0.5),
        &sphere,
        window(1.0, 1.5, -0.5, 0.5),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_proven_empty());
}

#[test]
fn pole_bounded_regions_retain_whole_patch_evidence() {
    let sphere = world_sphere();
    let half_pi = core::f64::consts::FRAC_PI_2;
    let hit = intersect_bounded_spheres(
        &sphere,
        window(0.0, 1.0, -half_pi, half_pi),
        &sphere,
        window(0.25, 0.75, -half_pi, half_pi),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&hit, &sphere, &sphere);
    assert_eq!(hit.regions.len(), 1);
    assert_eq!(hit.regions[0].boundary[0].point.z, -1.0);
    assert_eq!(hit.regions[0].boundary[2].point.z, 1.0);
}

#[test]
fn unsupported_chart_domains_and_near_coincidence_fail_closed() {
    let sphere = world_sphere();
    let tau = core::f64::consts::TAU;
    let half_pi = core::f64::consts::FRAC_PI_2;
    let overwide = intersect_bounded_spheres(
        &sphere,
        window(0.0, tau + 0.1, -0.5, 0.5),
        &sphere,
        window(0.0, tau, -0.5, 0.5),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        overwide,
        Error::InvalidGeometry {
            reason: "coincident sphere longitude windows cannot span more than one turn"
        }
    );

    let latitude = intersect_bounded_spheres(
        &sphere,
        window(0.0, 1.0, -half_pi - 0.1, 0.5),
        &sphere,
        window(0.0, 1.0, -half_pi, 0.5),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        latitude,
        Error::InvalidGeometry {
            reason: "coincident sphere latitude windows must stay inside the natural pole range"
        }
    );

    let near = Sphere::new(
        Frame::new(
            Point3::new(1.0e-8, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let near_error = intersect_bounded_spheres(
        &sphere,
        window(0.0, 1.0, -0.5, 0.5),
        &near,
        window(0.0, 1.0, -0.5, 0.5),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        near_error,
        Error::InvalidGeometry {
            reason: "near-coincident non-identical spheres require the general certified fallback"
        }
    );
}

#[test]
fn whole_region_bound_scales_with_large_periodic_parameters() {
    let sphere = world_sphere();
    let u = 1.0e8 * core::f64::consts::TAU;
    let hit = intersect_bounded_spheres(
        &sphere,
        window(u, u + 0.5, -0.5, 0.5),
        &sphere,
        window(u + 0.1, u + 0.4, -0.25, 0.25),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&hit, &sphere, &sphere);
}

fn bilinear_uv(
    region: &kops::intersect::SurfaceSurfaceRegion,
    first: bool,
    u_weight: f64,
    v_weight: f64,
) -> [f64; 2] {
    let uv = |index: usize| {
        if first {
            region.boundary[index].uv_a
        } else {
            region.boundary[index].uv_b
        }
    };
    let bottom = lerp2(uv(0), uv(1), u_weight);
    let top = lerp2(uv(3), uv(2), u_weight);
    lerp2(bottom, top, v_weight)
}

fn lerp2(a: [f64; 2], b: [f64; 2], weight: f64) -> [f64; 2] {
    [
        a[0] * (1.0 - weight) + b[0] * weight,
        a[1] * (1.0 - weight) + b[1] * weight,
    ]
}
