//! Coincident bounded plane/plane region and collapsed-contact conformance.

use kcore::error::Error;
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Surface};
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    ContactKind, SurfaceIntersectionCurve, SurfaceRegionOrientation, SurfaceSurfaceIntersections,
    SurfaceSurfaceRegion, SurfaceSurfaceRegionVertex, intersect_bounded_planes,
};

fn range(lo: f64, hi: f64) -> ParamRange {
    ParamRange::new(lo, hi)
}

fn window(u_lo: f64, u_hi: f64, v_lo: f64, v_hi: f64) -> [ParamRange; 2] {
    [range(u_lo, u_hi), range(v_lo, v_hi)]
}

fn world_plane() -> Plane {
    Plane::new(Frame::world())
}

fn assert_region_lifts(hit: &SurfaceSurfaceIntersections, a: &Plane, b: &Plane) {
    assert!(hit.is_complete());
    assert!(!hit.is_empty());
    assert!(hit.points.is_empty());
    assert!(hit.curves.is_empty());
    assert_eq!(hit.regions.len(), 1);
    let region = &hit.regions[0];
    assert!(region.boundary.len() >= 3);
    for vertex in &region.boundary {
        let pa = a.eval(vertex.uv_a);
        let pb = b.eval(vertex.uv_b);
        assert_eq!(vertex.point, (pa + pb) / 2.0);
        assert_eq!(vertex.residual, pa.dist(pb));
        assert!(vertex.residual <= region.max_residual);
    }
}

#[test]
fn aligned_partial_overlap_and_full_containment_are_complete_regions() {
    let plane = world_plane();
    let partial = intersect_bounded_planes(
        &plane,
        window(0.0, 2.0, 0.0, 2.0),
        &plane,
        window(1.0, 3.0, -1.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_region_lifts(&partial, &plane, &plane);
    assert_eq!(
        partial.regions[0]
            .boundary
            .iter()
            .map(|vertex| vertex.uv_a)
            .collect::<Vec<_>>(),
        vec![[1.0, 0.0], [2.0, 0.0], [2.0, 1.0], [1.0, 1.0]]
    );
    assert_eq!(
        partial.regions[0].orientation,
        SurfaceRegionOrientation::Same
    );

    let contained = intersect_bounded_planes(
        &plane,
        window(-2.0, 2.0, -2.0, 2.0),
        &plane,
        window(-0.5, 0.5, -1.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_region_lifts(&contained, &plane, &plane);
    assert_eq!(contained.regions[0].boundary.len(), 4);
    assert_eq!(contained.regions[0].boundary[0].uv_a, [-0.5, -1.0]);
}

#[test]
fn translated_rotated_and_antiparallel_charts_retain_paired_boundaries() {
    let a = world_plane();
    let translated = Plane::new(
        Frame::new(
            Point3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    );
    let translated_hit = intersect_bounded_planes(
        &a,
        window(0.0, 2.0, -1.0, 1.0),
        &translated,
        window(-1.0, 1.0, -1.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_region_lifts(&translated_hit, &a, &translated);
    for vertex in &translated_hit.regions[0].boundary {
        assert_eq!(vertex.uv_b[0], vertex.uv_a[0] - 1.0);
        assert_eq!(vertex.uv_b[1], vertex.uv_a[1]);
    }

    let angle = core::f64::consts::FRAC_PI_4;
    let rotated = Plane::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(math::cos(angle), math::sin(angle), 0.0),
        )
        .unwrap(),
    );
    let rotated_hit = intersect_bounded_planes(
        &a,
        window(-1.0, 1.0, -1.0, 1.0),
        &rotated,
        window(-1.0, 1.0, -1.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_region_lifts(&rotated_hit, &a, &rotated);
    assert_eq!(rotated_hit.regions[0].boundary.len(), 8);
    assert_eq!(
        rotated_hit.regions[0].orientation,
        SurfaceRegionOrientation::Same
    );

    let antiparallel = Plane::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    );
    let antiparallel_hit = intersect_bounded_planes(
        &a,
        window(-1.0, 1.0, -1.0, 1.0),
        &antiparallel,
        window(-1.0, 1.0, -1.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_region_lifts(&antiparallel_hit, &a, &antiparallel);
    assert_eq!(
        antiparallel_hit.regions[0].orientation,
        SurfaceRegionOrientation::Reversed
    );
}

#[test]
fn area_edge_point_and_disjoint_boundaries_are_dimensionally_truthful() {
    let plane = world_plane();
    let base = window(0.0, 1.0, 0.0, 1.0);

    let edge = intersect_bounded_planes(
        &plane,
        base,
        &plane,
        window(1.0, 2.0, 0.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert!(edge.is_complete());
    assert!(edge.points.is_empty());
    assert!(edge.regions.is_empty());
    assert_eq!(edge.curves.len(), 1);
    assert_eq!(edge.curves[0].kind, ContactKind::Tangent);
    assert_eq!(edge.curves[0].uv_a_start, [1.0, 0.0]);
    assert_eq!(edge.curves[0].uv_a_end, [1.0, 1.0]);
    assert!(matches!(
        edge.curves[0].curve,
        SurfaceIntersectionCurve::Line(_)
    ));

    let point = intersect_bounded_planes(
        &plane,
        base,
        &plane,
        window(1.0, 2.0, 1.0, 2.0),
        Tolerances::default(),
    )
    .unwrap();
    assert!(point.is_complete());
    assert!(point.curves.is_empty());
    assert!(point.regions.is_empty());
    assert_eq!(point.points.len(), 1);
    assert_eq!(point.points[0].kind, ContactKind::Tangent);
    assert_eq!(point.points[0].uv_a, [1.0, 1.0]);
    assert_eq!(point.points[0].uv_b, [1.0, 1.0]);

    let miss = intersect_bounded_planes(
        &plane,
        base,
        &plane,
        window(2.0, 3.0, 2.0, 3.0),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_proven_empty());
}

#[test]
fn degenerate_input_rectangles_collapse_to_curve_and_point() {
    let plane = world_plane();
    let curve = intersect_bounded_planes(
        &plane,
        window(0.5, 0.5, 0.0, 1.0),
        &plane,
        window(0.0, 1.0, 0.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(curve.curves.len(), 1);
    assert_eq!(curve.curves[0].kind, ContactKind::Tangent);

    let point = intersect_bounded_planes(
        &plane,
        window(0.5, 0.5, 0.25, 0.25),
        &plane,
        window(0.0, 1.0, 0.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(point.points.len(), 1);
    assert_eq!(point.points[0].uv_a, [0.5, 0.25]);
}

#[test]
fn swapped_regions_restore_first_chart_canonical_winding() {
    let a = world_plane();
    let b = Plane::new(
        Frame::new(
            Point3::new(0.5, 0.0, 0.0),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    );
    let a_range = window(-1.0, 1.0, -1.0, 1.0);
    let b_range = window(-0.5, 1.0, -0.75, 0.75);
    let hit = intersect_bounded_planes(&a, a_range, &b, b_range, Tolerances::default()).unwrap();
    let swapped = hit.clone().swapped();
    let direct_swapped =
        intersect_bounded_planes(&b, b_range, &a, a_range, Tolerances::default()).unwrap();
    assert_region_lifts(&swapped, &b, &a);
    assert_region_lifts(&direct_swapped, &b, &a);
    assert_eq!(swapped, direct_swapped);
    assert_eq!(
        swapped.regions[0].orientation,
        SurfaceRegionOrientation::Reversed
    );
}

#[test]
fn region_validation_rejects_bad_residual_orientation_and_degeneracy() {
    let vertices = vec![
        vertex([0.0, 0.0], [0.0, 0.0], 0.0),
        vertex([1.0, 0.0], [1.0, 0.0], 0.0),
        vertex([1.0, 1.0], [1.0, 1.0], 0.0),
        vertex([0.0, 1.0], [0.0, 1.0], 0.0),
    ];
    let bad_residual = SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
        Vec::new(),
        Vec::new(),
        vec![SurfaceSurfaceRegion {
            boundary: {
                let mut boundary = vertices.clone();
                boundary[0].residual = 1.0;
                boundary
            },
            orientation: SurfaceRegionOrientation::Same,
            correspondence: kops::intersect::SurfaceRegionCorrespondence::Polygonal,
            max_residual: 0.5,
        }],
    )
    .unwrap_err();
    assert!(matches!(bad_residual, Error::InvalidGeometry { .. }));

    let bad_orientation = SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
        Vec::new(),
        Vec::new(),
        vec![SurfaceSurfaceRegion {
            boundary: vertices,
            orientation: SurfaceRegionOrientation::Reversed,
            correspondence: kops::intersect::SurfaceRegionCorrespondence::Polygonal,
            max_residual: 0.0,
        }],
    )
    .unwrap_err();
    assert_eq!(
        bad_orientation,
        Error::InvalidGeometry {
            reason: "surface/surface region orientation disagrees with paired chart winding"
        }
    );

    let degenerate = SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
        Vec::new(),
        Vec::new(),
        vec![SurfaceSurfaceRegion {
            boundary: vec![
                vertex([0.0, 0.0], [0.0, 0.0], 0.0),
                vertex([1.0, 0.0], [1.0, 0.0], 0.0),
                vertex([2.0, 0.0], [2.0, 0.0], 0.0),
            ],
            orientation: SurfaceRegionOrientation::Same,
            correspondence: kops::intersect::SurfaceRegionCorrespondence::Polygonal,
            max_residual: 0.0,
        }],
    )
    .unwrap_err();
    assert!(matches!(degenerate, Error::InvalidGeometry { .. }));
}

#[test]
fn region_convexity_uses_exact_turns_and_canonicalizes_deterministically() {
    const M: i64 = (1_i64 << 52) - 1;
    let coordinates = vec![[0, 0], [M, M - 1], [2 * M + 1, 2 * M - 1], [0, 2 * M + 1]];

    assert!((0..coordinates.len()).all(|index| {
        exact_integer_turn(
            coordinates[index],
            coordinates[(index + 1) % coordinates.len()],
            coordinates[(index + 2) % coordinates.len()],
        ) > 0
    }));
    let [a, b, c] = [coordinates[0], coordinates[1], coordinates[2]].map(integer_uv);
    let rounded_turn = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
    assert_eq!(
        exact_integer_turn(coordinates[0], coordinates[1], coordinates[2]),
        1
    );
    assert_eq!(rounded_turn, 0.0);

    let canonicalize = |coordinates: &[[i64; 2]]| {
        SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
            Vec::new(),
            Vec::new(),
            vec![integer_polygonal_region(coordinates)],
        )
    };
    let expected = canonicalize(&coordinates).unwrap();
    assert!(expected.is_complete());
    assert_eq!(expected.regions.len(), 1);
    assert_eq!(canonicalize(&coordinates).unwrap(), expected);

    let mut rotated = coordinates.clone();
    rotated.rotate_left(2);
    assert_eq!(canonicalize(&rotated).unwrap(), expected);

    let mut reversed = coordinates.clone();
    reversed.reverse();
    assert_eq!(canonicalize(&reversed).unwrap(), expected);

    let mut collinear = coordinates.clone();
    collinear[2] = [2 * M, 2 * M - 2];
    assert_eq!(
        exact_integer_turn(collinear[0], collinear[1], collinear[2]),
        0
    );
    assert!(matches!(
        canonicalize(&collinear),
        Err(Error::InvalidGeometry { .. })
    ));

    assert!(matches!(
        canonicalize(&coordinates[..2]),
        Err(Error::InvalidGeometry { .. })
    ));
    let mut non_finite = integer_polygonal_region(&coordinates);
    non_finite.boundary[1].uv_a[0] = f64::NAN;
    assert!(matches!(
        SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
            Vec::new(),
            Vec::new(),
            vec![non_finite],
        ),
        Err(Error::InvalidGeometry { .. })
    ));
}

#[test]
fn region_winding_uses_exact_polygon_orientation_in_both_charts() {
    const M: i64 = (1_i64 << 52) - 1;
    let coordinates = vec![[0, 0], [M, M - 1], [2 * M + 1, 2 * M - 1], [M + 1, M]];

    assert_eq!(exact_integer_polygon_twice_area(&coordinates), 2);
    assert_eq!(rounded_origin_relative_twice_area(&coordinates), 0.0);
    assert!((0..coordinates.len()).all(|index| {
        exact_integer_turn(
            coordinates[index],
            coordinates[(index + 1) % coordinates.len()],
            coordinates[(index + 2) % coordinates.len()],
        ) == 1
    }));

    let canonicalize = |coordinates: &[[i64; 2]], orientation| {
        SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
            Vec::new(),
            Vec::new(),
            vec![integer_polygonal_region_with_orientation(
                coordinates,
                orientation,
            )],
        )
    };
    for orientation in [
        SurfaceRegionOrientation::Same,
        SurfaceRegionOrientation::Reversed,
    ] {
        let expected = canonicalize(&coordinates, orientation).unwrap();
        assert!(expected.is_complete());
        assert_eq!(expected.regions.len(), 1);
        assert_eq!(expected.regions[0].orientation, orientation);
        assert_eq!(canonicalize(&coordinates, orientation).unwrap(), expected);

        let mut rotated = coordinates.clone();
        rotated.rotate_left(2);
        assert_eq!(canonicalize(&rotated, orientation).unwrap(), expected);

        let mut reversed = coordinates.clone();
        reversed.reverse();
        assert_eq!(canonicalize(&reversed, orientation).unwrap(), expected);
    }

    let exact_zero = [[0, 0], [M, M - 1], [2 * M, 2 * M - 2]];
    assert_eq!(exact_integer_polygon_twice_area(&exact_zero), 0);
    assert_eq!(
        canonicalize(&exact_zero, SurfaceRegionOrientation::Same).unwrap_err(),
        Error::InvalidGeometry {
            reason: "surface/surface region boundaries must have positive area in both charts"
        }
    );
}

#[test]
fn region_residual_bound_covers_whole_affine_patch_and_bits_repeat() {
    let a = world_plane();
    let b = Plane::new(
        Frame::new(
            Point3::new(0.0, 0.0, 5.0e-9),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    );
    let a_range = window(-1.0, 2.0, -2.0, 1.0);
    let b_range = window(-0.5, 1.5, -1.5, 0.5);
    let first = intersect_bounded_planes(&a, a_range, &b, b_range, Tolerances::default()).unwrap();
    let second = intersect_bounded_planes(&a, a_range, &b, b_range, Tolerances::default()).unwrap();
    assert_eq!(region_bits(&first), region_bits(&second));

    let region = &first.regions[0];
    for index in 0..region.boundary.len() {
        let current = region.boundary[index];
        let next = region.boundary[(index + 1) % region.boundary.len()];
        for weight in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let uv_a = lerp2(current.uv_a, next.uv_a, weight);
            let uv_b = lerp2(current.uv_b, next.uv_b, weight);
            assert!(a.eval(uv_a).dist(b.eval(uv_b)) <= region.max_residual);
        }
    }
    let uv_a = average_uv(region.boundary.iter().map(|vertex| vertex.uv_a));
    let uv_b = average_uv(region.boundary.iter().map(|vertex| vertex.uv_b));
    assert!(a.eval(uv_a).dist(b.eval(uv_b)) <= region.max_residual);
}

fn vertex(uv_a: [f64; 2], uv_b: [f64; 2], residual: f64) -> SurfaceSurfaceRegionVertex {
    SurfaceSurfaceRegionVertex {
        point: Point3::new(uv_a[0], uv_a[1], 0.0),
        uv_a,
        uv_b,
        residual,
    }
}

fn integer_polygonal_region(coordinates: &[[i64; 2]]) -> SurfaceSurfaceRegion {
    SurfaceSurfaceRegion {
        boundary: coordinates
            .iter()
            .copied()
            .map(integer_uv)
            .map(|uv| vertex(uv, uv, 0.0))
            .collect(),
        orientation: SurfaceRegionOrientation::Same,
        correspondence: kops::intersect::SurfaceRegionCorrespondence::Polygonal,
        max_residual: 0.0,
    }
}

fn integer_polygonal_region_with_orientation(
    coordinates: &[[i64; 2]],
    orientation: SurfaceRegionOrientation,
) -> SurfaceSurfaceRegion {
    SurfaceSurfaceRegion {
        boundary: coordinates
            .iter()
            .copied()
            .map(|coordinate| {
                let uv_a = integer_uv(coordinate);
                let uv_b = match orientation {
                    SurfaceRegionOrientation::Same => uv_a,
                    SurfaceRegionOrientation::Reversed => [uv_a[0], -uv_a[1]],
                };
                vertex(uv_a, uv_b, 0.0)
            })
            .collect(),
        orientation,
        correspondence: kops::intersect::SurfaceRegionCorrespondence::Polygonal,
        max_residual: 0.0,
    }
}

fn integer_uv([u, v]: [i64; 2]) -> [f64; 2] {
    [u as f64, v as f64]
}

fn exact_integer_turn(a: [i64; 2], b: [i64; 2], c: [i64; 2]) -> i128 {
    let ab = [
        i128::from(b[0]) - i128::from(a[0]),
        i128::from(b[1]) - i128::from(a[1]),
    ];
    let bc = [
        i128::from(c[0]) - i128::from(b[0]),
        i128::from(c[1]) - i128::from(b[1]),
    ];
    ab[0] * bc[1] - ab[1] * bc[0]
}

fn exact_integer_polygon_twice_area(coordinates: &[[i64; 2]]) -> i128 {
    coordinates
        .iter()
        .zip(coordinates.iter().cycle().skip(1))
        .map(|(point, next)| {
            i128::from(point[0]) * i128::from(next[1]) - i128::from(point[1]) * i128::from(next[0])
        })
        .sum()
}

fn rounded_origin_relative_twice_area(coordinates: &[[i64; 2]]) -> f64 {
    let origin = integer_uv(coordinates[0]);
    coordinates
        .iter()
        .zip(coordinates.iter().cycle().skip(1))
        .map(|(point, next)| {
            let point = integer_uv(*point);
            let next = integer_uv(*next);
            let point = [point[0] - origin[0], point[1] - origin[1]];
            let next = [next[0] - origin[0], next[1] - origin[1]];
            point[0] * next[1] - point[1] * next[0]
        })
        .sum()
}

fn lerp2(a: [f64; 2], b: [f64; 2], weight: f64) -> [f64; 2] {
    [
        a[0] * (1.0 - weight) + b[0] * weight,
        a[1] * (1.0 - weight) + b[1] * weight,
    ]
}

fn average_uv(parameters: impl Iterator<Item = [f64; 2]>) -> [f64; 2] {
    let parameters = parameters.collect::<Vec<_>>();
    let sum = parameters
        .iter()
        .fold([0.0, 0.0], |sum, uv| [sum[0] + uv[0], sum[1] + uv[1]]);
    [
        sum[0] / parameters.len() as f64,
        sum[1] / parameters.len() as f64,
    ]
}

fn region_bits(hit: &SurfaceSurfaceIntersections) -> Vec<u64> {
    hit.regions
        .iter()
        .flat_map(|region| {
            core::iter::once(region.max_residual.to_bits()).chain(region.boundary.iter().flat_map(
                |vertex| {
                    [
                        vertex.point.x.to_bits(),
                        vertex.point.y.to_bits(),
                        vertex.point.z.to_bits(),
                        vertex.uv_a[0].to_bits(),
                        vertex.uv_a[1].to_bits(),
                        vertex.uv_b[0].to_bits(),
                        vertex.uv_b[1].to_bits(),
                        vertex.residual.to_bits(),
                    ]
                },
            ))
        })
        .collect()
}
