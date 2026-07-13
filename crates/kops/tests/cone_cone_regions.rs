//! Exact coincident bounded cone/cone region conformance.

use kcore::error::Error;
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cone, Surface};
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    ContactKind, SurfaceIntersectionCurve, SurfaceRegionCorrespondence, SurfaceRegionOrientation,
    SurfaceSurfaceIntersections, SurfaceSurfaceRegion, intersect_bounded_cones,
};

fn range(lo: f64, hi: f64) -> ParamRange {
    ParamRange::new(lo, hi)
}

fn window(u_lo: f64, u_hi: f64, v_lo: f64, v_hi: f64) -> [ParamRange; 2] {
    [range(u_lo, u_hi), range(v_lo, v_hi)]
}

fn world_cone() -> Cone {
    Cone::new(Frame::world(), 1.0, core::f64::consts::FRAC_PI_6).unwrap()
}

fn assert_regions_lift(hit: &SurfaceSurfaceIntersections, a: &Cone, b: &Cone) {
    assert!(hit.is_complete());
    assert!(hit.points.is_empty());
    assert!(hit.curves.is_empty());
    assert!(!hit.regions.is_empty());
    for region in &hit.regions {
        assert_eq!(region.boundary.len(), 4);
        assert_eq!(region.orientation, SurfaceRegionOrientation::Same);
        assert_eq!(
            region.correspondence,
            SurfaceRegionCorrespondence::Polygonal
        );
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

#[test]
fn aligned_partial_containment_and_seam_split_are_complete_regions() {
    let cone = world_cone();
    let partial = intersect_bounded_cones(
        &cone,
        window(0.0, 2.0, 0.0, 2.0),
        &cone,
        window(1.0, 3.0, 1.0, 3.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&partial, &cone, &cone);
    assert_eq!(partial.regions.len(), 1);
    assert_eq!(partial.regions[0].boundary[0].uv_a, [1.0, 1.0]);
    assert_eq!(partial.regions[0].boundary[2].uv_a, [2.0, 2.0]);

    let contained = intersect_bounded_cones(
        &cone,
        window(-1.0, 2.0, -1.0, 2.0),
        &cone,
        window(-0.5, 0.5, 0.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&contained, &cone, &cone);

    let tau = core::f64::consts::TAU;
    let seam = intersect_bounded_cones(
        &cone,
        window(0.0, tau, 0.0, 1.0),
        &cone,
        window(1.0, 1.0 + tau, 0.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&seam, &cone, &cone);
    assert_eq!(seam.regions.len(), 2);
    assert_eq!(seam.regions[0].boundary[0].uv_a[0], 0.0);
    assert_eq!(seam.regions[1].boundary[1].uv_a[0], tau);
}

#[test]
fn rotated_shifted_and_antiparallel_exact_charts_preserve_regions_and_swap() {
    let a = world_cone();
    let angle = 0.4_f64;
    let rotated = Cone::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(math::cos(angle), math::sin(angle), 0.0),
        )
        .unwrap(),
        1.0,
        a.half_angle(),
    )
    .unwrap();
    let a_range = window(0.4, 1.4, 0.0, 1.0);
    let rotated_range = window(0.6 - angle, 1.2 - angle, 0.25, 0.75);
    let hit = intersect_bounded_cones(&a, a_range, &rotated, rotated_range, Tolerances::default())
        .unwrap();
    assert_regions_lift(&hit, &a, &rotated);
    let direct_swapped =
        intersect_bounded_cones(&rotated, rotated_range, &a, a_range, Tolerances::default())
            .unwrap();
    assert_eq!(hit.clone().swapped(), direct_swapped);

    let (sin_angle, cos_angle) = math::sincos(a.half_angle());
    assert_eq!(sin_angle, 0.5);
    let shifted = Cone::new(
        Frame::world().with_origin(Point3::new(0.0, 0.0, 2.0 * cos_angle)),
        2.0,
        a.half_angle(),
    )
    .unwrap();
    assert_eq!(a.apex(), shifted.apex());
    let shifted_hit = intersect_bounded_cones(
        &a,
        window(0.2, 1.0, 0.0, 1.0),
        &shifted,
        window(0.2, 1.0, -2.0, -1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&shifted_hit, &a, &shifted);
    for vertex in &shifted_hit.regions[0].boundary {
        assert_eq!(vertex.uv_b[1], vertex.uv_a[1] - 2.0);
    }

    let antiparallel = Cone::new(
        Frame::new(
            Point3::new(0.0, 0.0, -4.0 * cos_angle),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
        a.half_angle(),
    )
    .unwrap();
    assert_eq!(a.apex(), antiparallel.apex());
    let anti_a_range = window(0.2, 1.0, 0.0, 1.0);
    let anti_b_range = window(
        core::f64::consts::PI - 1.0,
        core::f64::consts::PI - 0.2,
        -5.0,
        -4.0,
    );
    let anti = intersect_bounded_cones(
        &a,
        anti_a_range,
        &antiparallel,
        anti_b_range,
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&anti, &a, &antiparallel);
    for vertex in &anti.regions[0].boundary {
        assert!((vertex.uv_b[0] - (core::f64::consts::PI - vertex.uv_a[0])).abs() < 1.0e-14);
        assert_eq!(vertex.uv_b[1], -vertex.uv_a[1] - 4.0);
    }
}

#[test]
fn apex_splits_regions_and_collapses_latitudes_to_one_singular_point() {
    let cone = world_cone();
    let apex = cone.apex_v();
    let split = intersect_bounded_cones(
        &cone,
        window(0.0, 1.0, apex - 1.0, apex + 1.0),
        &cone,
        window(0.25, 0.75, apex - 0.5, apex + 0.5),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&split, &cone, &cone);
    assert_eq!(split.regions.len(), 2);
    assert!(
        split
            .regions
            .iter()
            .all(|region| region.boundary.iter().any(|vertex| vertex.uv_a[1] == apex))
    );

    let apex_point = intersect_bounded_cones(
        &cone,
        window(0.0, 1.0, apex, apex),
        &cone,
        window(2.0, 3.0, apex, apex),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(apex_point.points.len(), 1);
    assert_eq!(apex_point.points[0].kind, ContactKind::Singular);
    assert!(apex_point.curves.is_empty());
    assert!(apex_point.regions.is_empty());

    let disjoint_u_apex = intersect_bounded_cones(
        &cone,
        window(0.0, 0.5, apex - 0.25, apex + 0.25),
        &cone,
        window(1.0, 1.5, apex - 0.25, apex + 0.25),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(disjoint_u_apex.points.len(), 1);
    assert_eq!(disjoint_u_apex.points[0].kind, ContactKind::Singular);

    let ruling_split = intersect_bounded_cones(
        &cone,
        window(0.5, 0.5, apex - 1.0, apex + 1.0),
        &cone,
        window(0.5, 0.5, apex - 0.5, apex + 0.5),
        Tolerances::default(),
    )
    .unwrap();
    assert!(ruling_split.points.is_empty());
    assert_eq!(ruling_split.curves.len(), 2);
    assert!(ruling_split.regions.is_empty());
    assert!(
        ruling_split
            .curves
            .iter()
            .all(|curve| matches!(curve.curve, SurfaceIntersectionCurve::Line(_)))
    );

    let spill = 0.5 * Tolerances::default().linear();
    let tolerance_boundary = intersect_bounded_cones(
        &cone,
        window(0.0, 0.5, apex, apex),
        &cone,
        window(1.0, 1.5, apex + spill, apex + 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(tolerance_boundary.points.len(), 1);
    let point = tolerance_boundary.points[0];
    assert!(window(0.0, 0.5, apex, apex)[1].contains(point.uv_a[1]));
    assert!(window(1.0, 1.5, apex + spill, apex + 1.0)[1].contains(point.uv_b[1]));
    assert_eq!(point.uv_b[1], apex + spill);
}

#[test]
fn collapsed_windows_emit_circle_ruling_point_and_complete_miss() {
    let cone = world_cone();
    let circle = intersect_bounded_cones(
        &cone,
        window(0.0, 1.0, 0.5, 0.5),
        &cone,
        window(0.2, 0.8, 0.5, 0.5),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(circle.curves.len(), 1);
    assert!(matches!(
        circle.curves[0].curve,
        SurfaceIntersectionCurve::Circle(_)
    ));

    let ruling = intersect_bounded_cones(
        &cone,
        window(0.5, 0.5, 0.0, 1.0),
        &cone,
        window(0.5, 0.5, 0.25, 0.75),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(ruling.curves.len(), 1);
    assert!(matches!(
        ruling.curves[0].curve,
        SurfaceIntersectionCurve::Line(_)
    ));

    let point = intersect_bounded_cones(
        &cone,
        window(0.5, 0.5, 0.25, 0.25),
        &cone,
        window(0.5, 0.5, 0.25, 0.25),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(point.points.len(), 1);
    assert_eq!(point.points[0].kind, ContactKind::Tangent);

    let miss = intersect_bounded_cones(
        &cone,
        window(0.0, 0.5, 0.0, 1.0),
        &cone,
        window(1.0, 1.5, 2.0, 3.0),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_proven_empty());
}

#[test]
fn noncanonical_and_roundoff_unsafe_coincident_domains_fail_closed() {
    let cone = world_cone();
    let tau = core::f64::consts::TAU;
    let overwide = intersect_bounded_cones(
        &cone,
        window(0.0, tau + 0.1, 0.0, 1.0),
        &cone,
        window(0.0, tau, 0.0, 1.0),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        overwide,
        Error::InvalidGeometry {
            reason: "coincident cone longitude windows cannot span more than one turn"
        }
    );

    let angle = 0.4_f64;
    let rotated = Cone::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(math::cos(angle), math::sin(angle), 0.0),
        )
        .unwrap(),
        cone.radius(),
        cone.half_angle(),
    )
    .unwrap();
    let remote = 1.0e8 * tau;
    let unsafe_map = intersect_bounded_cones(
        &cone,
        window(remote, remote + 1.0, 0.0, 1.0),
        &rotated,
        window(remote - angle, remote + 1.0 - angle, 0.0, 1.0),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        unsafe_map,
        Error::InvalidGeometry {
            reason: "coincident cone chart map exceeds the certified parameter-roundoff corridor"
        }
    );

    let identical_extreme = intersect_bounded_cones(
        &cone,
        window(remote, remote + 1.0, 1.0e12, 1.0e12 + 1.0),
        &cone,
        window(remote, remote + 1.0, 1.0e12, 1.0e12 + 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&identical_extreme, &cone, &cone);

    let overflowing_first = intersect_bounded_cones(
        &cone,
        window(0.0, 1.0, -f64::MAX, f64::MAX),
        &cone,
        window(0.0, 1.0, 0.0, 1.0),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        overflowing_first,
        Error::InvalidGeometry {
            reason: "cone/cone intersection requires finite non-reversed first-cone ranges"
        }
    );

    let overflowing_second = intersect_bounded_cones(
        &cone,
        window(0.0, 1.0, 0.0, 1.0),
        &cone,
        window(0.0, 1.0, -f64::MAX, f64::MAX),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        overflowing_second,
        Error::InvalidGeometry {
            reason: "cone/cone intersection requires finite non-reversed second-cone ranges"
        }
    );
}

#[test]
fn public_region_validation_rejects_mutated_cone_evidence() {
    let cone = world_cone();
    let hit = intersect_bounded_cones(
        &cone,
        window(0.0, 1.0, 0.0, 1.0),
        &cone,
        window(0.0, 1.0, 0.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    let mut invalid: SurfaceSurfaceRegion = hit.regions[0].clone();
    invalid.max_residual = -1.0;
    let error = SurfaceSurfaceIntersections::canonicalized_complete_with_regions(
        Vec::new(),
        Vec::new(),
        vec![invalid],
    )
    .unwrap_err();
    assert_eq!(
        error,
        Error::InvalidGeometry {
            reason: "surface/surface region data must be finite, nonnegative, and have at least three bounded vertices"
        }
    );
}

fn bilinear_uv(
    region: &SurfaceSurfaceRegion,
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
