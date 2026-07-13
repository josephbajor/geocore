//! Coincident bounded cylinder/cylinder region conformance.

use kcore::error::Error;
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Surface};
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    ContactKind, SurfaceIntersectionCurve, SurfaceRegionOrientation, SurfaceSurfaceIntersections,
    intersect_bounded_cylinders,
};

fn range(lo: f64, hi: f64) -> ParamRange {
    ParamRange::new(lo, hi)
}

fn window(u_lo: f64, u_hi: f64, v_lo: f64, v_hi: f64) -> [ParamRange; 2] {
    [range(u_lo, u_hi), range(v_lo, v_hi)]
}

fn world_cylinder() -> Cylinder {
    Cylinder::new(Frame::world(), 1.0).unwrap()
}

fn assert_regions_lift(hit: &SurfaceSurfaceIntersections, a: &Cylinder, b: &Cylinder) {
    assert!(hit.is_complete());
    assert!(hit.points.is_empty());
    assert!(hit.curves.is_empty());
    assert!(!hit.regions.is_empty());
    for region in &hit.regions {
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

#[test]
fn aligned_partial_overlap_and_containment_are_complete_regions() {
    let cylinder = world_cylinder();
    let partial = intersect_bounded_cylinders(
        &cylinder,
        window(0.0, 2.0, 0.0, 2.0),
        &cylinder,
        window(1.0, 3.0, -1.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&partial, &cylinder, &cylinder);
    assert_eq!(partial.regions.len(), 1);
    assert_eq!(partial.regions[0].boundary[0].uv_a, [1.0, 0.0]);
    assert_eq!(partial.regions[0].boundary[2].uv_a, [2.0, 1.0]);

    let contained = intersect_bounded_cylinders(
        &cylinder,
        window(-1.0, 2.0, -2.0, 2.0),
        &cylinder,
        window(-0.5, 0.5, -1.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&contained, &cylinder, &cylinder);
    assert_eq!(contained.regions[0].boundary[0].uv_a, [-0.5, -1.0]);
}

#[test]
fn translated_rotated_and_antiparallel_charts_preserve_paired_regions_and_swap() {
    let a = world_cylinder();
    let angle = 0.4_f64;
    let rotated = Cylinder::new(
        Frame::new(
            Point3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(math::cos(angle), math::sin(angle), 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let a_range = window(0.4, 1.4, 1.0, 3.0);
    let b_range = window(0.6 - angle, 1.2 - angle, 0.5, 1.5);
    let hit =
        intersect_bounded_cylinders(&a, a_range, &rotated, b_range, Tolerances::default()).unwrap();
    assert_regions_lift(&hit, &a, &rotated);
    for vertex in &hit.regions[0].boundary {
        assert!((vertex.uv_b[0] - (vertex.uv_a[0] - angle)).abs() < 1.0e-15);
        assert!((vertex.uv_b[1] - (vertex.uv_a[1] - 1.0)).abs() < 1.0e-15);
    }
    let direct_swapped =
        intersect_bounded_cylinders(&rotated, b_range, &a, a_range, Tolerances::default()).unwrap();
    assert_eq!(hit.clone().swapped(), direct_swapped);

    let antiparallel = Cylinder::new(
        Frame::new(
            Point3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let anti = intersect_bounded_cylinders(
        &a,
        window(0.2, 1.0, 0.0, 2.0),
        &antiparallel,
        window(-0.8, -0.4, -0.5, 0.5),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&anti, &a, &antiparallel);
    for vertex in &anti.regions[0].boundary {
        assert_eq!(vertex.uv_b[0], -vertex.uv_a[0]);
        assert_eq!(vertex.uv_b[1], 1.0 - vertex.uv_a[1]);
    }
}

#[test]
fn full_turn_overlap_splits_deterministically_at_the_other_chart_seam() {
    let cylinder = world_cylinder();
    let tau = core::f64::consts::TAU;
    let first = intersect_bounded_cylinders(
        &cylinder,
        window(0.0, tau, 0.0, 1.0),
        &cylinder,
        window(1.0, 1.0 + tau, 0.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    let second = intersect_bounded_cylinders(
        &cylinder,
        window(0.0, tau, 0.0, 1.0),
        &cylinder,
        window(1.0, 1.0 + tau, 0.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(first, second);
    assert_regions_lift(&first, &cylinder, &cylinder);
    assert_eq!(first.regions.len(), 2);
    assert_eq!(first.regions[0].boundary[0].uv_a[0], 0.0);
    assert_eq!(first.regions[0].boundary[1].uv_a[0], 1.0);
    assert_eq!(first.regions[1].boundary[0].uv_a[0], 1.0);
    assert_eq!(first.regions[1].boundary[1].uv_a[0], tau);
}

#[test]
fn collapsed_longitude_and_axial_overlaps_emit_curves_points_and_empty_evidence() {
    let cylinder = world_cylinder();
    let circle = intersect_bounded_cylinders(
        &cylinder,
        window(0.0, 1.0, 0.5, 0.5),
        &cylinder,
        window(0.2, 0.8, 0.5, 0.5),
        Tolerances::default(),
    )
    .unwrap();
    assert!(circle.regions.is_empty());
    assert!(circle.points.is_empty());
    assert_eq!(circle.curves.len(), 1);
    assert_eq!(circle.curves[0].kind, ContactKind::Tangent);
    assert!(matches!(
        circle.curves[0].curve,
        SurfaceIntersectionCurve::Circle(_)
    ));
    assert_eq!(circle.curves[0].curve_range, range(0.2, 0.8));

    let ruling = intersect_bounded_cylinders(
        &cylinder,
        window(0.5, 0.5, 0.0, 1.0),
        &cylinder,
        window(0.5, 0.5, 0.25, 0.75),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(ruling.curves.len(), 1);
    assert!(matches!(
        ruling.curves[0].curve,
        SurfaceIntersectionCurve::Line(_)
    ));
    assert_eq!(ruling.curves[0].curve_range, range(0.25, 0.75));

    let point = intersect_bounded_cylinders(
        &cylinder,
        window(0.5, 0.5, 0.25, 0.25),
        &cylinder,
        window(0.5, 0.5, 0.25, 0.25),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(point.points.len(), 1);
    assert_eq!(point.points[0].kind, ContactKind::Tangent);

    let miss = intersect_bounded_cylinders(
        &cylinder,
        window(0.0, 0.5, 0.0, 1.0),
        &cylinder,
        window(1.0, 1.5, 0.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_proven_empty());
}

#[test]
fn overwide_coincident_longitude_windows_are_rejected_without_affecting_other_cases() {
    let cylinder = world_cylinder();
    let tau = core::f64::consts::TAU;
    let error = intersect_bounded_cylinders(
        &cylinder,
        window(0.0, tau + 0.1, 0.0, 1.0),
        &cylinder,
        window(0.0, tau, 0.0, 1.0),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        error,
        Error::InvalidGeometry {
            reason: "coincident cylinder longitude windows cannot span more than one turn"
        }
    );
}

#[test]
fn whole_region_bound_scales_with_large_axial_and_periodic_parameters() {
    let cylinder = world_cylinder();
    let tau = core::f64::consts::TAU;
    let u = 1.0e8 * tau;
    let v = 1.0e12;
    let hit = intersect_bounded_cylinders(
        &cylinder,
        window(u, u + 0.5, v, v + 2.0),
        &cylinder,
        window(u + 0.1, u + 0.4, v + 0.5, v + 1.5),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&hit, &cylinder, &cylinder);
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
