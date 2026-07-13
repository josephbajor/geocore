//! Coincident bounded torus/torus region conformance.

use std::collections::BTreeSet;

use kcore::error::Error;
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Surface, Torus};
use kgeom::vec::{Point3, Vec3};
use kops::intersect::{
    ContactKind, SurfaceIntersectionCurve, SurfaceRegionCorrespondence, SurfaceRegionOrientation,
    SurfaceSurfaceIntersections, intersect_bounded_tori,
};

fn range(lo: f64, hi: f64) -> ParamRange {
    ParamRange::new(lo, hi)
}

fn window(u_lo: f64, u_hi: f64, v_lo: f64, v_hi: f64) -> [ParamRange; 2] {
    [range(u_lo, u_hi), range(v_lo, v_hi)]
}

fn world_torus() -> Torus {
    Torus::new(Frame::world(), 2.0, 0.5).unwrap()
}

fn assert_regions_lift(hit: &SurfaceSurfaceIntersections, a: &Torus, b: &Torus) {
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
                assert!(
                    a.eval(uv_a).dist(b.eval(uv_b)) <= region.max_residual,
                    "whole-region bound missed at {uv_a:?} -> {uv_b:?}"
                );
            }
        }
    }
}

fn assert_curves_lift(hit: &SurfaceSurfaceIntersections, a: &Torus, b: &Torus) {
    assert!(hit.is_complete());
    assert!(hit.points.is_empty());
    assert!(hit.regions.is_empty());
    assert!(!hit.curves.is_empty());
    for branch in &hit.curves {
        assert_eq!(branch.kind, ContactKind::Tangent);
        assert!(matches!(branch.curve, SurfaceIntersectionCurve::Circle(_)));
        let start = branch.curve.eval(branch.curve_range.lo);
        let end = branch.curve.eval(branch.curve_range.hi);
        assert!(a.eval(branch.uv_a_start).dist(start) < 1.0e-10);
        assert!(a.eval(branch.uv_a_end).dist(end) < 1.0e-10);
        assert!(b.eval(branch.uv_b_start).dist(start) < 1.0e-10);
        assert!(b.eval(branch.uv_b_end).dist(end) < 1.0e-10);
    }
}

fn assert_uv_evidence_stays_in_windows(
    hit: &SurfaceSurfaceIntersections,
    a_range: [ParamRange; 2],
    b_range: [ParamRange; 2],
) {
    let assert_uv = |uv: [f64; 2], ranges: [ParamRange; 2]| {
        assert!(ranges[0].contains(uv[0]), "u={uv:?} escaped {ranges:?}");
        assert!(ranges[1].contains(uv[1]), "v={uv:?} escaped {ranges:?}");
    };
    for point in &hit.points {
        assert_uv(point.uv_a, a_range);
        assert_uv(point.uv_b, b_range);
    }
    for curve in &hit.curves {
        assert_uv(curve.uv_a_start, a_range);
        assert_uv(curve.uv_a_end, a_range);
        assert_uv(curve.uv_b_start, b_range);
        assert_uv(curve.uv_b_end, b_range);
    }
    for region in &hit.regions {
        for vertex in &region.boundary {
            assert_uv(vertex.uv_a, a_range);
            assert_uv(vertex.uv_b, b_range);
        }
    }
}

#[test]
fn aligned_partial_overlap_and_containment_are_complete_regions() {
    let torus = world_torus();
    let partial = intersect_bounded_tori(
        &torus,
        window(0.0, 2.0, 0.0, 2.0),
        &torus,
        window(1.0, 3.0, 1.0, 3.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&partial, &torus, &torus);
    assert_eq!(partial.regions.len(), 1);
    assert_eq!(partial.regions[0].boundary[0].uv_a, [1.0, 1.0]);
    assert_eq!(partial.regions[0].boundary[2].uv_a, [2.0, 2.0]);

    let contained = intersect_bounded_tori(
        &torus,
        window(-1.0, 2.0, -2.0, 2.0),
        &torus,
        window(-0.5, 0.5, -1.0, 1.0),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&contained, &torus, &torus);
    assert_eq!(contained.regions[0].boundary[0].uv_a, [-0.5, -1.0]);
}

#[test]
fn rotated_and_antiparallel_exact_frames_preserve_regions_and_swap() {
    let a = world_torus();
    let angle = 0.4_f64;
    let rotated = Torus::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(math::cos(angle), math::sin(angle), 0.0),
        )
        .unwrap(),
        2.0,
        0.5,
    )
    .unwrap();
    let a_range = window(0.4, 1.4, -0.5, 0.75);
    let b_range = window(0.6 - angle, 1.2 - angle, -0.25, 0.5);
    let hit =
        intersect_bounded_tori(&a, a_range, &rotated, b_range, Tolerances::default()).unwrap();
    assert_regions_lift(&hit, &a, &rotated);
    assert_uv_evidence_stays_in_windows(&hit, a_range, b_range);
    for vertex in &hit.regions[0].boundary {
        assert!((vertex.uv_b[0] - (vertex.uv_a[0] - angle)).abs() < 1.0e-14);
        assert_eq!(vertex.uv_b[1], vertex.uv_a[1]);
    }
    let direct_swapped =
        intersect_bounded_tori(&rotated, b_range, &a, a_range, Tolerances::default()).unwrap();
    assert_eq!(hit.clone().swapped(), direct_swapped);

    let antiparallel = Torus::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        2.0,
        0.5,
    )
    .unwrap();
    let anti = intersect_bounded_tori(
        &a,
        window(0.2, 1.0, 0.1, 1.0),
        &antiparallel,
        window(-0.8, -0.4, -0.75, -0.25),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&anti, &a, &antiparallel);
    assert_uv_evidence_stays_in_windows(
        &anti,
        window(0.2, 1.0, 0.1, 1.0),
        window(-0.8, -0.4, -0.75, -0.25),
    );
    for vertex in &anti.regions[0].boundary {
        assert_eq!(vertex.uv_b[0], -vertex.uv_a[0]);
        assert_eq!(vertex.uv_b[1], -vertex.uv_a[1]);
    }
}

#[test]
fn both_periodic_seams_split_into_four_unique_paired_regions() {
    let torus = world_torus();
    let tau = core::f64::consts::TAU;
    let first = intersect_bounded_tori(
        &torus,
        window(0.0, tau, 0.0, tau),
        &torus,
        window(1.0, 1.0 + tau, 2.0, 2.0 + tau),
        Tolerances::default(),
    )
    .unwrap();
    let second = intersect_bounded_tori(
        &torus,
        window(0.0, tau, 0.0, tau),
        &torus,
        window(1.0, 1.0 + tau, 2.0, 2.0 + tau),
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(first, second);
    assert_regions_lift(&first, &torus, &torus);
    assert_uv_evidence_stays_in_windows(
        &first,
        window(0.0, tau, 0.0, tau),
        window(1.0, 1.0 + tau, 2.0, 2.0 + tau),
    );
    assert_eq!(first.regions.len(), 4);

    let rectangles = first
        .regions
        .iter()
        .map(|region| {
            let lo = region.boundary[0].uv_a;
            let hi = region.boundary[2].uv_a;
            (
                lo[0].to_bits(),
                hi[0].to_bits(),
                lo[1].to_bits(),
                hi[1].to_bits(),
            )
        })
        .collect::<BTreeSet<_>>();
    assert_eq!(rectangles.len(), 4, "no seam patch may be duplicated");
    assert!(rectangles.contains(&(
        0.0_f64.to_bits(),
        1.0_f64.to_bits(),
        0.0_f64.to_bits(),
        2.0_f64.to_bits()
    )));
    assert!(rectangles.contains(&(
        1.0_f64.to_bits(),
        tau.to_bits(),
        2.0_f64.to_bits(),
        tau.to_bits()
    )));
}

#[test]
fn collapsed_axes_emit_exact_latitude_meridian_point_and_empty_results() {
    let torus = world_torus();
    let latitude = intersect_bounded_tori(
        &torus,
        window(0.0, 1.0, 0.25, 0.25),
        &torus,
        window(0.2, 0.8, 0.25, 0.25),
        Tolerances::default(),
    )
    .unwrap();
    assert_curves_lift(&latitude, &torus, &torus);
    assert_uv_evidence_stays_in_windows(
        &latitude,
        window(0.0, 1.0, 0.25, 0.25),
        window(0.2, 0.8, 0.25, 0.25),
    );
    assert_eq!(latitude.curves.len(), 1);
    assert_eq!(latitude.curves[0].curve_range, range(0.2, 0.8));

    let meridian = intersect_bounded_tori(
        &torus,
        window(0.5, 0.5, 0.0, 1.0),
        &torus,
        window(0.5, 0.5, 0.25, 0.75),
        Tolerances::default(),
    )
    .unwrap();
    assert_curves_lift(&meridian, &torus, &torus);
    assert_uv_evidence_stays_in_windows(
        &meridian,
        window(0.5, 0.5, 0.0, 1.0),
        window(0.5, 0.5, 0.25, 0.75),
    );
    assert_eq!(meridian.curves.len(), 1);
    assert_eq!(meridian.curves[0].curve_range, range(0.25, 0.75));
    let SurfaceIntersectionCurve::Circle(circle) = meridian.curves[0].curve else {
        unreachable!()
    };
    let tangent = torus.frame().y() * math::cos(0.5) - torus.frame().x() * math::sin(0.5);
    assert!((circle.frame().z().dot(tangent).abs() - 1.0).abs() < 1.0e-14);

    let point = intersect_bounded_tori(
        &torus,
        window(0.5, 0.5, 0.25, 0.25),
        &torus,
        window(0.5, 0.5, 0.25, 0.25),
        Tolerances::default(),
    )
    .unwrap();
    assert!(point.is_complete());
    assert_uv_evidence_stays_in_windows(
        &point,
        window(0.5, 0.5, 0.25, 0.25),
        window(0.5, 0.5, 0.25, 0.25),
    );
    assert_eq!(point.points.len(), 1);
    assert_eq!(point.points[0].kind, ContactKind::Tangent);
    assert!(point.curves.is_empty());
    assert!(point.regions.is_empty());

    let miss = intersect_bounded_tori(
        &torus,
        window(0.0, 0.5, 0.0, 0.5),
        &torus,
        window(1.0, 1.5, 1.0, 1.5),
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.is_proven_empty());
}

#[test]
fn seam_split_collapses_have_no_duplicate_circle_pieces_and_swap_exactly() {
    let torus = world_torus();
    let tau = core::f64::consts::TAU;
    let latitude_a = window(0.0, tau, 0.5, 0.5);
    let latitude_b = window(1.0, 1.0 + tau, 0.5, 0.5);
    let latitude = intersect_bounded_tori(
        &torus,
        latitude_a,
        &torus,
        latitude_b,
        Tolerances::default(),
    )
    .unwrap();
    assert_curves_lift(&latitude, &torus, &torus);
    assert_uv_evidence_stays_in_windows(&latitude, latitude_a, latitude_b);
    assert_eq!(latitude.curves.len(), 2);
    assert!(
        (latitude
            .curves
            .iter()
            .map(|curve| curve.curve_range.width())
            .sum::<f64>()
            - tau)
            .abs()
            < 1.0e-14
    );
    let swapped = intersect_bounded_tori(
        &torus,
        latitude_b,
        &torus,
        latitude_a,
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(latitude.clone().swapped(), swapped);

    let meridian_a = window(0.5, 0.5, 0.0, tau);
    let meridian_b = window(0.5, 0.5, 1.0, 1.0 + tau);
    let meridian = intersect_bounded_tori(
        &torus,
        meridian_a,
        &torus,
        meridian_b,
        Tolerances::default(),
    )
    .unwrap();
    assert_curves_lift(&meridian, &torus, &torus);
    assert_uv_evidence_stays_in_windows(&meridian, meridian_a, meridian_b);
    assert_eq!(meridian.curves.len(), 2);
    assert!(
        (meridian
            .curves
            .iter()
            .map(|curve| curve.curve_range.width())
            .sum::<f64>()
            - tau)
            .abs()
            < 1.0e-14
    );
    let swapped = intersect_bounded_tori(
        &torus,
        meridian_b,
        &torus,
        meridian_a,
        Tolerances::default(),
    )
    .unwrap();
    assert_eq!(meridian.swapped(), swapped);
}

#[test]
fn overwide_periodic_windows_are_rejected_per_axis_and_operand() {
    let torus = world_torus();
    let tau = core::f64::consts::TAU;
    let first_longitude = intersect_bounded_tori(
        &torus,
        window(0.0, tau + 0.1, 0.0, 1.0),
        &torus,
        window(0.0, tau, 0.0, 1.0),
        Tolerances::default(),
    )
    .unwrap_err();
    // Same-class dispatch canonicalizes the narrower window first, so the
    // public first operand is intentionally reported as the internal second.
    assert_eq!(
        first_longitude,
        Error::InvalidGeometry {
            reason: "coincident torus second longitude window cannot span more than one turn"
        }
    );

    let second_latitude = intersect_bounded_tori(
        &torus,
        window(0.0, 1.0, 0.0, tau),
        &torus,
        window(0.0, 1.0, 0.0, tau + 0.1),
        Tolerances::default(),
    )
    .unwrap_err();
    assert_eq!(
        second_latitude,
        Error::InvalidGeometry {
            reason: "coincident torus second latitude window cannot span more than one turn"
        }
    );
}

#[test]
fn near_coincident_nonidentical_tori_fail_closed_before_dimension_changes() {
    let torus = world_torus();
    let cases = [
        Torus::new(
            Frame::world().with_origin(Point3::new(5.0e-9, 0.0, 0.0)),
            2.0,
            0.5,
        )
        .unwrap(),
        Torus::new(Frame::world(), 2.0 + 5.0e-9, 0.5).unwrap(),
        Torus::new(
            Frame::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 5.0e-12, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
            2.0,
            0.5,
        )
        .unwrap(),
    ];
    for near in cases {
        let error = intersect_bounded_tori(
            &torus,
            window(0.0, 1.0, 0.0, 1.0),
            &near,
            window(0.0, 1.0, 0.0, 1.0),
            Tolerances::default(),
        )
        .unwrap_err();
        assert_eq!(
            error,
            Error::InvalidGeometry {
                reason: "near-coincident non-identical tori require the general certified fallback"
            }
        );
    }
}

#[test]
fn remote_rotated_chart_map_fails_closed_while_identical_chart_remains_exact() {
    let torus = world_torus();
    let angle = 0.4_f64;
    let rotated = Torus::new(
        Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(math::cos(angle), math::sin(angle), 0.0),
        )
        .unwrap(),
        2.0,
        0.5,
    )
    .unwrap();
    let remote = 1.0e8 * core::f64::consts::TAU;
    let a_range = window(remote, remote + 0.5, 0.0, 0.5);
    let b_range = window(remote - angle, remote + 0.5 - angle, 0.0, 0.5);
    let error = intersect_bounded_tori(&torus, a_range, &rotated, b_range, Tolerances::default())
        .unwrap_err();
    assert_eq!(
        error,
        Error::InvalidGeometry {
            reason: "coincident torus chart map exceeds the certified parameter-roundoff corridor"
        }
    );

    let identical =
        intersect_bounded_tori(&torus, a_range, &torus, a_range, Tolerances::default()).unwrap();
    assert_regions_lift(&identical, &torus, &torus);
    assert_eq!(identical.regions.len(), 1);
}

#[test]
fn whole_region_residual_bound_is_outward_at_large_model_scale() {
    let origin = Point3::new(1.0e12, -2.0e12, 3.0e12);
    let a = Torus::new(Frame::world().with_origin(origin), 2.0e6, 0.5e6).unwrap();
    let angle = 0.4_f64;
    let b = Torus::new(
        Frame::new(
            origin,
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(math::cos(angle), math::sin(angle), 0.0),
        )
        .unwrap(),
        2.0e6,
        0.5e6,
    )
    .unwrap();
    let hit = intersect_bounded_tori(
        &a,
        window(0.4, 1.4, -0.5, 0.75),
        &b,
        window(0.6 - angle, 1.2 - angle, -0.25, 0.5),
        Tolerances::default(),
    )
    .unwrap();
    assert_regions_lift(&hit, &a, &b);
    assert!(hit.regions[0].max_residual.is_finite());
    assert!(hit.regions[0].max_residual > 0.0);
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
