//! Contract tests for certified procedural skew-cylinder sheet carriers.

use kcore::math;
use kgeom::curve::Curve;
use kgeom::curve2d::Curve2d;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Surface};
use kgeom::vec::{Vec2, Vec3};
use kgraph::{
    Curve2dDescriptor, CurveDescriptor, GeometryGraph, GeometryGraphError,
    IntersectionCertificateError, SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS, SkewCylinderSheet,
    certify_paired_skew_cylinder_branch_residuals,
    certify_paired_skew_cylinder_branch_subrange_residuals,
};

const TAU: f64 = core::f64::consts::TAU;

fn fixture() -> ([Cylinder; 2], [[ParamRange; 2]; 2]) {
    let first = Cylinder::new(Frame::world(), 1.0).unwrap();
    let second_frame = Frame::new(
        Vec3::default(),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let second = Cylinder::new(second_frame, 2.0).unwrap();
    (
        [first, second],
        [
            [ParamRange::new(0.0, TAU), ParamRange::new(-3.0, 3.0)],
            [ParamRange::new(0.0, TAU), ParamRange::new(-2.0, 2.0)],
        ],
    )
}

#[test]
fn certifies_known_values_lifts_and_three_derivative_orders() {
    let (cylinders, ranges) = fixture();
    for sheet in [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper] {
        let certificate =
            certify_paired_skew_cylinder_branch_residuals(cylinders, ranges, sheet, 1e-8).unwrap();
        assert_eq!(certificate.carrier().periodicity(), Some(TAU));
        assert_eq!(certificate.traces()[0].pcurve().periodicity(), None);
        assert_eq!(certificate.traces()[1].pcurve().periodicity(), None);
        assert!(
            (certificate.carrier().eval(0.37 + TAU) - certificate.carrier().eval(0.37)).norm()
                < 2e-14
        );
        assert_eq!(certificate.traces()[0].pcurve().eval(TAU).x, TAU);
        assert_eq!(certificate.parameter_maps()[0].scale(), 1.0);
        assert!(certificate.residual_bounds()[1] <= certificate.tolerance());

        for parameter in [0.0, 0.37, 1.7, 3.9, TAU] {
            let (sine, cosine) = math::sincos(parameter);
            let sign = if sheet == SkewCylinderSheet::Lower {
                -1.0
            } else {
                1.0
            };
            let height = sign * (4.0 - sine * sine).sqrt();
            let height_derivative = -sign * sine * cosine / height.abs();
            let carrier_derivs = certificate.carrier().eval_derivs(parameter, 3);
            assert!((carrier_derivs.d[0].z - height).abs() < 2e-14);
            assert!((carrier_derivs.d[1].z - height_derivative).abs() < 2e-13);
            assert!(carrier_derivs.d.into_iter().all(finite3));
            assert!(
                certificate
                    .carrier()
                    .bounding_box(ranges[0][0])
                    .contains(carrier_derivs.d[0])
            );

            let [first, second] = certificate.traces();
            let first_derivs = first.pcurve().eval_derivs(parameter, 3);
            assert!((first_derivs.d[0].y - height).abs() < 2e-14);
            assert!((first_derivs.d[1].y - height_derivative).abs() < 2e-13);
            let second_derivs = second.pcurve().eval_derivs(parameter, 3);
            let raw_longitude = math::atan2(height, sine);
            let expected_longitude = if sheet == SkewCylinderSheet::Lower {
                raw_longitude + TAU
            } else {
                raw_longitude
            };
            assert!((second_derivs.d[0].x - expected_longitude).abs() < 2e-14);
            assert!((second_derivs.d[0].y - cosine).abs() < 2e-14);
            assert!(
                (second_derivs.d[1].x - (sine * height_derivative - height * cosine) / 4.0).abs()
                    < 2e-13
            );
            assert!((second_derivs.d[1].y + sine).abs() < 2e-13);
            assert!(first_derivs.d.into_iter().all(finite2));
            assert!(second_derivs.d.into_iter().all(finite2));

            for trace in certificate.traces() {
                let uv = trace.pcurve().eval(parameter);
                let reconstructed = trace.surface().eval([uv.x, uv.y]);
                assert!((reconstructed - carrier_derivs.d[0]).norm() <= certificate.tolerance());
            }
        }
    }
}

#[test]
fn swaps_only_source_order_and_rejects_tight_windows() {
    let (cylinders, mut ranges) = fixture();
    let certificate = certify_paired_skew_cylinder_branch_residuals(
        cylinders,
        ranges,
        SkewCylinderSheet::Upper,
        1e-8,
    )
    .unwrap();
    let swapped = certificate.swapped();
    assert_eq!(swapped.carrier(), certificate.carrier());
    assert_eq!(swapped.traces()[0], certificate.traces()[1]);
    assert_eq!(
        swapped.residual_bounds()[0],
        certificate.residual_bounds()[1]
    );

    ranges[0][1] = ParamRange::new(-1.0, 1.0);
    assert!(matches!(
        certify_paired_skew_cylinder_branch_residuals(
            cylinders,
            ranges,
            SkewCylinderSheet::Upper,
            1e-8
        ),
        Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
    ));
}

#[test]
fn operation_local_descriptors_are_rejected_before_graph_mutation() {
    let (cylinders, ranges) = fixture();
    let certificate = certify_paired_skew_cylinder_branch_residuals(
        cylinders,
        ranges,
        SkewCylinderSheet::Upper,
        1e-8,
    )
    .unwrap();
    let mut graph = GeometryGraph::new();
    assert!(matches!(
        graph.insert_curve(CurveDescriptor::from(certificate.carrier())),
        Err(GeometryGraphError::InvalidDescriptor { .. })
    ));
    assert!(graph.is_empty());
    assert!(matches!(
        graph.insert_curve2d(Curve2dDescriptor::from(certificate.traces()[0].pcurve())),
        Err(GeometryGraphError::InvalidDescriptor { .. })
    ));
    assert!(graph.is_empty());
}

#[test]
fn strict_subrange_certifies_directly_when_the_whole_sheet_escapes_both_axial_windows() {
    let (cylinders, mut ranges) = fixture();
    ranges[0][1] = ParamRange::new(1.7, 1.9);
    ranges[1][1] = ParamRange::new(-0.5, 0.5);
    assert!(matches!(
        certify_paired_skew_cylinder_branch_residuals(
            cylinders,
            ranges,
            SkewCylinderSheet::Upper,
            1e-8,
        ),
        Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
    ));

    let subrange = ParamRange::new(1.25, 1.89);
    let certificate = certify_paired_skew_cylinder_branch_subrange_residuals(
        cylinders,
        ranges,
        subrange,
        SkewCylinderSheet::Upper,
        1e-8,
    )
    .unwrap();
    assert_eq!(certificate.carrier_range(), subrange);
    assert_eq!(certificate.carrier().param_range(), subrange);
    assert_eq!(
        certificate
            .traces()
            .map(|trace| trace.pcurve().param_range()),
        [subrange; 2]
    );
    assert_eq!(
        certificate.traces().map(|trace| trace.pcurve().operand()),
        [0, 1]
    );
    assert_eq!(certificate.traces().map(|trace| trace.surface()), cylinders);

    let carrier_box = certificate.carrier().bounding_box(subrange);
    assert!(
        carrier_box.max.x < 0.4,
        "the bounded proof box must not retain the full-cycle x=1 extremum"
    );
    for parameter in [subrange.lo, core::f64::consts::FRAC_PI_2, subrange.hi] {
        let (sine, cosine) = math::sincos(parameter);
        let height = (4.0 - sine * sine).sqrt();
        let point = certificate.carrier().eval(parameter);
        assert!((point - Vec3::new(cosine, sine, height)).norm() < 2e-14);
        assert!(carrier_box.contains(point));

        let [first, second] = certificate.traces();
        assert!((first.pcurve().eval(parameter) - Vec2::new(parameter, height)).norm() < 2e-14);
        assert!(
            (second.pcurve().eval(parameter) - Vec2::new(math::atan2(height, sine), cosine)).norm()
                < 2e-14
        );
        for trace in [first, second] {
            let uv = trace.pcurve().eval(parameter);
            assert!((trace.surface().eval([uv.x, uv.y]) - point).norm() <= certificate.tolerance());
            assert!(
                trace
                    .pcurve()
                    .bounding_box(subrange)
                    .contains(trace.pcurve().eval(parameter))
            );
        }
    }

    let swapped = certificate.swapped();
    assert_eq!(swapped.carrier(), certificate.carrier());
    assert_eq!(swapped.carrier_range(), subrange);
    assert_eq!(
        swapped.traces(),
        [certificate.traces()[1], certificate.traces()[0]]
    );
    assert_eq!(
        swapped.residual_bounds(),
        [
            certificate.residual_bounds()[1],
            certificate.residual_bounds()[0]
        ]
    );
}

#[test]
fn guarded_boundary_root_subrange_avoids_uniform_cell_dependency_leakage() {
    let (cylinders, mut ranges) = fixture();
    ranges[0][1] = ParamRange::new(1.8, 2.1);
    ranges[1][1] = ParamRange::new(-1.25, 0.0);
    let root_span = ParamRange::new(2.082_769_014_844_373_6, 4.200_416_292_335_213);
    assert!(matches!(
        certify_paired_skew_cylinder_branch_subrange_residuals(
            cylinders,
            ranges,
            root_span,
            SkewCylinderSheet::Upper,
            1e-8,
        ),
        Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
    ));

    let mut guarded_lo = root_span.lo;
    let mut guarded_hi = root_span.hi;
    for _ in 0..SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS {
        guarded_lo = guarded_lo.next_up();
        guarded_hi = guarded_hi.next_down();
    }
    let subrange = ParamRange::new(guarded_lo, guarded_hi);
    assert!(subrange.lo - root_span.lo < 1e-12);
    assert!(root_span.hi - subrange.hi < 1e-12);

    let certificate = certify_paired_skew_cylinder_branch_subrange_residuals(
        cylinders,
        ranges,
        subrange,
        SkewCylinderSheet::Upper,
        1e-8,
    )
    .unwrap();
    assert_eq!(certificate.carrier_range(), subrange);
    assert_eq!(certificate.carrier().param_range(), subrange);
    assert_eq!(
        certificate
            .traces()
            .map(|trace| trace.pcurve().param_range()),
        [subrange; 2]
    );

    for parameter in [subrange.lo, subrange.hi] {
        let (sine, cosine) = math::sincos(parameter);
        let canonical_height = (4.0 - sine * sine).sqrt();
        assert!(canonical_height >= ranges[0][1].lo);
        assert!(canonical_height < ranges[0][1].hi);
        assert!(cosine > ranges[1][1].lo && cosine < ranges[1][1].hi);
        let point = certificate.carrier().eval(parameter);
        assert!((point - Vec3::new(cosine, sine, canonical_height)).norm() < 2e-14);
    }
}

#[test]
fn strict_subrange_rejects_invalid_nonfinite_wrapping_and_mismatched_ranges() {
    let (cylinders, ranges) = fixture();
    let certify = |ranges, carrier_range| {
        certify_paired_skew_cylinder_branch_subrange_residuals(
            cylinders,
            ranges,
            carrier_range,
            SkewCylinderSheet::Upper,
            1e-8,
        )
    };

    assert_eq!(
        certify(
            ranges,
            ParamRange {
                lo: f64::NAN,
                hi: 2.0,
            },
        ),
        Err(IntersectionCertificateError::InvalidCarrierRange)
    );
    assert_eq!(
        certify(ranges, ParamRange::new(2.0, 2.0)),
        Err(IntersectionCertificateError::InvalidCarrierRange)
    );
    for range in [
        ParamRange::new(0.0, 1.0),
        ParamRange::new(1.0, TAU),
        ParamRange::new(5.8, TAU + 0.2),
    ] {
        assert!(matches!(
            certify(ranges, range),
            Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
        ));
    }

    let mut shifted_authored_range = ranges;
    shifted_authored_range[0][0] = ParamRange::new(TAU, 2.0 * TAU);
    assert!(matches!(
        certify(shifted_authored_range, ParamRange::new(1.25, 1.89)),
        Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
    ));

    let mut nonfinite_authored_range = ranges;
    nonfinite_authored_range[1][1].hi = f64::INFINITY;
    assert_eq!(
        certify(nonfinite_authored_range, ParamRange::new(1.25, 1.89)),
        Err(IntersectionCertificateError::InvalidCarrierRange)
    );
}

fn finite2(value: Vec2) -> bool {
    value.x.is_finite() && value.y.is_finite()
}

fn finite3(value: Vec3) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
}
