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
    IntersectionCertificateError, SkewCylinderSheet, certify_paired_skew_cylinder_branch_residuals,
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

fn finite2(value: Vec2) -> bool {
    value.x.is_finite() && value.y.is_finite()
}

fn finite3(value: Vec3) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
}
