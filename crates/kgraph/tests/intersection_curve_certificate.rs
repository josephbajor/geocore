//! Whole-interval certification tests for affine plane intersection traces.

use kgeom::curve::Line;
use kgeom::curve2d::Line2d;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::{Vec2, Vec3};
use kgraph::{
    AffineParamMap1d, IntersectionCertificateError, PairedTrace,
    certify_paired_plane_line_residuals,
};

fn carrier() -> Line {
    Line::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap()
}

fn planes() -> [Plane; 2] {
    [
        Plane::new(Frame::world()),
        Plane::new(
            Frame::new(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
        ),
    ]
}

fn identity_pcurves() -> [Line2d; 2] {
    [
        Line2d::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
        Line2d::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
    ]
}

fn identity_maps() -> [AffineParamMap1d; 2] {
    [
        AffineParamMap1d::new(1.0, 0.0).unwrap(),
        AffineParamMap1d::new(1.0, 0.0).unwrap(),
    ]
}

#[test]
fn exact_paired_plane_traces_are_certified_over_the_whole_range() {
    let range = ParamRange::new(-3.0, 7.0);
    let certificate = certify_paired_plane_line_residuals(
        carrier(),
        range,
        planes(),
        identity_pcurves(),
        identity_maps(),
        1.0e-12,
    )
    .unwrap();

    assert_eq!(certificate.carrier(), carrier());
    assert_eq!(certificate.carrier_range(), range);
    assert_eq!(certificate.surfaces(), planes());
    assert_eq!(certificate.pcurves(), identity_pcurves());
    assert_eq!(certificate.parameter_maps(), identity_maps());
    assert_eq!(certificate.tolerance(), 1.0e-12);
    assert!(
        certificate
            .residual_bounds()
            .into_iter()
            .all(|bound| bound <= certificate.tolerance())
    );
}

#[test]
fn reversed_and_nonidentity_parameter_maps_certify_and_roundtrip() {
    let pcurves = [
        Line2d::new(Vec2::new(5.0, 0.0), Vec2::new(-1.0, 0.0)).unwrap(),
        Line2d::new(Vec2::new(-2.0, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
    ];
    // First lift: u = 5 - (5 - t) = t. Second lift:
    // u = -2 + (2 + t) = t.
    let maps = [
        AffineParamMap1d::new(-1.0, 5.0).unwrap(),
        AffineParamMap1d::new(1.0, 2.0).unwrap(),
    ];
    let range = ParamRange::new(-4.0, 3.0);

    let certificate =
        certify_paired_plane_line_residuals(carrier(), range, planes(), pcurves, maps, 1.0e-12)
            .unwrap();

    assert_eq!(maps[0].map_range(range), ParamRange::new(2.0, 9.0));
    for map in maps {
        for parameter in [range.lo, 0.25, range.hi] {
            assert_eq!(map.inverse(map.map(parameter)), parameter);
        }
    }
    assert!(
        certificate
            .residual_bounds()
            .into_iter()
            .all(|bound| bound <= 1.0e-12)
    );
}

#[test]
fn perturbed_pcurve_is_rejected_with_the_failing_trace() {
    let mut pcurves = identity_pcurves();
    pcurves[1] = Line2d::new(Vec2::new(0.0, 0.01), Vec2::new(1.0, 0.0)).unwrap();

    let error = certify_paired_plane_line_residuals(
        carrier(),
        ParamRange::new(-1.0, 1.0),
        planes(),
        pcurves,
        identity_maps(),
        1.0e-6,
    )
    .unwrap_err();

    assert!(matches!(
        error,
        IntersectionCertificateError::ResidualExceedsTolerance {
            trace: PairedTrace::Second,
            residual_bound,
            tolerance: 1.0e-6,
        } if residual_bound >= 0.01
    ));
}

#[test]
fn nonfinite_reversed_ranges_maps_and_tolerances_are_rejected() {
    assert!(matches!(
        AffineParamMap1d::new(0.0, 1.0),
        Err(IntersectionCertificateError::InvalidParameterMap { .. })
    ));
    assert!(matches!(
        AffineParamMap1d::new(f64::NAN, 1.0),
        Err(IntersectionCertificateError::InvalidParameterMap { .. })
    ));
    assert!(matches!(
        AffineParamMap1d::new(1.0, f64::INFINITY),
        Err(IntersectionCertificateError::InvalidParameterMap { .. })
    ));

    for range in [
        ParamRange::unbounded(),
        ParamRange { lo: 2.0, hi: 1.0 },
        ParamRange {
            lo: f64::NAN,
            hi: 1.0,
        },
    ] {
        assert_eq!(
            certify_paired_plane_line_residuals(
                carrier(),
                range,
                planes(),
                identity_pcurves(),
                identity_maps(),
                1.0e-6,
            ),
            Err(IntersectionCertificateError::InvalidCarrierRange)
        );
    }

    for tolerance in [-1.0, f64::NAN, f64::INFINITY] {
        assert_eq!(
            certify_paired_plane_line_residuals(
                carrier(),
                ParamRange::new(-1.0, 1.0),
                planes(),
                identity_pcurves(),
                identity_maps(),
                tolerance,
            ),
            Err(IntersectionCertificateError::InvalidTolerance)
        );
    }

    let nonfinite_carrier =
        Line::new(Vec3::new(f64::NAN, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
    assert_eq!(
        certify_paired_plane_line_residuals(
            nonfinite_carrier,
            ParamRange::new(-1.0, 1.0),
            planes(),
            identity_pcurves(),
            identity_maps(),
            1.0e-6,
        ),
        Err(IntersectionCertificateError::NonFiniteGeometry)
    );
}

#[test]
fn certificate_minting_is_deterministic() {
    let mint = || {
        certify_paired_plane_line_residuals(
            carrier(),
            ParamRange::new(-17.25, 23.5),
            planes(),
            identity_pcurves(),
            identity_maps(),
            1.0e-12,
        )
        .unwrap()
    };

    assert_eq!(mint(), mint());
    assert_eq!(format!("{:?}", mint()), format!("{:?}", mint()));
}
