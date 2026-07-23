//! Exact-source interval boundary regressions.

use super::*;
use kgeom::frame::Frame;

fn fixture() -> ([Cylinder; 2], [[ParamRange; 2]; 2]) {
    let first = Cylinder::new(Frame::world(), 1.0).unwrap();
    let second = Cylinder::new(
        Frame::new(
            Vec3::default(),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap(),
        2.0,
    )
    .unwrap();
    (
        [first, second],
        [
            [ParamRange::new(0.0, TAU), ParamRange::new(-3.0, 3.0)],
            [ParamRange::new(0.0, TAU), ParamRange::new(-2.0, 2.0)],
        ],
    )
}

#[test]
fn exact_source_root_height_and_longitude_enclosures_gate_near_boundary_windows() {
    let (cylinders, ranges) = fixture();
    let algebra = build_algebra(cylinders, ranges[0][0], SkewCylinderSheet::Upper).unwrap();
    let coefficients = coefficient_proof(algebra).unwrap();
    let mut stored_root_lower = f64::INFINITY;
    let mut exact_root_lower = f64::INFINITY;
    let mut stored_height_lower = f64::INFINITY;
    let mut exact_height_lower = f64::INFINITY;
    let mut stored_longitude_lower = f64::INFINITY;
    let mut stored_longitude_upper = f64::NEG_INFINITY;
    let mut exact_longitude_lower = f64::INFINITY;
    let mut exact_longitude_upper = f64::NEG_INFINITY;
    for index in 0..SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS {
        let lo = ranges[0][0].lerp(index as f64 / SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS as f64);
        let hi = ranges[0][0].lerp((index + 1) as f64 / SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS as f64);
        let cosine = trig_interval(lo, hi, false);
        let sine = trig_interval(lo, hi, true);
        let roots = cell_root_enclosures(algebra, coefficients, cosine, sine).unwrap();
        stored_root_lower = stored_root_lower.min(roots.stored_v.lo());
        exact_root_lower = exact_root_lower.min(roots.exact_v.lo());

        let stored_z = algebra.z0.interval(cosine, sine).unwrap()
            + Interval::point(algebra.dz) * roots.stored_v;
        let exact_z = coefficients.harmonics_true[2]
            .interval(cosine, sine)
            .unwrap()
            + coefficients.directions_true[2] * roots.exact_v;
        stored_height_lower = stored_height_lower.min(
            stored_z
                .checked_div(Interval::point(algebra.e))
                .unwrap()
                .lo(),
        );
        exact_height_lower =
            exact_height_lower.min(exact_z.checked_div(coefficients.e_true).unwrap().lo());

        let stored_xy =
            [(algebra.x0, algebra.dx), (algebra.y0, algebra.dy)].map(|(harmonic, direction)| {
                harmonic.interval(cosine, sine).unwrap()
                    + Interval::point(direction) * roots.stored_v
            });
        let exact_xy = [0, 1].map(|coordinate| {
            coefficients.harmonics_true[coordinate]
                .interval(cosine, sine)
                .unwrap()
                + coefficients.directions_true[coordinate] * roots.exact_v
        });
        let stored_longitude = longitude_interval(
            stored_xy[0]
                .checked_div(Interval::point(algebra.e))
                .unwrap(),
            stored_xy[1]
                .checked_div(Interval::point(algebra.e))
                .unwrap(),
        );
        let exact_longitude = longitude_interval(
            exact_xy[0].checked_div(coefficients.e_true).unwrap(),
            exact_xy[1].checked_div(coefficients.e_true).unwrap(),
        );
        stored_longitude_lower = stored_longitude_lower.min(stored_longitude.lo());
        stored_longitude_upper = stored_longitude_upper.max(stored_longitude.hi());
        exact_longitude_lower = exact_longitude_lower.min(exact_longitude.lo());
        exact_longitude_upper = exact_longitude_upper.max(exact_longitude.hi());
    }

    let root_boundary = exact_root_lower.midpoint(stored_root_lower);
    assert!(exact_root_lower < root_boundary && root_boundary < stored_root_lower);
    let mut root_ranges = ranges;
    root_ranges[0][1].lo = root_boundary;
    assert!(matches!(
        certify_paired_skew_cylinder_branch_residuals(
            cylinders,
            root_ranges,
            SkewCylinderSheet::Upper,
            1e-8,
        ),
        Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
    ));

    let height_boundary = exact_height_lower.midpoint(stored_height_lower);
    assert!(exact_height_lower < height_boundary && height_boundary < stored_height_lower);
    let mut height_ranges = ranges;
    height_ranges[1][1].lo = height_boundary;
    assert!(matches!(
        certify_paired_skew_cylinder_branch_residuals(
            cylinders,
            height_ranges,
            SkewCylinderSheet::Upper,
            1e-8,
        ),
        Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
    ));

    let longitude_window = if exact_longitude_lower < stored_longitude_lower {
        let lower = exact_longitude_lower.midpoint(stored_longitude_lower);
        ParamRange::new(lower, lower + TAU)
    } else {
        assert!(exact_longitude_upper > stored_longitude_upper);
        let upper = exact_longitude_upper.midpoint(stored_longitude_upper);
        ParamRange::new(upper - TAU, upper)
    };
    assert_eq!(longitude_window.width(), TAU);
    assert!(
        stored_longitude_lower > longitude_window.lo
            && stored_longitude_upper < longitude_window.hi
    );
    assert!(
        exact_longitude_lower <= longitude_window.lo
            || exact_longitude_upper >= longitude_window.hi
    );
    let mut longitude_ranges = ranges;
    longitude_ranges[1][0] = longitude_window;
    assert!(matches!(
        certify_paired_skew_cylinder_branch_residuals(
            cylinders,
            longitude_ranges,
            SkewCylinderSheet::Upper,
            1e-8,
        ),
        Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
    ));
}
