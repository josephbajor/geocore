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

fn oblique_fixture() -> ([Cylinder; 2], [[ParamRange; 2]; 2]) {
    let frame = Frame::new(
        Vec3::new(2.5, -1.75, 0.625),
        Vec3::new(0.48, 0.64, 0.6),
        Vec3::new(0.8, -0.6, 0.0),
    )
    .unwrap();
    let first = Cylinder::new(frame, 1.0).unwrap();
    let second = Cylinder::new(
        Frame::new(frame.origin(), frame.x(), frame.y()).unwrap(),
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

#[test]
fn indexed_pcurve_cells_cover_guarded_range_and_enclose_independent_derivative_oracle() {
    let (cylinders, mut ranges) = fixture();
    ranges[0][1] = ParamRange::new(1.8, 2.1);
    ranges[1][1] = ParamRange::new(-1.25, 0.0);
    let root_span = ParamRange::new(2.082_769_014_844_373_6, 4.200_416_292_335_213);
    let mut guarded = root_span;
    for _ in 0..SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS {
        guarded.lo = guarded.lo.next_up();
        guarded.hi = guarded.hi.next_down();
    }
    let certificate = certify_paired_skew_cylinder_branch_subrange_residuals(
        cylinders,
        ranges,
        guarded,
        SkewCylinderSheet::Upper,
        1e-8,
    )
    .unwrap();

    let mut previous_hi = guarded.lo;
    let mut work = 0_u64;
    for index in 0..SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS {
        let cell = certificate.certify_pcurve_cell(index).unwrap();
        assert_eq!(cell.parameter().lo().to_bits(), previous_hi.to_bits());
        assert!(cell.parameter().width() > 0.0);
        previous_hi = cell.parameter().hi();
        work += cell.work();

        let parameter = cell.parameter().lo().midpoint(cell.parameter().hi());
        let (sine, cosine) = math::sincos(parameter);
        let height = (4.0 - sine * sine).sqrt();
        let height_derivative = -sine * cosine / height;
        let expected = [
            ([parameter, height], [1.0, height_derivative]),
            (
                [math::atan2(height, sine), cosine],
                [(sine * height_derivative - height * cosine) / 4.0, -sine],
            ),
        ];
        for (enclosure, (expected_uv, expected_derivative)) in
            cell.pcurves().into_iter().zip(expected)
        {
            assert!(enclosure.stored_is_strictly_regular());
            assert!(enclosure.source_is_strictly_regular());
            for coordinate in 0..2 {
                assert!(enclosure.stored_uv()[coordinate].contains(expected_uv[coordinate]));
                assert!(enclosure.source_uv()[coordinate].contains(expected_uv[coordinate]));
                assert!(
                    enclosure.stored_derivative()[coordinate]
                        .contains(expected_derivative[coordinate])
                );
                assert!(
                    enclosure.source_derivative()[coordinate]
                        .contains(expected_derivative[coordinate])
                );
            }
        }
    }
    assert_eq!(previous_hi.to_bits(), guarded.hi.to_bits());
    assert_eq!(work, SKEW_CYLINDER_BRANCH_PCURVE_ALL_CELLS_WORK);
    assert!(matches!(
        certificate.certify_pcurve_cell(SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS),
        Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
    ));

    let swapped = certificate.swapped().certify_pcurve_cell(0).unwrap();
    assert_eq!(
        swapped.pcurves().map(|enclosure| enclosure.operand()),
        [1, 0]
    );
}

#[test]
fn physical_root_corridors_reprove_root_uv_and_reject_wrong_guard_or_chart() {
    let (cylinders, mut ranges) = fixture();
    ranges[0][1] = ParamRange::new(1.8, 2.1);
    ranges[1][1] = ParamRange::new(-1.25, 0.0);
    let roots = [2.082_769_014_844_373_6, 4.200_416_292_335_213];
    let mut guarded = ParamRange::new(roots[0], roots[1]);
    for _ in 0..SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS {
        guarded.lo = guarded.lo.next_up();
        guarded.hi = guarded.hi.next_down();
    }
    let certificate = certify_paired_skew_cylinder_branch_subrange_residuals(
        cylinders,
        ranges,
        guarded,
        SkewCylinderSheet::Upper,
        1e-8,
    )
    .unwrap();

    let lower_root = Interval::new(roots[0].next_down(), roots[0].next_up());
    let upper_root = Interval::new(roots[1].next_down(), roots[1].next_up());
    let lower = certificate
        .certify_lower_pcurve_root_corridor(lower_root)
        .unwrap();
    let upper = certificate
        .certify_upper_pcurve_root_corridor(upper_root)
        .unwrap();
    assert_eq!(lower.guarded_end(), SkewCylinderBranchGuardedEnd::Lower);
    assert_eq!(upper.guarded_end(), SkewCylinderBranchGuardedEnd::Upper);
    assert_eq!(lower.corridor().parameter().lo(), lower_root.lo());
    assert_eq!(lower.corridor().parameter().hi(), guarded.lo);
    assert_eq!(upper.corridor().parameter().lo(), guarded.hi);
    assert_eq!(upper.corridor().parameter().hi(), upper_root.hi());
    assert_eq!(
        lower.work() + upper.work(),
        2 * SKEW_CYLINDER_BRANCH_PCURVE_ROOT_CORRIDOR_WORK
    );

    for proof in [lower, upper] {
        let parameter = proof.root_parameter().lo();
        let (sine, cosine) = math::sincos(parameter);
        let height = (4.0 - sine * sine).sqrt();
        let height_derivative = -sine * cosine / height;
        let expected = [
            ([parameter, height], [1.0, height_derivative]),
            (
                [math::atan2(height, sine), cosine],
                [(sine * height_derivative - height * cosine) / 4.0, -sine],
            ),
        ];
        for (enclosure, (expected_uv, expected_derivative)) in
            proof.root_pcurves().into_iter().zip(expected)
        {
            assert!(enclosure.stored_is_strictly_regular());
            assert!(enclosure.source_is_strictly_regular());
            for coordinate in 0..2 {
                assert!(enclosure.stored_uv()[coordinate].contains(expected_uv[coordinate]));
                assert!(enclosure.source_uv()[coordinate].contains(expected_uv[coordinate]));
                assert!(
                    enclosure.stored_derivative()[coordinate]
                        .contains(expected_derivative[coordinate])
                );
                assert!(
                    enclosure.source_derivative()[coordinate]
                        .contains(expected_derivative[coordinate])
                );
            }
        }
    }

    assert!(matches!(
        certificate.certify_upper_pcurve_root_corridor(Interval::point(roots[0])),
        Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
    ));
    assert!(matches!(
        certificate.certify_lower_pcurve_root_corridor(Interval::point(guarded.lo)),
        Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
    ));
    assert!(matches!(
        certificate.certify_lower_pcurve_root_corridor(Interval::point(-0.1)),
        Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
    ));
    assert_eq!(
        certificate.certify_lower_pcurve_root_corridor(Interval::point(f64::NEG_INFINITY)),
        Err(IntersectionCertificateError::InvalidCarrierRange)
    );
}

#[test]
fn oblique_rigid_copy_recertifies_every_guarded_cell_and_both_root_corridors() {
    let (cylinders, mut ranges) = oblique_fixture();
    ranges[0][1] = ParamRange::new(1.8, 2.1);
    ranges[1][1] = ParamRange::new(-1.25, 0.0);
    let roots = [2.082_769_014_844_373_6, 4.200_416_292_335_213];
    let mut guarded = ParamRange::new(roots[0], roots[1]);
    for _ in 0..SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS {
        guarded.lo = guarded.lo.next_up();
        guarded.hi = guarded.hi.next_down();
    }
    let certificate = certify_paired_skew_cylinder_branch_subrange_residuals(
        cylinders,
        ranges,
        guarded,
        SkewCylinderSheet::Upper,
        1e-8,
    )
    .unwrap();

    for index in 0..SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS {
        let cell = certificate.certify_pcurve_cell(index).unwrap();
        for pcurve in cell.pcurves() {
            assert!(pcurve.stored_is_strictly_regular());
            assert!(pcurve.source_is_strictly_regular());
        }
    }
    certificate
        .certify_lower_pcurve_root_corridor(Interval::new(roots[0].next_down(), roots[0].next_up()))
        .unwrap();
    certificate
        .certify_upper_pcurve_root_corridor(Interval::new(roots[1].next_down(), roots[1].next_up()))
        .unwrap();
}
