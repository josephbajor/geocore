//! Contract tests for certified procedural skew-cylinder sheet carriers.

use kcore::interval::Interval;
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::curve2d::Curve2d;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Surface};
use kgeom::vec::{Vec2, Vec3};
use kgraph::{
    Curve2dDescriptor, CurveDescriptor, EvalContext, EvalError, EvalLimits, GeometryGraph,
    GeometryGraphError, GeometryRef, IntersectionCertificateError,
    PERSISTENT_SKEW_CYLINDER_OPEN_SPAN_WORK, PersistentSkewCylinderOpenSpanCertificate,
    PersistentSkewCylinderOpenSpanOrientation, SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS,
    SkewCylinderSheet, certify_paired_skew_cylinder_branch_residuals,
    certify_paired_skew_cylinder_branch_subrange_residuals,
    certify_persistent_skew_cylinder_open_span,
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

fn persistent_fixture(
    orientation: PersistentSkewCylinderOpenSpanOrientation,
) -> ([Cylinder; 2], PersistentSkewCylinderOpenSpanCertificate) {
    let (cylinders, mut ranges) = fixture();
    ranges[0][1] = ParamRange::new(1.8, 2.1);
    ranges[1][1] = ParamRange::new(-1.25, 0.0);
    let roots = [2.082_769_014_844_373_6, 4.200_416_292_335_213];
    let mut guarded = ParamRange::new(roots[0], roots[1]);
    for _ in 0..SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS {
        guarded.lo = guarded.lo.next_up();
        guarded.hi = guarded.hi.next_down();
    }
    let residual = certify_paired_skew_cylinder_branch_subrange_residuals(
        cylinders,
        ranges,
        guarded,
        SkewCylinderSheet::Upper,
        1e-8,
    )
    .unwrap();
    let root_intervals = roots.map(|root| Interval::new(root.next_down(), root.next_up()));
    let corridors = [
        residual
            .certify_lower_pcurve_root_corridor(root_intervals[0])
            .unwrap(),
        residual
            .certify_upper_pcurve_root_corridor(root_intervals[1])
            .unwrap(),
    ];
    let endpoint_points = roots.map(|parameter| {
        let (sine, cosine) = math::sincos(parameter);
        Vec3::new(cosine, sine, (4.0 - sine * sine).sqrt())
    });
    let certificate = certify_persistent_skew_cylinder_open_span(
        residual,
        corridors,
        endpoint_points,
        orientation,
    )
    .unwrap();
    (cylinders, certificate)
}

#[test]
fn persistent_open_span_normalizes_corridors_without_inventing_root_scalars() {
    let (cylinders, forward) =
        persistent_fixture(PersistentSkewCylinderOpenSpanOrientation::Forward);
    let (_, reversed) = persistent_fixture(PersistentSkewCylinderOpenSpanOrientation::Reversed);
    let logical = ParamRange::new(0.0, 1.0);

    assert_eq!(forward.logical_range(), logical);
    assert_eq!(forward.carrier().param_range(), logical);
    assert_eq!(forward.carrier().periodicity(), None);
    assert_eq!(forward.work(), PERSISTENT_SKEW_CYLINDER_OPEN_SPAN_WORK);
    assert_eq!(forward.work(), 260);
    assert_eq!(
        reversed.endpoint_points(),
        [forward.endpoint_points()[1], forward.endpoint_points()[0]]
    );
    assert_eq!(forward.carrier().eval(0.0), reversed.carrier().eval(1.0));
    assert_eq!(forward.carrier().eval(1.0), reversed.carrier().eval(0.0));
    assert!(
        forward.carrier().eval_derivs(0.37, 1).d[0]
            .dist(reversed.carrier().eval_derivs(0.63, 1).d[0])
            < 2e-14
    );
    assert!(
        (forward.carrier().eval_derivs(0.37, 1).d[1]
            + reversed.carrier().eval_derivs(0.63, 1).d[1])
            .norm()
            < 2e-12
    );

    let guarded = forward.residual_certificate().carrier_range();
    assert!(forward.pcurves()[0].eval(0.0).x < guarded.lo);
    assert!(forward.pcurves()[0].eval(1.0).x > guarded.hi);
    assert!(
        forward
            .residual_bounds()
            .into_iter()
            .all(|bound| bound <= forward.required_edge_tolerance())
    );
    assert!(forward.required_edge_tolerance() <= forward.residual_certificate().tolerance());

    for endpoint in 0..2 {
        let logical_parameter = endpoint as f64;
        let root = forward.root_corridors()[endpoint];
        let carrier_point = forward.carrier().eval(logical_parameter);
        assert!(
            forward
                .carrier()
                .bounding_box(logical)
                .contains(carrier_point)
        );
        for (trace_index, ((trace, pcurve), enclosure)) in forward
            .residual_certificate()
            .traces()
            .into_iter()
            .zip(forward.pcurves())
            .zip(root.root_pcurves())
            .enumerate()
        {
            let uv = pcurve.eval(logical_parameter);
            assert!(enclosure.stored_uv()[0].contains(uv.x));
            assert!(enclosure.stored_uv()[1].contains(uv.y));
            assert!(pcurve.bounding_box(logical).contains(uv));
            let reconstructed = trace.surface().eval([uv.x, uv.y]);
            assert!(reconstructed.dist(carrier_point) <= forward.residual_bounds()[trace_index]);
            assert!(
                reconstructed.dist(forward.endpoint_points()[endpoint])
                    <= forward.required_edge_tolerance()
            );
        }
    }

    assert_eq!(
        forward
            .residual_certificate()
            .traces()
            .map(|trace| trace.surface()),
        cylinders
    );

    let canonical_points = forward.endpoint_points();
    let far_points = [
        canonical_points[0] + Vec3::new(1.0, 0.0, 0.0),
        canonical_points[1],
    ];
    assert!(matches!(
        certify_persistent_skew_cylinder_open_span(
            forward.residual_certificate(),
            forward.root_corridors(),
            far_points,
            PersistentSkewCylinderOpenSpanOrientation::Forward,
        ),
        Err(IntersectionCertificateError::UnsupportedCarrierParameterization { .. })
    ));

    let swapped_residual = forward.residual_certificate().swapped();
    let roots = forward
        .root_corridors()
        .map(|corridor| corridor.root_parameter());
    let mixed_corridors = [
        swapped_residual
            .certify_lower_pcurve_root_corridor(roots[0])
            .unwrap(),
        swapped_residual
            .certify_upper_pcurve_root_corridor(roots[1])
            .unwrap(),
    ];
    assert_eq!(
        certify_persistent_skew_cylinder_open_span(
            forward.residual_certificate(),
            mixed_corridors,
            canonical_points,
            PersistentSkewCylinderOpenSpanOrientation::Forward,
        ),
        Err(IntersectionCertificateError::InvalidTraceFamily)
    );
}

#[test]
fn persistent_open_span_graph_binding_is_ordered_atomic_and_protected() {
    let (cylinders, certificate) =
        persistent_fixture(PersistentSkewCylinderOpenSpanOrientation::Forward);
    let mut graph = GeometryGraph::new();
    let sources = [
        graph.insert_surface(cylinders[0]).unwrap(),
        graph.insert_surface(cylinders[1]).unwrap(),
    ];
    let pcurve_values = certificate.pcurves();
    let pcurves = [
        graph.insert_curve2d(pcurve_values[0]).unwrap(),
        graph.insert_curve2d(pcurve_values[1]).unwrap(),
    ];
    let curve_count = graph.curve_count();
    let altered_source = graph
        .insert_surface(Cylinder::new(Frame::world(), 1.125).unwrap())
        .unwrap();

    assert!(matches!(
        graph.insert_verified_skew_cylinder_open_span_curve(
            [sources[1], sources[0]],
            pcurves,
            certificate,
        ),
        Err(GeometryGraphError::InvalidDescriptor { .. })
    ));
    assert!(matches!(
        graph.insert_verified_skew_cylinder_open_span_curve(
            [altered_source, sources[1]],
            pcurves,
            certificate,
        ),
        Err(GeometryGraphError::InvalidDescriptor { .. })
    ));
    assert!(matches!(
        graph.insert_verified_skew_cylinder_open_span_curve(
            sources,
            [pcurves[1], pcurves[0]],
            certificate,
        ),
        Err(GeometryGraphError::InvalidDescriptor { .. })
    ));
    let stale = graph.insert_curve2d(pcurve_values[0]).unwrap();
    graph.remove_curve2d(stale).unwrap();
    assert!(matches!(
        graph.insert_verified_skew_cylinder_open_span_curve(
            sources,
            [stale, pcurves[1]],
            certificate,
        ),
        Err(GeometryGraphError::StaleGeometryHandle { .. })
    ));
    assert_eq!(graph.curve_count(), curve_count);

    let curve = graph
        .insert_verified_skew_cylinder_open_span_curve(sources, pcurves, certificate)
        .unwrap();
    assert_eq!(
        graph
            .direct_dependencies(GeometryRef::Curve(curve))
            .unwrap(),
        vec![
            GeometryRef::Surface(sources[0]),
            GeometryRef::Surface(sources[1]),
            GeometryRef::Curve2d(pcurves[0]),
            GeometryRef::Curve2d(pcurves[1]),
        ]
    );
    let descriptor = graph
        .curve(curve)
        .unwrap()
        .as_persistent_skew_cylinder_open_span()
        .copied()
        .unwrap();
    assert_eq!(descriptor.source_surfaces(), sources);
    assert_eq!(descriptor.pcurves(), pcurves);
    assert_eq!(descriptor.certificate(), certificate);

    let mut eval = EvalContext::new(&graph, EvalLimits::default(), Tolerances::default());
    assert_eq!(eval.curve_param_range(curve), Ok(ParamRange::new(0.0, 1.0)));
    assert_eq!(
        eval.eval_curve(curve, 1.01, 0),
        Err(EvalError::ParameterOutsideDomain)
    );
    for parameter in [0.0, 0.31, 1.0] {
        assert_eq!(
            eval.eval_curve(curve, parameter, 1).unwrap(),
            certificate.carrier().eval_derivs(parameter, 1)
        );
    }
    let dependent = vec![GeometryRef::Curve(curve)];
    assert_eq!(
        graph.replace_surface(sources[0], cylinders[0]),
        Err(GeometryGraphError::HasDependents {
            geometry: GeometryRef::Surface(sources[0]),
            dependents: dependent.clone(),
        })
    );
    assert_eq!(
        graph.replace_curve2d(pcurves[0], pcurve_values[0]),
        Err(GeometryGraphError::HasDependents {
            geometry: GeometryRef::Curve2d(pcurves[0]),
            dependents: dependent,
        })
    );
    graph.validate().unwrap();
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
