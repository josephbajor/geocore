use super::*;

use crate::GeometryGraph;
use kgeom::curve::Curve;
use kgeom::curve2d::Curve2d;
use kgeom::frame::Frame;
use kgeom::surface::Cylinder;
use kgeom::vec::Vec3;

#[derive(Clone, Copy)]
struct SpanParts {
    residual: PairedSkewCylinderBranchResidualCertificate,
    corridors: [SkewCylinderBranchPcurveRootCorridorCertificate; 2],
    roots: [Interval; 2],
}

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

fn reflected_fixture() -> ([Cylinder; 2], [[ParamRange; 2]; 2]) {
    let first = Cylinder::new(
        Frame::new(
            Vec3::default(),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let second = Cylinder::new(
        Frame::new(
            Vec3::default(),
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.0, -1.0, 0.0),
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

fn narrow_root(parameter: f64) -> Interval {
    Interval::new(parameter.next_down(), parameter.next_up())
}

fn span_parts(
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
    guarded: ParamRange,
    roots: [Interval; 2],
    sheet: SkewCylinderSheet,
    tolerance: f64,
) -> SpanParts {
    let residual = certify_paired_skew_cylinder_branch_subrange_residuals(
        cylinders, ranges, guarded, sheet, tolerance,
    )
    .unwrap();
    let corridors = [
        residual
            .certify_lower_pcurve_root_corridor(roots[0])
            .unwrap(),
        residual
            .certify_upper_pcurve_root_corridor(roots[1])
            .unwrap(),
    ];
    SpanParts {
        residual,
        corridors,
        roots,
    }
}

fn root_midpoints(roots: [Interval; 2]) -> [f64; 2] {
    roots.map(|root| 0.5 * root.lo() + 0.5 * root.hi())
}

fn certify_span(
    parts: SpanParts,
    orientation: PersistentSkewCylinderOpenSpanOrientation,
    endpoint_points: Option<[Vec3; 2]>,
) -> PersistentSkewCylinderOpenSpanCertificate {
    let points = endpoint_points.unwrap_or_else(|| {
        root_midpoints(parts.roots).map(|parameter| parts.residual.carrier().eval(parameter))
    });
    certify_persistent_skew_cylinder_open_span(parts.residual, parts.corridors, points, orientation)
        .unwrap()
}

fn ordinary_span(
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
    guarded: ParamRange,
    roots: [f64; 2],
    sheet: SkewCylinderSheet,
    orientation: PersistentSkewCylinderOpenSpanOrientation,
) -> PersistentSkewCylinderOpenSpanCertificate {
    certify_span(
        span_parts(
            cylinders,
            ranges,
            guarded,
            roots.map(narrow_root),
            sheet,
            1.0e-8,
        ),
        orientation,
        None,
    )
}

fn insert_sources(graph: &mut GeometryGraph, cylinders: [Cylinder; 2]) -> [SurfaceHandle; 2] {
    [
        graph.insert_surface(cylinders[0]).unwrap(),
        graph.insert_surface(cylinders[1]).unwrap(),
    ]
}

fn bind(
    graph: &mut GeometryGraph,
    sources: [SurfaceHandle; 2],
    certificate: PersistentSkewCylinderOpenSpanCertificate,
) -> VerifiedSkewCylinderOpenSpanCurveDescriptor {
    let values = certificate.pcurves();
    let pcurves = [
        graph.insert_curve2d(values[0]).unwrap(),
        graph.insert_curve2d(values[1]).unwrap(),
    ];
    let curve = graph
        .insert_verified_skew_cylinder_open_span_curve(sources, pcurves, certificate)
        .unwrap();
    graph
        .curve(curve)
        .unwrap()
        .as_persistent_skew_cylinder_open_span()
        .copied()
        .unwrap()
}

fn disjoint_request(
    order: PersistentSkewCylinderSpanRangeOrder,
) -> PersistentSkewCylinderSpanRelationshipRequest {
    PersistentSkewCylinderSpanRelationshipRequest::DisjointRange { order }
}

#[test]
fn disjoint_ranges_certify_distinct_handles_replay_swap_and_tamper_rejection() {
    let (cylinders, ranges) = fixture();
    let first_certificate = ordinary_span(
        cylinders,
        ranges,
        ParamRange::new(0.20, 0.45),
        [0.15, 0.50],
        SkewCylinderSheet::Upper,
        PersistentSkewCylinderOpenSpanOrientation::Forward,
    );
    let second_certificate = ordinary_span(
        cylinders,
        ranges,
        ParamRange::new(0.55, 0.80),
        [0.52, 0.85],
        SkewCylinderSheet::Upper,
        PersistentSkewCylinderOpenSpanOrientation::Reversed,
    );
    assert_eq!(first_certificate.work(), 260);
    assert_eq!(second_certificate.work(), 260);

    let mut graph = GeometryGraph::new();
    let first_sources = insert_sources(&mut graph, cylinders);
    let second_sources = insert_sources(&mut graph, cylinders);
    assert_ne!(first_sources, second_sources);
    let first = bind(&mut graph, first_sources, first_certificate);
    let second = bind(&mut graph, second_sources, second_certificate);
    let relation = certify_persistent_skew_cylinder_span_relationship(
        first,
        second,
        disjoint_request(PersistentSkewCylinderSpanRangeOrder::FirstBeforeSecond),
    )
    .unwrap();
    assert_eq!(
        relation.span_source_surfaces(),
        [first_sources, second_sources]
    );
    let PersistentSkewCylinderSpanRelationshipKind::DisjointRange {
        angular_gap_lower,
        radial_chord_lower,
        metric_clearance_lower,
        ..
    } = relation.kind();
    assert!(angular_gap_lower > 0.0 && angular_gap_lower <= core::f64::consts::PI);
    assert!(radial_chord_lower > metric_clearance_lower && metric_clearance_lower > 0.0);
    assert_eq!(
        relation,
        certify_persistent_skew_cylinder_span_relationship(
            first,
            second,
            disjoint_request(PersistentSkewCylinderSpanRangeOrder::FirstBeforeSecond),
        )
        .unwrap()
    );

    let swapped = certify_persistent_skew_cylinder_span_relationship(
        second,
        first,
        disjoint_request(PersistentSkewCylinderSpanRangeOrder::SecondBeforeFirst),
    )
    .unwrap();
    let PersistentSkewCylinderSpanRelationshipKind::DisjointRange {
        angular_gap_lower: swapped_gap,
        radial_chord_lower: swapped_chord,
        metric_clearance_lower: swapped_clearance,
        ..
    } = swapped.kind();
    assert_eq!(angular_gap_lower.to_bits(), swapped_gap.to_bits());
    assert_eq!(radial_chord_lower.to_bits(), swapped_chord.to_bits());
    assert_eq!(
        metric_clearance_lower.to_bits(),
        swapped_clearance.to_bits()
    );
    assert_eq!(
        swapped.span_directed_chart_integrals(),
        [
            relation.span_directed_chart_integrals()[1],
            relation.span_directed_chart_integrals()[0],
        ]
    );

    let mut orientation_tamper = second_certificate;
    orientation_tamper.orientation = PersistentSkewCylinderOpenSpanOrientation::Forward;
    let orientation_descriptor = VerifiedSkewCylinderOpenSpanCurveDescriptor::new(
        second_sources,
        second.pcurves(),
        orientation_tamper,
    );
    assert_eq!(
        certify_persistent_skew_cylinder_span_relationship(
            first,
            orientation_descriptor,
            disjoint_request(PersistentSkewCylinderSpanRangeOrder::FirstBeforeSecond),
        ),
        Err(PersistentSkewCylinderSpanRelationshipError::InvalidSealedSpan)
    );

    let mut corridor_tamper = second_certificate;
    corridor_tamper.root_corridors[0] = first_certificate.root_corridors[0];
    let corridor_descriptor = VerifiedSkewCylinderOpenSpanCurveDescriptor::new(
        second_sources,
        second.pcurves(),
        corridor_tamper,
    );
    assert_eq!(
        certify_persistent_skew_cylinder_span_relationship(
            first,
            corridor_descriptor,
            disjoint_request(PersistentSkewCylinderSpanRangeOrder::FirstBeforeSecond),
        ),
        Err(PersistentSkewCylinderSpanRelationshipError::InvalidSealedSpan)
    );

    let mut integral_tamper = second_certificate;
    integral_tamper.directed_chart_integrals[0].stored = Interval::point(f64::INFINITY);
    let integral_descriptor = VerifiedSkewCylinderOpenSpanCurveDescriptor::new(
        second_sources,
        second.pcurves(),
        integral_tamper,
    );
    assert_eq!(
        certify_persistent_skew_cylinder_span_relationship(
            first,
            integral_descriptor,
            disjoint_request(PersistentSkewCylinderSpanRangeOrder::FirstBeforeSecond),
        ),
        Err(PersistentSkewCylinderSpanRelationshipError::InvalidSealedSpan)
    );

    let mut endpoint_tamper = second_certificate;
    endpoint_tamper.endpoint_points[0] = first_certificate.endpoint_points[0];
    let endpoint_descriptor = VerifiedSkewCylinderOpenSpanCurveDescriptor::new(
        second_sources,
        second.pcurves(),
        endpoint_tamper,
    );
    assert_eq!(
        certify_persistent_skew_cylinder_span_relationship(
            first,
            endpoint_descriptor,
            disjoint_request(PersistentSkewCylinderSpanRangeOrder::FirstBeforeSecond),
        ),
        Err(PersistentSkewCylinderSpanRelationshipError::EndpointRelationMismatch)
    );
}

#[test]
fn disjoint_ranges_accept_different_sheets() {
    let (cylinders, ranges) = fixture();
    let first_certificate = ordinary_span(
        cylinders,
        ranges,
        ParamRange::new(0.20, 0.45),
        [0.15, 0.50],
        SkewCylinderSheet::Upper,
        PersistentSkewCylinderOpenSpanOrientation::Forward,
    );
    let second_certificate = ordinary_span(
        cylinders,
        ranges,
        ParamRange::new(0.55, 0.80),
        [0.52, 0.85],
        SkewCylinderSheet::Lower,
        PersistentSkewCylinderOpenSpanOrientation::Forward,
    );
    let mut graph = GeometryGraph::new();
    let sources = insert_sources(&mut graph, cylinders);
    let first = bind(&mut graph, sources, first_certificate);
    let second = bind(&mut graph, sources, second_certificate);
    assert!(matches!(
        certify_persistent_skew_cylinder_span_relationship(
            first,
            second,
            disjoint_request(PersistentSkewCylinderSpanRangeOrder::FirstBeforeSecond),
        )
        .unwrap()
        .kind(),
        PersistentSkewCylinderSpanRelationshipKind::DisjointRange { .. }
    ));
}

#[test]
fn lower_sheet_integral_applies_the_chart_lift_once_and_reflection_stays_certified() {
    let (cylinders, ranges) = fixture();
    let first_certificate = ordinary_span(
        cylinders,
        ranges,
        ParamRange::new(0.20, 0.45),
        [0.15, 0.50],
        SkewCylinderSheet::Lower,
        PersistentSkewCylinderOpenSpanOrientation::Forward,
    );
    let second_certificate = ordinary_span(
        cylinders,
        ranges,
        ParamRange::new(0.55, 0.80),
        [0.52, 0.85],
        SkewCylinderSheet::Lower,
        PersistentSkewCylinderOpenSpanOrientation::Forward,
    );
    let mut graph = GeometryGraph::new();
    let sources = insert_sources(&mut graph, cylinders);
    let first = bind(&mut graph, sources, first_certificate);
    let second = bind(&mut graph, sources, second_certificate);
    let relation = certify_persistent_skew_cylinder_span_relationship(
        first,
        second,
        disjoint_request(PersistentSkewCylinderSpanRangeOrder::FirstBeforeSecond),
    )
    .unwrap();
    let pcurve = first_certificate.pcurves()[1];
    assert!(pcurve.eval(0.5).x > core::f64::consts::PI);
    let oracle = simpson_directed_integral(pcurve);
    let witness = relation.span_directed_chart_integrals()[0][1];
    assert!(witness.stored_enclosure().contains(oracle));
    assert!(witness.source_enclosure().contains(oracle));
    let ordinate_delta = pcurve.eval(1.0).y - pcurve.eval(0.0).y;
    let double_applied = oracle + TAU * ordinate_delta;
    assert!(!witness.stored_enclosure().contains(double_applied));
    assert!(!witness.source_enclosure().contains(double_applied));

    let (reflected, reflected_ranges) = reflected_fixture();
    let reflected_first = ordinary_span(
        reflected,
        reflected_ranges,
        ParamRange::new(0.20, 0.45),
        [0.15, 0.50],
        SkewCylinderSheet::Lower,
        PersistentSkewCylinderOpenSpanOrientation::Forward,
    );
    let reflected_second = ordinary_span(
        reflected,
        reflected_ranges,
        ParamRange::new(0.55, 0.80),
        [0.52, 0.85],
        SkewCylinderSheet::Lower,
        PersistentSkewCylinderOpenSpanOrientation::Forward,
    );
    let mut reflected_graph = GeometryGraph::new();
    let reflected_sources = insert_sources(&mut reflected_graph, reflected);
    let reflected_relation = certify_persistent_skew_cylinder_span_relationship(
        bind(&mut reflected_graph, reflected_sources, reflected_first),
        bind(&mut reflected_graph, reflected_sources, reflected_second),
        disjoint_request(PersistentSkewCylinderSpanRangeOrder::FirstBeforeSecond),
    )
    .unwrap();
    assert!(matches!(
        reflected_relation.kind(),
        PersistentSkewCylinderSpanRelationshipKind::DisjointRange {
            metric_clearance_lower,
            ..
        } if metric_clearance_lower > 0.0
    ));
}

fn simpson_directed_integral(pcurve: PersistentSkewCylinderOpenSpanPcurve) -> f64 {
    const PANELS: usize = 4096;
    let integrand = |parameter: f64| {
        let derivatives = pcurve.eval_derivs(parameter, 1);
        derivatives.d[0].x * derivatives.d[1].y - derivatives.d[0].y * derivatives.d[1].x
    };
    let mut weighted = integrand(0.0) + integrand(1.0);
    for index in 1..PANELS {
        let weight = if index % 2 == 0 { 2.0 } else { 4.0 };
        weighted += weight * integrand(index as f64 / PANELS as f64);
    }
    weighted / (3.0 * PANELS as f64)
}

#[test]
fn full_root_enclosures_and_exact_geometry_fail_closed() {
    let (cylinders, ranges) = fixture();
    let first = certify_span(
        span_parts(
            cylinders,
            ranges,
            ParamRange::new(0.20, 0.45),
            [Interval::new(0.10, 0.15), Interval::new(0.46, 0.54)],
            SkewCylinderSheet::Upper,
            1.0e-8,
        ),
        PersistentSkewCylinderOpenSpanOrientation::Forward,
        None,
    );
    let second = certify_span(
        span_parts(
            cylinders,
            ranges,
            ParamRange::new(0.55, 0.80),
            [Interval::new(0.49, 0.54), Interval::new(0.85, 0.90)],
            SkewCylinderSheet::Upper,
            1.0e-8,
        ),
        PersistentSkewCylinderOpenSpanOrientation::Forward,
        None,
    );
    let first_midpoint_end =
        root_midpoints(first.root_corridors().map(|root| root.root_parameter()))[1];
    let second_midpoint_start =
        root_midpoints(second.root_corridors().map(|root| root.root_parameter()))[0];
    assert!(first_midpoint_end < second_midpoint_start);
    assert!(
        first.root_corridors()[1].root_parameter().hi()
            > second.root_corridors()[0].root_parameter().lo()
    );
    let mut graph = GeometryGraph::new();
    let sources = insert_sources(&mut graph, cylinders);
    let first_descriptor = bind(&mut graph, sources, first);
    let second_descriptor = bind(&mut graph, sources, second);
    assert_eq!(
        certify_persistent_skew_cylinder_span_relationship(
            first_descriptor,
            second_descriptor,
            disjoint_request(PersistentSkewCylinderSpanRangeOrder::FirstBeforeSecond),
        ),
        Err(PersistentSkewCylinderSpanRelationshipError::RangeRelationMismatch)
    );

    let changed = [Cylinder::new(Frame::world(), 1.125).unwrap(), cylinders[1]];
    let changed_certificate = ordinary_span(
        changed,
        ranges,
        ParamRange::new(0.55, 0.80),
        [0.52, 0.85],
        SkewCylinderSheet::Upper,
        PersistentSkewCylinderOpenSpanOrientation::Forward,
    );
    let changed_sources = insert_sources(&mut graph, changed);
    let changed_descriptor = bind(&mut graph, changed_sources, changed_certificate);
    assert_eq!(
        certify_persistent_skew_cylinder_span_relationship(
            first_descriptor,
            changed_descriptor,
            disjoint_request(PersistentSkewCylinderSpanRangeOrder::FirstBeforeSecond),
        ),
        Err(PersistentSkewCylinderSpanRelationshipError::SourceOrderMismatch)
    );

    let mut shifted_chart = ranges;
    shifted_chart[1][0] = ParamRange::new(-core::f64::consts::PI, core::f64::consts::PI);
    let shifted = ordinary_span(
        cylinders,
        shifted_chart,
        ParamRange::new(0.55, 0.80),
        [0.52, 0.85],
        SkewCylinderSheet::Upper,
        PersistentSkewCylinderOpenSpanOrientation::Forward,
    );
    let shifted_descriptor = bind(&mut graph, sources, shifted);
    assert_eq!(
        certify_persistent_skew_cylinder_span_relationship(
            first_descriptor,
            shifted_descriptor,
            disjoint_request(PersistentSkewCylinderSpanRangeOrder::FirstBeforeSecond),
        ),
        Err(PersistentSkewCylinderSpanRelationshipError::ChartMismatch)
    );
}
