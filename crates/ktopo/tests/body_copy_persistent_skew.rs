//! Proof-safe lower rigid-copy regression for family-bound skew composites.
//! Wall-time budget: less than 30 seconds for one oblique family reissue.

use kgeom::curve::Curve;
use kgeom::curve2d::Curve2d;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Surface};
use kgeom::vec::{Point3, Vec3};
use kgraph::{
    PersistentSkewCylinderFiniteWindowMemberInput, PersistentSkewCylinderOpenSpanCertificate,
    PersistentSkewCylinderOpenSpanOrientation, SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
    SkewCylinderAxialBoundProvenance, SkewCylinderAxialBoundary,
    SkewCylinderExactDiscriminantTopology, SkewCylinderFiniteSheetTopology,
    SkewCylinderOpenSpanTopologyInput, SkewCylinderSheet,
    certify_paired_skew_cylinder_branch_subrange_residuals,
    certify_persistent_skew_cylinder_finite_window_family,
    certify_persistent_skew_cylinder_open_span_in_family, classify_skew_cylinder_axial_bound,
    classify_skew_cylinder_exact_discriminant, classify_skew_cylinder_open_spans,
};
use ktopo::analytic_shell::{
    AnalyticEdgeKey, AnalyticFaceKey, AnalyticShellFace, AnalyticShellFin, AnalyticShellInput,
    AnalyticShellLoop, AnalyticShellSkewCylinderOpenSpan, AnalyticShellSurface, AnalyticVertexKey,
};
use ktopo::check::{CheckLevel, CheckOutcome, check_body_report};
use ktopo::entity::{EntityRef, FaceDomain, PcurveChart, Sense};
use ktopo::store::Store;
use ktopo::transaction::LineageEvent;

const TOLERANCE: f64 = 1.0e-8;
const V0: AnalyticVertexKey = AnalyticVertexKey::new(0);
const V1: AnalyticVertexKey = AnalyticVertexKey::new(1);
const E0: AnalyticEdgeKey = AnalyticEdgeKey::new(0);
const E1: AnalyticEdgeKey = AnalyticEdgeKey::new(1);
const F0: AnalyticFaceKey = AnalyticFaceKey::new(0);
const F1: AnalyticFaceKey = AnalyticFaceKey::new(1);

#[test]
fn family_bound_persistent_skew_copy_reissues_every_live_dependency() {
    let certificate = family_bound_certificate();
    let mut store = Store::new();
    let source = assemble_scaffold(&mut store, certificate);
    let placement = Frame::new(
        Point3::new(-2.5, 1.25, 3.75),
        Vec3::new(1.0, 2.0, 3.0).normalized().unwrap(),
        Vec3::new(2.0, -1.0, 0.0).normalized().unwrap(),
    )
    .unwrap();

    let mut transaction = store.transaction().unwrap();
    let copied = transaction
        .copy_body_rigid_with_source(source, placement)
        .unwrap();
    let journal = transaction.commit_checked_body(copied).unwrap();

    assert_ne!(copied, source);
    assert_eq!(journal.lineage().len(), journal.mutations().len());
    assert!(journal.lineage().iter().all(|event| matches!(
        event,
        LineageEvent::DerivedFrom { derived, source } if derived != source
    )));
    assert!(journal.lineage().iter().any(|event| matches!(
        event,
        LineageEvent::DerivedFrom {
            derived: EntityRef::Body(derived),
            source: EntityRef::Body(original),
        } if *derived == copied && *original == source
    )));
    assert_eq!(
        check_body_report(&store, source, CheckLevel::Fast)
            .unwrap()
            .outcome(),
        CheckOutcome::Valid
    );
    assert_eq!(
        check_body_report(&store, copied, CheckLevel::Fast)
            .unwrap()
            .outcome(),
        CheckOutcome::Valid
    );
    assert_reissued_geometry(&store, source, copied, placement);
    store.geometry().validate().unwrap();
}

fn family_bound_certificate() -> PersistentSkewCylinderOpenSpanCertificate {
    let cylinders = [
        Cylinder::new(Frame::world(), 1.0).unwrap(),
        Cylinder::new(
            Frame::new(
                Point3::default(),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            )
            .unwrap(),
            2.0,
        )
        .unwrap(),
    ];
    let ranges = [
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(1.8, 1.9),
        ],
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(-1.25, 1.25),
        ],
    ];
    let topologies = [
        (0, SkewCylinderAxialBoundary::Lower, ranges[0][1].lo),
        (0, SkewCylinderAxialBoundary::Upper, ranges[0][1].hi),
        (1, SkewCylinderAxialBoundary::Lower, ranges[1][1].lo),
        (1, SkewCylinderAxialBoundary::Upper, ranges[1][1].hi),
    ]
    .map(|(source_operand, boundary, value)| {
        classify_skew_cylinder_axial_bound(
            cylinders,
            [0, 1],
            SkewCylinderAxialBoundProvenance {
                source_operand,
                boundary,
                value,
            },
            SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
        )
        .unwrap()
    });
    let topology = classify_skew_cylinder_open_spans(SkewCylinderOpenSpanTopologyInput {
        topologies: &topologies,
        ranges,
        canonical_to_source: [0, 1],
    })
    .unwrap();
    let mut spans = Vec::new();
    for sheet in [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper] {
        if let SkewCylinderFiniteSheetTopology::Open(open) = topology.sheet(sheet) {
            spans.extend(open.iter().copied());
        }
    }
    let members = spans
        .into_iter()
        .map(|span| {
            let residual = certify_paired_skew_cylinder_branch_subrange_residuals(
                cylinders, ranges, span.range, span.sheet, TOLERANCE,
            )
            .unwrap();
            let roots = span.root_longitude_intervals(ranges[0][0]).unwrap();
            PersistentSkewCylinderFiniteWindowMemberInput {
                residual,
                root_corridors: [
                    residual
                        .certify_lower_pcurve_root_corridor(roots[0])
                        .unwrap(),
                    residual
                        .certify_upper_pcurve_root_corridor(roots[1])
                        .unwrap(),
                ],
            }
        })
        .collect::<Vec<_>>();
    let SkewCylinderExactDiscriminantTopology::StrictPositive(admission) =
        classify_skew_cylinder_exact_discriminant(cylinders, SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK)
            .unwrap()
    else {
        panic!("fixture must retain two strict-positive sheets");
    };
    let family = certify_persistent_skew_cylinder_finite_window_family(
        admission, &topology, &members, TOLERANCE,
    )
    .unwrap();
    let member = members[0];
    let endpoints = member.root_corridors.map(|corridor| {
        let root = corridor.root_parameter();
        member
            .residual
            .carrier()
            .eval(0.5 * root.lo() + 0.5 * root.hi())
    });
    certify_persistent_skew_cylinder_open_span_in_family(
        member.residual,
        member.root_corridors,
        endpoints,
        PersistentSkewCylinderOpenSpanOrientation::Forward,
        family.membership(0).unwrap(),
    )
    .unwrap()
}

fn assemble_scaffold(
    store: &mut Store,
    certificate: PersistentSkewCylinderOpenSpanCertificate,
) -> ktopo::entity::BodyId {
    let first = AnalyticShellSkewCylinderOpenSpan::new(E0, [V0, V1], certificate);
    let second = AnalyticShellSkewCylinderOpenSpan::new(E1, [V0, V1], certificate);
    let cylinders = certificate.carrier().cylinders();
    let pcurves = first.pcurves();
    let charts = [PcurveChart::identity(), PcurveChart::shifted([1, 0])];
    let uses = [
        pcurves[0].with_chart(charts[0]),
        pcurves[1].with_chart(charts[1]),
    ];
    let faces = vec![
        AnalyticShellFace::new(
            F0,
            AnalyticShellSurface::Cylinder(cylinders[0]),
            Sense::Forward,
            pcurve_domain(certificate.pcurves()[0], charts[0], cylinders[0]),
            vec![AnalyticShellLoop::new(vec![
                AnalyticShellFin::new(E0, Sense::Forward, uses[0]),
                AnalyticShellFin::new(E1, Sense::Reversed, uses[0]),
            ])],
        ),
        AnalyticShellFace::new(
            F1,
            AnalyticShellSurface::Cylinder(cylinders[1]),
            Sense::Forward,
            pcurve_domain(certificate.pcurves()[1], charts[1], cylinders[1]),
            vec![AnalyticShellLoop::new(vec![
                AnalyticShellFin::new(E0, Sense::Reversed, uses[1]),
                AnalyticShellFin::new(E1, Sense::Forward, uses[1]),
            ])],
        ),
    ];
    let input = AnalyticShellInput::new(
        first.vertices().to_vec(),
        vec![first.edge(), second.edge()],
        faces,
    );
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&input, 1.0e-12)
        .unwrap();
    let body = output.body();
    transaction.commit_checked_body(body).unwrap();
    body
}

fn pcurve_domain(
    pcurve: kgraph::PersistentSkewCylinderOpenSpanPcurve,
    chart: PcurveChart,
    cylinder: Cylinder,
) -> FaceDomain {
    let bounds = pcurve.bounding_box(ParamRange::new(0.0, 1.0));
    let periods = cylinder.periodicity();
    let min = chart.apply(bounds.min, periods).unwrap();
    let max = chart.apply(bounds.max, periods).unwrap();
    FaceDomain::from_bounds(min.x, max.x, min.y, max.y).unwrap()
}

fn assert_reissued_geometry(
    store: &Store,
    source: ktopo::entity::BodyId,
    copied: ktopo::entity::BodyId,
    placement: Frame,
) {
    let source_edges = store.edges_of_body(source).unwrap();
    let copied_edges = store.edges_of_body(copied).unwrap();
    assert_eq!(source_edges.len(), 2);
    assert_eq!(copied_edges.len(), source_edges.len());
    for (source_edge, copied_edge) in source_edges.into_iter().zip(copied_edges) {
        let source_curve = store.get(source_edge).unwrap().curve().unwrap();
        let copied_curve = store.get(copied_edge).unwrap().curve().unwrap();
        assert_ne!(source_curve, copied_curve);
        let source_descriptor = store
            .curve(source_curve)
            .unwrap()
            .as_persistent_skew_cylinder_open_span()
            .unwrap();
        let copied_descriptor = store
            .curve(copied_curve)
            .unwrap()
            .as_persistent_skew_cylinder_open_span()
            .unwrap();
        let source_certificate = source_descriptor.certificate();
        let copied_certificate = copied_descriptor.certificate();
        assert_ne!(
            source_certificate
                .finite_window_family_membership()
                .unwrap()
                .family(),
            copied_certificate
                .finite_window_family_membership()
                .unwrap()
                .family()
        );
        assert_eq!(
            source_certificate
                .finite_window_family_membership()
                .unwrap()
                .ordinal(),
            copied_certificate
                .finite_window_family_membership()
                .unwrap()
                .ordinal()
        );
        for parameter in [0.0, 0.37, 1.0] {
            let source_point = source_certificate.carrier().eval(parameter);
            let expected = placement.point_at(source_point.x, source_point.y, source_point.z);
            assert!(copied_certificate.carrier().eval(parameter).dist(expected) <= 1.0e-12);
        }
        for index in 0..2 {
            assert_ne!(
                source_descriptor.source_surfaces()[index],
                copied_descriptor.source_surfaces()[index]
            );
            assert_ne!(
                source_descriptor.pcurves()[index],
                copied_descriptor.pcurves()[index]
            );
            assert_eq!(
                store
                    .get(copied_descriptor.pcurves()[index])
                    .unwrap()
                    .as_persistent_skew_cylinder_open_span()
                    .copied(),
                Some(copied_certificate.pcurves()[index])
            );
            assert_eq!(
                store
                    .get(copied_descriptor.source_surfaces()[index])
                    .unwrap()
                    .as_cylinder()
                    .copied(),
                Some(
                    copied_certificate
                        .finite_window_family_membership()
                        .unwrap()
                        .family()
                        .source_cylinders()[index]
                )
            );
        }
    }
}
