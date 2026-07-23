use super::assemble::AnalyticShellAssemblyError;
use super::*;
use crate::check::{CheckLevel, CheckOutcome, VerificationGapKind, check_body_report};
use crate::entity::{
    Body, Edge, EntityRef, Face, Fin, FinPcurve, Loop, ParamMap1d, PcurveChart, PcurveEndpointKind,
    PcurveSeam, Region, SeamSide, Sense, Shell, SurfaceParameter, Vertex,
};
use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use crate::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};
use crate::store::Store;
use crate::tolerance::{EntityTolerance, ToleranceOrigin};
use crate::transaction::{FullCommitRequirement, MutationKind, Transaction};
use kcore::interval::Interval;
use kcore::math;
use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::curve::Line;
use kgeom::curve2d::Curve2d;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Surface};
use kgeom::vec::{Point3, Vec3};
use kgraph::{
    PersistentSkewCylinderOpenSpanCertificate, PersistentSkewCylinderOpenSpanOrientation,
    SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS, SkewCylinderSheet,
    certify_paired_skew_cylinder_branch_subrange_residuals,
    certify_persistent_skew_cylinder_open_span,
};

const V0: AnalyticVertexKey = AnalyticVertexKey::new(0);
const V1: AnalyticVertexKey = AnalyticVertexKey::new(1);
const E0: AnalyticEdgeKey = AnalyticEdgeKey::new(0);
const E1: AnalyticEdgeKey = AnalyticEdgeKey::new(1);
const F0: AnalyticFaceKey = AnalyticFaceKey::new(0);
const F1: AnalyticFaceKey = AnalyticFaceKey::new(1);

fn persistent_certificate(endpoint_offset: f64) -> PersistentSkewCylinderOpenSpanCertificate {
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
    let cylinders = [first, second];
    let ranges = [
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(1.8, 2.1),
        ],
        [
            ParamRange::new(0.0, core::f64::consts::TAU),
            ParamRange::new(-1.25, 0.0),
        ],
    ];
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
        if endpoint_offset == 0.0 {
            LINEAR_RESOLUTION
        } else {
            1.0e-6
        },
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
        Vec3::new(cosine + endpoint_offset, sine, (4.0 - sine * sine).sqrt())
    });
    certify_persistent_skew_cylinder_open_span(
        residual,
        corridors,
        endpoint_points,
        PersistentSkewCylinderOpenSpanOrientation::Forward,
    )
    .unwrap()
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

fn scaffold_input() -> (
    AnalyticShellInput,
    PersistentSkewCylinderOpenSpanCertificate,
) {
    scaffold_input_with_certificate(persistent_certificate(4.0 * LINEAR_RESOLUTION))
}

fn scaffold_input_with_certificate(
    certificate: PersistentSkewCylinderOpenSpanCertificate,
) -> (
    AnalyticShellInput,
    PersistentSkewCylinderOpenSpanCertificate,
) {
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
    (
        AnalyticShellInput::new(
            first.vertices().to_vec(),
            vec![first.edge(), second.edge()],
            faces,
        ),
        certificate,
    )
}

fn with_persistent_incidence_scaffold(
    test: impl FnOnce(
        &mut Transaction<'_>,
        &AnalyticShellOutput,
        PersistentSkewCylinderOpenSpanCertificate,
    ),
) {
    let (input, certificate) = scaffold_input();
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&input, 1.0e-12)
        .unwrap();
    test(&mut transaction, &output, certificate);
    transaction.rollback().unwrap();
}

fn persistent_use(
    store: &Store,
    output: &AnalyticShellOutput,
    source_slot: usize,
) -> (
    crate::entity::EdgeId,
    crate::entity::FaceId,
    crate::entity::LoopId,
    crate::entity::FinId,
) {
    let edge_id = output.edges()[0].1;
    let edge = store.get(edge_id).unwrap();
    let descriptor = store
        .curve(edge.curve().unwrap())
        .unwrap()
        .as_persistent_skew_cylinder_open_span()
        .unwrap();
    let source = descriptor.source_surfaces()[source_slot];
    let fin_id = edge
        .fins()
        .iter()
        .copied()
        .find(|fin| {
            let loop_id = store.get(*fin).unwrap().parent();
            let face_id = store.get(loop_id).unwrap().face();
            store.get(face_id).unwrap().surface() == source
        })
        .unwrap();
    let loop_id = store.get(fin_id).unwrap().parent();
    let face_id = store.get(loop_id).unwrap().face();
    (edge_id, face_id, loop_id, fin_id)
}

fn assert_persistent_incidence(
    store: &Store,
    face: crate::entity::FaceId,
    loop_: crate::entity::LoopId,
    fin: crate::entity::FinId,
    expected: WholeFinIncidence,
) {
    let edge = store.get(store.get(fin).unwrap().edge()).unwrap();
    let effective = edge.tolerance().unwrap().value();
    assert_eq!(
        certify_whole_fin_incidence(store, face, loop_, fin, effective),
        expected
    );
}

#[test]
fn persistent_skew_scaffold_is_fast_valid_bound_and_journaled() {
    let (input, certificate) = scaffold_input();
    let mut permuted = input.clone();
    permuted.vertices.reverse();
    permuted.edges.reverse();
    permuted.faces.reverse();
    for face in &mut permuted.faces {
        face.loops[0].fins.rotate_left(1);
    }
    assert_eq!(
        prepare_analytic_shell(&input, &Store::new(), 1.0e-12).unwrap(),
        prepare_analytic_shell(&permuted, &Store::new(), 1.0e-12).unwrap()
    );

    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&input, 1.0e-12)
        .unwrap();
    assert_eq!(output.vertices().len(), 2);
    assert_eq!(output.edges().len(), 2);
    assert_eq!(output.faces().len(), 2);

    let face_ids = [output.faces()[0].1, output.faces()[1].1];
    let source_surfaces = face_ids.map(|face| transaction.store().get(face).unwrap().surface());
    let expected_tolerance = certificate.required_edge_tolerance().max(LINEAR_RESOLUTION);
    assert!(expected_tolerance > LINEAR_RESOLUTION);
    for &(_, edge_id) in output.edges() {
        let edge = transaction.store().get(edge_id).unwrap();
        assert_eq!(edge.bounds(), Some((0.0, 1.0)));
        assert_eq!(edge.tolerance().unwrap().value(), expected_tolerance);
        let descriptor = transaction
            .store()
            .curve(edge.curve().unwrap())
            .unwrap()
            .as_persistent_skew_cylinder_open_span()
            .copied()
            .unwrap();
        assert_eq!(descriptor.certificate(), certificate);
        assert_eq!(descriptor.source_surfaces(), source_surfaces);
        for source_index in 0..2 {
            let fin = edge
                .fins()
                .iter()
                .map(|fin| transaction.store().get(*fin).unwrap())
                .find(|fin| {
                    let loop_ = transaction.store().get(fin.parent()).unwrap();
                    transaction.store().get(loop_.face()).unwrap().surface()
                        == source_surfaces[source_index]
                })
                .unwrap();
            let pcurve = fin.pcurve().unwrap();
            assert_eq!(pcurve.range(), ParamRange::new(0.0, 1.0));
            assert_eq!(pcurve.edge_to_pcurve(), ParamMap1d::identity());
            assert_eq!(
                pcurve.chart().period_shifts(),
                [[0, 0], [1, 0]][source_index]
            );
            assert_eq!(descriptor.pcurves()[source_index], pcurve.curve());
        }
    }

    let fast = check_body_report(transaction.store(), output.body(), CheckLevel::Fast).unwrap();
    assert_eq!(fast.outcome(), CheckOutcome::Valid, "{fast:#?}");
    let full = check_body_report(transaction.store(), output.body(), CheckLevel::Full).unwrap();
    assert_eq!(full.outcome(), CheckOutcome::Indeterminate, "{full:#?}");
    assert!(full.faults.is_empty(), "{full:#?}");
    assert!(!full.gaps.is_empty());
    assert!(
        full.gaps
            .iter()
            .all(|gap| gap.kind != VerificationGapKind::PcurveSurfaceIncidence),
        "{full:#?}"
    );

    let edge_ids = output
        .edges()
        .iter()
        .map(|(_, edge)| EntityRef::Edge(*edge))
        .collect::<Vec<_>>();
    let journal = transaction.commit_checked(&[output.body()]).unwrap();
    assert_eq!(journal.tolerance_budgets().len(), 1);
    let budget = journal.tolerance_budgets()[0];
    let expected_growth = 2.0 * (expected_tolerance - LINEAR_RESOLUTION).max(0.0);
    assert_eq!(budget.operation(), "analytic-shell.skew-cylinder-composite");
    assert_eq!(budget.limit(), expected_growth);
    assert_eq!(budget.consumed(), expected_growth);
    assert_eq!(
        journal
            .tolerance_events()
            .iter()
            .map(|event| event.entity())
            .collect::<Vec<_>>(),
        edge_ids
    );
    for event in journal.tolerance_events() {
        assert_eq!(event.previous(), None);
        assert_eq!(event.current().value(), expected_tolerance);
        assert_eq!(
            event.current().origin(),
            ToleranceOrigin::Operation("analytic-shell.skew-cylinder-composite")
        );
        assert_eq!(
            event.current().last_operation(),
            Some("analytic-shell.skew-cylinder-composite")
        );
    }
    assert!(edge_ids.iter().all(|edge| {
        journal
            .mutations()
            .iter()
            .any(|mutation| mutation.entity == *edge && mutation.kind == MutationKind::Created)
    }));
}

#[test]
fn persistent_skew_whole_fin_incidence_certifies_both_sources_and_all_fins() {
    with_persistent_incidence_scaffold(|transaction, output, _| {
        let mut per_source = [0_usize; 2];
        for &(_, edge_id) in output.edges() {
            let edge = transaction.store().get(edge_id).unwrap();
            let descriptor = transaction
                .store()
                .curve(edge.curve().unwrap())
                .unwrap()
                .as_persistent_skew_cylinder_open_span()
                .copied()
                .unwrap();
            for &fin_id in edge.fins() {
                let loop_id = transaction.store().get(fin_id).unwrap().parent();
                let face_id = transaction.store().get(loop_id).unwrap().face();
                let surface = transaction.store().get(face_id).unwrap().surface();
                let source_slot = descriptor
                    .source_surfaces()
                    .iter()
                    .position(|candidate| *candidate == surface)
                    .unwrap();
                per_source[source_slot] += 1;
                assert_persistent_incidence(
                    transaction.store(),
                    face_id,
                    loop_id,
                    fin_id,
                    WholeFinIncidence::Certified,
                );
            }
        }
        assert_eq!(per_source, [2, 2]);

        let (_, face, loop_, fin) = persistent_use(transaction.store(), output, 1);
        let old = transaction.store().get(fin).unwrap().pcurve().unwrap();
        transaction.assembly().get_mut(fin).unwrap().pcurve =
            Some(old.with_chart(PcurveChart::shifted([-7, 0])));
        assert_persistent_incidence(
            transaction.store(),
            face,
            loop_,
            fin,
            WholeFinIncidence::Certified,
        );
    });
}

#[test]
fn persistent_skew_whole_fin_incidence_rejects_edge_endpoint_and_tolerance_tampers() {
    with_persistent_incidence_scaffold(|transaction, output, certificate| {
        let (edge_id, face, loop_, fin) = persistent_use(transaction.store(), output, 0);
        let original = transaction.store().get(edge_id).unwrap().clone();

        transaction.assembly().get_mut(edge_id).unwrap().bounds = Some((0.0_f64.next_up(), 1.0));
        assert_persistent_incidence(
            transaction.store(),
            face,
            loop_,
            fin,
            WholeFinIncidence::Indeterminate,
        );
        *transaction.assembly().get_mut(edge_id).unwrap() = original.clone();

        let line = transaction
            .assembly()
            .insert_curve(CurveGeom::Line(
                Line::new(Point3::default(), Vec3::new(1.0, 0.0, 0.0)).unwrap(),
            ))
            .unwrap();
        transaction.assembly().get_mut(edge_id).unwrap().curve = Some(line);
        assert_persistent_incidence(
            transaction.store(),
            face,
            loop_,
            fin,
            WholeFinIncidence::Indeterminate,
        );
        *transaction.assembly().get_mut(edge_id).unwrap() = original.clone();

        let vertices = original.vertices();
        transaction.assembly().get_mut(edge_id).unwrap().vertices = [vertices[0], vertices[0]];
        assert_persistent_incidence(
            transaction.store(),
            face,
            loop_,
            fin,
            WholeFinIncidence::Indeterminate,
        );
        *transaction.assembly().get_mut(edge_id).unwrap() = original.clone();

        let vertex = vertices[0].unwrap();
        let point = transaction.store().get(vertex).unwrap().point();
        let original_point = *transaction.store().get(point).unwrap();
        *transaction.assembly().get_mut(point).unwrap() = Point3::new(
            original_point.x.next_up(),
            original_point.y,
            original_point.z,
        );
        assert_persistent_incidence(
            transaction.store(),
            face,
            loop_,
            fin,
            WholeFinIncidence::Indeterminate,
        );
        *transaction.assembly().get_mut(point).unwrap() = original_point;

        transaction.assembly().get_mut(edge_id).unwrap().tolerance = None;
        assert_eq!(
            certify_whole_fin_incidence(
                transaction.store(),
                face,
                loop_,
                fin,
                certificate.required_edge_tolerance().max(LINEAR_RESOLUTION),
            ),
            WholeFinIncidence::Indeterminate
        );
        *transaction.assembly().get_mut(edge_id).unwrap() = original.clone();

        let required = certificate.required_edge_tolerance().max(LINEAR_RESOLUTION);
        assert!(required > LINEAR_RESOLUTION);
        transaction.assembly().get_mut(edge_id).unwrap().tolerance = Some(
            EntityTolerance::operation(
                LINEAR_RESOLUTION + (required - LINEAR_RESOLUTION) * 0.5,
                "persistent-skew-incidence-test",
            )
            .unwrap(),
        );
        assert_eq!(
            certify_whole_fin_incidence(transaction.store(), face, loop_, fin, required),
            WholeFinIncidence::Indeterminate
        );
        *transaction.assembly().get_mut(edge_id).unwrap() = original;

        transaction.assembly().remove(vertex).unwrap();
        assert_persistent_incidence(
            transaction.store(),
            face,
            loop_,
            fin,
            WholeFinIncidence::Indeterminate,
        );
    });
}

#[test]
fn persistent_skew_whole_fin_incidence_rejects_source_and_pcurve_tampers() {
    with_persistent_incidence_scaffold(|transaction, output, certificate| {
        let (_, face, loop_, fin) = persistent_use(transaction.store(), output, 1);
        let source = transaction.store().get(face).unwrap().surface();
        let old_use = transaction.store().get(fin).unwrap().pcurve().unwrap();

        let alias = transaction
            .assembly()
            .insert_surface(SurfaceGeom::Cylinder(certificate.carrier().cylinders()[1]))
            .unwrap();
        transaction.assembly().get_mut(face).unwrap().surface = alias;
        assert_persistent_incidence(
            transaction.store(),
            face,
            loop_,
            fin,
            WholeFinIncidence::Indeterminate,
        );
        transaction.assembly().get_mut(face).unwrap().surface = source;

        let live_source = transaction.store().surface(source).unwrap().clone();
        assert!(
            transaction
                .assembly()
                .replace_surface(
                    source,
                    SurfaceGeom::Cylinder(certificate.carrier().cylinders()[0]),
                )
                .is_err()
        );
        assert_eq!(transaction.store().surface(source).unwrap(), &live_source);
        assert_persistent_incidence(
            transaction.store(),
            face,
            loop_,
            fin,
            WholeFinIncidence::Certified,
        );

        let live_pcurve = transaction.store().pcurve(old_use.curve()).unwrap().clone();
        assert!(
            transaction
                .assembly()
                .replace_pcurve(old_use.curve(), Curve2dGeom::from(certificate.pcurves()[0]),)
                .is_err()
        );
        assert_eq!(
            transaction.store().pcurve(old_use.curve()).unwrap(),
            &live_pcurve
        );
        assert_persistent_incidence(
            transaction.store(),
            face,
            loop_,
            fin,
            WholeFinIncidence::Certified,
        );

        let descriptor = transaction
            .store()
            .curve(
                transaction
                    .store()
                    .get(transaction.store().get(fin).unwrap().edge())
                    .unwrap()
                    .curve()
                    .unwrap(),
            )
            .unwrap()
            .as_persistent_skew_cylinder_open_span()
            .copied()
            .unwrap();
        let wrong_handle = FinPcurve::new(
            descriptor.pcurves()[0],
            old_use.range(),
            old_use.edge_to_pcurve(),
        )
        .unwrap()
        .with_chart(old_use.chart());
        transaction.assembly().get_mut(fin).unwrap().pcurve = Some(wrong_handle);
        assert_persistent_incidence(
            transaction.store(),
            face,
            loop_,
            fin,
            WholeFinIncidence::Indeterminate,
        );
        transaction.assembly().get_mut(fin).unwrap().pcurve = Some(old_use);

        let malformed = [
            FinPcurve::new(
                old_use.curve(),
                ParamRange::new(0.0, 0.5),
                old_use.edge_to_pcurve(),
            )
            .unwrap()
            .with_chart(old_use.chart()),
            FinPcurve::new(
                old_use.curve(),
                old_use.range(),
                ParamMap1d::affine(1.0, 0.25).unwrap(),
            )
            .unwrap()
            .with_chart(old_use.chart()),
            old_use.with_endpoint_kinds([
                PcurveEndpointKind::SurfaceSingularity,
                PcurveEndpointKind::Regular,
            ]),
            old_use.with_chart(PcurveChart::shifted([0, 1])),
            old_use.with_chart(PcurveChart::shifted([i32::MAX, 0])),
            old_use.with_closure_winding([0, 0]),
            old_use.with_seam(PcurveSeam::new(SurfaceParameter::U, SeamSide::Lower)),
        ];
        for candidate in malformed {
            transaction.assembly().get_mut(fin).unwrap().pcurve = Some(candidate);
            assert_persistent_incidence(
                transaction.store(),
                face,
                loop_,
                fin,
                WholeFinIncidence::Indeterminate,
            );
        }
        transaction.assembly().get_mut(fin).unwrap().pcurve = Some(old_use);

        let residual = certificate.residual_bounds()[1];
        assert!(residual > 0.0);
        assert_eq!(
            certify_whole_fin_incidence(
                transaction.store(),
                face,
                loop_,
                fin,
                residual.next_down(),
            ),
            WholeFinIncidence::Indeterminate
        );
    });
}

#[test]
fn persistent_skew_rollback_restores_tolerances_and_future_ids() {
    let (input, certificate) = scaffold_input();
    assert!(certificate.required_edge_tolerance() > LINEAR_RESOLUTION);
    let mut store = Store::new();
    let before = counts(&store);
    let first = {
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&input, 1.0e-12)
            .unwrap();
        assert!(output.edges().iter().all(|(_, edge)| {
            transaction
                .store()
                .get(*edge)
                .unwrap()
                .tolerance()
                .is_some()
        }));
        transaction.rollback().unwrap();
        output
    };
    assert_eq!(counts(&store), before);

    let mut transaction = store.transaction().unwrap();
    let replay = transaction
        .assemble_analytic_shell(&input, 1.0e-12)
        .unwrap();
    assert_eq!(replay, first);
    transaction.rollback().unwrap();
    assert_eq!(counts(&store), before);
}

#[test]
fn persistent_skew_require_valid_rejects_and_rolls_back_future_ids() {
    let (input, _) = scaffold_input();
    let mut store = Store::new();
    let before = counts(&store);
    let rejected_output = {
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&input, 1.0e-12)
            .unwrap();
        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(!decision.is_committed());
        assert!(decision.journal().is_none());
        assert_eq!(decision.checks().len(), 1);
        let report = decision.checks()[0].report();
        assert_eq!(report.outcome(), CheckOutcome::Indeterminate, "{report:#?}");
        assert!(report.faults.is_empty(), "{report:#?}");
        assert!(!report.gaps.is_empty());
        output
    };
    assert_eq!(counts(&store), before);

    let mut transaction = store.transaction().unwrap();
    let replay = transaction
        .assemble_analytic_shell(&input, 1.0e-12)
        .unwrap();
    assert_eq!(replay, rejected_output);
    transaction.rollback().unwrap();
    assert_eq!(counts(&store), before);
}

#[test]
fn persistent_skew_malformed_swapped_and_stale_pairings_refuse_before_allocation() {
    let (mut unsealed, certificate) = scaffold_input();
    unsealed.edges[0] = AnalyticShellEdge::new(
        E0,
        [V0, V1],
        AnalyticShellCurve::PersistentSkewCylinderOpenSpan(certificate.carrier()),
        ParamRange::new(0.0, 1.0),
    );
    assert_preflight_refusal(&mut Store::new(), &unsealed, |error| {
        matches!(error, AnalyticShellPlanError::InvalidGeometry { .. })
    });

    let (mut swapped_pcurve, certificate) = scaffold_input();
    let wrong_use = AnalyticPcurveUse::new(
        AnalyticShellPcurve::PersistentSkewCylinderOpenSpan(certificate.pcurves()[1]),
        AffineParamMap1d::new(1.0, 0.0).unwrap(),
    );
    for fin in &mut swapped_pcurve.faces[0].loops[0].fins {
        fin.pcurve = wrong_use;
    }
    swapped_pcurve.faces[0].domain = pcurve_domain(
        certificate.pcurves()[1],
        PcurveChart::identity(),
        certificate.carrier().cylinders()[0],
    );
    assert_preflight_refusal(&mut Store::new(), &swapped_pcurve, |error| {
        matches!(
            error,
            AnalyticShellPlanError::PairingCertification {
                source: IntersectionCertificateError::InvalidTraceFamily,
                ..
            }
        )
    });

    let (mut stale, _) = scaffold_input();
    let mut store = Store::new();
    let stale_edge = {
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&stale, 1.0e-12)
            .unwrap();
        let edge = output.edges()[0].1;
        transaction.rollback().unwrap();
        edge
    };
    stale.edges[0] = stale.edges[0].with_source(EntityRef::Edge(stale_edge));
    assert_preflight_refusal(&mut store, &stale, |error| {
        matches!(
            error,
            AnalyticShellPlanError::StaleLineage(EntityRef::Edge(edge)) if *edge == stale_edge
        )
    });
}

#[test]
fn persistent_skew_source_order_is_geometric_not_face_key_order() {
    let (mut input, certificate) = scaffold_input();
    let keys = [input.faces[0].key, input.faces[1].key];
    input.faces[0].key = keys[1];
    input.faces[1].key = keys[0];
    let prepared = prepare_analytic_shell(&input, &Store::new(), 1.0e-12).unwrap();
    for edge in prepared.edges() {
        assert_eq!(edge.uses().map(AnalyticEdgeUseRef::face), [F1, F0]);
    }

    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&input, 1.0e-12)
        .unwrap();
    for &(_, edge) in output.edges() {
        let descriptor = transaction
            .store()
            .curve(transaction.store().get(edge).unwrap().curve().unwrap())
            .unwrap()
            .as_persistent_skew_cylinder_open_span()
            .unwrap();
        assert_eq!(
            descriptor.source_surfaces().map(|surface| transaction
                .store()
                .surface(surface)
                .unwrap()
                .clone()),
            certificate.carrier().cylinders().map(SurfaceGeom::Cylinder)
        );
    }
    transaction.rollback().unwrap();
}

#[test]
fn persistent_skew_exact_floor_is_installed_and_budgeted() {
    let certificate = persistent_certificate(0.0);
    assert!(certificate.required_edge_tolerance() <= LINEAR_RESOLUTION);
    let (input, _) = scaffold_input_with_certificate(certificate);
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&input, 1.0e-12)
        .unwrap();
    for &(_, edge) in output.edges() {
        assert_eq!(
            transaction
                .store()
                .get(edge)
                .unwrap()
                .tolerance()
                .unwrap()
                .value(),
            LINEAR_RESOLUTION
        );
    }
    let journal = transaction.commit_checked(&[output.body()]).unwrap();
    assert_eq!(journal.tolerance_budgets().len(), 1);
    assert_eq!(journal.tolerance_budgets()[0].limit(), 0.0);
    assert_eq!(journal.tolerance_budgets()[0].consumed(), 0.0);
}

fn assert_preflight_refusal(
    store: &mut Store,
    input: &AnalyticShellInput,
    expected: impl FnOnce(&AnalyticShellPlanError) -> bool,
) {
    let before = counts(store);
    let mut transaction = store.transaction().unwrap();
    let error = transaction
        .assemble_analytic_shell(input, 1.0e-12)
        .unwrap_err();
    let AnalyticShellAssemblyError::Preflight(error) = error else {
        panic!("expected allocation-free analytic-shell refusal")
    };
    assert!(expected(&error), "unexpected preflight error: {error:?}");
    assert_eq!(counts(transaction.store()), before);
    transaction.rollback().unwrap();
}

fn counts(store: &Store) -> [usize; 12] {
    [
        store.count::<Body>(),
        store.count::<Region>(),
        store.count::<Shell>(),
        store.count::<Face>(),
        store.count::<Loop>(),
        store.count::<Fin>(),
        store.count::<Edge>(),
        store.count::<Vertex>(),
        store.count::<CurveGeom>(),
        store.count::<SurfaceGeom>(),
        store.count::<Point3>(),
        store.count::<Curve2dGeom>(),
    ]
}
