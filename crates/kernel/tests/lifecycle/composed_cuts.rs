//! Application sequence: extrude, subtract, then reuse each result as an operand.
//! Part of the existing lifecycle target; release wall-time budget: 10 seconds.

use super::*;
use kernel::{BOOLEAN_SOURCE_EXTRACTION_WORK, PointBodyVerdict};

const CUTS: [(f64, f64, f64); 3] = [(-2.5, -1.0, 0.75), (2.0, -1.0, 0.5), (0.0, 2.0, 0.625)];

fn cutter(session: &mut Session, part: &PartId, frame: Frame, cut: (f64, f64, f64)) -> BodyId {
    session
        .edit_part(part.clone())
        .unwrap()
        .create_cylinder(CylinderRequest::new(
            frame.with_origin(frame.point_at(cut.0, cut.1, -1.0)),
            cut.2,
            4.0,
        ))
        .unwrap()
        .into_result()
        .unwrap()
        .body()
}

fn fixture(frame: Frame) -> BooleanFixture {
    fixture_with_first_cut(frame, CUTS[0])
}

fn fixture_with_first_cut(frame: Frame, first: (f64, f64, f64)) -> BooleanFixture {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let outer = vec![
        Point2::new(-5.0, -4.0),
        Point2::new(5.0, -4.0),
        Point2::new(5.0, 4.0),
        Point2::new(-5.0, 4.0),
    ];
    let left = session
        .edit_part(part.clone())
        .unwrap()
        .extrude_profile(ExtrudeProfileRequest::new(frame, outer, vec![], 2.0))
        .unwrap()
        .into_result()
        .unwrap()
        .body();
    let right = cutter(&mut session, &part, frame, first);
    BooleanFixture {
        session,
        part,
        left,
        right,
    }
}

fn export(fixture: &BooleanFixture, body: BodyId) -> Vec<u8> {
    fixture
        .session
        .part(fixture.part.clone())
        .unwrap()
        .export_xt(ExportXtRequest::new(body))
        .unwrap()
        .into_result()
        .unwrap()
        .bytes()
        .to_vec()
}

fn cut(fixture: &mut BooleanFixture) -> kernel::BooleanCreatedResult {
    let BooleanResult::Created(created) = boolean_success(run_boolean(
        fixture,
        BooleanOperation::Subtract,
        OperationSettings::new(),
    )) else {
        panic!("a supported through cut must create one solid")
    };
    assert_eq!(created.bodies().len(), 1);
    assert_boolean_created_full_valid(&created);
    created
}

fn second_cut_fixture(next: (f64, f64, f64)) -> BooleanFixture {
    let mut fixture = fixture(Frame::world());
    fixture.left = cut(&mut fixture).bodies()[0].clone();
    fixture.right = cutter(&mut fixture.session, &fixture.part, Frame::world(), next);
    fixture
}

fn assert_lineage(
    fixture: &BooleanFixture,
    created: &kernel::BooleanCreatedResult,
    expected_sources: [usize; 2],
) {
    let part = fixture.session.part(fixture.part.clone()).unwrap();
    let original = part
        .body(fixture.left.clone())
        .unwrap()
        .faces()
        .unwrap()
        .collect::<Vec<_>>();
    let tool = part
        .body(fixture.right.clone())
        .unwrap()
        .faces()
        .unwrap()
        .collect::<Vec<_>>();
    let result = part
        .body(created.bodies()[0].clone())
        .unwrap()
        .faces()
        .unwrap()
        .collect::<Vec<_>>();
    let mut derived = Vec::new();
    let mut counts = [0, 0];
    for event in created.journal().lineage() {
        if let LineageView::DerivedFrom {
            derived: JournalEntity::Edge(edge),
            source: JournalEntity::Edge(source),
        } = &event
        {
            assert!(
                part.body(created.bodies()[0].clone())
                    .unwrap()
                    .edges()
                    .unwrap()
                    .any(|id| id == *edge)
            );
            assert!([&fixture.left, &fixture.right].iter().any(|body| {
                part.body((*body).clone())
                    .unwrap()
                    .edges()
                    .unwrap()
                    .any(|id| id == *source)
            }));
            continue;
        }
        if let LineageView::DerivedFrom {
            derived: JournalEntity::Vertex(vertex),
            source: JournalEntity::Edge(source),
        } = &event
        {
            assert!(
                part.body(created.bodies()[0].clone())
                    .unwrap()
                    .vertices()
                    .unwrap()
                    .any(|id| id == *vertex)
            );
            assert!([&fixture.left, &fixture.right].iter().any(|body| {
                part.body((*body).clone())
                    .unwrap()
                    .edges()
                    .unwrap()
                    .any(|id| id == *source)
            }));
            continue;
        }
        let LineageView::DerivedFrom {
            derived: JournalEntity::Face(face),
            source: JournalEntity::Face(source),
        } = event
        else {
            panic!("expected face-only source lineage: {event:?}")
        };
        assert!(result.contains(&face) && !derived.contains(&face));
        derived.push(face);
        if original.contains(&source) {
            counts[0] += 1;
        } else {
            assert!(tool.contains(&source));
            counts[1] += 1;
        }
    }
    assert_eq!(counts, expected_sources);
    assert_eq!(derived.len(), result.len());
    assert!(
        created
            .journal()
            .mutations()
            .all(|event| event.kind() == MutationKind::Created)
    );
}

fn frames() -> [Frame; 4] {
    [
        Frame::world(),
        Frame::world().with_origin(Point3::new(4.0, -3.0, 2.0)),
        Frame::new(
            Point3::new(-2.0, 3.5, 1.25),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap(),
        Frame::new(
            Point3::new(3.0, -2.0, 1.25),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.8, -0.6, 0.0),
        )
        .unwrap(),
    ]
}

#[test]
fn composed_cuts_reuse_results_preserve_holes_and_replay_under_rigid_frames() {
    for frame in frames() {
        let mut previous_exports = None;
        for _ in 0..2 {
            let mut fixture = fixture(frame);
            let mut exports = Vec::new();
            for (index, &next) in CUTS.iter().enumerate() {
                if index != 0 {
                    fixture.right = cutter(&mut fixture.session, &fixture.part, frame, next);
                }
                let sources = [
                    export(&fixture, fixture.left.clone()),
                    export(&fixture, fixture.right.clone()),
                ];
                let section = fixture
                    .session
                    .part(fixture.part.clone())
                    .unwrap()
                    .section_bodies(SectionBodiesRequest::new(
                        fixture.left.clone(),
                        fixture.right.clone(),
                    ))
                    .unwrap()
                    .into_result()
                    .unwrap();
                assert_eq!(
                    section.completion(),
                    SectionCompletion::Complete,
                    "{section:?}"
                );
                assert!(section.gaps().is_empty());
                assert_eq!(section.curve_fragments().len(), 2);
                let created = cut(&mut fixture);
                let body = created.bodies()[0].clone();
                assert_eq!(
                    boolean_body_topology_signature(&fixture, body.clone()),
                    [7 + index, 14 + 2 * index, 8]
                );
                assert_lineage(&fixture, &created, [6 + index, 1]);
                assert_eq!(export(&fixture, fixture.left.clone()), sources[0]);
                assert_eq!(export(&fixture, fixture.right.clone()), sources[1]);
                let part = fixture.session.part(fixture.part.clone()).unwrap();
                let full = part
                    .check_body(CheckBodyRequest::new(body.clone(), CheckLevel::Full))
                    .unwrap()
                    .into_result()
                    .unwrap();
                assert_eq!(full.outcome(), CheckOutcome::Valid, "{full:?}");
                for (x, y, expected) in CUTS[..=index]
                    .iter()
                    .map(|cut| (cut.0, cut.1, kernel::PointBodyVerdict::Exterior))
                    .chain([
                        (4.0, 3.0, kernel::PointBodyVerdict::Interior),
                        (6.0, 0.0, kernel::PointBodyVerdict::Exterior),
                    ])
                {
                    let classified = part
                        .classify_point_in_body(ClassifyPointInBodyRequest::new(
                            body.clone(),
                            frame.point_at(x, y, 1.0),
                        ))
                        .unwrap()
                        .into_result()
                        .unwrap();
                    assert_eq!(classified.verdict(), &expected, "{classified:?}");
                }
                let mesh = part
                    .tessellate_body(TessellateBodyRequest::new(
                        body.clone(),
                        TessOptions {
                            chord_tol: 1.0e-3,
                            max_edge_len: None,
                        },
                    ))
                    .unwrap()
                    .into_result()
                    .unwrap();
                let six_volume = mesh.triangles().iter().fold(0.0, |sum, triangle| {
                    let [a, b, c] = triangle.map(|i| mesh.positions()[i as usize] - frame.origin());
                    sum + a.dot(b.cross(c))
                });
                let expected = 160.0
                    - 2.0
                        * core::f64::consts::PI
                        * CUTS[..=index].iter().map(|cut| cut.2 * cut.2).sum::<f64>();
                assert!(
                    (six_volume.abs() / 6.0 - expected).abs() < 0.015,
                    "holes={}: mesh={}, independent volume={expected}",
                    index + 1,
                    six_volume.abs() / 6.0
                );
                exports.push(assert_deterministic_xt_and_fast_self_import(
                    &mut fixture,
                    std::slice::from_ref(&body),
                ));
                fixture.left = body;
            }
            if let Some(previous) = previous_exports {
                assert_eq!(exports, previous);
            }
            previous_exports = Some(exports);
        }
    }
}

#[test]
fn composed_cuts_budget_boundaries_are_exact_and_failure_atomic() {
    assert_work_boundaries(|| second_cut_fixture(CUTS[1]));
}

fn assert_work_boundaries(mut make_fixture: impl FnMut() -> BooleanFixture) {
    let baseline = run_boolean(
        &mut make_fixture(),
        BooleanOperation::Subtract,
        OperationSettings::new(),
    );
    assert!(matches!(
        baseline.result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));
    for stage in [
        BOOLEAN_SOURCE_EXTRACTION_WORK,
        BOOLEAN_BSP_WORK,
        BOOLEAN_POST_SELECTION_WORK,
        SECTION_WORK,
        kernel::POINT_CLASSIFICATION_WORK,
        kernel::StageId::new("ktopo.check.shell-surgery-work").unwrap(),
        kernel::StageId::new("ktopo.check.mixed-profile-prism-work").unwrap(),
    ] {
        let usage = *baseline
            .report()
            .usage()
            .iter()
            .find(|usage| usage.stage == stage && usage.resource == ResourceKind::Work)
            .unwrap();
        if usage.consumed == 0
            && (stage == kernel::StageId::new("ktopo.check.mixed-profile-prism-work").unwrap()
                || stage == kernel::StageId::new("ktopo.check.shell-surgery-work").unwrap())
        {
            continue; // These independent shell proofs discharge different representations.
        }
        assert!(usage.consumed > 0, "unused budget stage: {stage:?}");
        let settings = |allowed| {
            OperationSettings::new().with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    stage,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            )
        };
        let admitted = run_boolean(
            &mut make_fixture(),
            BooleanOperation::Subtract,
            settings(usage.consumed),
        );
        assert!(matches!(
            admitted.result().unwrap(),
            BooleanOutcome::Success(BooleanResult::Created(_))
        ));
        let mut fixture = make_fixture();
        let before = boolean_topology_counts(&fixture);
        let source = export(&fixture, fixture.left.clone());
        let denied = run_boolean(
            &mut fixture,
            BooleanOperation::Subtract,
            settings(usage.consumed - 1),
        );
        let expected = kernel::LimitSnapshot {
            allowed: usage.consumed - 1,
            ..usage
        };
        assert_eq!(denied.result().unwrap_err().limit(), Some(expected));
        assert_eq!(denied.report().limit_events(), &[expected]);
        assert_eq!(boolean_topology_counts(&fixture), before);
        assert_eq!(export(&fixture, fixture.left.clone()), source);
        cut(&mut fixture);
    }
}

#[test]
fn composed_cuts_refuse_tangent_existing_holes_without_mutation() {
    // External/internal tangency and unresolved near contact fail closed.
    for next in [
        (-1.0, -1.0, 0.75),
        (-2.0, -1.0, 1.25),
        (-2.0_f64.next_down(), -1.0, 1.25),
    ] {
        let mut fixture = second_cut_fixture(next);
        let before = boolean_topology_counts(&fixture);
        let source = export(&fixture, fixture.left.clone());
        let outcome = run_boolean(
            &mut fixture,
            BooleanOperation::Subtract,
            OperationSettings::new(),
        );
        assert!(matches!(
            outcome.into_result().unwrap(),
            BooleanOutcome::Refused(_)
        ));
        assert_eq!(boolean_topology_counts(&fixture), before);
        assert_eq!(export(&fixture, fixture.left.clone()), source);
    }
}

fn nested_cut_fixture(
    frame: Frame,
    holes: &[(f64, f64, f64)],
    next: (f64, f64, f64),
) -> BooleanFixture {
    let mut fixture = fixture_with_first_cut(frame, holes[0]);
    for (index, &hole) in holes.iter().enumerate() {
        if index != 0 {
            fixture.right = cutter(&mut fixture.session, &fixture.part, frame, hole);
        }
        fixture.left = cut(&mut fixture).bodies()[0].clone();
    }
    fixture.right = cutter(&mut fixture.session, &fixture.part, frame, next);
    fixture
}

#[test]
fn composed_cuts_replace_nested_holes_and_preserve_unaffected_material() {
    struct Case {
        holes: &'static [(f64, f64, f64)],
        cut: (f64, f64, f64),
        result_holes: &'static [(f64, f64, f64)],
        unchanged: bool,
    }
    let cases = [
        Case {
            holes: &[(0.0, 0.0, 0.5)],
            cut: (0.0, 0.0, 1.0),
            result_holes: &[(0.0, 0.0, 1.0)],
            unchanged: false,
        },
        Case {
            holes: &[(0.0, 0.0, 0.5)],
            cut: (0.25, 0.125, 1.25),
            result_holes: &[(0.25, 0.125, 1.25)],
            unchanged: false,
        },
        Case {
            holes: &[(-1.0, 0.0, 0.25), (1.0, 0.0, 0.375), (3.0, 2.0, 0.25)],
            cut: (0.0, 0.0, 1.75),
            result_holes: &[(0.0, 0.0, 1.75), (3.0, 2.0, 0.25)],
            unchanged: false,
        },
        Case {
            holes: &[(-1.0, 0.0, 0.25), (0.0, 1.0, 0.25), (1.0, 0.0, 0.25)],
            cut: (0.0, 0.0, 1.75),
            result_holes: &[(0.0, 0.0, 1.75)],
            unchanged: false,
        },
        Case {
            holes: &[(0.0, 0.0, 1.5)],
            cut: (0.125, 0.125, 0.5),
            result_holes: &[(0.0, 0.0, 1.5)],
            unchanged: true,
        },
    ];
    for frame in frames() {
        for case in &cases {
            let mut prior = None;
            for _ in 0..2 {
                let mut fixture = nested_cut_fixture(frame, case.holes, case.cut);
                let before = [
                    export(&fixture, fixture.left.clone()),
                    export(&fixture, fixture.right.clone()),
                ];
                let source_counts = boolean_topology_counts(&fixture);
                let part = fixture.session.part(fixture.part.clone()).unwrap();
                let request =
                    SectionBodiesRequest::new(fixture.left.clone(), fixture.right.clone());
                let graph = part
                    .section_bodies(request.clone())
                    .unwrap()
                    .into_result()
                    .unwrap();
                assert_eq!(
                    graph,
                    part.section_bodies(request).unwrap().into_result().unwrap()
                );
                assert_eq!(graph.completion(), SectionCompletion::Complete, "{graph:?}");
                assert!(graph.gaps().is_empty());
                assert_eq!(
                    graph.curve_fragments().len(),
                    if case.unchanged { 0 } else { 2 }
                );
                let swapped = part
                    .section_bodies(SectionBodiesRequest::new(
                        fixture.right.clone(),
                        fixture.left.clone(),
                    ))
                    .unwrap()
                    .into_result()
                    .unwrap();
                assert_eq!(swapped.completion(), SectionCompletion::Complete);
                assert!(swapped.gaps().is_empty());
                assert_eq!(
                    swapped.curve_fragments().len(),
                    graph.curve_fragments().len()
                );
                // A contained void never acquires a material-disjointness witness.
                let exterior_pairs = if case.unchanged {
                    0
                } else {
                    case.result_holes.len() - 1
                };
                assert_eq!(
                    graph.cylinder_cylinder_exterior_radial_separations().len(),
                    exterior_pairs
                );
                assert_eq!(
                    swapped
                        .cylinder_cylinder_exterior_radial_separations()
                        .len(),
                    exterior_pairs
                );
                assert_eq!(boolean_topology_counts(&fixture), source_counts);
                let created = cut(&mut fixture);
                let body = created.bodies()[0].clone();
                let holes = case.result_holes.len();
                assert_eq!(
                    boolean_body_topology_signature(&fixture, body.clone()),
                    [6 + holes, 12 + 2 * holes, 8]
                );
                assert_lineage(
                    &fixture,
                    &created,
                    if case.unchanged {
                        [6 + holes, 0]
                    } else {
                        [5 + holes, 1]
                    },
                );
                assert_eq!(export(&fixture, fixture.left.clone()), before[0]);
                assert_eq!(export(&fixture, fixture.right.clone()), before[1]);
                let part = fixture.session.part(fixture.part.clone()).unwrap();
                let report = part
                    .check_body(CheckBodyRequest::new(body.clone(), CheckLevel::Full))
                    .unwrap()
                    .into_result()
                    .unwrap();
                assert_eq!(report.outcome(), CheckOutcome::Valid);
                let mesh = part
                    .tessellate_body(TessellateBodyRequest::new(
                        body.clone(),
                        TessOptions {
                            chord_tol: 2.0e-3,
                            max_edge_len: None,
                        },
                    ))
                    .unwrap()
                    .into_result()
                    .unwrap();
                let volume = mesh
                    .triangles()
                    .iter()
                    .map(|triangle| {
                        let [a, b, c] =
                            triangle.map(|i| mesh.positions()[i as usize] - frame.origin());
                        a.dot(b.cross(c)) / 6.0
                    })
                    .sum::<f64>()
                    .abs();
                let expected = 160.0
                    - 2.0
                        * core::f64::consts::PI
                        * case
                            .result_holes
                            .iter()
                            .map(|cut| cut.2 * cut.2)
                            .sum::<f64>();
                assert!(
                    (volume - expected).abs() < 0.05,
                    "volume {volume}, expected {expected}"
                );
                for (x, y, expected) in case
                    .holes
                    .iter()
                    .chain(case.result_holes)
                    .map(|hole| (hole.0, hole.1, kernel::PointBodyVerdict::Exterior))
                    .chain([(4.0, -3.0, kernel::PointBodyVerdict::Interior)])
                {
                    let value = part
                        .classify_point_in_body(ClassifyPointInBodyRequest::new(
                            body.clone(),
                            frame.point_at(x, y, 1.0),
                        ))
                        .unwrap()
                        .into_result()
                        .unwrap();
                    assert_eq!(value.verdict(), &expected, "{value:?}");
                }
                let exports = assert_deterministic_xt_and_fast_self_import(
                    &mut fixture,
                    std::slice::from_ref(&body),
                );
                if let Some(prior) = &prior {
                    assert_eq!(&exports, prior);
                }
                prior = Some(exports);
                // The enlarged result remains a usable operand for a later cut.
                fixture.left = body;
                fixture.right = cutter(
                    &mut fixture.session,
                    &fixture.part,
                    frame,
                    (-3.0, -2.0, 0.5),
                );
                let next = cut(&mut fixture);
                assert_eq!(
                    boolean_body_topology_signature(&fixture, next.bodies()[0].clone()),
                    [7 + holes, 14 + 2 * holes, 8]
                );
            }
        }
    }
}

#[test]
fn composed_cuts_nested_replacement_budget_denial_is_failure_atomic() {
    assert_work_boundaries(|| {
        nested_cut_fixture(
            Frame::world(),
            &[(-1.0, 0.0, 0.25), (1.0, 0.0, 0.375), (3.0, 2.0, 0.25)],
            (0.0, 0.0, 1.75),
        )
    });
}

/// Independent circle-lens formula; the pre-existing holes are disjoint.
fn disk_overlap(first: (f64, f64, f64), second: (f64, f64, f64)) -> f64 {
    let d = ((first.0 - second.0).powi(2) + (first.1 - second.1).powi(2)).sqrt();
    let (r, s) = (first.2, second.2);
    if d >= r + s {
        return 0.0;
    }
    if d <= (r - s).abs() {
        return core::f64::consts::PI * r.min(s).powi(2);
    }
    let acos = |x: f64| kcore::math::atan2((1.0 - x * x).sqrt(), x);
    r * r * acos((d * d + r * r - s * s) / (2.0 * d * r))
        + s * s * acos((d * d + s * s - r * r) / (2.0 * d * s))
        - 0.5 * ((-d + r + s) * (d + r - s) * (d - r + s) * (d + r + s)).sqrt()
}

#[test]
fn composed_cuts_split_crossing_holes_with_full_proof_and_independent_volume() {
    type CircleCut = (f64, f64, f64);
    let cases: &[(&[CircleCut], CircleCut)] = &[
        (&[(0.0, 0.0, 0.75)], (0.75, 0.0, 0.75)),
        (&[(-2.5, -1.0, 0.75)], (-2.0, -1.0, 0.75)),
        (&[(0.0, 0.0, 0.75)], (0.625, 0.25, 0.5)),
        (&[(0.0, 0.0, 0.75)], (-0.75, 0.0, 0.75)),
        (&[(0.0, 0.0, 0.75)], (0.25, 0.625, 0.5)),
        (&[(0.0, 0.0, 0.75), (3.0, 2.0, 0.25)], (0.75, 0.0, 0.75)),
        (&[(-1.0, 0.0, 0.5), (1.0, 0.0, 0.5)], (0.0, 0.0, 1.125)),
        (
            &[(-1.0, 0.0, 0.5), (1.0, 0.0, 0.5), (0.0, 0.0, 0.125)],
            (0.0, 0.0, 1.125),
        ),
    ];
    for frame in frames() {
        for &(holes, next) in cases {
            let mut previous = None;
            for _ in 0..2 {
                let mut fixture = nested_cut_fixture(frame, holes, next);
                let sources = [
                    export(&fixture, fixture.left.clone()),
                    export(&fixture, fixture.right.clone()),
                ];
                let part = fixture.session.part(fixture.part.clone()).unwrap();
                let graph = part
                    .section_bodies(SectionBodiesRequest::new(
                        fixture.left.clone(),
                        fixture.right.clone(),
                    ))
                    .unwrap()
                    .into_result()
                    .unwrap();
                assert_eq!(
                    graph.completion(),
                    SectionCompletion::Complete,
                    "holes={holes:?}, cut={next:?}: {graph:?}"
                );
                let crossed = holes
                    .iter()
                    .filter(|&&hole| {
                        let d = ((hole.0 - next.0).powi(2) + (hole.1 - next.1).powi(2)).sqrt();
                        d > (hole.2 - next.2).abs() && d < hole.2 + next.2
                    })
                    .count();
                let swallowed = holes
                    .iter()
                    .filter(|&&hole| {
                        ((hole.0 - next.0).powi(2) + (hole.1 - next.1).powi(2)).sqrt() + hole.2
                            < next.2
                    })
                    .count();
                let swapped = part
                    .section_bodies(SectionBodiesRequest::new(
                        fixture.right.clone(),
                        fixture.left.clone(),
                    ))
                    .unwrap()
                    .into_result()
                    .unwrap();
                assert_eq!(swapped.completion(), SectionCompletion::Complete);
                assert_eq!(
                    swapped.curve_fragments().len(),
                    graph.curve_fragments().len()
                );
                assert_eq!(graph.curve_fragments().len(), 4 * crossed);
                let created = cut(&mut fixture);
                let body = created.bodies()[0].clone();
                let retained = holes.len() - swallowed;
                assert_lineage(&fixture, &created, [6 + retained, crossed]);
                assert_eq!(
                    boolean_body_topology_signature(&fixture, body.clone()),
                    [
                        6 + retained + crossed,
                        12 + 2 * retained + 4 * crossed,
                        8 + 4 * crossed
                    ]
                );
                assert_eq!(export(&fixture, fixture.left.clone()), sources[0]);
                assert_eq!(export(&fixture, fixture.right.clone()), sources[1]);
                let part = fixture.session.part(fixture.part.clone()).unwrap();
                let checked = part
                    .check_body(CheckBodyRequest::new(body.clone(), CheckLevel::Full))
                    .unwrap()
                    .into_result()
                    .unwrap();
                assert_eq!(checked.outcome(), CheckOutcome::Valid, "{checked:?}");
                let mesh = part
                    .tessellate_body(TessellateBodyRequest::new(
                        body.clone(),
                        TessOptions {
                            chord_tol: 1e-3,
                            max_edge_len: None,
                        },
                    ))
                    .unwrap()
                    .into_result()
                    .unwrap();
                let volume = mesh
                    .triangles()
                    .iter()
                    .map(|triangle| {
                        let [a, b, c] =
                            triangle.map(|i| mesh.positions()[i as usize] - frame.origin());
                        a.dot(b.cross(c)) / 6.0
                    })
                    .sum::<f64>()
                    .abs();
                let area = core::f64::consts::PI
                    * (next.2.powi(2) + holes.iter().map(|h| h.2.powi(2)).sum::<f64>())
                    - holes.iter().map(|&h| disk_overlap(h, next)).sum::<f64>();
                assert!(
                    (volume - (160.0 - 2.0 * area)).abs() < 0.04,
                    "holes={holes:?}, cut={next:?}, volume={volume}, expected={}",
                    160.0 - 2.0 * area
                );
                let bytes = assert_deterministic_xt_and_fast_self_import(
                    &mut fixture,
                    std::slice::from_ref(&body),
                );
                if let Some(previous) = &previous {
                    assert_eq!(&bytes, previous);
                }
                previous = Some(bytes);
            }
        }
    }
}

#[test]
fn composed_crossing_cut_budget_denial_is_failure_atomic() {
    assert_work_boundaries(|| {
        nested_cut_fixture(Frame::world(), &[(0.0, 0.0, 0.75)], (0.75, 0.0, 0.75))
    });
}

fn reused_arc_fixture(frame: Frame, next: (f64, f64, f64)) -> BooleanFixture {
    let mut fixture = nested_cut_fixture(
        frame,
        &[(0.0, 0.0, 0.75), (3.0, 2.0, 0.25)],
        (0.75, 0.0, 0.75),
    );
    fixture.left = cut(&mut fixture).bodies()[0].clone();
    fixture.right = cutter(&mut fixture.session, &fixture.part, frame, next);
    fixture
}

#[test]
fn composed_arc_results_support_separated_and_crossing_followup_cuts() {
    for frame in frames() {
        for next in [(3.0, -2.0, 0.5), (1.5, 0.0, 0.5), (1.375, 0.375, 0.5)] {
            let mut previous = None;
            for _ in 0..2 {
                let mut fixture = reused_arc_fixture(frame, next);
                let graph = fixture
                    .session
                    .part(fixture.part.clone())
                    .unwrap()
                    .section_bodies(SectionBodiesRequest::new(
                        fixture.left.clone(),
                        fixture.right.clone(),
                    ))
                    .unwrap()
                    .into_result()
                    .unwrap();
                assert_eq!(
                    graph.completion(),
                    SectionCompletion::Complete,
                    "next={next:?}, frame={frame:?}: {graph:?}"
                );
                let sources = [
                    export(&fixture, fixture.left.clone()),
                    export(&fixture, fixture.right.clone()),
                ];
                let created = cut(&mut fixture);
                let body = created.bodies()[0].clone();
                let crossing = disk_overlap((0.75, 0.0, 0.75), next) > 0.0;
                assert_lineage(&fixture, &created, [if crossing { 10 } else { 9 }, 1]);
                assert_eq!(
                    boolean_body_topology_signature(&fixture, body.clone()),
                    if crossing { [11, 26, 16] } else { [10, 22, 12] }
                );
                assert_eq!(export(&fixture, fixture.left.clone()), sources[0]);
                assert_eq!(export(&fixture, fixture.right.clone()), sources[1]);
                let part = fixture.session.part(fixture.part.clone()).unwrap();
                for (x, y, expected) in [
                    (0.0, 0.0, PointBodyVerdict::Exterior),
                    (0.75, 0.0, PointBodyVerdict::Exterior),
                    (next.0, next.1, PointBodyVerdict::Exterior),
                    (3.0, 2.0, PointBodyVerdict::Exterior),
                    (4.0, -3.0, PointBodyVerdict::Interior),
                ] {
                    let value = part
                        .classify_point_in_body(ClassifyPointInBodyRequest::new(
                            body.clone(),
                            frame.point_at(x, y, 1.0),
                        ))
                        .unwrap()
                        .into_result()
                        .unwrap();
                    assert_eq!(value.verdict(), &expected, "next={next:?}: {value:?}");
                }
                let wall = part
                    .classify_point_in_body(ClassifyPointInBodyRequest::new(
                        body.clone(),
                        frame.point_at(-0.75, 0.0, 1.0),
                    ))
                    .unwrap()
                    .into_result()
                    .unwrap();
                assert!(
                    matches!(wall.verdict(), PointBodyVerdict::Boundary { .. }),
                    "{wall:?}"
                );
                let mesh = part
                    .tessellate_body(TessellateBodyRequest::new(
                        body.clone(),
                        TessOptions {
                            chord_tol: 1e-3,
                            max_edge_len: None,
                        },
                    ))
                    .unwrap()
                    .into_result()
                    .unwrap();
                let volume = mesh
                    .triangles()
                    .iter()
                    .map(|triangle| {
                        let [a, b, c] =
                            triangle.map(|i| mesh.positions()[i as usize] - frame.origin());
                        a.dot(b.cross(c)) / 6.0
                    })
                    .sum::<f64>()
                    .abs();
                // The first and third intersecting disks are disjoint, so no
                // triple-overlap term is present in this independent oracle.
                let first = (0.0, 0.0, 0.75);
                let second = (0.75, 0.0, 0.75);
                let area = core::f64::consts::PI
                    * (2.0 * 0.75_f64.powi(2) + 0.25_f64.powi(2) + next.2.powi(2))
                    - disk_overlap(first, second)
                    - disk_overlap(second, next);
                assert!(
                    (volume - (160.0 - 2.0 * area)).abs() < 0.04,
                    "next={next:?}, volume={volume}"
                );
                let bytes = assert_deterministic_xt_and_fast_self_import(
                    &mut fixture,
                    std::slice::from_ref(&body),
                );
                if let Some(previous) = &previous {
                    assert_eq!(&bytes, previous);
                }
                previous = Some(bytes);
            }
        }
    }
}

#[test]
fn composed_arc_reuse_budget_denial_is_failure_atomic() {
    for next in [(3.0, -2.0, 0.5), (1.5, 0.0, 0.5)] {
        assert_work_boundaries(|| reused_arc_fixture(Frame::world(), next));
    }
}
