//! Application sequence: extrude, subtract, then reuse each result as an operand.
//! Part of the existing lifecycle target; release wall-time budget: 10 seconds.

use super::*;
use kernel::BOOLEAN_SOURCE_EXTRACTION_WORK;

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
    let right = cutter(&mut session, &part, frame, CUTS[0]);
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
        panic!("a separated through cut must create one solid")
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

fn assert_lineage(fixture: &BooleanFixture, created: &kernel::BooleanCreatedResult, holes: usize) {
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
        let LineageView::DerivedFrom {
            derived: JournalEntity::Face(face),
            source: JournalEntity::Face(source),
        } = event
        else {
            panic!("expected face-only source lineage")
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
    assert_eq!(counts, [5 + holes, 1]);
    assert_eq!(derived.len(), result.len());
    assert!(
        created
            .journal()
            .mutations()
            .all(|event| event.kind() == MutationKind::Created)
    );
}

#[test]
fn composed_cuts_reuse_results_preserve_holes_and_replay_under_rigid_frames() {
    let frames = [
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
    ];
    for frame in frames {
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
                assert_lineage(&fixture, &created, index + 1);
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
    let baseline = run_boolean(
        &mut second_cut_fixture(CUTS[1]),
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
        kernel::StageId::new("ktopo.check.shell-surgery-work").unwrap(),
    ] {
        let usage = *baseline
            .report()
            .usage()
            .iter()
            .find(|usage| usage.stage == stage && usage.resource == ResourceKind::Work)
            .unwrap();
        assert!(usage.consumed > 0);
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
            &mut second_cut_fixture(CUTS[1]),
            BooleanOperation::Subtract,
            settings(usage.consumed),
        );
        assert!(matches!(
            admitted.result().unwrap(),
            BooleanOutcome::Success(BooleanResult::Created(_))
        ));
        let mut fixture = second_cut_fixture(CUTS[1]);
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
fn composed_cuts_refuse_interacting_existing_holes_without_mutation() {
    // Overlap, tangency, and a cutter surrounding the first hole are outside
    // the untouched-ring contract, even though their set differences exist.
    for next in [(-2.0, -1.0, 0.75), (-1.0, -1.0, 0.75), (-2.5, -1.0, 1.0)] {
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
