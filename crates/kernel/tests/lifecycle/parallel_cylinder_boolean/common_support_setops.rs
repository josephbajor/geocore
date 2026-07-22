//! Facade-only set-operation evidence for exact common-cylinder supports.
//! Wall-time budget: less than 60 seconds for the semantic and rigid-frame matrices.

use super::*;

const COMMON_SUPPORT_BAND_WORK: u64 = 420;
const COMMON_SUPPORT_RELATION_WORK: u64 = 64;
const COMMON_SUPPORT_PROPERTIES_WORK: u64 = 3_953;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CommonSupportOperation {
    Intersect,
    Unite,
    AMinusB,
    BMinusA,
}

impl CommonSupportOperation {
    const fn request(self) -> (BooleanOperation, bool) {
        match self {
            Self::Intersect => (BooleanOperation::Intersect, false),
            Self::Unite => (BooleanOperation::Unite, false),
            Self::AMinusB => (BooleanOperation::Subtract, false),
            Self::BMinusA => (BooleanOperation::Subtract, true),
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct SemanticRow {
    name: &'static str,
    intervals: [[f64; 2]; 2],
    operation: CommonSupportOperation,
    expected: &'static [[f64; 2]],
}

const SEMANTIC_ROWS: [SemanticRow; 20] = [
    SemanticRow {
        name: "crossing intersect",
        intervals: [[-2.0, 1.0], [-1.0, 2.0]],
        operation: CommonSupportOperation::Intersect,
        expected: &[[-1.0, 1.0]],
    },
    SemanticRow {
        name: "crossing unite",
        intervals: [[-2.0, 1.0], [-1.0, 2.0]],
        operation: CommonSupportOperation::Unite,
        expected: &[[-2.0, 2.0]],
    },
    SemanticRow {
        name: "crossing A-B",
        intervals: [[-2.0, 1.0], [-1.0, 2.0]],
        operation: CommonSupportOperation::AMinusB,
        expected: &[[-2.0, -1.0]],
    },
    SemanticRow {
        name: "crossing B-A",
        intervals: [[-2.0, 1.0], [-1.0, 2.0]],
        operation: CommonSupportOperation::BMinusA,
        expected: &[[1.0, 2.0]],
    },
    SemanticRow {
        name: "nested intersect",
        intervals: [[-2.0, 2.0], [-1.0, 1.0]],
        operation: CommonSupportOperation::Intersect,
        expected: &[[-1.0, 1.0]],
    },
    SemanticRow {
        name: "nested unite",
        intervals: [[-2.0, 2.0], [-1.0, 1.0]],
        operation: CommonSupportOperation::Unite,
        expected: &[[-2.0, 2.0]],
    },
    SemanticRow {
        name: "nested A-B",
        intervals: [[-2.0, 2.0], [-1.0, 1.0]],
        operation: CommonSupportOperation::AMinusB,
        expected: &[[-2.0, -1.0], [1.0, 2.0]],
    },
    SemanticRow {
        name: "nested B-A",
        intervals: [[-2.0, 2.0], [-1.0, 1.0]],
        operation: CommonSupportOperation::BMinusA,
        expected: &[],
    },
    SemanticRow {
        name: "shared-low intersect",
        intervals: [[-2.0, 2.0], [-2.0, 0.0]],
        operation: CommonSupportOperation::Intersect,
        expected: &[[-2.0, 0.0]],
    },
    SemanticRow {
        name: "shared-low unite",
        intervals: [[-2.0, 2.0], [-2.0, 0.0]],
        operation: CommonSupportOperation::Unite,
        expected: &[[-2.0, 2.0]],
    },
    SemanticRow {
        name: "shared-low A-B",
        intervals: [[-2.0, 2.0], [-2.0, 0.0]],
        operation: CommonSupportOperation::AMinusB,
        expected: &[[0.0, 2.0]],
    },
    SemanticRow {
        name: "shared-low B-A",
        intervals: [[-2.0, 2.0], [-2.0, 0.0]],
        operation: CommonSupportOperation::BMinusA,
        expected: &[],
    },
    SemanticRow {
        name: "shared-high intersect",
        intervals: [[-2.0, 0.0], [-1.0, 0.0]],
        operation: CommonSupportOperation::Intersect,
        expected: &[[-1.0, 0.0]],
    },
    SemanticRow {
        name: "shared-high unite",
        intervals: [[-2.0, 0.0], [-1.0, 0.0]],
        operation: CommonSupportOperation::Unite,
        expected: &[[-2.0, 0.0]],
    },
    SemanticRow {
        name: "shared-high A-B",
        intervals: [[-2.0, 0.0], [-1.0, 0.0]],
        operation: CommonSupportOperation::AMinusB,
        expected: &[[-2.0, -1.0]],
    },
    SemanticRow {
        name: "shared-high B-A",
        intervals: [[-2.0, 0.0], [-1.0, 0.0]],
        operation: CommonSupportOperation::BMinusA,
        expected: &[],
    },
    SemanticRow {
        name: "equal intersect",
        intervals: [[-1.0, 1.0], [-1.0, 1.0]],
        operation: CommonSupportOperation::Intersect,
        expected: &[[-1.0, 1.0]],
    },
    SemanticRow {
        name: "equal unite",
        intervals: [[-1.0, 1.0], [-1.0, 1.0]],
        operation: CommonSupportOperation::Unite,
        expected: &[[-1.0, 1.0]],
    },
    SemanticRow {
        name: "equal A-B",
        intervals: [[-1.0, 1.0], [-1.0, 1.0]],
        operation: CommonSupportOperation::AMinusB,
        expected: &[],
    },
    SemanticRow {
        name: "equal B-A",
        intervals: [[-1.0, 1.0], [-1.0, 1.0]],
        operation: CommonSupportOperation::BMinusA,
        expected: &[],
    },
];

fn common_support_frame(placement: Placement) -> Frame {
    match placement {
        Placement::World => Frame::world(),
        Placement::Oblique => Frame::new(
            // The translated 7-24-25 direction keeps all crossing-fixture
            // cap evaluations exact for height three in either authored axis
            // direction. That makes this a positive conformance case rather
            // than the rounded all-nonzero boundary refusal covered below.
            Point3::new(0.5, 0.0, 0.0),
            Vec3::new(0.0, 0.28, 0.96),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    }
}

fn common_support_fixture(
    placement: Placement,
    intervals: [[f64; 2]; 2],
    reversed_axes: [bool; 2],
) -> Fixture {
    common_support_fixture_with_radii(placement, intervals, reversed_axes, [RADIUS; 2])
}

fn common_support_fixture_with_radii(
    placement: Placement,
    intervals: [[f64; 2]; 2],
    reversed_axes: [bool; 2],
    radii: [f64; 2],
) -> Fixture {
    let frame = common_support_frame(placement);
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let bodies: [BodyId; 2] = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        core::array::from_fn(|operand| {
            let [low, high] = intervals[operand];
            assert!(low < high);
            let reversed = reversed_axes[operand];
            let origin = axial_point(frame, if reversed { high } else { low });
            let source_frame = if reversed {
                Frame::new(origin, -frame.z(), frame.x()).unwrap()
            } else {
                frame.with_origin(origin)
            };
            edit.create_cylinder(CylinderRequest::new(
                source_frame,
                radii[operand],
                high - low,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body()
        })
    };
    Fixture {
        session,
        part_id,
        outer: bodies[0].clone(),
        inner: bodies[1].clone(),
        frame,
    }
}

fn axial_point(frame: Frame, parameter: f64) -> Point3 {
    let origin = frame.origin();
    let axis = frame.z();
    Point3::new(
        origin.x + axis.x * parameter,
        origin.y + axis.y * parameter,
        origin.z + axis.z * parameter,
    )
}

fn run_common_support(
    fixture: &mut Fixture,
    operation: BooleanOperation,
    swapped: bool,
    settings: OperationSettings,
) -> OperationOutcome<BooleanOutcome> {
    let bodies = if swapped {
        [fixture.inner.clone(), fixture.outer.clone()]
    } else {
        [fixture.outer.clone(), fixture.inner.clone()]
    };
    fixture
        .session
        .edit_part(fixture.part_id.clone())
        .unwrap()
        .boolean_bodies(
            BooleanBodiesRequest::new(operation, bodies[0].clone(), bodies[1].clone())
                .with_settings(settings),
        )
        .unwrap()
}

fn usage_at(
    outcome: &OperationOutcome<BooleanOutcome>,
    stage: kernel::StageId,
    resource: ResourceKind,
) -> Option<u64> {
    outcome
        .report()
        .usage()
        .iter()
        .find(|usage| usage.stage == stage && usage.resource == resource)
        .map(|usage| usage.consumed)
}

fn settings_at(stage: kernel::StageId, allowed: u64) -> OperationSettings {
    OperationSettings::new().with_budget_overrides(
        BudgetPlan::new([LimitSpec::new(
            stage,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            allowed,
        )])
        .unwrap(),
    )
}

fn assert_work_limit(
    outcome: &OperationOutcome<BooleanOutcome>,
    stage: kernel::StageId,
    expected_work: u64,
) {
    let limit = *outcome
        .report()
        .limit_events()
        .first()
        .expect("common-support N-1 refusal recorded no limit event");
    assert_eq!(limit.stage, stage);
    assert_eq!(limit.resource, ResourceKind::Work);
    assert_eq!(limit.allowed, expected_work - 1);
    assert_eq!(limit.consumed, expected_work);
    assert_eq!(outcome.result().unwrap_err().limit(), Some(limit));
    assert_eq!(outcome.report().limit_events(), &[limit]);
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct CommonLineageSummary {
    derived_faces: usize,
    derived_edges: usize,
    split_faces: usize,
    merge_faces: usize,
    merge_edges: usize,
}

impl CommonLineageSummary {
    const fn event_count(self) -> usize {
        self.derived_faces
            + self.derived_edges
            + self.split_faces
            + self.merge_faces
            + self.merge_edges
    }
}

fn assert_complete_common_lineage(
    fixture: &Fixture,
    created: &kernel::BooleanCreatedResult,
) -> CommonLineageSummary {
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    let source_entities = [fixture.outer.clone(), fixture.inner.clone()]
        .into_iter()
        .flat_map(|body| {
            let body = part.body(body).unwrap();
            body.faces()
                .unwrap()
                .into_iter()
                .map(JournalEntity::Face)
                .chain(body.edges().unwrap().into_iter().map(JournalEntity::Edge))
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let result_entities = created
        .bodies()
        .iter()
        .flat_map(|body| {
            let body = part.body(body.clone()).unwrap();
            body.faces()
                .unwrap()
                .into_iter()
                .map(JournalEntity::Face)
                .chain(body.edges().unwrap().into_iter().map(JournalEntity::Edge))
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let mut covered = Vec::with_capacity(result_entities.len());
    let mut summary = CommonLineageSummary::default();
    for event in created.journal().lineage() {
        match event {
            LineageView::DerivedFrom { derived, source } => {
                assert!(
                    source_entities.contains(&source),
                    "derived source escaped both operands"
                );
                assert!(
                    result_entities.contains(&derived),
                    "derived result escaped output bodies"
                );
                assert_eq!(derived.kind(), source.kind());
                assert!(
                    !covered.contains(&derived),
                    "result identity received duplicate lineage"
                );
                match derived.kind() {
                    EntityKind::Face => summary.derived_faces += 1,
                    EntityKind::Edge => summary.derived_edges += 1,
                    kind => panic!("common-support lineage derived unsupported {kind:?}"),
                }
                covered.push(derived);
            }
            LineageView::Split { source, pieces } => {
                assert!(
                    source_entities.contains(&source),
                    "split source escaped both operands"
                );
                assert_eq!(source.kind(), EntityKind::Face);
                let pieces = pieces.collect::<Vec<_>>();
                assert_eq!(pieces.len(), 2);
                for piece in pieces {
                    assert_eq!(piece.kind(), EntityKind::Face);
                    assert!(result_entities.contains(&piece));
                    assert!(
                        !covered.contains(&piece),
                        "split piece received duplicate lineage"
                    );
                    covered.push(piece);
                }
                summary.split_faces += 1;
            }
            LineageView::Merge { sources, result } => {
                assert!(result_entities.contains(&result));
                assert!(
                    !covered.contains(&result),
                    "merge result received duplicate lineage"
                );
                let sources = sources.collect::<Vec<_>>();
                assert_eq!(sources.len(), 2);
                assert!(sources.iter().all(|source| source.kind() == result.kind()));
                assert!(
                    sources
                        .iter()
                        .all(|source| source_entities.contains(source))
                );
                match result.kind() {
                    EntityKind::Face => summary.merge_faces += 1,
                    EntityKind::Edge => summary.merge_edges += 1,
                    kind => panic!("common-support lineage merged unsupported {kind:?}"),
                }
                covered.push(result);
            }
            lineage => panic!("common-support result published unexpected lineage {lineage:?}"),
        }
    }
    assert_eq!(covered.len(), result_entities.len());
    assert!(
        result_entities
            .iter()
            .all(|entity| covered.contains(entity))
    );
    assert_eq!(summary.event_count(), created.journal().lineage_count());
    summary
}

fn assert_span_properties(fixture: &Fixture, body: BodyId, span: [f64; 2], exact_budget: bool) {
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    let properties = if exact_budget {
        certified_properties_at_exact_budget(
            &part,
            body,
            COMMON_SUPPORT_PROPERTIES_WORK,
            "common-support band",
        )
    } else {
        let outcome = part
            .body_properties(BodyPropertiesRequest::new(body))
            .unwrap();
        assert_eq!(
            outcome
                .report()
                .usage()
                .iter()
                .find(|usage| {
                    usage.stage == kernel::BODY_PROPERTIES_ANALYTIC_WORK
                        && usage.resource == ResourceKind::Work
                })
                .map(|usage| usage.consumed),
            Some(COMMON_SUPPORT_PROPERTIES_WORK)
        );
        let BodyPropertiesOutcome::Certified {
            properties,
            full_check,
        } = outcome.into_result().unwrap()
        else {
            panic!("common-support analytic properties were refused")
        };
        assert_eq!(full_check.outcome(), CheckOutcome::Valid);
        properties
    };

    let height = span[1] - span[0];
    let volume = core::f64::consts::PI * height;
    assert_scalar_matches_analytic(properties.volume(), volume, "common-support volume");
    assert_scalar_matches_analytic(
        properties.surface_area(),
        2.0 * core::f64::consts::PI * (height + 1.0),
        "common-support surface area",
    );
    assert_point_matches_analytic(
        properties.centroid(),
        axial_point(fixture.frame, (span[0] + span[1]) / 2.0),
    );
    let transverse = volume * (3.0 + height.powi(2)) / 12.0;
    let axial = volume / 2.0;
    assert_inertia_matches_analytic(
        properties.centroidal_inertia().value(),
        properties.centroidal_inertia().error_bound(),
        rotate_tensor(
            fixture.frame,
            [
                [transverse, 0.0, 0.0],
                [0.0, transverse, 0.0],
                [0.0, 0.0, axial],
            ],
        ),
    );
}

fn assert_common_support_result(
    fixture: &mut Fixture,
    before: FixtureSignature,
    outcome: OperationOutcome<BooleanOutcome>,
    expected: &[[f64; 2]],
    check_properties: bool,
    label: &str,
) -> Vec<Vec<u8>> {
    assert_eq!(
        usage_at(&outcome, BOOLEAN_BSP_WORK, ResourceKind::Work),
        Some(COMMON_SUPPORT_RELATION_WORK),
        "{label}: relation work changed"
    );
    assert_eq!(
        usage_at(&outcome, BOOLEAN_REALIZED_VERTICES, ResourceKind::Items),
        Some(0),
        "{label}: endpoint-free result allocated vertices"
    );
    let expected_work = COMMON_SUPPORT_BAND_WORK * expected.len() as u64;
    assert_eq!(
        usage_at(&outcome, BOOLEAN_POST_SELECTION_WORK, ResourceKind::Work),
        Some(expected_work),
        "{label}: analytic-shell work changed"
    );
    let result = outcome.into_result().unwrap();
    if expected.is_empty() {
        assert!(
            matches!(result, BooleanOutcome::Success(BooleanResult::ProvenEmpty)),
            "{label}: empty interval result returned {result:#?}"
        );
        assert_eq!(
            fixture_signature(fixture),
            before,
            "{label}: empty result mutated the part"
        );
        return Vec::new();
    }

    let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
        panic!("{label}: common-support Boolean returned {result:#?}")
    };
    assert_eq!(created.bodies().len(), expected.len());
    assert_eq!(created.reports().len(), expected.len());
    assert!(
        created
            .reports()
            .iter()
            .all(|report| report.report().outcome() == CheckOutcome::Valid)
    );
    let summary = assert_complete_common_lineage(fixture, &created);
    if expected.len() == 2 {
        assert_eq!(
            summary,
            CommonLineageSummary {
                derived_faces: 4,
                derived_edges: 4,
                split_faces: 1,
                merge_faces: 0,
                merge_edges: 0,
            },
            "{label}: two-band split lineage changed"
        );
    } else {
        assert_eq!(
            summary.event_count(),
            5,
            "{label}: one-band lineage changed"
        );
        assert_eq!(
            summary.split_faces, 0,
            "{label}: one band spuriously split a side"
        );
    }

    let bodies = created.bodies().to_vec();
    let mut exports = Vec::with_capacity(bodies.len());
    {
        let part = fixture.session.part(fixture.part_id.clone()).unwrap();
        for body in &bodies {
            assert_eq!(body_topology(&part, body.clone()), CYLINDER_TOPOLOGY);
            let full = part
                .check_body(CheckBodyRequest::new(body.clone(), CheckLevel::Full))
                .unwrap()
                .into_result()
                .unwrap();
            assert_eq!(full.outcome(), CheckOutcome::Valid, "{label}: {full:#?}");
            let first = part
                .export_xt(ExportXtRequest::new(body.clone()))
                .unwrap()
                .into_result()
                .unwrap();
            let second = part
                .export_xt(ExportXtRequest::new(body.clone()))
                .unwrap()
                .into_result()
                .unwrap();
            assert_eq!(
                first.bytes(),
                second.bytes(),
                "{label}: repeat export changed bytes"
            );
            exports.push(first.bytes().to_vec());
        }
    }
    if check_properties {
        for (body, span) in bodies.iter().cloned().zip(expected.iter().copied()) {
            assert_span_properties(fixture, body, span, false);
        }
    }
    assert_source_bodies_preserved(fixture, 2 + expected.len());
    exports
}

#[test]
fn exact_common_support_executes_the_complete_twenty_row_interval_table() {
    for row in SEMANTIC_ROWS {
        let mut fixture = common_support_fixture(Placement::World, row.intervals, [false; 2]);
        let before = fixture_signature(&fixture);
        let (operation, swapped) = row.operation.request();
        let outcome =
            run_common_support(&mut fixture, operation, swapped, OperationSettings::new());
        assert_common_support_result(&mut fixture, before, outcome, row.expected, true, row.name);
    }
}

#[test]
fn exact_common_support_crossing_is_deterministic_across_rigid_frames_and_orders() {
    let intervals = [[-2.0, 1.0], [-1.0, 2.0]];
    for placement in [Placement::World, Placement::Oblique] {
        for reversed_axes in [[false, false], [false, true], [true, false], [true, true]] {
            for (operation, expected, request_orders) in [
                (
                    BooleanOperation::Unite,
                    &[[-2.0, 2.0]][..],
                    &[false, true][..],
                ),
                (
                    BooleanOperation::Intersect,
                    &[[-1.0, 1.0]][..],
                    &[false, true][..],
                ),
                (
                    BooleanOperation::Subtract,
                    &[[-2.0, -1.0]][..],
                    &[false][..],
                ),
                (BooleanOperation::Subtract, &[[1.0, 2.0]][..], &[true][..]),
            ] {
                let mut canonical: Option<Vec<Vec<u8>>> = None;
                for swapped in request_orders {
                    for repeat in 0..2 {
                        let mut fixture =
                            common_support_fixture(placement, intervals, reversed_axes);
                        let before = fixture_signature(&fixture);
                        let outcome = run_common_support(
                            &mut fixture,
                            operation,
                            *swapped,
                            OperationSettings::new(),
                        );
                        let label = format!(
                            "{placement:?} {reversed_axes:?} {operation:?} swapped={swapped} repeat={repeat}"
                        );
                        let exports = assert_common_support_result(
                            &mut fixture,
                            before,
                            outcome,
                            expected,
                            false,
                            &label,
                        );
                        if let Some(canonical) = canonical.as_ref() {
                            assert_eq!(exports.len(), canonical.len());
                            for (actual, expected) in exports.iter().zip(canonical) {
                                assert_xt_equal(
                                    actual,
                                    expected,
                                    "common-support request order or repeat changed X_T",
                                );
                            }
                        } else {
                            canonical = Some(exports.clone());
                        }
                        for bytes in &exports {
                            assert_fast_self_import(&mut fixture.session, bytes);
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn exact_shared_endpoint_merges_are_not_inferred_across_one_ulp() {
    let cases = [
        (
            -2.0,
            CommonLineageSummary {
                derived_faces: 1,
                derived_edges: 1,
                split_faces: 0,
                merge_faces: 2,
                merge_edges: 1,
            },
        ),
        (
            (-2.0_f64).next_up(),
            CommonLineageSummary {
                derived_faces: 2,
                derived_edges: 2,
                split_faces: 0,
                merge_faces: 1,
                merge_edges: 0,
            },
        ),
    ];
    let mut canonical_geometry: Option<Vec<u8>> = None;
    for (second_low, expected_lineage) in cases {
        let mut fixture = common_support_fixture(
            Placement::World,
            [[-2.0, 2.0], [second_low, 0.0]],
            [false; 2],
        );
        let outcome = run_common_support(
            &mut fixture,
            BooleanOperation::Unite,
            false,
            OperationSettings::new(),
        );
        let result = outcome.into_result().unwrap();
        let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
            panic!("shared-end Unite returned {result:#?}")
        };
        assert_eq!(
            assert_complete_common_lineage(&fixture, &created),
            expected_lineage
        );
        let body = created.bodies()[0].clone();
        let bytes = fixture
            .session
            .part(fixture.part_id.clone())
            .unwrap()
            .export_xt(ExportXtRequest::new(body))
            .unwrap()
            .into_result()
            .unwrap()
            .bytes()
            .to_vec();
        if let Some(canonical) = canonical_geometry.as_ref() {
            assert_xt_equal(
                &bytes,
                canonical,
                "one-ULP endpoint separation changed union geometry",
            );
        } else {
            canonical_geometry = Some(bytes);
        }
        assert_source_bodies_preserved(&fixture, 3);
    }
}

#[test]
fn one_ulp_radius_drift_remains_an_allocation_free_boundary_refusal() {
    for settings in [
        OperationSettings::new(),
        OperationSettings::new().with_tolerances(Tolerances::with_linear(1.0e-6).unwrap()),
    ] {
        let mut fixture = common_support_fixture_with_radii(
            Placement::World,
            [[-2.0, 1.0], [-1.0, 2.0]],
            [false; 2],
            [RADIUS, RADIUS.next_up()],
        );
        let before = fixture_signature(&fixture);
        let outcome = run_common_support(&mut fixture, BooleanOperation::Unite, false, settings);
        assert_eq!(
            usage_at(&outcome, BOOLEAN_BSP_WORK, ResourceKind::Work),
            Some(COMMON_SUPPORT_RELATION_WORK)
        );
        assert_eq!(
            usage_at(&outcome, BOOLEAN_POST_SELECTION_WORK, ResourceKind::Work),
            Some(0)
        );
        assert!(matches!(
            outcome.into_result().unwrap(),
            BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
        ));
        assert_eq!(fixture_signature(&fixture), before);
        assert_source_bodies_preserved(&fixture, 2);
    }
}

fn assert_common_support_realization_frontier(
    intervals: [[f64; 2]; 2],
    operation: CommonSupportOperation,
    expected: &[[f64; 2]],
) {
    let (request, swapped) = operation.request();
    let expected_work = COMMON_SUPPORT_BAND_WORK * expected.len() as u64;

    let mut baseline = common_support_fixture(Placement::World, intervals, [false; 2]);
    let outcome = run_common_support(&mut baseline, request, swapped, OperationSettings::new());
    assert_eq!(
        usage_at(&outcome, BOOLEAN_POST_SELECTION_WORK, ResourceKind::Work),
        Some(expected_work)
    );
    assert!(matches!(
        outcome.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut admitted = common_support_fixture(Placement::World, intervals, [false; 2]);
    let outcome = run_common_support(
        &mut admitted,
        request,
        swapped,
        settings_at(BOOLEAN_POST_SELECTION_WORK, expected_work),
    );
    assert!(matches!(
        outcome.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut denied = common_support_fixture(Placement::World, intervals, [false; 2]);
    let before = fixture_signature(&denied);
    let outcome = run_common_support(
        &mut denied,
        request,
        swapped,
        settings_at(BOOLEAN_POST_SELECTION_WORK, expected_work - 1),
    );
    assert_work_limit(&outcome, BOOLEAN_POST_SELECTION_WORK, expected_work);
    assert_eq!(fixture_signature(&denied), before);
    assert_source_bodies_preserved(&denied, 2);

    let replay = run_common_support(&mut denied, request, swapped, OperationSettings::new());
    assert_common_support_result(
        &mut denied,
        before,
        replay,
        expected,
        false,
        "post-refusal replay",
    );
}

#[test]
fn exact_common_support_relation_and_realization_work_have_atomic_frontiers() {
    let intervals = [[-2.0, 1.0], [-1.0, 2.0]];
    let mut admitted = common_support_fixture(Placement::World, intervals, [false; 2]);
    let outcome = run_common_support(
        &mut admitted,
        BooleanOperation::Intersect,
        false,
        settings_at(BOOLEAN_BSP_WORK, COMMON_SUPPORT_RELATION_WORK),
    );
    assert!(matches!(
        outcome.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut denied = common_support_fixture(Placement::World, intervals, [false; 2]);
    let before = fixture_signature(&denied);
    let outcome = run_common_support(
        &mut denied,
        BooleanOperation::Intersect,
        false,
        settings_at(BOOLEAN_BSP_WORK, COMMON_SUPPORT_RELATION_WORK - 1),
    );
    assert_work_limit(&outcome, BOOLEAN_BSP_WORK, COMMON_SUPPORT_RELATION_WORK);
    assert_eq!(fixture_signature(&denied), before);
    let replay = run_common_support(
        &mut denied,
        BooleanOperation::Intersect,
        false,
        OperationSettings::new(),
    );
    assert_common_support_result(
        &mut denied,
        before,
        replay,
        &[[-1.0, 1.0]],
        false,
        "relation post-refusal replay",
    );

    assert_common_support_realization_frontier(
        intervals,
        CommonSupportOperation::Intersect,
        &[[-1.0, 1.0]],
    );
    assert_common_support_realization_frontier(
        [[-2.0, 2.0], [-1.0, 1.0]],
        CommonSupportOperation::AMinusB,
        &[[-2.0, -1.0], [1.0, 2.0]],
    );
}

#[test]
fn common_support_band_properties_accept_n_and_refuse_n_minus_one() {
    let mut fixture = common_support_fixture(
        Placement::Oblique,
        [[-2.0, 1.0], [-1.0, 2.0]],
        [true, false],
    );
    let outcome = run_common_support(
        &mut fixture,
        BooleanOperation::Intersect,
        false,
        OperationSettings::new(),
    );
    let result = outcome.into_result().unwrap();
    let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
        panic!("common-support property fixture returned {result:#?}")
    };
    assert_span_properties(&fixture, created.bodies()[0].clone(), [-1.0, 1.0], true);
}
