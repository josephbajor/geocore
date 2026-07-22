//! Facade-only lifecycle evidence for disjoint radial-miss Unite/Subtract.
//! Wall-time budget: less than 60 seconds for the rigid-frame/order matrix.

use super::*;

// Identity-copy precharge: body 1 + regions 2 + shell 1 + faces 6 +
// loops 4 + fin/pcurve pairs 8 + edge/curve pairs 4 = 26.
const ONE_CYLINDER_COPY_WORK: u64 = 26;
const TWO_CYLINDER_COPY_WORK: u64 = 2 * ONE_CYLINDER_COPY_WORK;
const ONE_CYLINDER_COPY_IDENTITIES: usize = 26;
const RADIAL_MISS_RELATION_WORK: u64 = 64;

#[derive(Debug, Clone, Copy)]
struct DisjointCylinderCase {
    radii: [f64; 2],
    radial_offset: [f64; 2],
}

const DISJOINT: DisjointCylinderCase = DisjointCylinderCase {
    radii: [0.75, 1.25],
    radial_offset: [1.5, 2.0],
};

#[derive(Debug, Clone, Copy)]
enum SetOperation {
    Unite,
    Subtract { swapped: bool },
}

impl SetOperation {
    const fn operation(self) -> BooleanOperation {
        match self {
            Self::Unite => BooleanOperation::Unite,
            Self::Subtract { .. } => BooleanOperation::Subtract,
        }
    }

    const fn swapped(self) -> bool {
        match self {
            Self::Unite => false,
            Self::Subtract { swapped } => swapped,
        }
    }

    const fn result_body_count(self) -> usize {
        match self {
            Self::Unite => 2,
            Self::Subtract { .. } => 1,
        }
    }

    const fn realization_work(self) -> u64 {
        match self {
            Self::Unite => TWO_CYLINDER_COPY_WORK,
            Self::Subtract { .. } => ONE_CYLINDER_COPY_WORK,
        }
    }
}

struct CreatedEvidence {
    exports: Vec<Vec<u8>>,
    report: kernel::OperationReport,
}

fn fixture(placement: Placement, antiparallel: bool) -> Fixture {
    let distance_squared = DISJOINT.radial_offset[0].powi(2) + DISJOINT.radial_offset[1].powi(2);
    assert!(distance_squared > (DISJOINT.radii[0] + DISJOINT.radii[1]).powi(2));

    let frame = shared_frame(placement);
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (outer, inner) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let outer = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, -1.0)),
                DISJOINT.radii[0],
                2.0,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let inner_origin = frame.point_at(
            DISJOINT.radial_offset[0],
            DISJOINT.radial_offset[1],
            if antiparallel { 1.0 } else { -1.0 },
        );
        let inner_frame = if antiparallel {
            Frame::new(inner_origin, -frame.z(), frame.x()).unwrap()
        } else {
            frame.with_origin(inner_origin)
        };
        let inner = edit
            .create_cylinder(CylinderRequest::new(inner_frame, DISJOINT.radii[1], 2.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (outer, inner)
    };
    Fixture {
        session,
        part_id,
        outer,
        inner,
        frame,
    }
}

fn run_set_operation(
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

fn assert_full_valid(created: &kernel::BooleanCreatedResult) {
    assert_eq!(created.reports().len(), created.bodies().len());
    for (report, body) in created.reports().iter().zip(created.bodies()) {
        assert_eq!(report.body(), *body);
        assert_eq!(report.report().level(), CheckLevel::Full);
        assert_eq!(report.report().outcome(), CheckOutcome::Valid);
        assert!(report.report().faults().is_empty());
        assert!(report.report().gaps().is_empty());
    }
}

fn source_copy_lineage(fixture: &Fixture, created: &kernel::BooleanCreatedResult) -> Vec<BodyId> {
    assert_eq!(created.journal().part(), fixture.part_id);
    let mutations = created.journal().mutations().collect::<Vec<_>>();
    assert!(!mutations.is_empty());
    assert!(
        mutations
            .iter()
            .all(|mutation| mutation.kind() == MutationKind::Created)
    );
    assert_eq!(created.journal().lineage_count(), mutations.len());
    assert_eq!(
        mutations.len(),
        ONE_CYLINDER_COPY_IDENTITIES * created.bodies().len()
    );

    let mut derived = Vec::with_capacity(mutations.len());
    let mut body_pairs = Vec::new();
    let mut face_pairs = Vec::new();
    for event in created.journal().lineage() {
        let LineageView::DerivedFrom {
            derived: derived_entity,
            source,
        } = event
        else {
            panic!("whole-cylinder copy lineage must contain only DerivedFrom events")
        };
        assert!(!derived.contains(&derived_entity));
        assert_eq!(derived_entity.kind(), source.kind());
        derived.push(derived_entity.clone());
        match (derived_entity, source) {
            (JournalEntity::Body(result), JournalEntity::Body(source)) => {
                body_pairs.push((result, source));
            }
            (JournalEntity::Face(result), JournalEntity::Face(source)) => {
                face_pairs.push((result, source));
            }
            _ => {}
        }
    }
    assert!(
        mutations
            .iter()
            .all(|mutation| derived.contains(mutation.entity()))
    );
    for (kind, identities_per_source) in [
        (EntityKind::Body, 1),
        (EntityKind::Region, 2),
        (EntityKind::Shell, 1),
        (EntityKind::Face, 3),
        (EntityKind::Loop, 4),
        (EntityKind::Fin, 4),
        (EntityKind::Edge, 2),
        (EntityKind::Vertex, 0),
        (EntityKind::Curve, 2),
        (EntityKind::Surface, 3),
        (EntityKind::Point, 0),
        (EntityKind::Pcurve, 4),
    ] {
        assert_eq!(
            derived
                .iter()
                .filter(|entity| entity.kind() == kind)
                .count(),
            identities_per_source * created.bodies().len(),
            "unexpected {kind:?} copy inventory"
        );
    }
    assert_eq!(body_pairs.len(), created.bodies().len());
    assert_eq!(
        body_pairs
            .iter()
            .map(|(result, _)| result.clone())
            .collect::<Vec<_>>(),
        created.bodies()
    );

    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    for (result, source) in &body_pairs {
        assert_ne!(result, source);
        let result_faces = part
            .body(result.clone())
            .unwrap()
            .faces()
            .unwrap()
            .collect::<Vec<_>>();
        let source_faces = part
            .body(source.clone())
            .unwrap()
            .faces()
            .unwrap()
            .collect::<Vec<_>>();
        assert_eq!(result_faces.len(), source_faces.len());
        assert!(result_faces.iter().all(|result_face| {
            face_pairs
                .iter()
                .filter(|(derived, source)| derived == result_face && source_faces.contains(source))
                .count()
                == 1
        }));
    }
    body_pairs.into_iter().map(|(_, source)| source).collect()
}

fn assert_analytic_cylinder(fixture: &Fixture, body: BodyId, source: BodyId) {
    let (radius, centroid) = if source == fixture.outer {
        (DISJOINT.radii[0], fixture.frame.origin())
    } else if source == fixture.inner {
        (
            DISJOINT.radii[1],
            fixture
                .frame
                .point_at(DISJOINT.radial_offset[0], DISJOINT.radial_offset[1], 0.0),
        )
    } else {
        panic!("whole-cylinder result escaped both source bodies")
    };
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    assert_eq!(body_topology(&part, body.clone()), CYLINDER_TOPOLOGY);
    let outcome = part
        .body_properties(BodyPropertiesRequest::new(body))
        .unwrap();
    let BodyPropertiesOutcome::Certified {
        properties,
        full_check,
    } = outcome.into_result().unwrap()
    else {
        panic!("whole-cylinder copy properties were not certified")
    };
    assert_eq!(full_check.level(), CheckLevel::Full);
    assert_eq!(full_check.outcome(), CheckOutcome::Valid);
    assert_scalar_matches_analytic(
        properties.volume(),
        2.0 * core::f64::consts::PI * radius.powi(2),
        "whole-cylinder volume",
    );
    assert_scalar_matches_analytic(
        properties.surface_area(),
        4.0 * core::f64::consts::PI * radius + 2.0 * core::f64::consts::PI * radius.powi(2),
        "whole-cylinder surface area",
    );
    assert_point_matches_analytic(properties.centroid(), centroid);
}

fn deterministic_exports(fixture: &mut Fixture, bodies: &[BodyId]) -> Vec<Vec<u8>> {
    let exports = {
        let part = fixture.session.part(fixture.part_id.clone()).unwrap();
        bodies
            .iter()
            .map(|body| {
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
                assert_eq!(first.bytes(), second.bytes());
                first.bytes().to_vec()
            })
            .collect::<Vec<_>>()
    };
    for bytes in &exports {
        assert_fast_self_import(&mut fixture.session, bytes);
    }
    exports
}

fn assert_created(
    fixture: &mut Fixture,
    outcome: OperationOutcome<BooleanOutcome>,
    operation: SetOperation,
) -> CreatedEvidence {
    let report = outcome.report().clone();
    let result = outcome.into_result().unwrap();
    let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
        panic!("radially disjoint {operation:?} returned {result:#?}")
    };
    assert_eq!(created.bodies().len(), operation.result_body_count());
    assert_full_valid(&created);
    let sources = source_copy_lineage(fixture, &created);
    match operation {
        SetOperation::Unite => {
            assert_eq!(sources, [fixture.outer.clone(), fixture.inner.clone()]);
        }
        SetOperation::Subtract { swapped: false } => {
            assert_eq!(sources, [fixture.outer.clone()]);
        }
        SetOperation::Subtract { swapped: true } => {
            assert_eq!(sources, [fixture.inner.clone()]);
        }
    }
    let bodies = created.bodies().to_vec();
    for (body, source) in bodies.iter().cloned().zip(sources) {
        assert_analytic_cylinder(fixture, body, source);
    }
    let exports = deterministic_exports(fixture, &bodies);
    CreatedEvidence { exports, report }
}

fn assert_same_evidence(actual: &CreatedEvidence, expected: &CreatedEvidence, label: &str) {
    assert_eq!(actual.report, expected.report, "{label}: report changed");
    assert_eq!(actual.exports.len(), expected.exports.len(), "{label}");
    for (actual, expected) in actual.exports.iter().zip(&expected.exports) {
        assert_xt_equal(actual, expected, label);
    }
}

#[test]
fn exterior_radial_miss_unite_and_ordered_subtract_copy_whole_sources_deterministically() {
    let mut executions = 0_usize;
    for placement in [Placement::World, Placement::Oblique] {
        for antiparallel in [false, true] {
            let mut canonical_unite = None;
            for swapped in [false, true] {
                for repeat in 0..2 {
                    let mut fixture = fixture(placement, antiparallel);
                    assert_source_bodies_preserved(&fixture, 2);
                    let outcome = run_set_operation(
                        &mut fixture,
                        BooleanOperation::Unite,
                        swapped,
                        OperationSettings::new(),
                    );
                    let evidence = assert_created(&mut fixture, outcome, SetOperation::Unite);
                    assert_source_bodies_preserved(&fixture, 4);
                    if let Some(canonical) = canonical_unite.as_ref() {
                        assert_same_evidence(
                            &evidence,
                            canonical,
                            &format!(
                                "{placement:?} antiparallel={antiparallel} Unite swapped={swapped} repeat={repeat}"
                            ),
                        );
                    } else {
                        canonical_unite = Some(evidence);
                    }
                    executions += 1;
                }
            }

            for swapped in [false, true] {
                let mut canonical_subtract = None;
                for repeat in 0..2 {
                    let operation = SetOperation::Subtract { swapped };
                    let mut fixture = fixture(placement, antiparallel);
                    assert_source_bodies_preserved(&fixture, 2);
                    let outcome = run_set_operation(
                        &mut fixture,
                        operation.operation(),
                        operation.swapped(),
                        OperationSettings::new(),
                    );
                    let evidence = assert_created(&mut fixture, outcome, operation);
                    assert_source_bodies_preserved(&fixture, 3);
                    if let Some(canonical) = canonical_subtract.as_ref() {
                        assert_same_evidence(
                            &evidence,
                            canonical,
                            &format!(
                                "{placement:?} antiparallel={antiparallel} Subtract swapped={swapped} repeat={repeat}"
                            ),
                        );
                    } else {
                        canonical_subtract = Some(evidence);
                    }
                    executions += 1;
                }
            }
        }
    }
    assert_eq!(executions, 32);
}

fn settings_at_realization_work(allowed: u64) -> OperationSettings {
    OperationSettings::new().with_budget_overrides(
        BudgetPlan::new([LimitSpec::new(
            BOOLEAN_POST_SELECTION_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            allowed,
        )])
        .unwrap(),
    )
}

fn realization_work(outcome: &OperationOutcome<BooleanOutcome>) -> u64 {
    outcome
        .report()
        .usage()
        .iter()
        .find(|usage| {
            usage.stage == BOOLEAN_POST_SELECTION_WORK && usage.resource == ResourceKind::Work
        })
        .expect("radial-miss set operation did not charge realization work")
        .consumed
}

fn realized_vertices(outcome: &OperationOutcome<BooleanOutcome>) -> u64 {
    outcome
        .report()
        .usage()
        .iter()
        .find(|usage| {
            usage.stage == BOOLEAN_REALIZED_VERTICES && usage.resource == ResourceKind::Items
        })
        .expect("radial-miss set operation did not report realized vertices")
        .consumed
}

#[test]
fn exterior_radial_miss_whole_copy_work_accepts_n_and_refuses_n_minus_one_atomically() {
    let operations = [
        SetOperation::Unite,
        SetOperation::Subtract { swapped: false },
        SetOperation::Subtract { swapped: true },
    ];
    for antiparallel in [false, true] {
        for operation in operations {
            let expected_work = operation.realization_work();
            let mut baseline = fixture(Placement::World, antiparallel);
            let outcome = run_set_operation(
                &mut baseline,
                operation.operation(),
                operation.swapped(),
                OperationSettings::new(),
            );
            assert_eq!(
                outcome
                    .report()
                    .usage()
                    .iter()
                    .find(|usage| {
                        usage.stage == BOOLEAN_BSP_WORK && usage.resource == ResourceKind::Work
                    })
                    .expect("radial-miss set operation did not report relation work")
                    .consumed,
                RADIAL_MISS_RELATION_WORK,
                "{operation:?}"
            );
            assert_eq!(realization_work(&outcome), expected_work, "{operation:?}");
            assert_eq!(realized_vertices(&outcome), 0, "{operation:?}");
            assert!(matches!(
                outcome.into_result().unwrap(),
                BooleanOutcome::Success(BooleanResult::Created(_))
            ));
            assert_source_bodies_preserved(&baseline, 2 + operation.result_body_count());

            let mut admitted = fixture(Placement::World, antiparallel);
            let outcome = run_set_operation(
                &mut admitted,
                operation.operation(),
                operation.swapped(),
                settings_at_realization_work(expected_work),
            );
            assert!(matches!(
                outcome.into_result().unwrap(),
                BooleanOutcome::Success(BooleanResult::Created(_))
            ));
            assert_source_bodies_preserved(&admitted, 2 + operation.result_body_count());

            let mut denied = fixture(Placement::World, antiparallel);
            let before = fixture_signature(&denied);
            let outcome = run_set_operation(
                &mut denied,
                operation.operation(),
                operation.swapped(),
                settings_at_realization_work(expected_work - 1),
            );
            let limit = *outcome
                .report()
                .limit_events()
                .first()
                .expect("radial-miss N-1 refusal recorded no limit event");
            assert_eq!(limit.stage, BOOLEAN_POST_SELECTION_WORK);
            assert_eq!(limit.resource, ResourceKind::Work);
            assert_eq!(limit.allowed, expected_work - 1);
            assert_eq!(limit.consumed, expected_work);
            assert_eq!(outcome.result().unwrap_err().limit(), Some(limit));
            assert_eq!(outcome.report().limit_events(), &[limit]);
            assert_eq!(fixture_signature(&denied), before);
            assert_source_bodies_preserved(&denied, 2);
        }
    }
}
