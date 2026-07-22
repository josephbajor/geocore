//! Desired facade contract for certified exterior radial Cylinder/Cylinder misses.
//! Wall-time budget: less than 60 seconds for the case/direction/order matrix.

use super::*;

const RADIAL_MISS_RELATION_WORK: u64 = 64;

#[derive(Debug, Clone, Copy)]
struct RadialCase {
    name: &'static str,
    radii: [f64; 2],
    radial_offset: [f64; 2],
}

const CLEAR_MISSES: [RadialCase; 2] = [
    RadialCase {
        name: "equal-radius clear miss",
        radii: [1.0, 1.0],
        radial_offset: [3.0, 0.0],
    },
    RadialCase {
        name: "unequal-radius off-axis clear miss",
        radii: [0.75, 1.25],
        radial_offset: [1.5, 2.0],
    },
];

fn radial_fixture(case: RadialCase, placement: Placement, antiparallel: bool) -> Fixture {
    assert!(case.radii.into_iter().all(|radius| radius > 0.0));
    let frame = shared_frame(placement);
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (outer, inner) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let outer = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, -1.0)),
                case.radii[0],
                2.0,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let inner_origin = frame.point_at(
            case.radial_offset[0],
            case.radial_offset[1],
            if antiparallel { 1.0 } else { -1.0 },
        );
        let inner_frame = if antiparallel {
            Frame::new(inner_origin, -frame.z(), frame.x()).unwrap()
        } else {
            frame.with_origin(inner_origin)
        };
        let inner = edit
            .create_cylinder(CylinderRequest::new(inner_frame, case.radii[1], 2.0))
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

fn assert_proven_empty_without_mutation(
    fixture: &mut Fixture,
    swapped: bool,
    label: &str,
) -> kernel::OperationReport {
    let before = fixture_signature(fixture);
    assert_source_bodies_preserved(fixture, 2);
    let outcome = run(fixture, swapped, OperationSettings::new());
    let report = outcome.report().clone();
    let result = outcome.into_result().unwrap();
    assert!(
        matches!(result, BooleanOutcome::Success(BooleanResult::ProvenEmpty)),
        "{label} returned {result:#?}"
    );
    let BooleanOutcome::Success(result) = result else {
        unreachable!()
    };
    assert!(result.is_empty(), "{label}");
    assert!(result.bodies().is_empty(), "{label}");
    assert!(result.created().is_none(), "{label}");
    assert_eq!(fixture_signature(fixture), before, "{label}");
    assert_source_bodies_preserved(fixture, 2);
    report
}

fn assert_refused_without_mutation(fixture: &mut Fixture, swapped: bool, label: &str) {
    let before = fixture_signature(fixture);
    let result = run(fixture, swapped, OperationSettings::new())
        .into_result()
        .unwrap();
    assert!(
        matches!(result, BooleanOutcome::Refused(_)),
        "{label} must remain a typed refusal, got {result:#?}"
    );
    assert_eq!(fixture_signature(fixture), before, "{label}");
    assert_source_bodies_preserved(fixture, 2);
}

#[test]
fn exterior_radial_miss_intersection_is_proven_empty_without_mutation_in_every_order() {
    let mut executions = 0_usize;
    for case in CLEAR_MISSES {
        let distance_squared = case.radial_offset[0].powi(2) + case.radial_offset[1].powi(2);
        assert!(distance_squared > (case.radii[0] + case.radii[1]).powi(2));
        for placement in [Placement::World, Placement::Oblique] {
            for antiparallel in [false, true] {
                let mut fixture = radial_fixture(case, placement, antiparallel);
                let mut canonical_report = None;
                for swapped in [false, true] {
                    for repeat in 0..2 {
                        let label = format!(
                            "{} {placement:?} antiparallel={antiparallel} swapped={swapped} repeat={repeat}",
                            case.name
                        );
                        let report =
                            assert_proven_empty_without_mutation(&mut fixture, swapped, &label);
                        if let Some(canonical) = canonical_report.as_ref() {
                            assert_eq!(
                                &report, canonical,
                                "{label}: swap or repeat changed the operation report"
                            );
                        } else {
                            canonical_report = Some(report);
                        }
                        executions += 1;
                    }
                }
            }
        }
    }
    assert_eq!(executions, 32);
}

#[test]
fn exact_radial_boundary_distinguishes_adjacent_representable_offsets() {
    let radius_sum = 2.0_f64;
    let cases = [
        ("one ULP inside tangent", radius_sum.next_down(), false),
        ("exact tangent", radius_sum, true),
        ("one ULP outside tangent", radius_sum.next_up(), true),
    ];
    assert!(cases[0].1 < radius_sum);
    assert_eq!(cases[1].1, radius_sum);
    assert!(cases[2].1 > radius_sum);

    let mut executions = 0_usize;
    for (name, separation, proven_empty) in cases {
        let case = RadialCase {
            name,
            radii: [1.0, 1.0],
            radial_offset: [separation, 0.0],
        };
        for antiparallel in [false, true] {
            let mut fixture = radial_fixture(case, Placement::World, antiparallel);
            for swapped in [false, true] {
                for repeat in 0..2 {
                    let label = format!(
                        "{name} antiparallel={antiparallel} swapped={swapped} repeat={repeat}"
                    );
                    if proven_empty {
                        let _ = assert_proven_empty_without_mutation(&mut fixture, swapped, &label);
                    } else {
                        assert_refused_without_mutation(&mut fixture, swapped, &label);
                    }
                    executions += 1;
                }
            }
        }
    }
    assert_eq!(executions, 24);
}

#[test]
fn internal_radial_nonsecancy_is_not_misreported_as_an_empty_intersection() {
    let case = RadialCase {
        name: "strict internal radial containment",
        radii: [2.0, 0.5],
        radial_offset: [0.3, 0.4],
    };
    let distance_squared = case.radial_offset[0].powi(2) + case.radial_offset[1].powi(2);
    assert!(distance_squared < (case.radii[0] - case.radii[1]).powi(2));

    let mut executions = 0_usize;
    for placement in [Placement::World, Placement::Oblique] {
        for antiparallel in [false, true] {
            let mut fixture = radial_fixture(case, placement, antiparallel);
            for swapped in [false, true] {
                for repeat in 0..2 {
                    let label = format!(
                        "{} {placement:?} antiparallel={antiparallel} swapped={swapped} repeat={repeat}",
                        case.name
                    );
                    assert_refused_without_mutation(&mut fixture, swapped, &label);
                    executions += 1;
                }
            }
        }
    }
    assert_eq!(executions, 16);
}

fn settings_at_relation_work(allowed: u64) -> OperationSettings {
    OperationSettings::new().with_budget_overrides(
        BudgetPlan::new([LimitSpec::new(
            BOOLEAN_BSP_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            allowed,
        )])
        .unwrap(),
    )
}

fn work_at(outcome: &OperationOutcome<BooleanOutcome>, stage: kernel::StageId) -> u64 {
    outcome
        .report()
        .usage()
        .iter()
        .find(|usage| usage.stage == stage && usage.resource == ResourceKind::Work)
        .unwrap_or_else(|| panic!("radial-miss Intersect did not report {stage:?} work"))
        .consumed
}

#[test]
fn exterior_radial_miss_relation_work_accepts_n_and_refuses_n_minus_one_atomically() {
    for antiparallel in [false, true] {
        let mut baseline = radial_fixture(CLEAR_MISSES[0], Placement::World, antiparallel);
        let before = fixture_signature(&baseline);
        let baseline_outcome = run(&mut baseline, false, OperationSettings::new());
        assert_eq!(
            work_at(&baseline_outcome, BOOLEAN_BSP_WORK),
            RADIAL_MISS_RELATION_WORK
        );
        assert_eq!(work_at(&baseline_outcome, BOOLEAN_POST_SELECTION_WORK), 0);
        assert!(matches!(
            baseline_outcome.into_result().unwrap(),
            BooleanOutcome::Success(BooleanResult::ProvenEmpty)
        ));
        assert_eq!(fixture_signature(&baseline), before);

        let mut admitted = radial_fixture(CLEAR_MISSES[0], Placement::World, antiparallel);
        let before = fixture_signature(&admitted);
        let admitted_outcome = run(
            &mut admitted,
            false,
            settings_at_relation_work(RADIAL_MISS_RELATION_WORK),
        );
        assert!(matches!(
            admitted_outcome.into_result().unwrap(),
            BooleanOutcome::Success(BooleanResult::ProvenEmpty)
        ));
        assert_eq!(fixture_signature(&admitted), before);

        let mut denied = radial_fixture(CLEAR_MISSES[0], Placement::World, antiparallel);
        let before = fixture_signature(&denied);
        let denied_outcome = run(
            &mut denied,
            false,
            settings_at_relation_work(RADIAL_MISS_RELATION_WORK - 1),
        );
        let limit = *denied_outcome
            .report()
            .limit_events()
            .first()
            .expect("N-1 radial-miss refusal recorded no limit event");
        assert_eq!(limit.stage, BOOLEAN_BSP_WORK);
        assert_eq!(limit.resource, ResourceKind::Work);
        assert_eq!(limit.allowed, RADIAL_MISS_RELATION_WORK - 1);
        assert_eq!(limit.consumed, RADIAL_MISS_RELATION_WORK);
        assert_eq!(denied_outcome.result().unwrap_err().limit(), Some(limit));
        assert_eq!(fixture_signature(&denied), before);
        assert_source_bodies_preserved(&denied, 2);
    }
}
