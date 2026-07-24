//! Facade-only lifecycle evidence for cylinder lenses with coincident cap cells.
//! Wall-time budget: less than 60 seconds for the full case/direction/order matrix.

use super::*;

fn coincident_cap_shell_stage() -> kernel::StageId {
    kernel::StageId::new("ktopo.check.mixed-profile-prism-work").unwrap()
}

#[derive(Debug, Clone, Copy)]
struct AxialOverlapCase {
    name: &'static str,
    outer: [f64; 2],
    inner: [f64; 2],
    overlap: [f64; 2],
}

const AXIAL_OVERLAPS: [AxialOverlapCase; 3] = [
    AxialOverlapCase {
        name: "equal height",
        outer: [-1.0, 1.0],
        inner: [-1.0, 1.0],
        overlap: [-1.0, 1.0],
    },
    AxialOverlapCase {
        name: "shared lower end",
        outer: [-2.0, 1.0],
        inner: [-2.0, 2.0],
        overlap: [-2.0, 1.0],
    },
    AxialOverlapCase {
        name: "shared upper end",
        outer: [-2.0, 2.0],
        inner: [-1.0, 2.0],
        overlap: [-1.0, 2.0],
    },
];

#[derive(Debug, Clone, Copy)]
struct LensPrismOracle {
    volume: f64,
    surface_area: f64,
    centroid: Point3,
    centroidal_inertia: [[f64; 3]; 3],
}

fn independent_lens_prism_oracle(case: AxialOverlapCase, frame: Frame) -> LensPrismOracle {
    let height = case.overlap[1] - case.overlap[0];
    let axial_center = (case.overlap[0] + case.overlap[1]) / 2.0;
    let pi = core::f64::consts::PI;
    let root_three = 3.0_f64.sqrt();

    // Direct integration of two unit disks whose centers are one unit apart.
    // L is the lens area, while X and Y are its raw planar second moments.
    let lens_area = 2.0 * pi / 3.0 - root_three / 2.0;
    let x_squared = pi / 3.0 - 9.0 * root_three / 16.0;
    let y_squared = pi / 6.0 - 3.0 * root_three / 16.0;
    let axial_squared = lens_area * height.powi(3) / 12.0;
    let local_inertia = [
        [height * y_squared + axial_squared, 0.0, 0.0],
        [0.0, height * x_squared + axial_squared, 0.0],
        [0.0, 0.0, height * (x_squared + y_squared)],
    ];

    LensPrismOracle {
        volume: height * lens_area,
        surface_area: 2.0 * lens_area + height * 4.0 * pi / 3.0,
        centroid: frame.point_at(0.0, 0.0, axial_center),
        centroidal_inertia: rotate_tensor(frame, local_inertia),
    }
}

fn fixture(case: AxialOverlapCase, placement: Placement, antiparallel: bool) -> Fixture {
    fixture_with_axial_intervals_and_inner_direction(
        placement,
        case.outer,
        case.inner,
        antiparallel,
    )
}

fn charged_work(
    outcome: &OperationOutcome<BooleanOutcome>,
    stage: kernel::StageId,
    label: &str,
) -> u64 {
    outcome
        .report()
        .usage()
        .iter()
        .find(|usage| usage.stage == stage && usage.resource == ResourceKind::Work)
        .unwrap_or_else(|| panic!("{label} did not charge {stage:?}"))
        .consumed
}

fn certified_properties(
    part: &kernel::Part<'_>,
    body: BodyId,
    case: AxialOverlapCase,
) -> kernel::BodyProperties {
    let outcome = part
        .body_properties(BodyPropertiesRequest::new(body))
        .unwrap();
    let work = outcome
        .report()
        .usage()
        .iter()
        .find(|usage| {
            usage.stage == kernel::BODY_PROPERTIES_ANALYTIC_WORK
                && usage.resource == ResourceKind::Work
        })
        .unwrap_or_else(|| panic!("{} properties charged no analytic work", case.name));
    assert!(work.consumed > 0, "{} property work not metered", case.name);
    let BodyPropertiesOutcome::Certified {
        properties,
        full_check,
    } = outcome.into_result().unwrap()
    else {
        panic!("{} properties were not certified", case.name)
    };
    assert_eq!(full_check.outcome(), CheckOutcome::Valid);
    properties
}

fn assert_created_lens(
    fixture: &Fixture,
    case: AxialOverlapCase,
    outcome: OperationOutcome<BooleanOutcome>,
) -> Vec<u8> {
    let realization_work = charged_work(
        &outcome,
        BOOLEAN_POST_SELECTION_WORK,
        "coincident-cap Intersect realization",
    );
    let shell_work = charged_work(
        &outcome,
        coincident_cap_shell_stage(),
        "coincident-cap Intersect shell proof",
    );
    let result = outcome.into_result().unwrap();
    let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
        panic!(
            "{} Intersect returned {result:#?}; realization_work={realization_work}, \
             shell_work={shell_work}",
            case.name
        )
    };
    assert!(
        realization_work > 0,
        "{} realization work not metered",
        case.name
    );
    assert!(shell_work > 0, "{} shell-proof work not metered", case.name);
    assert_eq!(created.bodies().len(), 1);
    assert_eq!(created.reports().len(), 1);
    assert_eq!(created.reports()[0].report().outcome(), CheckOutcome::Valid);

    let body = created.bodies()[0].clone();
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    assert_eq!(body_topology(&part, body.clone()), [4, 6, 4]);
    let full = part
        .check_body(CheckBodyRequest::new(body.clone(), CheckLevel::Full))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(full.outcome(), CheckOutcome::Valid, "{full:#?}");

    let properties = certified_properties(&part, body.clone(), case);
    let oracle = independent_lens_prism_oracle(case, fixture.frame);
    assert_scalar_matches_analytic(properties.volume(), oracle.volume, "volume");
    assert_scalar_matches_analytic(
        properties.surface_area(),
        oracle.surface_area,
        "surface area",
    );
    assert_point_matches_analytic(properties.centroid(), oracle.centroid);
    assert_inertia_matches_analytic(
        properties.centroidal_inertia().value(),
        properties.centroidal_inertia().error_bound(),
        oracle.centroidal_inertia,
    );

    let first = part
        .export_xt(ExportXtRequest::new(body.clone()))
        .unwrap()
        .into_result()
        .unwrap();
    let second = part
        .export_xt(ExportXtRequest::new(body))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(first.bytes(), second.bytes());
    first.bytes().to_vec()
}

#[test]
fn coincident_cap_intersections_full_commit_against_one_table_driven_oracle_matrix() {
    for case in AXIAL_OVERLAPS {
        for placement in [Placement::World, Placement::Oblique] {
            for antiparallel in [false, true] {
                // Axis-reversed charts can preserve a distinct signed zero;
                // caller order and repeated evaluation must not affect bytes
                // within one authored direction.
                let mut canonical_bytes: Option<Vec<u8>> = None;
                for swapped in [false, true] {
                    for _ in 0..2 {
                        let mut fixture = fixture(case, placement, antiparallel);
                        assert_source_bodies_preserved(&fixture, 2);
                        let outcome = run(&mut fixture, swapped, OperationSettings::new());
                        let bytes = assert_created_lens(&fixture, case, outcome);
                        assert_source_bodies_preserved(&fixture, 3);
                        if let Some(canonical) = canonical_bytes.as_ref() {
                            assert_xt_equal(
                                &bytes,
                                canonical,
                                "operand swap or repeat changed direction-local X_T bytes",
                            );
                        } else {
                            canonical_bytes = Some(bytes.clone());
                        }
                        assert_fast_self_import(&mut fixture.session, &bytes);
                    }
                }
            }
        }
    }
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

fn assert_operation_budget_frontier(
    case: AxialOverlapCase,
    antiparallel: bool,
    stage: kernel::StageId,
) {
    let mut baseline = fixture(case, Placement::World, antiparallel);
    let baseline_outcome = run(&mut baseline, false, OperationSettings::new());
    let measured = charged_work(&baseline_outcome, stage, case.name);
    assert!(measured > 0, "{} metered no work at {stage:?}", case.name);
    assert!(matches!(
        baseline_outcome.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut admitted = fixture(case, Placement::World, antiparallel);
    let admitted_outcome = run(&mut admitted, false, settings_at(stage, measured));
    assert!(matches!(
        admitted_outcome.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut denied = fixture(case, Placement::World, antiparallel);
    let before = fixture_signature(&denied);
    let denied_outcome = run(&mut denied, false, settings_at(stage, measured - 1));
    let usage = *denied_outcome
        .report()
        .limit_events()
        .first()
        .expect("N-1 refusal recorded no limit event");
    assert_eq!(usage.stage, stage);
    assert_eq!(usage.resource, ResourceKind::Work);
    assert_eq!(usage.allowed, measured - 1);
    assert_eq!(usage.consumed, measured);
    assert_eq!(denied_outcome.result().unwrap_err().limit(), Some(usage));
    assert_eq!(fixture_signature(&denied), before);
}

#[test]
fn coincident_cap_intersection_work_frontiers_accept_n_and_refuse_n_minus_one_atomically() {
    for case in [AXIAL_OVERLAPS[0], AXIAL_OVERLAPS[1]] {
        for antiparallel in [false, true] {
            assert_operation_budget_frontier(case, antiparallel, BOOLEAN_POST_SELECTION_WORK);
            assert_operation_budget_frontier(case, antiparallel, coincident_cap_shell_stage());

            let mut fixture = fixture(case, Placement::World, antiparallel);
            let outcome = run(&mut fixture, false, OperationSettings::new());
            let result = outcome.into_result().unwrap();
            let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
                panic!("{} property-budget fixture was refused", case.name)
            };
            let part = fixture.session.part(fixture.part_id.clone()).unwrap();
            let _ =
                certified_properties_at_exact_budget(&part, created.bodies()[0].clone(), case.name);
        }
    }
}
