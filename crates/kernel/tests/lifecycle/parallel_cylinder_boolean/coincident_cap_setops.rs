//! Desired lifecycle contract for coincident-cap Unite and ordered Subtract.
//! Wall-time budget: less than 60 seconds for the case/direction/order matrix.

use super::*;

const FIRST_CENTER_X: f64 = -0.5;
const SECOND_CENTER_X: f64 = 0.5;

fn mixed_profile_shell_stage() -> kernel::StageId {
    kernel::StageId::new("ktopo.check.mixed-profile-prism-work").unwrap()
}

fn cap_reaching_shell_stage() -> kernel::StageId {
    kernel::StageId::new("ktopo.check.cap-reaching-cylinder-shell-work").unwrap()
}

#[derive(Debug, Clone, Copy)]
struct AxialCase {
    name: &'static str,
    first: [f64; 2],
    second: [f64; 2],
    overlap: [f64; 2],
    topology: [[usize; 3]; 3],
    volume: [[f64; 2]; 3],
    area: [[f64; 2]; 3],
}

const CASES: [AxialCase; 3] = [
    AxialCase {
        name: "equal height",
        first: [-1.0, 1.0],
        second: [-1.0, 1.0],
        overlap: [-1.0, 1.0],
        topology: [[4, 6, 4], [4, 6, 4], [4, 6, 4]],
        volume: [[8.0 / 3.0, 1.0], [2.0 / 3.0, 1.0], [2.0 / 3.0, 1.0]],
        area: [[8.0, 1.0], [14.0 / 3.0, 1.0], [14.0 / 3.0, 1.0]],
    },
    AxialCase {
        name: "shared lower end",
        first: [-2.0, 1.0],
        second: [-2.0, 2.0],
        overlap: [-2.0, 1.0],
        topology: [[5, 7, 4], [4, 6, 4], [5, 7, 4]],
        volume: [[5.0, 1.5], [1.0, 1.5], [2.0, 1.5]],
        area: [[38.0 / 3.0, 1.0], [20.0 / 3.0, 1.0], [10.0, 0.0]],
    },
    AxialCase {
        name: "shared upper end",
        first: [-2.0, 2.0],
        second: [-1.0, 2.0],
        overlap: [-1.0, 2.0],
        topology: [[5, 7, 4], [5, 7, 4], [4, 6, 4]],
        volume: [[5.0, 1.5], [2.0, 1.5], [1.0, 1.5]],
        area: [[38.0 / 3.0, 1.0], [10.0, 0.0], [20.0 / 3.0, 1.0]],
    },
];

#[derive(Debug, Clone, Copy)]
enum SetOperation {
    Unite,
    FirstMinusSecond,
    SecondMinusFirst,
}

impl SetOperation {
    const ALL: [Self; 3] = [Self::Unite, Self::FirstMinusSecond, Self::SecondMinusFirst];

    const fn index(self) -> usize {
        match self {
            Self::Unite => 0,
            Self::FirstMinusSecond => 1,
            Self::SecondMinusFirst => 2,
        }
    }

    const fn contains(self, first: bool, second: bool) -> bool {
        match self {
            Self::Unite => first || second,
            Self::FirstMinusSecond => first && !second,
            Self::SecondMinusFirst => second && !first,
        }
    }

    const fn caller_orders(self) -> &'static [bool] {
        match self {
            Self::Unite => &[false, true],
            Self::FirstMinusSecond | Self::SecondMinusFirst => &[false],
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct RawMoments {
    volume: f64,
    first: [f64; 3],
    second: [[f64; 3]; 3],
}

impl RawMoments {
    const fn zero() -> Self {
        Self {
            volume: 0.0,
            first: [0.0; 3],
            second: [[0.0; 3]; 3],
        }
    }

    fn add_scaled(&mut self, other: Self, scale: f64) {
        self.volume += scale * other.volume;
        for axis in 0..3 {
            self.first[axis] += scale * other.first[axis];
            for peer in 0..3 {
                self.second[axis][peer] += scale * other.second[axis][peer];
            }
        }
    }

    fn centroid(self) -> [f64; 3] {
        self.first.map(|value| value / self.volume)
    }

    fn centroidal_inertia(self) -> [[f64; 3]; 3] {
        let centroid = self.centroid();
        let central: [[f64; 3]; 3] = core::array::from_fn(|row| {
            core::array::from_fn(|column| {
                self.second[row][column] - self.volume * centroid[row] * centroid[column]
            })
        });
        let trace = central[0][0] + central[1][1] + central[2][2];
        core::array::from_fn(|row| {
            core::array::from_fn(|column| {
                if row == column {
                    trace - central[row][column]
                } else {
                    -central[row][column]
                }
            })
        })
    }
}

#[derive(Debug, Clone, Copy)]
struct SetOracle {
    volume: f64,
    surface_area: f64,
    centroid: Point3,
    centroidal_inertia: [[f64; 3]; 3],
}

fn cylinder_raw_moments(interval: [f64; 2], center_x: f64) -> RawMoments {
    let height = interval[1] - interval[0];
    let center_z = (interval[0] + interval[1]) / 2.0;
    let volume = core::f64::consts::PI * height;
    let cross_second = core::f64::consts::PI * height / 4.0;
    let axial_second = core::f64::consts::PI * height.powi(3) / 12.0;
    let mut second = [[0.0; 3]; 3];
    second[0][0] = cross_second + volume * center_x.powi(2);
    second[1][1] = cross_second;
    second[2][2] = axial_second + volume * center_z.powi(2);
    second[0][2] = volume * center_x * center_z;
    second[2][0] = second[0][2];
    RawMoments {
        volume,
        first: [volume * center_x, 0.0, volume * center_z],
        second,
    }
}

fn lens_raw_moments(interval: [f64; 2]) -> RawMoments {
    let height = interval[1] - interval[0];
    let center_z = (interval[0] + interval[1]) / 2.0;
    let pi = core::f64::consts::PI;
    let root_three = 3.0_f64.sqrt();
    let area = 2.0 * pi / 3.0 - root_three / 2.0;
    let x_squared = pi / 3.0 - 9.0 * root_three / 16.0;
    let y_squared = pi / 6.0 - 3.0 * root_three / 16.0;
    let volume = height * area;
    RawMoments {
        volume,
        first: [0.0, 0.0, volume * center_z],
        second: [
            [height * x_squared, 0.0, 0.0],
            [0.0, height * y_squared, 0.0],
            [
                0.0,
                0.0,
                area * (height.powi(3) / 12.0 + height * center_z.powi(2)),
            ],
        ],
    }
}

fn combined_raw_moments(case: AxialCase, operation: SetOperation) -> RawMoments {
    let first = cylinder_raw_moments(case.first, FIRST_CENTER_X);
    let second = cylinder_raw_moments(case.second, SECOND_CENTER_X);
    let lens = lens_raw_moments(case.overlap);
    let mut combined = RawMoments::zero();
    match operation {
        SetOperation::Unite => {
            combined.add_scaled(first, 1.0);
            combined.add_scaled(second, 1.0);
            combined.add_scaled(lens, -1.0);
        }
        SetOperation::FirstMinusSecond => {
            combined.add_scaled(first, 1.0);
            combined.add_scaled(lens, -1.0);
        }
        SetOperation::SecondMinusFirst => {
            combined.add_scaled(second, 1.0);
            combined.add_scaled(lens, -1.0);
        }
    }
    combined
}

fn interval_present(interval: [f64; 2], parameter: f64) -> bool {
    interval[0] < parameter && parameter < interval[1]
}

fn sorted_axial_events(case: AxialCase) -> Vec<f64> {
    let mut events = vec![case.first[0], case.first[1], case.second[0], case.second[1]];
    events.sort_by(f64::total_cmp);
    events.dedup_by(|left, right| left.to_bits() == right.to_bits());
    events
}

fn atom_areas() -> [(bool, bool, f64); 3] {
    let lens = 2.0 * core::f64::consts::PI / 3.0 - 3.0_f64.sqrt() / 2.0;
    let crescent = core::f64::consts::PI - lens;
    [
        (true, false, crescent),
        (false, true, crescent),
        (true, true, lens),
    ]
}

fn planar_boundary_area(case: AxialCase, operation: SetOperation, events: &[f64]) -> f64 {
    events
        .iter()
        .map(|&event| {
            let before = [
                case.first[0] < event && event <= case.first[1],
                case.second[0] < event && event <= case.second[1],
            ];
            let after = [
                case.first[0] <= event && event < case.first[1],
                case.second[0] <= event && event < case.second[1],
            ];
            atom_areas()
                .into_iter()
                .filter_map(|(in_first, in_second, area)| {
                    let below = operation.contains(before[0] && in_first, before[1] && in_second);
                    let above = operation.contains(after[0] && in_first, after[1] && in_second);
                    (below != above).then_some(area)
                })
                .sum::<f64>()
        })
        .sum()
}

fn lateral_boundary_area(case: AxialCase, operation: SetOperation, events: &[f64]) -> f64 {
    let arc_lengths = [
        (false, 4.0 * core::f64::consts::PI / 3.0),
        (true, 2.0 * core::f64::consts::PI / 3.0),
    ];
    events
        .windows(2)
        .map(|window| {
            let height = window[1] - window[0];
            let midpoint = window[0] * 0.5 + window[1] * 0.5;
            let first_present = interval_present(case.first, midpoint);
            let second_present = interval_present(case.second, midpoint);
            let mut perimeter = 0.0;
            if first_present {
                for &(inside_second, length) in &arc_lengths {
                    let second = second_present && inside_second;
                    if operation.contains(true, second) != operation.contains(false, second) {
                        perimeter += length;
                    }
                }
            }
            if second_present {
                for &(inside_first, length) in &arc_lengths {
                    let first = first_present && inside_first;
                    if operation.contains(first, true) != operation.contains(first, false) {
                        perimeter += length;
                    }
                }
            }
            height * perimeter
        })
        .sum()
}

fn independent_set_oracle(case: AxialCase, operation: SetOperation, frame: Frame) -> SetOracle {
    let raw = combined_raw_moments(case, operation);
    let local_centroid = raw.centroid();
    let events = sorted_axial_events(case);
    SetOracle {
        volume: raw.volume,
        surface_area: planar_boundary_area(case, operation, &events)
            + lateral_boundary_area(case, operation, &events),
        centroid: frame.point_at(local_centroid[0], local_centroid[1], local_centroid[2]),
        centroidal_inertia: rotate_tensor(frame, raw.centroidal_inertia()),
    }
}

fn exact_pi_root_three(coefficients: [f64; 2]) -> f64 {
    coefficients[0] * core::f64::consts::PI + coefficients[1] * 3.0_f64.sqrt()
}

fn fixture(case: AxialCase, placement: Placement, antiparallel: bool) -> Fixture {
    fixture_with_axial_intervals_and_inner_direction(
        placement,
        case.first,
        case.second,
        antiparallel,
    )
}

fn run_operation(
    fixture: &mut Fixture,
    operation: SetOperation,
    swapped: bool,
) -> OperationOutcome<BooleanOutcome> {
    run_operation_with_settings(fixture, operation, swapped, OperationSettings::new())
}

fn run_operation_with_settings(
    fixture: &mut Fixture,
    operation: SetOperation,
    swapped: bool,
    settings: OperationSettings,
) -> OperationOutcome<BooleanOutcome> {
    match operation {
        SetOperation::Unite => run_unite(fixture, swapped, settings),
        SetOperation::FirstMinusSecond => run_subtract(fixture, true, settings),
        SetOperation::SecondMinusFirst => run_subtract(fixture, false, settings),
    }
}

fn assert_connected_solid(part: &kernel::Part<'_>, body: BodyId) {
    let body = part.body(body).unwrap();
    assert_eq!(body.kind(), BodyKind::Solid);
    let regions = body.regions().collect::<Vec<_>>();
    assert_eq!(regions.len(), 2);
    let exterior = part.region(regions[0].clone()).unwrap();
    let material = part.region(regions[1].clone()).unwrap();
    assert_eq!(exterior.kind(), RegionKind::Void);
    assert_eq!(exterior.shells().len(), 0);
    assert_eq!(material.kind(), RegionKind::Solid);
    assert_eq!(material.shells().len(), 1);
}

fn assert_result_face_lineage(
    fixture: &Fixture,
    body: BodyId,
    created: &kernel::BooleanCreatedResult,
) {
    assert_eq!(created.journal().part(), fixture.part_id);
    assert!(created.journal().mutation_count() > 0);
    assert!(
        created
            .journal()
            .mutations()
            .all(|mutation| mutation.kind() == MutationKind::Created)
    );

    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    let result_faces = part
        .body(body.clone())
        .unwrap()
        .faces()
        .unwrap()
        .collect::<Vec<_>>();
    let result_edges = part
        .body(body)
        .unwrap()
        .edges()
        .unwrap()
        .collect::<Vec<_>>();
    let source_faces = [fixture.outer.clone(), fixture.inner.clone()]
        .iter()
        .cloned()
        .flat_map(|source| {
            part.body(source)
                .unwrap()
                .faces()
                .unwrap()
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let source_edges = [fixture.outer.clone(), fixture.inner.clone()]
        .into_iter()
        .flat_map(|source| {
            part.body(source)
                .unwrap()
                .edges()
                .unwrap()
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let mut derived_faces = Vec::new();
    let mut derived_edges = Vec::new();
    for event in created.journal().lineage() {
        match event {
            LineageView::DerivedFrom {
                derived: JournalEntity::Face(derived),
                source: JournalEntity::Face(source),
            } => {
                assert!(result_faces.contains(&derived));
                assert!(source_faces.contains(&source));
                assert!(!derived_faces.contains(&derived));
                derived_faces.push(derived);
            }
            LineageView::DerivedFrom {
                derived: JournalEntity::Edge(derived),
                source: JournalEntity::Edge(source),
            } => {
                assert!(result_edges.contains(&derived));
                assert!(source_edges.contains(&source));
                assert!(!derived_edges.contains(&derived));
                derived_edges.push(derived);
            }
            _ => panic!("coincident-cap Boolean lineage escaped source faces and edges"),
        }
    }
    assert_eq!(derived_faces.len(), result_faces.len());
}

fn assert_created_result(
    fixture: &Fixture,
    case: AxialCase,
    operation: SetOperation,
    oracle: SetOracle,
    created: kernel::BooleanCreatedResult,
) -> Vec<u8> {
    assert_eq!(created.bodies().len(), 1);
    assert_eq!(created.reports().len(), 1);
    assert_eq!(created.reports()[0].report().outcome(), CheckOutcome::Valid);
    let body = created.bodies()[0].clone();
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    assert_eq!(
        body_topology(&part, body.clone()),
        case.topology[operation.index()]
    );
    assert_connected_solid(&part, body.clone());
    assert_result_face_lineage(fixture, body.clone(), &created);
    let full = part
        .check_body(CheckBodyRequest::new(body.clone(), CheckLevel::Full))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(full.outcome(), CheckOutcome::Valid, "{full:#?}");

    let properties = part
        .body_properties(BodyPropertiesRequest::new(body.clone()))
        .unwrap();
    let BodyPropertiesOutcome::Certified {
        properties,
        full_check,
    } = properties.into_result().unwrap()
    else {
        panic!("{case:?} {operation:?} properties were not certified")
    };
    assert_eq!(full_check.outcome(), CheckOutcome::Valid);
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
fn coincident_cap_unite_and_ordered_subtract_match_independent_set_oracles() {
    let mut refusals = Vec::new();
    let mut executions = 0_usize;
    for case in CASES {
        for placement in [Placement::World, Placement::Oblique] {
            for antiparallel in [false, true] {
                for operation in SetOperation::ALL {
                    let mut canonical_bytes: Option<Vec<u8>> = None;
                    for &swapped in operation.caller_orders() {
                        for _ in 0..2 {
                            let mut fixture = fixture(case, placement, antiparallel);
                            assert_source_bodies_preserved(&fixture, 2);
                            let oracle = independent_set_oracle(case, operation, fixture.frame);
                            assert!(oracle.volume.is_finite() && oracle.volume > 0.0);
                            assert!(oracle.surface_area.is_finite() && oracle.surface_area > 0.0);
                            assert!(
                                (oracle.volume
                                    - exact_pi_root_three(case.volume[operation.index()]))
                                .abs()
                                    <= 1.0e-12,
                                "{} {operation:?} volume oracle drifted",
                                case.name
                            );
                            assert!(
                                (oracle.surface_area
                                    - exact_pi_root_three(case.area[operation.index()]))
                                .abs()
                                    <= 1.0e-12,
                                "{} {operation:?} area oracle drifted",
                                case.name
                            );
                            let before = fixture_signature(&fixture);
                            executions += 1;
                            let outcome = run_operation(&mut fixture, operation, swapped);
                            let result = outcome.into_result().unwrap();
                            let created = match result {
                                BooleanOutcome::Success(BooleanResult::Created(created)) => created,
                                BooleanOutcome::Refused(refusal) => {
                                    assert_eq!(fixture_signature(&fixture), before);
                                    refusals.push(format!(
                                        "{} {placement:?} antiparallel={antiparallel} \
                                         {operation:?} swapped={swapped}: {refusal:?}",
                                        case.name
                                    ));
                                    break;
                                }
                                other => panic!(
                                    "{} {operation:?} returned unexpected {other:#?}",
                                    case.name
                                ),
                            };
                            let bytes =
                                assert_created_result(&fixture, case, operation, oracle, created);
                            assert_source_bodies_preserved(&fixture, 3);
                            if let Some(canonical) = canonical_bytes.as_ref() {
                                assert_xt_equal(
                                    &bytes,
                                    canonical,
                                    "caller order or repeat changed direction-local X_T bytes",
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
    assert!(
        refusals.is_empty(),
        "{} coincident-cap set-operation rows remain refused:\n{}",
        refusals.len(),
        refusals.join("\n")
    );
    assert_eq!(executions, 96);
}

#[derive(Debug, Clone, Copy)]
struct WorkFrontierCase {
    axial: AxialCase,
    operation: SetOperation,
    shell_stage: fn() -> kernel::StageId,
}

const WORK_FRONTIERS: [WorkFrontierCase; 4] = [
    WorkFrontierCase {
        axial: CASES[0],
        operation: SetOperation::Unite,
        shell_stage: mixed_profile_shell_stage,
    },
    WorkFrontierCase {
        axial: CASES[1],
        operation: SetOperation::Unite,
        shell_stage: cap_reaching_shell_stage,
    },
    WorkFrontierCase {
        axial: CASES[1],
        operation: SetOperation::FirstMinusSecond,
        shell_stage: mixed_profile_shell_stage,
    },
    WorkFrontierCase {
        axial: CASES[1],
        operation: SetOperation::SecondMinusFirst,
        shell_stage: cap_reaching_shell_stage,
    },
];

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

fn charged_work(
    outcome: &OperationOutcome<BooleanOutcome>,
    stage: kernel::StageId,
    frontier: WorkFrontierCase,
) -> u64 {
    outcome
        .report()
        .usage()
        .iter()
        .find(|usage| usage.stage == stage && usage.resource == ResourceKind::Work)
        .unwrap_or_else(|| {
            panic!(
                "{} {:?} did not charge {stage:?}",
                frontier.axial.name, frontier.operation
            )
        })
        .consumed
}

fn assert_operation_work_frontier(
    frontier: WorkFrontierCase,
    antiparallel: bool,
    stage: kernel::StageId,
) {
    let run_at = |fixture: &mut Fixture, settings| {
        run_operation_with_settings(fixture, frontier.operation, false, settings)
    };

    let mut baseline = fixture(frontier.axial, Placement::World, antiparallel);
    let baseline_outcome = run_at(&mut baseline, OperationSettings::new());
    let measured = charged_work(&baseline_outcome, stage, frontier);
    assert!(
        measured > 0,
        "{} {:?} metered no work at {stage:?}",
        frontier.axial.name,
        frontier.operation
    );
    assert!(matches!(
        baseline_outcome.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut admitted = fixture(frontier.axial, Placement::World, antiparallel);
    let admitted_outcome = run_at(&mut admitted, settings_at(stage, measured));
    assert!(matches!(
        admitted_outcome.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut denied = fixture(frontier.axial, Placement::World, antiparallel);
    let before = fixture_signature(&denied);
    let denied_outcome = run_at(&mut denied, settings_at(stage, measured - 1));
    let limit = *denied_outcome
        .report()
        .limit_events()
        .first()
        .expect("N-1 refusal recorded no limit event");
    assert_eq!(limit.stage, stage);
    assert_eq!(limit.resource, ResourceKind::Work);
    assert_eq!(limit.allowed, measured - 1);
    assert_eq!(limit.consumed, measured);
    assert_eq!(denied_outcome.result().unwrap_err().limit(), Some(limit));
    assert_eq!(fixture_signature(&denied), before);
}

#[test]
fn coincident_cap_set_operation_work_frontiers_accept_n_and_refuse_n_minus_one_atomically() {
    for frontier in WORK_FRONTIERS {
        for antiparallel in [false, true] {
            assert_operation_work_frontier(frontier, antiparallel, BOOLEAN_POST_SELECTION_WORK);
            assert_operation_work_frontier(frontier, antiparallel, (frontier.shell_stage)());

            let mut fixture = fixture(frontier.axial, Placement::World, antiparallel);
            let outcome = run_operation(&mut fixture, frontier.operation, false);
            let BooleanOutcome::Success(BooleanResult::Created(created)) =
                outcome.into_result().unwrap()
            else {
                panic!(
                    "{} {:?} property frontier fixture was refused",
                    frontier.axial.name, frontier.operation
                )
            };
            let before = fixture_signature(&fixture);
            let part = fixture.session.part(fixture.part_id.clone()).unwrap();
            let _ = certified_properties_at_exact_budget(
                &part,
                created.bodies()[0].clone(),
                frontier.axial.name,
            );
            assert_eq!(fixture_signature(&fixture), before);
        }
    }
}
