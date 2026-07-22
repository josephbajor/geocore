//! Facade-only lifecycle evidence for parallel-cylinder lens/crescent prisms.
//! Wall-time budget: less than 60 seconds for the rigid-frame/order matrix.

use super::*;
use kernel::{
    BodyPropertiesOutcome, BodyPropertiesRequest, ImportXtRequest, OperationOutcome,
    Point3Enclosure, ScalarEnclosure,
};

const RADIUS: f64 = 1.0;
const AXIS_OFFSET: f64 = 0.5;
const OUTER_HALF_HEIGHT: f64 = 2.0;
const INNER_HALF_HEIGHT: f64 = 1.0;
const ANALYTIC_ORACLE_TOLERANCE: f64 = 1.0e-10;

#[derive(Debug, Clone, Copy)]
enum Placement {
    World,
    Oblique,
}

struct Fixture {
    session: Session,
    part_id: PartId,
    outer: BodyId,
    inner: BodyId,
    frame: Frame,
}

fn shared_frame(placement: Placement) -> Frame {
    match placement {
        Placement::World => Frame::world(),
        Placement::Oblique => Frame::new(
            Point3::new(2.5, -1.75, 0.625),
            Vec3::new(0.48, 0.64, 0.6),
            Vec3::new(0.8, -0.6, 0.0),
        )
        .unwrap(),
    }
}

fn fixture(placement: Placement) -> Fixture {
    let frame = shared_frame(placement);
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (outer, inner) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let outer = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(-AXIS_OFFSET, 0.0, -OUTER_HALF_HEIGHT)),
                RADIUS,
                2.0 * OUTER_HALF_HEIGHT,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let inner = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(AXIS_OFFSET, 0.0, -INNER_HALF_HEIGHT)),
                RADIUS,
                2.0 * INNER_HALF_HEIGHT,
            ))
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

fn body_topology(part: &kernel::Part<'_>, body: BodyId) -> [usize; 3] {
    let body = part.body(body).unwrap();
    [
        body.faces().unwrap().len(),
        body.edges().unwrap().len(),
        body.vertices().unwrap().len(),
    ]
}

fn source_signature(fixture: &Fixture) -> ([usize; 3], [usize; 3], usize) {
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    (
        body_topology(&part, fixture.outer.clone()),
        body_topology(&part, fixture.inner.clone()),
        part.bodies().len(),
    )
}

fn run(
    fixture: &mut Fixture,
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
            BooleanBodiesRequest::new(
                BooleanOperation::Intersect,
                bodies[0].clone(),
                bodies[1].clone(),
            )
            .with_settings(settings),
        )
        .unwrap()
}

fn run_subtract(
    fixture: &mut Fixture,
    reverse: bool,
    settings: OperationSettings,
) -> OperationOutcome<BooleanOutcome> {
    let bodies = if reverse {
        [fixture.outer.clone(), fixture.inner.clone()]
    } else {
        [fixture.inner.clone(), fixture.outer.clone()]
    };
    fixture
        .session
        .edit_part(fixture.part_id.clone())
        .unwrap()
        .boolean_bodies(
            BooleanBodiesRequest::new(
                BooleanOperation::Subtract,
                bodies[0].clone(),
                bodies[1].clone(),
            )
            .with_settings(settings),
        )
        .unwrap()
}

fn expected_volume() -> f64 {
    4.0 * core::f64::consts::PI / 3.0 - 3.0_f64.sqrt()
}

fn expected_surface_area() -> f64 {
    4.0 * core::f64::consts::PI - 3.0_f64.sqrt()
}

fn expected_subtract_cross_section_area() -> f64 {
    core::f64::consts::PI / 3.0 + 3.0_f64.sqrt() / 2.0
}

fn expected_subtract_volume() -> f64 {
    2.0 * expected_subtract_cross_section_area()
}

fn expected_subtract_surface_area() -> f64 {
    2.0 * expected_subtract_cross_section_area() + 4.0 * core::f64::consts::PI
}

fn expected_subtract_centroid_x() -> f64 {
    (core::f64::consts::PI / 2.0) / expected_subtract_cross_section_area()
}

fn expected_subtract_centroidal_inertia(frame: Frame) -> [[f64; 3]; 3] {
    let pi = core::f64::consts::PI;
    let root_three = 3.0_f64.sqrt();
    let area = expected_subtract_cross_section_area();
    // Raw planar moments of the right unit disk after subtracting the
    // symmetric unit-disk lens. The lens moments follow by integrating its
    // two exact circular segments over x in [-1/2, 1/2].
    let raw_x_squared = pi / 6.0 + 9.0 * root_three / 16.0;
    let raw_y_squared = pi / 12.0 + 3.0 * root_three / 16.0;
    let central_x_squared = raw_x_squared - (pi / 2.0).powi(2) / area;
    let axial_translation_moment = 2.0 * area / 3.0;
    let local = [
        [2.0 * raw_y_squared + axial_translation_moment, 0.0, 0.0],
        [0.0, 2.0 * central_x_squared + axial_translation_moment, 0.0],
        [0.0, 0.0, 2.0 * (central_x_squared + raw_y_squared)],
    ];
    rotate_tensor(frame, local)
}

fn rotate_tensor(frame: Frame, local: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let axes = [frame.x(), frame.y(), frame.z()].map(|axis| [axis.x, axis.y, axis.z]);
    core::array::from_fn(|row| {
        core::array::from_fn(|column| {
            (0..3)
                .flat_map(|left| (0..3).map(move |right| (left, right)))
                .map(|(left, right)| axes[left][row] * local[left][right] * axes[right][column])
                .sum()
        })
    })
}

fn assert_scalar_matches_analytic(actual: ScalarEnclosure, expected: f64, label: &str) {
    assert!(
        (actual.value() - expected).abs() <= ANALYTIC_ORACLE_TOLERANCE,
        "{label}={actual:?}, expected={expected}"
    );
    assert!(
        actual.error_bound() <= ANALYTIC_ORACLE_TOLERANCE,
        "{label} enclosure is too wide: {actual:?}"
    );
}

fn assert_point_matches_analytic(actual: Point3Enclosure, expected: Point3) {
    assert!(
        (actual.value() - expected).norm() <= ANALYTIC_ORACLE_TOLERANCE,
        "centroid={actual:?}, expected={expected:?}"
    );
    assert!(
        actual.error_bound() <= ANALYTIC_ORACLE_TOLERANCE,
        "centroid enclosure is too wide: {actual:?}"
    );
}

fn assert_fast_self_import(session: &mut Session, bytes: &[u8]) {
    let imported_part = session.create_part();
    let imported = session
        .edit_part(imported_part.clone())
        .unwrap()
        .import_xt(ImportXtRequest::new(bytes))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(imported.bodies().len(), 1);
    let report = session
        .part(imported_part)
        .unwrap()
        .check_body(CheckBodyRequest::new(
            imported.bodies()[0].clone(),
            CheckLevel::Fast,
        ))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(report.outcome(), CheckOutcome::Valid);
}

fn assert_xt_equal(actual: &[u8], expected: &[u8], message: &str) {
    if actual == expected {
        return;
    }
    let actual = String::from_utf8_lossy(actual);
    let expected = String::from_utf8_lossy(expected);
    let difference = actual
        .lines()
        .zip(expected.lines())
        .enumerate()
        .find(|(_, (left, right))| left != right);
    let Some((line, (left, right))) = difference else {
        panic!(
            "{message}: equal shared lines but byte lengths differ ({} != {})",
            actual.len(),
            expected.len()
        );
    };
    panic!(
        "{message} at X_T line {}:\nleft: {left}\nright: {right}",
        line + 1
    );
}

fn assert_subtract_created(
    fixture: &Fixture,
    outcome: OperationOutcome<BooleanOutcome>,
) -> Vec<u8> {
    let result = outcome.into_result().unwrap();
    let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
        panic!("parallel-cylinder inner-minus-outer Subtract returned {result:#?}")
    };
    assert_eq!(created.bodies().len(), 1);
    assert_eq!(created.reports().len(), 1);
    assert_eq!(created.reports()[0].report().outcome(), CheckOutcome::Valid);
    let result = created.bodies()[0].clone();
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    assert_eq!(body_topology(&part, result.clone()), [4, 6, 4]);
    let full = part
        .check_body(CheckBodyRequest::new(result.clone(), CheckLevel::Full))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(full.outcome(), CheckOutcome::Valid, "{full:#?}");
    let BodyPropertiesOutcome::Certified {
        properties,
        full_check,
    } = part
        .body_properties(BodyPropertiesRequest::new(result.clone()))
        .unwrap()
        .into_result()
        .unwrap()
    else {
        panic!("crescent-prism analytic properties were refused")
    };
    assert_eq!(full_check.outcome(), CheckOutcome::Valid);
    assert_scalar_matches_analytic(properties.volume(), expected_subtract_volume(), "volume");
    assert_scalar_matches_analytic(
        properties.surface_area(),
        expected_subtract_surface_area(),
        "surface area",
    );
    assert_point_matches_analytic(
        properties.centroid(),
        fixture
            .frame
            .point_at(expected_subtract_centroid_x(), 0.0, 0.0),
    );
    let expected_inertia = expected_subtract_centroidal_inertia(fixture.frame);
    let actual_inertia = properties.centroidal_inertia().value();
    // The oracle describes the ideal CSG circles, while the committed B-rep
    // owns certified floating trim representatives. Compare their values at
    // the same tolerance used for volume/area and separately bound the
    // certificate width; an ideal value need not lie inside the realized
    // trim enclosure bit-for-bit.
    let inertia_error = (0..3)
        .flat_map(|row| (0..3).map(move |column| (row, column)))
        .map(|(row, column)| (actual_inertia[row][column] - expected_inertia[row][column]).abs())
        .fold(0.0, f64::max);
    assert!(
        inertia_error <= ANALYTIC_ORACLE_TOLERANCE,
        "inertia={actual_inertia:?}, expected={expected_inertia:?}, enclosure {:?}",
        properties.centroidal_inertia()
    );
    assert!(
        properties.centroidal_inertia().error_bound() <= 1.0e-8,
        "centroidal inertia enclosure is too wide: {:?}",
        properties.centroidal_inertia()
    );
    let first = part
        .export_xt(ExportXtRequest::new(result.clone()))
        .unwrap()
        .into_result()
        .unwrap();
    let second = part
        .export_xt(ExportXtRequest::new(result))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(first.bytes(), second.bytes());
    first.bytes().to_vec()
}

#[test]
fn parallel_cylinder_intersection_full_commits_a_deterministic_lens_prism() {
    for placement in [Placement::World, Placement::Oblique] {
        let mut canonical_bytes: Option<Vec<u8>> = None;
        for swapped in [false, true] {
            let mut fixture = fixture(placement);
            assert_eq!(source_signature(&fixture), ([3, 2, 0], [3, 2, 0], 2));
            let outcome = run(&mut fixture, swapped, OperationSettings::new());
            let result = outcome.into_result().unwrap();
            let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
                panic!("parallel-cylinder Intersect returned {result:#?}")
            };
            assert_eq!(created.bodies().len(), 1);
            assert_eq!(created.reports().len(), 1);
            assert_eq!(created.reports()[0].report().outcome(), CheckOutcome::Valid);
            let result = created.bodies()[0].clone();
            let bytes = {
                let part = fixture.session.part(fixture.part_id.clone()).unwrap();
                assert_eq!(body_topology(&part, result.clone()), [4, 6, 4]);
                let full = part
                    .check_body(CheckBodyRequest::new(result.clone(), CheckLevel::Full))
                    .unwrap()
                    .into_result()
                    .unwrap();
                assert_eq!(full.outcome(), CheckOutcome::Valid, "{full:#?}");
                let BodyPropertiesOutcome::Certified {
                    properties,
                    full_check,
                } = part
                    .body_properties(BodyPropertiesRequest::new(result.clone()))
                    .unwrap()
                    .into_result()
                    .unwrap()
                else {
                    panic!("lens-prism analytic properties were refused")
                };
                assert_eq!(full_check.outcome(), CheckOutcome::Valid);
                assert_scalar_matches_analytic(properties.volume(), expected_volume(), "volume");
                assert_scalar_matches_analytic(
                    properties.surface_area(),
                    expected_surface_area(),
                    "surface area",
                );
                assert_point_matches_analytic(properties.centroid(), fixture.frame.origin());
                let first = part
                    .export_xt(ExportXtRequest::new(result.clone()))
                    .unwrap()
                    .into_result()
                    .unwrap();
                let second = part
                    .export_xt(ExportXtRequest::new(result))
                    .unwrap()
                    .into_result()
                    .unwrap();
                assert_eq!(first.bytes(), second.bytes());
                first.bytes().to_vec()
            };
            assert_eq!(source_signature(&fixture), ([3, 2, 0], [3, 2, 0], 3));
            if let Some(canonical) = canonical_bytes.as_ref() {
                assert_xt_equal(&bytes, canonical, "operand swap changed X_T bytes");
            } else {
                canonical_bytes = Some(bytes.clone());
            }
            assert_fast_self_import(&mut fixture.session, &bytes);
        }
    }
}

#[test]
fn parallel_cylinder_inner_minus_outer_full_commits_a_deterministic_crescent_prism() {
    for placement in [Placement::World, Placement::Oblique] {
        let mut canonical_bytes: Option<Vec<u8>> = None;
        for _ in 0..2 {
            let mut fixture = fixture(placement);
            assert_eq!(source_signature(&fixture), ([3, 2, 0], [3, 2, 0], 2));
            let outcome = run_subtract(&mut fixture, false, OperationSettings::new());
            let bytes = assert_subtract_created(&fixture, outcome);
            assert_eq!(source_signature(&fixture), ([3, 2, 0], [3, 2, 0], 3));
            if let Some(canonical) = canonical_bytes.as_ref() {
                assert_xt_equal(&bytes, canonical, "repeat Subtract changed X_T bytes");
            } else {
                canonical_bytes = Some(bytes.clone());
            }
            assert_fast_self_import(&mut fixture.session, &bytes);
        }

        let mut reverse = fixture(placement);
        let before = source_signature(&reverse);
        let outcome = run_subtract(&mut reverse, true, OperationSettings::new());
        assert!(matches!(
            outcome.into_result().unwrap(),
            BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
        ));
        assert_eq!(source_signature(&reverse), before);
    }
}

#[derive(Clone, Copy)]
enum RealizationCase {
    Intersection,
    InnerMinusOuter,
}

fn run_realization_case(
    fixture: &mut Fixture,
    case: RealizationCase,
    settings: OperationSettings,
) -> OperationOutcome<BooleanOutcome> {
    match case {
        RealizationCase::Intersection => run(fixture, false, settings),
        RealizationCase::InnerMinusOuter => run_subtract(fixture, false, settings),
    }
}

fn assert_realization_budget_case(case: RealizationCase) {
    let baseline = run_realization_case(
        &mut fixture(Placement::World),
        case,
        OperationSettings::new(),
    );
    let usage = *baseline
        .report()
        .usage()
        .iter()
        .find(|usage| {
            usage.stage == BOOLEAN_POST_SELECTION_WORK && usage.resource == ResourceKind::Work
        })
        .expect("parallel-cylinder realization did not charge its shared stage");
    assert!(usage.consumed > 0);
    assert!(matches!(
        baseline.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));
    let settings_at = |allowed| {
        OperationSettings::new().with_budget_overrides(
            BudgetPlan::new([LimitSpec::new(
                BOOLEAN_POST_SELECTION_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            )])
            .unwrap(),
        )
    };
    let admitted = run_realization_case(
        &mut fixture(Placement::World),
        case,
        settings_at(usage.consumed),
    );
    assert!(matches!(
        admitted.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut denied_fixture = fixture(Placement::World);
    let before = source_signature(&denied_fixture);
    let denied = run_realization_case(&mut denied_fixture, case, settings_at(usage.consumed - 1));
    let expected = kernel::LimitSnapshot {
        allowed: usage.consumed - 1,
        ..usage
    };
    assert_eq!(denied.result().unwrap_err().limit(), Some(expected));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(source_signature(&denied_fixture), before);
}

#[test]
fn parallel_cylinder_realization_budget_accepts_n_and_refuses_n_minus_one_atomically() {
    for case in [
        RealizationCase::Intersection,
        RealizationCase::InnerMinusOuter,
    ] {
        assert_realization_budget_case(case);
    }
}
