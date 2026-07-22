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
const CYLINDER_TOPOLOGY: [usize; 3] = [3, 2, 0];
const PARTIAL_SUBTRACT_REALIZATION_WORK: u64 = 4_103;
const PARTIAL_SUBTRACT_BODY_PROPERTIES_WORK: u64 = 15_617;

#[derive(Debug, Clone, Copy)]
enum Placement {
    World,
    Oblique,
}

#[derive(Debug, Clone, Copy)]
enum PartialSubtractMeaning {
    AMinusB,
    BMinusA,
}

impl PartialSubtractMeaning {
    const fn reverse(self) -> bool {
        matches!(self, Self::AMinusB)
    }

    const fn centroid_sign(self) -> f64 {
        match self {
            Self::AMinusB => -1.0,
            Self::BMinusA => 1.0,
        }
    }
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
    fixture_with_half_heights(placement, OUTER_HALF_HEIGHT, INNER_HALF_HEIGHT)
}

fn partial_overlap_fixture(placement: Placement) -> Fixture {
    fixture_with_axial_intervals(placement, [-2.0, 1.0], [-1.0, 2.0])
}

fn fixture_with_half_heights(
    placement: Placement,
    outer_half_height: f64,
    inner_half_height: f64,
) -> Fixture {
    fixture_with_axial_intervals(
        placement,
        [-outer_half_height, outer_half_height],
        [-inner_half_height, inner_half_height],
    )
}

fn fixture_with_axial_intervals(
    placement: Placement,
    outer_interval: [f64; 2],
    inner_interval: [f64; 2],
) -> Fixture {
    assert!(outer_interval[0] < outer_interval[1]);
    assert!(inner_interval[0] < inner_interval[1]);
    let frame = shared_frame(placement);
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (outer, inner) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let outer = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(-AXIS_OFFSET, 0.0, outer_interval[0])),
                RADIUS,
                outer_interval[1] - outer_interval[0],
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let inner = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(AXIS_OFFSET, 0.0, inner_interval[0])),
                RADIUS,
                inner_interval[1] - inner_interval[0],
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PartEntityCounts {
    bodies: usize,
    regions: usize,
    shells: usize,
    faces: usize,
    loops: usize,
    fins: usize,
    edges: usize,
    vertices: usize,
    curves: usize,
    surfaces: usize,
    pcurves: usize,
}

impl PartEntityCounts {
    fn from_part(part: &kernel::Part<'_>) -> Self {
        Self {
            bodies: part.bodies().len(),
            regions: part.regions().len(),
            shells: part.shells().len(),
            faces: part.faces().len(),
            loops: part.loops().len(),
            fins: part.fins().len(),
            edges: part.edges().len(),
            vertices: part.vertices().len(),
            curves: part.curves().len(),
            surfaces: part.surfaces().len(),
            pcurves: part.pcurves().len(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct FixtureSignature {
    outer_topology: [usize; 3],
    inner_topology: [usize; 3],
    part_entities: PartEntityCounts,
}

fn fixture_signature(fixture: &Fixture) -> FixtureSignature {
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    FixtureSignature {
        outer_topology: body_topology(&part, fixture.outer.clone()),
        inner_topology: body_topology(&part, fixture.inner.clone()),
        part_entities: PartEntityCounts::from_part(&part),
    }
}

fn assert_source_bodies_preserved(fixture: &Fixture, expected_body_count: usize) {
    let signature = fixture_signature(fixture);
    assert_eq!(signature.outer_topology, CYLINDER_TOPOLOGY);
    assert_eq!(signature.inner_topology, CYLINDER_TOPOLOGY);
    assert_eq!(signature.part_entities.bodies, expected_body_count);
}

fn run(
    fixture: &mut Fixture,
    swapped: bool,
    settings: OperationSettings,
) -> OperationOutcome<BooleanOutcome> {
    run_commutative(fixture, BooleanOperation::Intersect, swapped, settings)
}

fn run_unite(
    fixture: &mut Fixture,
    swapped: bool,
    settings: OperationSettings,
) -> OperationOutcome<BooleanOutcome> {
    run_commutative(fixture, BooleanOperation::Unite, swapped, settings)
}

fn run_commutative(
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

fn expected_intersection_centroidal_inertia(frame: Frame) -> [[f64; 3]; 3] {
    let pi = core::f64::consts::PI;
    let root_three = 3.0_f64.sqrt();
    let area = expected_volume() / 2.0;
    // Direct integration of the symmetric unit-circle lens gives its planar
    // second moments. Extruding that lens over z in [-1, 1] gives the axial
    // moment below. This oracle therefore applies equally to the nested and
    // partial-overlap fixtures whenever their intersection is that same prism.
    let planar_x_squared = pi / 3.0 - 9.0 * root_three / 16.0;
    let planar_y_squared = pi / 6.0 - 3.0 * root_three / 16.0;
    let axial_squared = 2.0 * area / 3.0;
    let local = [
        [2.0 * planar_y_squared + axial_squared, 0.0, 0.0],
        [0.0, 2.0 * planar_x_squared + axial_squared, 0.0],
        [0.0, 0.0, 2.0 * (planar_x_squared + planar_y_squared)],
    ];
    rotate_tensor(frame, local)
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

fn expected_outer_subtract_volume() -> f64 {
    8.0 * core::f64::consts::PI / 3.0 + 3.0_f64.sqrt()
}

fn expected_outer_subtract_surface_area() -> f64 {
    34.0 * core::f64::consts::PI / 3.0 - 3.0_f64.sqrt()
}

fn expected_outer_subtract_centroid_x() -> f64 {
    -2.0 * core::f64::consts::PI / expected_outer_subtract_volume()
}

fn expected_outer_subtract_centroidal_inertia(frame: Frame) -> [[f64; 3]; 3] {
    let pi = core::f64::consts::PI;
    let root_three = 3.0_f64.sqrt();
    let volume = expected_outer_subtract_volume();
    // Subtract the height-two right-cylinder overlap from the height-four
    // left cylinder. These are the remaining body's raw local moments.
    let raw_x_squared = 4.0 * pi / 3.0 + 9.0 * root_three / 8.0;
    let raw_y_squared = 2.0 * pi / 3.0 + 3.0 * root_three / 8.0;
    let raw_z_squared = 44.0 * pi / 9.0 + root_three / 3.0;
    let central_x_squared = raw_x_squared - 4.0 * pi.powi(2) / volume;
    let local = [
        [raw_y_squared + raw_z_squared, 0.0, 0.0],
        [0.0, central_x_squared + raw_z_squared, 0.0],
        [0.0, 0.0, central_x_squared + raw_y_squared],
    ];
    rotate_tensor(frame, local)
}

fn expected_partial_subtract_volume() -> f64 {
    5.0 * core::f64::consts::PI / 3.0 + 3.0_f64.sqrt()
}

fn expected_partial_subtract_centroid_offset() -> f64 {
    3.0 * core::f64::consts::PI / (2.0 * expected_partial_subtract_volume())
}

fn expected_partial_subtract_centroidal_inertia(frame: Frame) -> [[f64; 3]; 3] {
    let pi = core::f64::consts::PI;
    let root_three = 3.0_f64.sqrt();
    let volume = expected_partial_subtract_volume();
    // Direct integration of either congruent ordered crescent solid in its
    // local cylinder frame. Both centroids change x/z sign together, so their
    // parallel-axis correction and centroidal xz product are identical.
    let translation = 9.0 * pi.powi(2) / (4.0 * volume);
    let central_x_squared = 5.0 * pi / 6.0 + 9.0 * root_three / 8.0 - translation;
    let central_y_squared = 5.0 * pi / 12.0 + 3.0 * root_three / 8.0;
    let central_z_squared = 23.0 * pi / 9.0 + root_three / 3.0 - translation;
    let central_xz = 3.0 * pi / 4.0 - translation;
    let local = [
        [central_y_squared + central_z_squared, 0.0, -central_xz],
        [0.0, central_x_squared + central_z_squared, 0.0],
        [-central_xz, 0.0, central_x_squared + central_y_squared],
    ];
    rotate_tensor(frame, local)
}

fn expected_unite_volume() -> f64 {
    14.0 * core::f64::consts::PI / 3.0 + 3.0_f64.sqrt()
}

fn expected_unite_surface_area() -> f64 {
    12.0 * core::f64::consts::PI + 3.0_f64.sqrt()
}

fn expected_unite_centroid_x() -> f64 {
    -core::f64::consts::PI / expected_unite_volume()
}

fn expected_unite_centroidal_inertia(frame: Frame) -> [[f64; 3]; 3] {
    let pi = core::f64::consts::PI;
    let root_three = 3.0_f64.sqrt();
    let volume = expected_unite_volume();
    // Inclusion-exclusion of the height-four left cylinder, height-two right
    // cylinder, and their symmetric height-two lens gives these raw moments.
    let raw_x_squared = 7.0 * pi / 3.0 + 9.0 * root_three / 8.0;
    let raw_y_squared = 7.0 * pi / 6.0 + 3.0 * root_three / 8.0;
    let raw_z_squared = 50.0 * pi / 9.0 + root_three / 3.0;
    let central_x_squared = raw_x_squared - pi.powi(2) / volume;
    let local = [
        [raw_y_squared + raw_z_squared, 0.0, 0.0],
        [0.0, central_x_squared + raw_z_squared, 0.0],
        [0.0, 0.0, central_x_squared + raw_y_squared],
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

fn assert_inertia_matches_analytic(
    actual: [[f64; 3]; 3],
    error_bound: f64,
    expected: [[f64; 3]; 3],
) {
    let error = (0..3)
        .flat_map(|row| (0..3).map(move |column| (row, column)))
        .map(|(row, column)| (actual[row][column] - expected[row][column]).abs())
        .fold(0.0, f64::max);
    assert!(
        error <= ANALYTIC_ORACLE_TOLERANCE,
        "inertia={actual:?}, expected={expected:?}, error_bound={error_bound}"
    );
    assert!(
        error_bound <= 1.0e-8,
        "centroidal inertia enclosure is too wide: {error_bound}"
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

fn assert_partial_intersection_created(
    fixture: &Fixture,
    outcome: OperationOutcome<BooleanOutcome>,
) -> Vec<u8> {
    let result = outcome.into_result().unwrap();
    let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
        panic!("partial-overlap parallel-cylinder Intersect returned {result:#?}")
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
        panic!("partial-overlap lens-prism analytic properties were refused")
    };
    assert_eq!(full_check.outcome(), CheckOutcome::Valid);
    // The two height-three cylinders overlap over z in [-1, 1], so their
    // result is mathematically identical to the independently integrated
    // height-two lens prism used by these analytic oracles.
    assert_scalar_matches_analytic(properties.volume(), expected_volume(), "volume");
    assert_scalar_matches_analytic(
        properties.surface_area(),
        expected_surface_area(),
        "surface area",
    );
    assert_point_matches_analytic(properties.centroid(), fixture.frame.origin());
    assert_inertia_matches_analytic(
        properties.centroidal_inertia().value(),
        properties.centroidal_inertia().error_bound(),
        expected_intersection_centroidal_inertia(fixture.frame),
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

fn assert_inner_subtract_created(
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
    // The oracle describes the ideal CSG circles, while the committed B-rep
    // owns certified floating trim representatives. Compare their values at
    // the same tolerance used for volume/area and separately bound the
    // certificate width; an ideal value need not lie inside the realized
    // trim enclosure bit-for-bit.
    assert_inertia_matches_analytic(
        properties.centroidal_inertia().value(),
        properties.centroidal_inertia().error_bound(),
        expected_subtract_centroidal_inertia(fixture.frame),
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

fn assert_outer_subtract_created(
    fixture: &Fixture,
    outcome: OperationOutcome<BooleanOutcome>,
) -> Vec<u8> {
    let result = outcome.into_result().unwrap();
    let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
        panic!("parallel-cylinder outer-minus-inner Subtract returned {result:#?}")
    };
    assert_eq!(created.bodies().len(), 1);
    assert_eq!(created.reports().len(), 1);
    assert_eq!(created.reports()[0].report().outcome(), CheckOutcome::Valid);
    let result = created.bodies()[0].clone();
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    assert_eq!(body_topology(&part, result.clone()), [6, 8, 4]);
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
        panic!("outer-minus-inner analytic properties were refused")
    };
    assert_eq!(full_check.outcome(), CheckOutcome::Valid);
    assert_scalar_matches_analytic(
        properties.volume(),
        expected_outer_subtract_volume(),
        "volume",
    );
    assert_scalar_matches_analytic(
        properties.surface_area(),
        expected_outer_subtract_surface_area(),
        "surface area",
    );
    assert_point_matches_analytic(
        properties.centroid(),
        fixture
            .frame
            .point_at(expected_outer_subtract_centroid_x(), 0.0, 0.0),
    );
    assert_inertia_matches_analytic(
        properties.centroidal_inertia().value(),
        properties.centroidal_inertia().error_bound(),
        expected_outer_subtract_centroidal_inertia(fixture.frame),
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

fn assert_partial_subtract_created(
    fixture: &Fixture,
    meaning: PartialSubtractMeaning,
    outcome: OperationOutcome<BooleanOutcome>,
) -> Vec<u8> {
    let result = outcome.into_result().unwrap();
    let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
        panic!("partial-overlap parallel-cylinder {meaning:?} returned {result:#?}")
    };
    assert_eq!(created.bodies().len(), 1);
    assert_eq!(created.reports().len(), 1);
    assert_eq!(created.reports()[0].report().outcome(), CheckOutcome::Valid);
    let result = created.bodies()[0].clone();
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    assert_eq!(body_topology(&part, result.clone()), [5, 7, 4]);
    let full = part
        .check_body(CheckBodyRequest::new(result.clone(), CheckLevel::Full))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(full.outcome(), CheckOutcome::Valid, "{full:#?}");
    let properties_outcome = part
        .body_properties(BodyPropertiesRequest::new(result.clone()))
        .unwrap();
    let analytic_usage = *properties_outcome
        .report()
        .usage()
        .iter()
        .find(|usage| {
            usage.stage == kernel::BODY_PROPERTIES_ANALYTIC_WORK
                && usage.resource == ResourceKind::Work
        })
        .expect("partial Subtract properties did not charge analytic work");
    assert_eq!(
        analytic_usage.consumed, PARTIAL_SUBTRACT_BODY_PROPERTIES_WORK,
        "partial-overlap {meaning:?} changed analytic property work"
    );
    let BodyPropertiesOutcome::Certified {
        properties,
        full_check,
    } = properties_outcome.into_result().unwrap()
    else {
        panic!("partial-overlap {meaning:?} analytic properties were refused")
    };
    assert_eq!(full_check.outcome(), CheckOutcome::Valid);
    assert_scalar_matches_analytic(
        properties.volume(),
        expected_partial_subtract_volume(),
        "volume",
    );
    assert_scalar_matches_analytic(
        properties.surface_area(),
        8.0 * core::f64::consts::PI,
        "surface area",
    );
    let offset = meaning.centroid_sign() * expected_partial_subtract_centroid_offset();
    assert_point_matches_analytic(
        properties.centroid(),
        fixture.frame.point_at(offset, 0.0, offset),
    );
    assert_inertia_matches_analytic(
        properties.centroidal_inertia().value(),
        properties.centroidal_inertia().error_bound(),
        expected_partial_subtract_centroidal_inertia(fixture.frame),
    );
    let property_settings_at = |allowed| {
        OperationSettings::new().with_budget_overrides(
            BudgetPlan::new([LimitSpec::new(
                kernel::BODY_PROPERTIES_ANALYTIC_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            )])
            .unwrap(),
        )
    };
    let admitted = part
        .body_properties(
            BodyPropertiesRequest::new(result.clone())
                .with_settings(property_settings_at(analytic_usage.consumed)),
        )
        .unwrap();
    assert!(matches!(
        admitted.into_result().unwrap(),
        BodyPropertiesOutcome::Certified { .. }
    ));
    let refused = part
        .body_properties(
            BodyPropertiesRequest::new(result.clone())
                .with_settings(property_settings_at(analytic_usage.consumed - 1)),
        )
        .unwrap();
    let expected_limit = kernel::LimitSnapshot {
        allowed: analytic_usage.consumed - 1,
        ..analytic_usage
    };
    assert_eq!(refused.result().unwrap_err().limit(), Some(expected_limit));
    assert_eq!(refused.report().limit_events(), &[expected_limit]);
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

fn assert_unite_created(fixture: &Fixture, outcome: OperationOutcome<BooleanOutcome>) -> Vec<u8> {
    let result = outcome.into_result().unwrap();
    let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
        panic!("connected parallel-cylinder Unite returned {result:#?}")
    };
    assert_eq!(created.bodies().len(), 1);
    assert_eq!(created.reports().len(), 1);
    assert_eq!(created.reports()[0].report().outcome(), CheckOutcome::Valid);
    let result = created.bodies()[0].clone();
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    assert_eq!(body_topology(&part, result.clone()), [6, 8, 4]);
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
        panic!("connected-union analytic properties were refused")
    };
    assert_eq!(full_check.outcome(), CheckOutcome::Valid);
    assert_scalar_matches_analytic(properties.volume(), expected_unite_volume(), "volume");
    assert_scalar_matches_analytic(
        properties.surface_area(),
        expected_unite_surface_area(),
        "surface area",
    );
    assert_point_matches_analytic(
        properties.centroid(),
        fixture
            .frame
            .point_at(expected_unite_centroid_x(), 0.0, 0.0),
    );
    assert_inertia_matches_analytic(
        properties.centroidal_inertia().value(),
        properties.centroidal_inertia().error_bound(),
        expected_unite_centroidal_inertia(fixture.frame),
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
            assert_source_bodies_preserved(&fixture, 2);
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
            assert_source_bodies_preserved(&fixture, 3);
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
fn partial_axial_overlap_intersection_full_commits_a_deterministic_lens_prism() {
    for placement in [Placement::World, Placement::Oblique] {
        let mut canonical_bytes: Option<Vec<u8>> = None;
        for swapped in [false, true] {
            for _ in 0..2 {
                let mut fixture = partial_overlap_fixture(placement);
                assert_source_bodies_preserved(&fixture, 2);
                let outcome = run(&mut fixture, swapped, OperationSettings::new());
                let bytes = assert_partial_intersection_created(&fixture, outcome);
                assert_source_bodies_preserved(&fixture, 3);
                if let Some(canonical) = canonical_bytes.as_ref() {
                    assert_xt_equal(
                        &bytes,
                        canonical,
                        "operand swap or repeat changed partial-overlap Intersect X_T bytes",
                    );
                } else {
                    canonical_bytes = Some(bytes.clone());
                }
                assert_fast_self_import(&mut fixture.session, &bytes);
            }
        }
    }
}

#[test]
fn parallel_cylinder_unite_full_commits_a_deterministic_connected_union() {
    for placement in [Placement::World, Placement::Oblique] {
        let mut canonical_bytes: Option<Vec<u8>> = None;
        for swapped in [false, true] {
            for _ in 0..2 {
                let mut fixture = fixture(placement);
                assert_source_bodies_preserved(&fixture, 2);
                let outcome = run_unite(&mut fixture, swapped, OperationSettings::new());
                let bytes = assert_unite_created(&fixture, outcome);
                assert_source_bodies_preserved(&fixture, 3);
                if let Some(canonical) = canonical_bytes.as_ref() {
                    assert_xt_equal(
                        &bytes,
                        canonical,
                        "operand swap or repeat changed Unite X_T bytes",
                    );
                } else {
                    canonical_bytes = Some(bytes.clone());
                }
                assert_fast_self_import(&mut fixture.session, &bytes);
            }
        }
    }
}

#[test]
fn unsupported_equal_height_parallel_cylinder_unite_refuses_atomically() {
    for placement in [Placement::World, Placement::Oblique] {
        for swapped in [false, true] {
            let mut fixture =
                fixture_with_half_heights(placement, INNER_HALF_HEIGHT, INNER_HALF_HEIGHT);
            assert_source_bodies_preserved(&fixture, 2);
            let before = fixture_signature(&fixture);
            let outcome = run_unite(&mut fixture, swapped, OperationSettings::new());
            assert!(matches!(
                outcome.into_result().unwrap(),
                BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
            ));
            assert_eq!(
                fixture_signature(&fixture),
                before,
                "equal-height Unite mutated the part for {placement:?}, swapped={swapped}"
            );
        }
    }
}

#[test]
fn unsupported_partial_overlap_unite_refuses_atomically() {
    for placement in [Placement::World, Placement::Oblique] {
        for swapped in [false, true] {
            let mut fixture = partial_overlap_fixture(placement);
            assert_source_bodies_preserved(&fixture, 2);
            let before = fixture_signature(&fixture);
            let outcome = run_unite(&mut fixture, swapped, OperationSettings::new());
            assert!(matches!(
                outcome.into_result().unwrap(),
                BooleanOutcome::Refused(BooleanRefusal::CurvedResultTopologyUnsupported)
            ));
            assert_eq!(
                fixture_signature(&fixture),
                before,
                "partial-overlap Unite mutated the part for {placement:?}, swapped={swapped}"
            );
            assert_source_bodies_preserved(&fixture, 2);
        }
    }
}

#[test]
fn partial_axial_overlap_both_ordered_subtractions_commit_deterministically() {
    for placement in [Placement::World, Placement::Oblique] {
        for meaning in [
            PartialSubtractMeaning::AMinusB,
            PartialSubtractMeaning::BMinusA,
        ] {
            let mut canonical_bytes: Option<Vec<u8>> = None;
            for _ in 0..2 {
                let mut fixture = partial_overlap_fixture(placement);
                assert_source_bodies_preserved(&fixture, 2);
                let outcome =
                    run_subtract(&mut fixture, meaning.reverse(), OperationSettings::new());
                let bytes = assert_partial_subtract_created(&fixture, meaning, outcome);
                assert_source_bodies_preserved(&fixture, 3);
                if let Some(canonical) = canonical_bytes.as_ref() {
                    assert_xt_equal(
                        &bytes,
                        canonical,
                        "repeat changed partial-overlap ordered Subtract X_T bytes",
                    );
                } else {
                    canonical_bytes = Some(bytes.clone());
                }
                assert_fast_self_import(&mut fixture.session, &bytes);
            }
        }
    }
}

#[test]
fn parallel_cylinder_inner_minus_outer_full_commits_a_deterministic_crescent_prism() {
    for placement in [Placement::World, Placement::Oblique] {
        let mut canonical_bytes: Option<Vec<u8>> = None;
        for _ in 0..2 {
            let mut fixture = fixture(placement);
            assert_source_bodies_preserved(&fixture, 2);
            let outcome = run_subtract(&mut fixture, false, OperationSettings::new());
            let bytes = assert_inner_subtract_created(&fixture, outcome);
            assert_source_bodies_preserved(&fixture, 3);
            if let Some(canonical) = canonical_bytes.as_ref() {
                assert_xt_equal(&bytes, canonical, "repeat Subtract changed X_T bytes");
            } else {
                canonical_bytes = Some(bytes.clone());
            }
            assert_fast_self_import(&mut fixture.session, &bytes);
        }
    }
}

#[test]
fn parallel_cylinder_outer_minus_inner_full_commits_a_deterministic_notched_cylinder() {
    for placement in [Placement::World, Placement::Oblique] {
        let mut canonical_bytes: Option<Vec<u8>> = None;
        for _ in 0..2 {
            let mut fixture = fixture(placement);
            assert_source_bodies_preserved(&fixture, 2);
            let outcome = run_subtract(&mut fixture, true, OperationSettings::new());
            let bytes = assert_outer_subtract_created(&fixture, outcome);
            assert_source_bodies_preserved(&fixture, 3);
            if let Some(canonical) = canonical_bytes.as_ref() {
                assert_xt_equal(&bytes, canonical, "repeat Subtract changed X_T bytes");
            } else {
                canonical_bytes = Some(bytes.clone());
            }
            assert_fast_self_import(&mut fixture.session, &bytes);
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum RealizationCase {
    Intersection,
    PartialOverlapIntersection,
    PartialOverlapAMinusB,
    PartialOverlapBMinusA,
    Unite,
    InnerMinusOuter,
    OuterMinusInner,
}

fn realization_fixture(case: RealizationCase, placement: Placement) -> Fixture {
    match case {
        RealizationCase::PartialOverlapIntersection
        | RealizationCase::PartialOverlapAMinusB
        | RealizationCase::PartialOverlapBMinusA => partial_overlap_fixture(placement),
        RealizationCase::Intersection
        | RealizationCase::Unite
        | RealizationCase::InnerMinusOuter
        | RealizationCase::OuterMinusInner => fixture(placement),
    }
}

fn run_realization_case(
    fixture: &mut Fixture,
    case: RealizationCase,
    settings: OperationSettings,
) -> OperationOutcome<BooleanOutcome> {
    match case {
        RealizationCase::Intersection | RealizationCase::PartialOverlapIntersection => {
            run(fixture, false, settings)
        }
        RealizationCase::PartialOverlapAMinusB => run_subtract(fixture, true, settings),
        RealizationCase::PartialOverlapBMinusA => run_subtract(fixture, false, settings),
        RealizationCase::Unite => run_unite(fixture, false, settings),
        RealizationCase::InnerMinusOuter => run_subtract(fixture, false, settings),
        RealizationCase::OuterMinusInner => run_subtract(fixture, true, settings),
    }
}

fn assert_realization_budget_case(case: RealizationCase) {
    let baseline = run_realization_case(
        &mut realization_fixture(case, Placement::World),
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
    if matches!(
        case,
        RealizationCase::PartialOverlapAMinusB | RealizationCase::PartialOverlapBMinusA
    ) {
        assert_eq!(usage.consumed, PARTIAL_SUBTRACT_REALIZATION_WORK);
    }
    let baseline_result = baseline.into_result().unwrap();
    assert!(
        matches!(
            baseline_result,
            BooleanOutcome::Success(BooleanResult::Created(_))
        ),
        "{case:?} baseline used {} realization work but returned {baseline_result:#?}",
        usage.consumed
    );
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
        &mut realization_fixture(case, Placement::World),
        case,
        settings_at(usage.consumed),
    );
    assert!(matches!(
        admitted.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut denied_fixture = realization_fixture(case, Placement::World);
    let before = fixture_signature(&denied_fixture);
    let denied = run_realization_case(&mut denied_fixture, case, settings_at(usage.consumed - 1));
    let expected = kernel::LimitSnapshot {
        allowed: usage.consumed - 1,
        ..usage
    };
    assert_eq!(denied.result().unwrap_err().limit(), Some(expected));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(fixture_signature(&denied_fixture), before);
}

#[test]
fn parallel_cylinder_realization_budget_accepts_n_and_refuses_n_minus_one_atomically() {
    for case in [
        RealizationCase::Intersection,
        RealizationCase::PartialOverlapIntersection,
        RealizationCase::PartialOverlapAMinusB,
        RealizationCase::PartialOverlapBMinusA,
        RealizationCase::Unite,
        RealizationCase::InnerMinusOuter,
        RealizationCase::OuterMinusInner,
    ] {
        assert_realization_budget_case(case);
    }
}
