//! Desired facade-only lifecycle contract for positive-area axial-contact Unite.
//! Wall-time budget: less than 60 seconds for the exact frame/direction/order matrix.

use super::*;

const AXIAL_CONTACT_RELATION_WORK: u64 = 64;
// `N = 1 + 5F + 8L + 8U = 22`; the theorem charges `N^2 + 32N`.
const INTERNAL_CONTACT_SHELL_WORK: u64 = 1_188;
// These heights and the contact-specific translated oblique origin below are
// verified against the stored oblique axis: both cap reconstructions retain
// one exact zero affine projection in every authored axis direction.
const LOWER_HEIGHT: f64 = 8.0;
const UPPER_HEIGHT: f64 = 0.5;

// Existing analytic-shell precharge is `N^2 + 16N`, where
// `N = 1 + V + Eb + Ec + F + L + U`. The independent boundary inventories are:
// internal `(0,0,4,5,8,8) => N=26` and coincident
// `(0,0,2,3,4,4) => N=14`.

fn internal_contact_shell_stage() -> kernel::StageId {
    kernel::StageId::new("ktopo.check.parallel-cylinder-contact-shell-work").unwrap()
}

#[derive(Debug, Clone, Copy)]
enum RadialOverlap {
    StrictInternal,
    Coincident,
}

#[derive(Debug, Clone, Copy)]
struct ContactCase {
    name: &'static str,
    radii: [f64; 2],
    upper_radial_scale: f64,
    radial_overlap: RadialOverlap,
    topology: [usize; 3],
    source_face_counts: [usize; 2],
    realization_work: u64,
    realized_vertices: u64,
}

// Tangent disks have no positive-area shared cap cell. Their point-only union
// cannot close one manifold shell and remains covered by the boundary-contact
// refusal matrix in `radial_miss_setops`.
const CONTACT_CASES: [ContactCase; 3] = [
    ContactCase {
        name: "lower cap strictly contains upper cap",
        radii: [3.0, 0.5],
        upper_radial_scale: 2.0,
        radial_overlap: RadialOverlap::StrictInternal,
        topology: [5, 4, 0],
        source_face_counts: [3, 2],
        realization_work: 1_092,
        realized_vertices: 0,
    },
    ContactCase {
        name: "upper cap strictly contains lower cap",
        radii: [0.5, 3.0],
        upper_radial_scale: 2.0,
        radial_overlap: RadialOverlap::StrictInternal,
        topology: [5, 4, 0],
        source_face_counts: [2, 3],
        realization_work: 1_092,
        realized_vertices: 0,
    },
    ContactCase {
        name: "coincident shared cap",
        radii: [1.0, 1.0],
        upper_radial_scale: 0.0,
        radial_overlap: RadialOverlap::Coincident,
        topology: [3, 2, 0],
        source_face_counts: [2, 2],
        realization_work: 420,
        realized_vertices: 0,
    },
];

#[derive(Debug, Clone, Copy)]
struct AnalyticCylinder {
    radius: f64,
    height: f64,
    center: Point3,
    axis: Vec3,
}

struct ContactFixture {
    model: Fixture,
    cylinders: [AnalyticCylinder; 2],
    radial_distance: f64,
}

#[derive(Debug, Clone, Copy)]
struct UnionOracle {
    volume: f64,
    surface_area: f64,
    centroid: Point3,
    centroidal_inertia: [[f64; 3]; 3],
}

struct UniteEvidence {
    report: kernel::OperationReport,
    xt: Vec<u8>,
}

fn exact_radial_direction(frame: Frame) -> Vec3 {
    let axis = frame.z();
    if axis.x == 0.0 && axis.y == 0.0 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        // The expansion-evaluated dot with `axis` is identically zero even
        // when `Frame` normalization made its stored components non-dyadic.
        Vec3::new(axis.y, -axis.x, 0.0)
    }
}

fn translated(point: Point3, vector: Vec3, scale: f64) -> Point3 {
    Point3::new(
        point.x + vector.x * scale,
        point.y + vector.y * scale,
        point.z + vector.z * scale,
    )
}

fn contact_fixture(
    case: ContactCase,
    placement: Placement,
    reversed_axes: [bool; 2],
) -> ContactFixture {
    let frame = match placement {
        Placement::World => shared_frame(placement),
        Placement::Oblique => shared_frame(placement).with_origin(Point3::new(0.5, -0.25, 0.125)),
    };
    let axis = frame.z();
    let radial = exact_radial_direction(frame) * case.upper_radial_scale;
    let lower_contact = frame.origin();
    let upper_contact = translated(lower_contact, radial, 1.0);
    let lower_frame = if reversed_axes[0] {
        Frame::new(lower_contact, -axis, frame.x()).unwrap()
    } else {
        frame.with_origin(translated(lower_contact, axis, -LOWER_HEIGHT))
    };
    let upper_frame = if reversed_axes[1] {
        Frame::new(
            translated(upper_contact, axis, UPPER_HEIGHT),
            -axis,
            frame.x(),
        )
        .unwrap()
    } else {
        frame.with_origin(upper_contact)
    };

    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (outer, inner) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let lower = edit
            .create_cylinder(CylinderRequest::new(
                lower_frame,
                case.radii[0],
                LOWER_HEIGHT,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let upper = edit
            .create_cylinder(CylinderRequest::new(
                upper_frame,
                case.radii[1],
                UPPER_HEIGHT,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (lower, upper)
    };
    let model = Fixture {
        session,
        part_id,
        outer,
        inner,
        frame,
    };
    assert_source_bodies_preserved(&model, 2);

    ContactFixture {
        model,
        cylinders: [
            AnalyticCylinder {
                radius: case.radii[0],
                height: LOWER_HEIGHT,
                center: translated(lower_contact, axis, -LOWER_HEIGHT / 2.0),
                axis,
            },
            AnalyticCylinder {
                radius: case.radii[1],
                height: UPPER_HEIGHT,
                center: translated(upper_contact, axis, UPPER_HEIGHT / 2.0),
                axis,
            },
        ],
        radial_distance: radial.norm(),
    }
}

fn assert_positive_area_relation(case: ContactCase, fixture: &ContactFixture) {
    let distance = fixture.radial_distance;
    let [first, second] = case.radii;
    match case.radial_overlap {
        RadialOverlap::StrictInternal => {
            assert!(distance > 0.0, "{}", case.name);
            assert!(
                distance + first.min(second) < first.max(second),
                "{}",
                case.name
            );
        }
        RadialOverlap::Coincident => {
            assert_eq!(distance.to_bits(), 0.0_f64.to_bits(), "{}", case.name);
            assert_eq!(first.to_bits(), second.to_bits(), "{}", case.name);
        }
    }
}

fn disk_overlap_area(case: ContactCase) -> f64 {
    let [first, second] = case.radii;
    match case.radial_overlap {
        RadialOverlap::Coincident => core::f64::consts::PI * first.powi(2),
        RadialOverlap::StrictInternal => core::f64::consts::PI * first.min(second).powi(2),
    }
}

fn cylinder_volume(cylinder: AnalyticCylinder) -> f64 {
    core::f64::consts::PI * cylinder.radius.powi(2) * cylinder.height
}

fn union_centroid(cylinders: [AnalyticCylinder; 2]) -> Point3 {
    let weights = cylinders.map(cylinder_volume);
    let total = weights[0] + weights[1];
    Point3::new(
        (weights[0] * cylinders[0].center.x + weights[1] * cylinders[1].center.x) / total,
        (weights[0] * cylinders[0].center.y + weights[1] * cylinders[1].center.y) / total,
        (weights[0] * cylinders[0].center.z + weights[1] * cylinders[1].center.z) / total,
    )
}

fn cylinder_centroidal_inertia(cylinder: AnalyticCylinder) -> [[f64; 3]; 3] {
    let volume = cylinder_volume(cylinder);
    let axial = volume * cylinder.radius.powi(2) / 2.0;
    let transverse = volume * (3.0 * cylinder.radius.powi(2) + cylinder.height.powi(2)) / 12.0;
    let axis = cylinder.axis.to_array();
    core::array::from_fn(|row| {
        core::array::from_fn(|column| {
            let identity = if row == column { 1.0 } else { 0.0 };
            transverse * identity + (axial - transverse) * axis[row] * axis[column]
        })
    })
}

fn union_centroidal_inertia(cylinders: [AnalyticCylinder; 2], centroid: Point3) -> [[f64; 3]; 3] {
    let mut result = [[0.0; 3]; 3];
    let centroid = centroid.to_array();
    for cylinder in cylinders {
        let local = cylinder_centroidal_inertia(cylinder);
        let center = cylinder.center.to_array();
        let offset: [f64; 3] = core::array::from_fn(|axis| center[axis] - centroid[axis]);
        let squared_distance = offset.iter().map(|value| value.powi(2)).sum::<f64>();
        let volume = cylinder_volume(cylinder);
        for row in 0..3 {
            for column in 0..3 {
                let identity = if row == column { 1.0 } else { 0.0 };
                result[row][column] += local[row][column]
                    + volume * (squared_distance * identity - offset[row] * offset[column]);
            }
        }
    }
    result
}

fn independent_union_oracle(case: ContactCase, fixture: &ContactFixture) -> UnionOracle {
    let cylinders = fixture.cylinders;
    let volume = cylinders.into_iter().map(cylinder_volume).sum();
    let source_area = cylinders.into_iter().map(|cylinder| {
        2.0 * core::f64::consts::PI * cylinder.radius * cylinder.height
            + 2.0 * core::f64::consts::PI * cylinder.radius.powi(2)
    });
    let centroid = union_centroid(cylinders);
    UnionOracle {
        volume,
        surface_area: source_area.sum::<f64>() - 2.0 * disk_overlap_area(case),
        centroid,
        centroidal_inertia: union_centroidal_inertia(cylinders, centroid),
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

fn assert_full_reports(created: &kernel::BooleanCreatedResult) {
    assert_eq!(created.bodies().len(), 1);
    assert_eq!(created.reports().len(), 1);
    assert_eq!(created.reports()[0].body(), created.bodies()[0]);
    let report = created.reports()[0].report();
    assert_eq!(report.level(), CheckLevel::Full);
    assert_eq!(report.outcome(), CheckOutcome::Valid, "{report:#?}");
    assert!(report.faults().is_empty());
    assert!(report.gaps().is_empty());
}

fn assert_face_lineage(
    fixture: &ContactFixture,
    case: ContactCase,
    body: BodyId,
    created: &kernel::BooleanCreatedResult,
) {
    assert_eq!(created.journal().part(), fixture.model.part_id);
    assert!(created.journal().mutation_count() > 0);
    assert!(
        created
            .journal()
            .mutations()
            .all(|mutation| mutation.kind() == MutationKind::Created)
    );

    let part = fixture
        .model
        .session
        .part(fixture.model.part_id.clone())
        .unwrap();
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
    let source_faces = [fixture.model.outer.clone(), fixture.model.inner.clone()].map(|source| {
        part.body(source)
            .unwrap()
            .faces()
            .unwrap()
            .collect::<Vec<_>>()
    });
    let source_edges = [fixture.model.outer.clone(), fixture.model.inner.clone()].map(|source| {
        part.body(source)
            .unwrap()
            .edges()
            .unwrap()
            .collect::<Vec<_>>()
    });
    let mut derived_faces = Vec::new();
    let mut derived_edges = Vec::new();
    let mut face_sources = [0_usize; 2];
    for event in created.journal().lineage() {
        match event {
            LineageView::DerivedFrom {
                derived: JournalEntity::Face(derived),
                source: JournalEntity::Face(source),
            } => {
                assert!(result_faces.contains(&derived), "{}", case.name);
                assert!(!derived_faces.contains(&derived), "{}", case.name);
                derived_faces.push(derived);
                let source_index = source_faces
                    .iter()
                    .position(|faces| faces.contains(&source))
                    .unwrap_or_else(|| panic!("{} face lineage escaped both sources", case.name));
                face_sources[source_index] += 1;
            }
            LineageView::DerivedFrom {
                derived: JournalEntity::Edge(derived),
                source: JournalEntity::Edge(source),
            } => {
                assert!(result_edges.contains(&derived), "{}", case.name);
                assert!(!derived_edges.contains(&derived), "{}", case.name);
                derived_edges.push(derived);
                assert!(
                    source_edges.iter().any(|edges| edges.contains(&source)),
                    "{}",
                    case.name
                );
            }
            LineageView::Merge { sources, result } => {
                assert!(matches!(case.radial_overlap, RadialOverlap::Coincident));
                let JournalEntity::Face(result) = result else {
                    panic!("{} merge result changed entity kind", case.name)
                };
                assert!(result_faces.contains(&result), "{}", case.name);
                assert!(!derived_faces.contains(&result), "{}", case.name);
                derived_faces.push(result);
                let sources = sources.collect::<Vec<_>>();
                assert_eq!(sources.len(), 2, "{}", case.name);
                for (source_index, source) in sources.into_iter().enumerate() {
                    let JournalEntity::Face(source) = source else {
                        panic!("{} merge source changed entity kind", case.name)
                    };
                    assert!(
                        source_faces[source_index].contains(&source),
                        "{}",
                        case.name
                    );
                    assert_eq!(
                        part.face(source).unwrap().loops().len(),
                        2,
                        "{} merge source must be the cylinder side",
                        case.name
                    );
                    face_sources[source_index] += 1;
                }
            }
            _ => panic!("{} Unite lineage changed entity kind", case.name),
        }
    }
    assert_eq!(derived_faces.len(), result_faces.len(), "{}", case.name);
    assert_eq!(derived_edges.len(), result_edges.len(), "{}", case.name);
    assert_eq!(
        created.journal().lineage_count(),
        result_faces.len() + result_edges.len(),
        "{}",
        case.name
    );
    assert_eq!(face_sources, case.source_face_counts, "{}", case.name);
}

fn assert_properties(
    fixture: &ContactFixture,
    case: ContactCase,
    body: BodyId,
    oracle: UnionOracle,
) {
    let part = fixture
        .model
        .session
        .part(fixture.model.part_id.clone())
        .unwrap();
    let outcome = part
        .body_properties(BodyPropertiesRequest::new(body))
        .unwrap();
    let BodyPropertiesOutcome::Certified {
        properties,
        full_check,
    } = outcome.into_result().unwrap()
    else {
        panic!("{} Unite properties were not certified", case.name)
    };
    assert_eq!(full_check.level(), CheckLevel::Full);
    assert_eq!(full_check.outcome(), CheckOutcome::Valid, "{full_check:#?}");
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
}

fn deterministic_xt(fixture: &mut ContactFixture, body: BodyId) -> Vec<u8> {
    let bytes = {
        let part = fixture
            .model
            .session
            .part(fixture.model.part_id.clone())
            .unwrap();
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
    };
    assert_fast_self_import(&mut fixture.model.session, &bytes);
    bytes
}

fn run_unite_case(
    case: ContactCase,
    placement: Placement,
    reversed_axes: [bool; 2],
    swapped: bool,
) -> UniteEvidence {
    run_unite_case_with_settings(
        case,
        placement,
        reversed_axes,
        swapped,
        OperationSettings::new(),
    )
}

fn run_unite_case_with_settings(
    case: ContactCase,
    placement: Placement,
    reversed_axes: [bool; 2],
    swapped: bool,
    settings: OperationSettings,
) -> UniteEvidence {
    let mut fixture = contact_fixture(case, placement, reversed_axes);
    assert_positive_area_relation(case, &fixture);
    let before = fixture_signature(&fixture.model);
    let outcome = run_unite(&mut fixture.model, swapped, settings);
    assert_eq!(
        work_at(&outcome, BOOLEAN_BSP_WORK),
        Some(AXIAL_CONTACT_RELATION_WORK),
        "{} {placement:?} reversed_axes={reversed_axes:?} swapped={swapped}",
        case.name
    );
    assert_eq!(
        work_at(&outcome, BOOLEAN_POST_SELECTION_WORK),
        Some(case.realization_work),
        "{} {placement:?} reversed_axes={reversed_axes:?} swapped={swapped}",
        case.name
    );
    assert_eq!(
        item_usage_at(&outcome, BOOLEAN_REALIZED_VERTICES),
        Some(case.realized_vertices),
        "{} {placement:?} reversed_axes={reversed_axes:?} swapped={swapped}",
        case.name
    );
    let report = outcome.report().clone();
    let result = outcome.into_result().unwrap();
    let BooleanOutcome::Success(BooleanResult::Created(created)) = result else {
        assert_eq!(fixture_signature(&fixture.model), before, "{}", case.name);
        panic!(
            "{} {placement:?} reversed_axes={reversed_axes:?} swapped={swapped} returned {result:#?}",
            case.name
        )
    };
    assert_full_reports(&created);
    let body = created.bodies()[0].clone();
    {
        let part = fixture
            .model
            .session
            .part(fixture.model.part_id.clone())
            .unwrap();
        assert_eq!(
            body_topology(&part, body.clone()),
            case.topology,
            "{}",
            case.name
        );
        assert_connected_solid(&part, body.clone());
    }
    assert_face_lineage(&fixture, case, body.clone(), &created);
    assert_properties(
        &fixture,
        case,
        body.clone(),
        independent_union_oracle(case, &fixture),
    );
    assert_source_bodies_preserved(&fixture.model, 3);
    let xt = deterministic_xt(&mut fixture, body);
    UniteEvidence { report, xt }
}

fn work_at(outcome: &OperationOutcome<BooleanOutcome>, stage: kernel::StageId) -> Option<u64> {
    outcome
        .report()
        .usage()
        .iter()
        .find(|usage| usage.stage == stage && usage.resource == ResourceKind::Work)
        .map(|usage| usage.consumed)
}

fn item_usage_at(
    outcome: &OperationOutcome<BooleanOutcome>,
    stage: kernel::StageId,
) -> Option<u64> {
    outcome
        .report()
        .usage()
        .iter()
        .find(|usage| usage.stage == stage && usage.resource == ResourceKind::Items)
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

fn assert_limit(
    outcome: &OperationOutcome<BooleanOutcome>,
    stage: kernel::StageId,
    expected_work: u64,
) {
    let limit = *outcome
        .report()
        .limit_events()
        .first()
        .expect("axial-contact Unite N-1 refusal recorded no limit event");
    assert_eq!(limit.stage, stage);
    assert_eq!(limit.resource, ResourceKind::Work);
    assert_eq!(limit.allowed, expected_work - 1);
    assert_eq!(limit.consumed, expected_work);
    assert_eq!(outcome.result().unwrap_err().limit(), Some(limit));
    assert_eq!(outcome.report().limit_events(), &[limit]);
}

fn assert_work_frontier(
    case: ContactCase,
    reversed_axes: [bool; 2],
    stage: kernel::StageId,
    expected_work: u64,
) {
    let mut baseline = contact_fixture(case, Placement::World, reversed_axes);
    let outcome = run_unite(&mut baseline.model, false, OperationSettings::new());
    assert_eq!(
        work_at(&outcome, stage),
        Some(expected_work),
        "{}",
        case.name
    );
    assert!(matches!(
        outcome.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));
    assert_source_bodies_preserved(&baseline.model, 3);

    let mut admitted = contact_fixture(case, Placement::World, reversed_axes);
    let outcome = run_unite(
        &mut admitted.model,
        false,
        settings_at(stage, expected_work),
    );
    assert!(matches!(
        outcome.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));
    assert_source_bodies_preserved(&admitted.model, 3);

    let mut denied = contact_fixture(case, Placement::World, reversed_axes);
    let before = fixture_signature(&denied.model);
    let outcome = run_unite(
        &mut denied.model,
        false,
        settings_at(stage, expected_work - 1),
    );
    assert_limit(&outcome, stage, expected_work);
    assert_eq!(fixture_signature(&denied.model), before, "{}", case.name);
    assert_source_bodies_preserved(&denied.model, 2);
}

fn assert_exact_general_matrix(cases: &[ContactCase]) {
    let mut executions = 0_usize;
    for &case in cases {
        for placement in [Placement::World, Placement::Oblique] {
            for reversed_axes in [[false, false], [false, true], [true, false], [true, true]] {
                let mut canonical: Option<UniteEvidence> = None;
                for swapped in [false, true] {
                    for repeat in 0..2 {
                        let evidence = run_unite_case(case, placement, reversed_axes, swapped);
                        if let Some(expected) = canonical.as_ref() {
                            assert_eq!(
                                evidence.report, expected.report,
                                "{} {placement:?} reversed_axes={reversed_axes:?} swapped={swapped} repeat={repeat}: report changed",
                                case.name
                            );
                            assert_xt_equal(
                                &evidence.xt,
                                &expected.xt,
                                &format!(
                                    "{} {placement:?} reversed_axes={reversed_axes:?} swapped={swapped} repeat={repeat}: X_T changed",
                                    case.name
                                ),
                            );
                        } else {
                            canonical = Some(evidence);
                        }
                        executions += 1;
                    }
                }
            }
        }
    }
    assert_eq!(executions, cases.len() * 32);
}

fn assert_exact_work_frontiers(cases: &[ContactCase]) {
    for &case in cases {
        for reversed_axes in [[false, false], [true, false]] {
            assert_work_frontier(
                case,
                reversed_axes,
                BOOLEAN_BSP_WORK,
                AXIAL_CONTACT_RELATION_WORK,
            );
            assert_work_frontier(
                case,
                reversed_axes,
                BOOLEAN_POST_SELECTION_WORK,
                case.realization_work,
            );
        }
    }
}

#[test]
fn strict_internal_axial_contact_unite_full_commits_both_containment_directions() {
    assert_exact_general_matrix(&CONTACT_CASES[..2]);
}

#[test]
fn coincident_axial_contact_unite_full_commits_the_exact_general_matrix() {
    assert_exact_general_matrix(&CONTACT_CASES[2..]);
}

#[test]
fn internal_axial_contact_unite_work_accepts_n_and_refuses_n_minus_one_atomically() {
    assert_exact_work_frontiers(&CONTACT_CASES[..2]);
    for &case in &CONTACT_CASES[..2] {
        for reversed_axes in [[false, false], [true, false]] {
            assert_work_frontier(
                case,
                reversed_axes,
                internal_contact_shell_stage(),
                INTERNAL_CONTACT_SHELL_WORK,
            );
        }
    }
}

#[test]
fn coincident_axial_contact_unite_work_accepts_n_and_refuses_n_minus_one_atomically() {
    assert_exact_work_frontiers(&CONTACT_CASES[2..]);
}

#[test]
fn coincident_axial_contact_refuses_resolution_near_but_not_exact_coaxial_supports() {
    let contact = Point3::new(0.5, -0.25, 0.125);
    let frame = shared_frame(Placement::Oblique).with_origin(contact);
    let lower_height = 0.125;
    let lower_frame = frame.with_origin(translated(contact, frame.z(), -lower_height));

    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (outer, inner) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let lower = edit
            .create_cylinder(CylinderRequest::new(lower_frame, 1.0, lower_height))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let upper = edit
            .create_cylinder(CylinderRequest::new(frame, 1.0, UPPER_HEIGHT))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (lower, upper)
    };
    let mut fixture = Fixture {
        session,
        part_id,
        outer,
        inner,
        frame,
    };
    let before = fixture_signature(&fixture);
    let outcome = run_unite(&mut fixture, false, OperationSettings::new());
    assert_eq!(
        work_at(&outcome, BOOLEAN_BSP_WORK),
        Some(AXIAL_CONTACT_RELATION_WORK),
    );
    assert_eq!(work_at(&outcome, BOOLEAN_POST_SELECTION_WORK), Some(0));
    assert_eq!(item_usage_at(&outcome, BOOLEAN_REALIZED_VERTICES), Some(0),);
    assert!(matches!(
        outcome.into_result().unwrap(),
        BooleanOutcome::Refused(BooleanRefusal::BoundaryContact),
    ));
    assert_eq!(fixture_signature(&fixture), before);
    assert_source_bodies_preserved(&fixture, 2);
}

#[test]
fn exact_internal_contact_keeps_session_resolution_under_loose_operation_tolerance() {
    let resolution = Tolerances::default().linear();
    let case = ContactCase {
        name: "sub-operation-tolerance internal annulus",
        radii: [2.0, 1.0],
        upper_radial_scale: 1.0 - 4.0 * resolution,
        radial_overlap: RadialOverlap::StrictInternal,
        topology: [5, 4, 0],
        source_face_counts: [3, 2],
        realization_work: 1_092,
        realized_vertices: 0,
    };
    let baseline = run_unite_case_with_settings(
        case,
        Placement::World,
        [false, false],
        false,
        OperationSettings::new(),
    );
    let loose = run_unite_case_with_settings(
        case,
        Placement::World,
        [false, false],
        false,
        OperationSettings::new().with_tolerances(Tolerances::with_linear(1.0e-6).unwrap()),
    );

    assert_eq!(loose.report, baseline.report);
    assert_xt_equal(
        &loose.xt,
        &baseline.xt,
        "loose operation tolerance changed an exact contact result",
    );
}
