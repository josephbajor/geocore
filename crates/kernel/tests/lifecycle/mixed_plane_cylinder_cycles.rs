//! Facade-only lifecycle evidence for closed mixed Plane/Cylinder section cycles.
//!
//! Wall-time budget: less than 60 seconds for the table-driven matrix.

use super::*;
use kcore::math::atan2;
use kernel::{
    ClassifyPointOnFaceRequest, EdgeId, FaceId, PointFaceVerdict, SectionBranch,
    SectionBranchTopology, SectionCarrier, SectionCurveComponent, SectionCurveEndpointTopology,
    SectionCurveFragment, SectionCurveFragmentSpan, SectionEdgeParameterInterval,
    SectionPeriodicFaceEmbeddingEvidence, SectionSite, SectionSourceParameterKey, VertexId,
};

const RADIUS: f64 = 1.5;
const HALF_BLOCK_X: f64 = 1.0;
const ROOT_Y: f64 = 1.118_033_988_749_895;
const SLAB_LO: f64 = 0.5;
const SLAB_HI: f64 = 1.5;
const CYLINDER_HEIGHT: f64 = 2.0;
const GEOMETRY_TOLERANCE: f64 = 1.0e-9;
/// Independently evaluated `asin(HALF_BLOCK_X / RADIUS)` reference.
const STRIP_HALF_ANGLE: f64 = 0.729_727_656_226_966_3;

#[derive(Debug, Clone, Copy)]
enum Placement {
    World,
    Translated,
    AxisPermuted,
    Oblique,
}

#[derive(Debug, Clone, Copy)]
struct MixedCycleCase {
    name: &'static str,
    placement: Placement,
    swapped: bool,
}

const CASES: [MixedCycleCase; 8] = [
    MixedCycleCase {
        name: "world_block_first",
        placement: Placement::World,
        swapped: false,
    },
    MixedCycleCase {
        name: "world_cylinder_first",
        placement: Placement::World,
        swapped: true,
    },
    MixedCycleCase {
        name: "translated_block_first",
        placement: Placement::Translated,
        swapped: false,
    },
    MixedCycleCase {
        name: "translated_cylinder_first",
        placement: Placement::Translated,
        swapped: true,
    },
    MixedCycleCase {
        name: "axis_permuted_block_first",
        placement: Placement::AxisPermuted,
        swapped: false,
    },
    MixedCycleCase {
        name: "axis_permuted_cylinder_first",
        placement: Placement::AxisPermuted,
        swapped: true,
    },
    MixedCycleCase {
        name: "oblique_block_first",
        placement: Placement::Oblique,
        swapped: false,
    },
    MixedCycleCase {
        name: "oblique_cylinder_first",
        placement: Placement::Oblique,
        swapped: true,
    },
];

fn mixed_frame(placement: Placement) -> Frame {
    match placement {
        Placement::World => Frame::world(),
        Placement::Translated => Frame::world().with_origin(Point3::new(4.0, -3.0, 2.0)),
        Placement::AxisPermuted => Frame::new(
            Point3::new(-2.0, 3.5, 1.25),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap(),
        Placement::Oblique => Frame::new(
            Point3::new(3.0, -2.0, 1.25),
            Vec3::new(0.48, 0.64, 0.6),
            Vec3::new(0.8, -0.6, 0.0),
        )
        .unwrap(),
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct BodySignature {
    faces: Vec<FaceId>,
    edges: Vec<EdgeId>,
    vertices: Vec<VertexId>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct SourceSignature {
    block: BodySignature,
    cylinder: BodySignature,
    body_count: usize,
    geometry_counts: [usize; 3],
}

struct MixedCycleFixture {
    session: Session,
    part_id: PartId,
    block: BodyId,
    cylinder: BodyId,
    frame: Frame,
    before: SourceSignature,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FragmentKind {
    Arc,
    Line,
}

#[derive(Debug, Clone)]
struct EndpointOccurrence {
    endpoint: usize,
    kind: FragmentKind,
    point: Point3,
    source_parameter: SectionSourceParameterKey,
    edge_parameter: SectionEdgeParameterInterval,
}

fn body_signature(part: &kernel::Part<'_>, body: BodyId) -> BodySignature {
    let body = part.body(body).unwrap();
    BodySignature {
        faces: body.faces().unwrap().collect(),
        edges: body.edges().unwrap().collect(),
        vertices: body.vertices().unwrap().collect(),
    }
}

fn source_signature(
    session: &Session,
    part_id: &PartId,
    block: &BodyId,
    cylinder: &BodyId,
) -> SourceSignature {
    let part = session.part(part_id.clone()).unwrap();
    SourceSignature {
        block: body_signature(&part, block.clone()),
        cylinder: body_signature(&part, cylinder.clone()),
        body_count: part.bodies().len(),
        geometry_counts: [
            part.curves().len(),
            part.pcurves().len(),
            part.surfaces().len(),
        ],
    }
}

fn mixed_cycle_fixture(case: MixedCycleCase) -> MixedCycleFixture {
    let frame = mixed_frame(case.placement);
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (block, cylinder) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        // The cavity's x faces cut four rulings. The host's top and bottom
        // faces lie strictly inside the cylinder and cut four matching arcs.
        let block = edit
            .extrude_profile(ExtrudeProfileRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, SLAB_LO)),
                vec![
                    Point2::new(-3.0, -3.0),
                    Point2::new(3.0, -3.0),
                    Point2::new(3.0, 3.0),
                    Point2::new(-3.0, 3.0),
                ],
                vec![vec![
                    Point2::new(-HALF_BLOCK_X, -2.5),
                    Point2::new(-HALF_BLOCK_X, 2.5),
                    Point2::new(HALF_BLOCK_X, 2.5),
                    Point2::new(HALF_BLOCK_X, -2.5),
                ]],
                SLAB_HI - SLAB_LO,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let cylinder = edit
            .create_cylinder(CylinderRequest::new(frame, RADIUS, CYLINDER_HEIGHT))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (block, cylinder)
    };
    let before = source_signature(&session, &part_id, &block, &cylinder);
    assert_eq!(before.block.faces.len(), 10, "{}", case.name);
    assert_eq!(before.block.edges.len(), 24, "{}", case.name);
    assert_eq!(before.block.vertices.len(), 16, "{}", case.name);
    assert_eq!(before.cylinder.faces.len(), 3, "{}", case.name);
    assert_eq!(before.cylinder.edges.len(), 2, "{}", case.name);
    assert!(before.cylinder.vertices.is_empty(), "{}", case.name);
    assert_eq!(before.body_count, 2, "{}", case.name);
    MixedCycleFixture {
        session,
        part_id,
        block,
        cylinder,
        frame,
        before,
    }
}

fn convex_mixed_cycle_fixture(placement: Placement) -> MixedCycleFixture {
    let frame = mixed_frame(placement);
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (block, cylinder) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let block = edit
            .extrude_profile(ExtrudeProfileRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, SLAB_LO)),
                vec![
                    Point2::new(-HALF_BLOCK_X, -3.0),
                    Point2::new(HALF_BLOCK_X, -3.0),
                    Point2::new(HALF_BLOCK_X, 3.0),
                    Point2::new(-HALF_BLOCK_X, 3.0),
                ],
                Vec::new(),
                SLAB_HI - SLAB_LO,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let cylinder = edit
            .create_cylinder(CylinderRequest::new(frame, RADIUS, CYLINDER_HEIGHT))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (block, cylinder)
    };
    let before = source_signature(&session, &part_id, &block, &cylinder);
    assert_eq!(before.block.faces.len(), 6);
    assert_eq!(before.block.edges.len(), 12);
    assert_eq!(before.block.vertices.len(), 8);
    MixedCycleFixture {
        session,
        part_id,
        block,
        cylinder,
        frame,
        before,
    }
}

fn convex_three_patch_fixture(placement: Placement) -> MixedCycleFixture {
    let frame = mixed_frame(placement);
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (block, cylinder) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        // Every triangular side has support distance below the cylinder
        // radius, while every vertex lies outside it. The intersection is a
        // general convex clipped cylinder with three disjoint side charts.
        let block = edit
            .extrude_profile(ExtrudeProfileRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, SLAB_LO)),
                vec![
                    Point2::new(-4.0, -1.25),
                    Point2::new(4.0, -1.25),
                    Point2::new(0.0, 1.75),
                ],
                Vec::new(),
                SLAB_HI - SLAB_LO,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let cylinder = edit
            .create_cylinder(CylinderRequest::new(frame, RADIUS, CYLINDER_HEIGHT))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (block, cylinder)
    };
    let before = source_signature(&session, &part_id, &block, &cylinder);
    assert_eq!(before.block.faces.len(), 5);
    assert_eq!(before.block.edges.len(), 9);
    assert_eq!(before.block.vertices.len(), 6);
    MixedCycleFixture {
        session,
        part_id,
        block,
        cylinder,
        frame,
        before,
    }
}

fn convex_five_patch_fixture(placement: Placement) -> MixedCycleFixture {
    let frame = mixed_frame(placement);
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (block, cylinder) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        // A convex five-support profile with every support strictly inside the
        // radius-3/2 disk and every vertex strictly outside it. The clipped
        // disk therefore alternates five source spans and five cylinder arcs.
        let block = edit
            .extrude_profile(ExtrudeProfileRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, SLAB_LO)),
                vec![
                    Point2::new(0.0, 1.6),
                    Point2::new(-1.521_690_426, 0.494_427_191),
                    Point2::new(-0.940_456_404, -1.294_427_191),
                    Point2::new(0.940_456_404, -1.294_427_191),
                    Point2::new(1.521_690_426, 0.494_427_191),
                ],
                Vec::new(),
                SLAB_HI - SLAB_LO,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let cylinder = edit
            .create_cylinder(CylinderRequest::new(frame, RADIUS, CYLINDER_HEIGHT))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (block, cylinder)
    };
    let before = source_signature(&session, &part_id, &block, &cylinder);
    assert_eq!(before.block.faces.len(), 7);
    assert_eq!(before.block.edges.len(), 15);
    assert_eq!(before.block.vertices.len(), 10);
    MixedCycleFixture {
        session,
        part_id,
        block,
        cylinder,
        frame,
        before,
    }
}

fn seam_crossing_five_patch_fixture(placement: Placement) -> MixedCycleFixture {
    let frame = mixed_frame(placement);
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (block, cylinder) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        // Circumradius 1.7 makes the portal centered nearest the cylinder's
        // periodic seam cross that seam. Every pentagon support remains
        // strictly inside the radius-1.5 disk and every vertex outside it.
        let block = edit
            .extrude_profile(ExtrudeProfileRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, SLAB_LO)),
                vec![
                    Point2::new(0.0, 1.7),
                    Point2::new(-1.616_796_077_701_761, 0.525_328_890_437_410_6),
                    Point2::new(-0.999_234_928_897_204_3, -1.375_328_890_437_410_5),
                    Point2::new(0.999_234_928_897_203_8, -1.375_328_890_437_411),
                    Point2::new(1.616_796_077_701_761, 0.525_328_890_437_41),
                ],
                Vec::new(),
                SLAB_HI - SLAB_LO,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let cylinder = edit
            .create_cylinder(CylinderRequest::new(frame, RADIUS, CYLINDER_HEIGHT))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (block, cylinder)
    };
    let before = source_signature(&session, &part_id, &block, &cylinder);
    assert_eq!(before.block.faces.len(), 7);
    assert_eq!(before.block.edges.len(), 15);
    assert_eq!(before.block.vertices.len(), 10);
    assert_eq!(before.cylinder.faces.len(), 3);
    assert_eq!(before.cylinder.edges.len(), 2);
    assert!(before.cylinder.vertices.is_empty());
    MixedCycleFixture {
        session,
        part_id,
        block,
        cylinder,
        frame,
        before,
    }
}

fn nonconvex_star_profile() -> Vec<Point2> {
    vec![
        Point2::new(0.0, 2.5),
        Point2::new(-0.587_785_252_292_473, 0.809_016_994_374_947_5),
        Point2::new(-2.377_641_290_737_884, 0.772_542_485_937_368_6),
        Point2::new(-0.951_056_516_295_153_6, -0.309_016_994_374_947_3),
        Point2::new(-1.469_463_130_731_183_2, -2.022_542_485_937_368),
        Point2::new(0.0, -1.0),
        Point2::new(1.469_463_130_731_182_5, -2.022_542_485_937_369),
        Point2::new(0.951_056_516_295_153_5, -0.309_016_994_374_947_6),
        Point2::new(2.377_641_290_737_884, 0.772_542_485_937_368_5),
        Point2::new(0.587_785_252_292_473_1, 0.809_016_994_374_947_5),
    ]
}

fn literal_segment_circle_root(start: Point2, end: Point2, radius: f64) -> Point2 {
    let direction = end - start;
    let a = direction.dot(direction);
    let b = 2.0 * start.dot(direction);
    let c = start.dot(start) - radius * radius;
    let discriminant = b * b - 4.0 * a * c;
    assert!(discriminant > 0.0);
    let denominator = 2.0 * a;
    let first = (-b - discriminant.sqrt()) / denominator;
    let second = (-b + discriminant.sqrt()) / denominator;
    let first_on_edge = (0.0..=1.0).contains(&first);
    let second_on_edge = (0.0..=1.0).contains(&second);
    assert_ne!(first_on_edge, second_on_edge);
    start + direction * if first_on_edge { first } else { second }
}

fn literal_star_disk_intersection_area(profile: &[Point2], radius: f64) -> f64 {
    assert_eq!(profile.len(), 10);
    let radius_squared = radius * radius;
    let mut line_area = 0.0;
    let mut arc_area = 0.0;
    for index in 0..profile.len() {
        let start = profile[index];
        let end = profile[(index + 1) % profile.len()];
        let start_inside = start.dot(start) < radius_squared;
        let end_inside = end.dot(end) < radius_squared;
        assert_ne!(start_inside, end_inside);
        let root = literal_segment_circle_root(start, end, radius);
        if start_inside {
            // Green's theorem contributes cross(p, q) / 2 on each retained
            // literal line span. The following edge enters the disk after the
            // intervening outer star vertex, so its two roots bound the
            // retained counter-clockwise circle arc.
            line_area += start.cross(root) * 0.5;
            let after_outer = profile[(index + 2) % profile.len()];
            let entry = literal_segment_circle_root(end, after_outer, radius);
            let exit_angle = atan2(root.y, root.x);
            let entry_angle = atan2(entry.y, entry.x);
            let arc_angle = (entry_angle - exit_angle).rem_euclid(std::f64::consts::TAU);
            assert!(arc_angle < std::f64::consts::PI);
            arc_area += radius_squared * arc_angle * 0.5;
        } else {
            line_area += root.cross(end) * 0.5;
        }
    }
    line_area + arc_area
}

fn nonconvex_star_five_patch_fixture(placement: Placement) -> MixedCycleFixture {
    let frame = mixed_frame(placement);
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (block, cylinder) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        // A simple ten-vertex star alternates exact authored outer and inner
        // radii 5/2 and 1. Every edge therefore crosses the radius-3/2 disk
        // exactly once, producing five bounded mixed components without a
        // convex planar-source certificate.
        let block = edit
            .extrude_profile(ExtrudeProfileRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, SLAB_LO)),
                nonconvex_star_profile(),
                Vec::new(),
                SLAB_HI - SLAB_LO,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let cylinder = edit
            .create_cylinder(CylinderRequest::new(frame, RADIUS, CYLINDER_HEIGHT))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (block, cylinder)
    };
    let before = source_signature(&session, &part_id, &block, &cylinder);
    assert_eq!(before.block.faces.len(), 12);
    assert_eq!(before.block.edges.len(), 30);
    assert_eq!(before.block.vertices.len(), 20);
    assert_eq!(before.cylinder.faces.len(), 3);
    assert_eq!(before.cylinder.edges.len(), 2);
    assert!(before.cylinder.vertices.is_empty());
    MixedCycleFixture {
        session,
        part_id,
        block,
        cylinder,
        frame,
        before,
    }
}

fn cap_retaining_fixture(placement: Placement) -> MixedCycleFixture {
    let frame = mixed_frame(placement);
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (block, cylinder) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let block = edit
            .create_block(BlockRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, 1.0)),
                [2.0, 5.0, 1.0],
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let cylinder = edit
            .create_cylinder(CylinderRequest::new(frame, RADIUS, CYLINDER_HEIGHT))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (block, cylinder)
    };
    let before = source_signature(&session, &part_id, &block, &cylinder);
    assert_eq!(before.block.faces.len(), 6);
    assert_eq!(before.block.edges.len(), 12);
    assert_eq!(before.block.vertices.len(), 8);
    assert_eq!(before.cylinder.faces.len(), 3);
    assert_eq!(before.cylinder.edges.len(), 2);
    assert!(before.cylinder.vertices.is_empty());
    MixedCycleFixture {
        session,
        part_id,
        block,
        cylinder,
        frame,
        before,
    }
}

fn assert_mixed_deterministic_xt_and_fast_self_import(
    session: &mut Session,
    part_id: &PartId,
    body: &BodyId,
) -> Vec<u8> {
    let bytes = {
        let part = session.part(part_id.clone()).unwrap();
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
    };
    let imported_part = session.create_part();
    let imported = session
        .edit_part(imported_part.clone())
        .unwrap()
        .import_xt(ImportXtRequest::new(&bytes))
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
    bytes
}

fn mesh_volume(positions: &[Point3], triangles: &[[u32; 3]]) -> f64 {
    let six_volume = triangles.iter().fold(0.0, |sum, triangle| {
        let first = positions[triangle[0] as usize];
        let second = positions[triangle[1] as usize];
        let third = positions[triangle[2] as usize];
        sum + first.dot(second.cross(third))
    });
    (six_volume / 6.0).abs()
}

fn assert_planar_minus_cylinder_components(
    mut fixture: MixedCycleFixture,
    expected_bodies: usize,
    expected_topology: (usize, usize, usize),
    expected_total_volume: Option<f64>,
) {
    let outcome = fixture
        .session
        .edit_part(fixture.part_id.clone())
        .unwrap()
        .boolean_bodies(BooleanBodiesRequest::new(
            BooleanOperation::Subtract,
            fixture.block.clone(),
            fixture.cylinder.clone(),
        ))
        .unwrap()
        .into_result()
        .unwrap();
    let created = match outcome {
        BooleanOutcome::Success(BooleanResult::Created(created)) => created,
        other => panic!("ordered planar-minus-cylinder did not commit: {other:?}"),
    };
    assert_eq!(created.bodies().len(), expected_bodies);
    assert_eq!(created.reports().len(), expected_bodies);
    assert!(
        created
            .reports()
            .iter()
            .all(|report| report.report().outcome() == CheckOutcome::Valid)
    );

    let mut total_volume = 0.0;
    for body in created.bodies() {
        let signature = body_signature(
            &fixture.session.part(fixture.part_id.clone()).unwrap(),
            body.clone(),
        );
        assert_eq!(
            (
                signature.faces.len(),
                signature.edges.len(),
                signature.vertices.len(),
            ),
            expected_topology,
        );
        let full = fixture
            .session
            .part(fixture.part_id.clone())
            .unwrap()
            .check_body(CheckBodyRequest::new(body.clone(), CheckLevel::Full))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(full.outcome(), CheckOutcome::Valid, "{full:?}");
        let mesh = fixture
            .session
            .part(fixture.part_id.clone())
            .unwrap()
            .tessellate_body(TessellateBodyRequest::new(
                body.clone(),
                TessOptions {
                    chord_tol: 1.0e-4,
                    max_edge_len: None,
                },
            ))
            .unwrap()
            .into_result()
            .unwrap();
        total_volume += mesh_volume(mesh.positions(), mesh.triangles());
        let _xt = assert_mixed_deterministic_xt_and_fast_self_import(
            &mut fixture.session,
            &fixture.part_id,
            body,
        );
    }
    if let Some(expected) = expected_total_volume {
        assert!(
            (total_volume - expected).abs() <= expected * 2.0e-4,
            "mesh volume {total_volume:.17e} differs from independent profile value {expected:.17e}"
        );
    }

    let after = source_signature(
        &fixture.session,
        &fixture.part_id,
        &fixture.block,
        &fixture.cylinder,
    );
    assert_eq!(after.block, fixture.before.block, "source prism mutated");
    assert_eq!(
        after.cylinder, fixture.before.cylinder,
        "source cylinder mutated"
    );
    assert_eq!(
        after.body_count,
        fixture.before.body_count + expected_bodies
    );
}

fn assert_on_faces(part: &kernel::Part<'_>, faces: &[FaceId; 2], point: Point3, context: &str) {
    for face in faces {
        let verdict = part
            .classify_point_on_face(ClassifyPointOnFaceRequest::new(face.clone(), point))
            .unwrap()
            .into_result()
            .unwrap();
        assert!(
            matches!(verdict.verdict(), PointFaceVerdict::On(_)),
            "{context}: point is not on both source faces: {:?}",
            verdict.verdict()
        );
    }
}

fn fragment_kind(fragment: &SectionCurveFragment) -> FragmentKind {
    match fragment.span() {
        SectionCurveFragmentSpan::Arc { .. } => FragmentKind::Arc,
        SectionCurveFragmentSpan::LineSegment { .. } => FragmentKind::Line,
        SectionCurveFragmentSpan::Whole => {
            panic!("finite mixed-cycle fixture published a whole-period fragment")
        }
        _ => panic!("finite mixed-cycle fixture published an unknown fragment family"),
    }
}

fn fragment_endpoints(fragment: &SectionCurveFragment) -> [usize; 2] {
    match fragment.span() {
        SectionCurveFragmentSpan::Arc { endpoints, .. } => {
            endpoints.each_ref().map(|end| end.endpoint())
        }
        SectionCurveFragmentSpan::LineSegment { endpoints } => {
            endpoints.each_ref().map(|end| end.endpoint())
        }
        SectionCurveFragmentSpan::Whole => {
            panic!("finite mixed-cycle fixture published a whole-period fragment")
        }
        _ => panic!("finite mixed-cycle fixture published an unknown fragment family"),
    }
}

fn assert_analytic_endpoint(point: Point3, frame: Frame, context: &str) {
    let local = frame.to_local(point);
    assert!(
        (local.x.abs() - HALF_BLOCK_X).abs() <= GEOMETRY_TOLERANCE,
        "{context}: endpoint escaped the two in-radius block faces: {local:?}"
    );
    assert!(
        (local.y.abs() - ROOT_Y).abs() <= GEOMETRY_TOLERANCE,
        "{context}: endpoint escaped x^2 + y^2 = r^2: {local:?}"
    );
    assert!(
        (local.z - SLAB_LO).abs() <= GEOMETRY_TOLERANCE
            || (local.z - SLAB_HI).abs() <= GEOMETRY_TOLERANCE,
        "{context}: endpoint escaped the slab caps: {local:?}"
    );
    assert!(
        (local.x * local.x + local.y * local.y - RADIUS * RADIUS).abs() <= GEOMETRY_TOLERANCE,
        "{context}: endpoint escaped the authored cylinder: {local:?}"
    );
}

#[allow(clippy::too_many_arguments)]
fn assert_trim_provenance(
    part: &kernel::Part<'_>,
    graph: &kernel::BodySectionGraph,
    branch: &SectionBranch,
    endpoint: usize,
    point: Point3,
    operand: usize,
    face: FaceId,
    loop_id: kernel::LoopId,
    fin: kernel::FinId,
    source_parameter: &SectionSourceParameterKey,
    edge_parameter: SectionEdgeParameterInterval,
    block_slot: usize,
    block_edges: &[EdgeId],
    frame: Frame,
    context: &str,
) {
    assert!(endpoint < graph.curve_endpoints().len(), "{context}");
    assert_eq!(operand, block_slot, "{context}: trim escaped block slot");
    assert_eq!(face, branch.faces()[operand], "{context}");
    assert_eq!(
        part.loop_(loop_id.clone()).unwrap().face(),
        face,
        "{context}"
    );
    assert_eq!(part.fin(fin.clone()).unwrap().loop_(), loop_id, "{context}");
    assert_eq!(
        part.fin(fin).unwrap().edge(),
        source_parameter.edge(),
        "{context}"
    );
    assert!(block_edges.contains(&source_parameter.edge()), "{context}");
    assert!(
        edge_parameter.lo().is_finite()
            && edge_parameter.hi().is_finite()
            && edge_parameter.lo() < edge_parameter.hi(),
        "{context}: invalid source-edge enclosure"
    );

    let public_endpoint = &graph.curve_endpoints()[endpoint];
    let SectionCurveEndpointTopology::Trim {
        sites,
        source_parameters,
    } = public_endpoint.topology()
    else {
        panic!("{context}: physical endpoint became a parameter seam")
    };
    assert_eq!(
        sites[block_slot],
        SectionSite::EdgeInterior(source_parameter.edge()),
        "{context}"
    );
    assert_eq!(
        source_parameters[block_slot].as_ref(),
        Some(source_parameter),
        "{context}"
    );
    assert_eq!(
        sites[1 - block_slot],
        SectionSite::FaceInterior(branch.faces()[1 - block_slot].clone()),
        "{context}"
    );
    assert!(source_parameters[1 - block_slot].is_none(), "{context}");
    assert!(
        public_endpoint.edge_parameters()[1 - block_slot].is_none(),
        "{context}"
    );
    let common = public_endpoint.edge_parameters()[block_slot]
        .expect("mixed endpoint must retain common source-edge evidence");
    assert!(
        common.lo() >= edge_parameter.lo() && common.hi() <= edge_parameter.hi(),
        "{context}: interned evidence escaped its occurrence enclosure"
    );
    assert_analytic_endpoint(point, frame, context);
    assert_on_faces(part, branch.faces(), point, context);
}

fn collect_endpoint_occurrences(
    part: &kernel::Part<'_>,
    graph: &kernel::BodySectionGraph,
    block_slot: usize,
    block_edges: &[EdgeId],
    frame: Frame,
    case: MixedCycleCase,
) -> Vec<EndpointOccurrence> {
    let mut occurrences = Vec::with_capacity(2 * graph.curve_fragments().len());
    for (fragment_index, fragment) in graph.curve_fragments().iter().enumerate() {
        let branch = graph
            .branches()
            .get(fragment.branch())
            .unwrap_or_else(|| panic!("{}: fragment escaped branch table", case.name));
        assert!(
            branch.range().is_finite() && branch.range().lo < branch.range().hi,
            "{}",
            case.name
        );
        let evidence = branch.evidence();
        assert!(evidence.tolerance().is_finite() && evidence.tolerance() > 0.0);
        assert!(evidence.residual_bounds().into_iter().all(|residual| {
            residual.is_finite() && residual >= 0.0 && residual <= evidence.tolerance()
        }));

        match fragment.span() {
            SectionCurveFragmentSpan::Arc { endpoints, .. } => {
                assert!(matches!(branch.carrier(), SectionCarrier::Circle { .. }));
                assert_eq!(branch.topology(), SectionBranchTopology::Closed);
                for end in endpoints.iter() {
                    let trim = end.trim();
                    let context = format!("{} arc fragment {fragment_index}", case.name);
                    assert!(
                        branch.range().contains(end.carrier_parameter()),
                        "{context}"
                    );
                    assert!(
                        trim.pcurve_half_angle().lo().is_finite()
                            && trim.pcurve_half_angle().hi().is_finite()
                            && trim.pcurve_half_angle().lo() < trim.pcurve_half_angle().hi(),
                        "{context}"
                    );
                    assert_trim_provenance(
                        part,
                        graph,
                        branch,
                        end.endpoint(),
                        end.point(),
                        trim.operand(),
                        trim.face(),
                        trim.loop_id(),
                        trim.fin(),
                        trim.source_parameter(),
                        trim.edge_parameter(),
                        block_slot,
                        block_edges,
                        frame,
                        &context,
                    );
                    occurrences.push(EndpointOccurrence {
                        endpoint: end.endpoint(),
                        kind: FragmentKind::Arc,
                        point: end.point(),
                        source_parameter: trim.source_parameter().clone(),
                        edge_parameter: trim.edge_parameter(),
                    });
                }
            }
            SectionCurveFragmentSpan::LineSegment { endpoints } => {
                assert!(matches!(branch.carrier(), SectionCarrier::Line { .. }));
                assert_eq!(branch.topology(), SectionBranchTopology::Open);
                for end in endpoints.iter() {
                    let context = format!("{} line fragment {fragment_index}", case.name);
                    assert!(
                        branch.range().contains(end.carrier_parameter()),
                        "{context}"
                    );
                    let trims = end.trims().iter().flatten().collect::<Vec<_>>();
                    assert_eq!(trims.len(), 1, "{context}");
                    let trim = trims[0];
                    let carrier = trim.carrier_parameter();
                    assert!(
                        carrier.lo().is_finite()
                            && carrier.hi().is_finite()
                            && carrier.lo() < carrier.hi()
                            && carrier.lo() <= end.carrier_parameter()
                            && end.carrier_parameter() <= carrier.hi(),
                        "{context}"
                    );
                    assert_trim_provenance(
                        part,
                        graph,
                        branch,
                        end.endpoint(),
                        end.point(),
                        trim.operand(),
                        trim.face(),
                        trim.loop_id(),
                        trim.fin(),
                        trim.source_parameter(),
                        trim.edge_parameter(),
                        block_slot,
                        block_edges,
                        frame,
                        &context,
                    );
                    occurrences.push(EndpointOccurrence {
                        endpoint: end.endpoint(),
                        kind: FragmentKind::Line,
                        point: end.point(),
                        source_parameter: trim.source_parameter().clone(),
                        edge_parameter: trim.edge_parameter(),
                    });
                }
            }
            SectionCurveFragmentSpan::Whole => {
                panic!("{}: finite fixture published a whole fragment", case.name)
            }
            _ => panic!(
                "{}: finite fixture published an unknown fragment",
                case.name
            ),
        }
    }
    occurrences
}

fn assert_components(graph: &kernel::BodySectionGraph, case: MixedCycleCase) {
    assert_eq!(graph.curve_components().len(), 2, "{}", case.name);
    let mut uses = vec![0usize; graph.curve_fragments().len()];
    for component in graph.curve_components() {
        assert!(component.closed(), "{}", case.name);
        assert_eq!(component.fragments().len(), 4, "{}", case.name);
        let mut kinds = Vec::with_capacity(component.fragments().len());
        for &fragment_index in component.fragments() {
            let fragment = graph
                .curve_fragments()
                .get(fragment_index)
                .unwrap_or_else(|| panic!("{}: unknown component fragment", case.name));
            uses[fragment_index] += 1;
            kinds.push(fragment_kind(fragment));
        }
        assert_eq!(
            kinds
                .iter()
                .filter(|&&kind| kind == FragmentKind::Arc)
                .count(),
            2,
            "{}",
            case.name
        );
        assert_eq!(
            kinds
                .iter()
                .filter(|&&kind| kind == FragmentKind::Line)
                .count(),
            2,
            "{}",
            case.name
        );
        for offset in 0..component.fragments().len() {
            let current = component.fragments()[offset];
            let next = component.fragments()[(offset + 1) % component.fragments().len()];
            assert_ne!(
                fragment_kind(&graph.curve_fragments()[current]),
                fragment_kind(&graph.curve_fragments()[next]),
                "{}",
                case.name
            );
            assert_eq!(
                fragment_endpoints(&graph.curve_fragments()[current])[1],
                fragment_endpoints(&graph.curve_fragments()[next])[0],
                "{}: component traversal is not a directed exact-endpoint cycle",
                case.name
            );
        }
    }
    assert_eq!(
        uses,
        vec![1; graph.curve_fragments().len()],
        "{}",
        case.name
    );
}

fn assert_shared_root_identity(
    graph: &kernel::BodySectionGraph,
    occurrences: &[EndpointOccurrence],
    case: MixedCycleCase,
) {
    assert_eq!(occurrences.len(), 16, "{}", case.name);
    let mut endpoint_keys = Vec::new();
    for endpoint in 0..graph.curve_endpoints().len() {
        let at_endpoint = occurrences
            .iter()
            .filter(|occurrence| occurrence.endpoint == endpoint)
            .collect::<Vec<_>>();
        assert_eq!(at_endpoint.len(), 2, "{} endpoint {endpoint}", case.name);
        assert_ne!(at_endpoint[0].kind, at_endpoint[1].kind, "{}", case.name);
        assert_eq!(
            at_endpoint[0].source_parameter, at_endpoint[1].source_parameter,
            "{}: arc/ruling join did not share exact root identity",
            case.name
        );
        assert!(
            at_endpoint[0].point.dist(at_endpoint[1].point) <= GEOMETRY_TOLERANCE,
            "{}",
            case.name
        );
        let common = graph.curve_endpoints()[endpoint]
            .edge_parameters()
            .iter()
            .flatten()
            .next()
            .expect("mixed endpoint lost its common parameter evidence");
        for occurrence in &at_endpoint {
            assert!(
                common.lo() >= occurrence.edge_parameter.lo()
                    && common.hi() <= occurrence.edge_parameter.hi(),
                "{}",
                case.name
            );
        }
        endpoint_keys.push(at_endpoint[0].source_parameter.clone());
    }
    assert_eq!(endpoint_keys.len(), 8, "{}", case.name);
    for (index, key) in endpoint_keys.iter().enumerate() {
        assert!(
            !endpoint_keys[..index].contains(key),
            "{}: distinct physical endpoints reused a root key",
            case.name
        );
    }

    let mut source_edges = Vec::new();
    for key in &endpoint_keys {
        if !source_edges.contains(&key.edge()) {
            source_edges.push(key.edge());
        }
    }
    assert_eq!(source_edges.len(), 4, "{}", case.name);
    for edge in source_edges {
        let mut ordinals = endpoint_keys
            .iter()
            .filter(|key| key.edge() == edge)
            .map(SectionSourceParameterKey::root_ordinal)
            .collect::<Vec<_>>();
        ordinals.sort_unstable();
        assert_eq!(ordinals, vec![0, 1], "{}", case.name);
    }
}

fn assert_graph_contract(
    fixture: &MixedCycleFixture,
    graph: &kernel::BodySectionGraph,
    body_a: &BodyId,
    body_b: &BodyId,
    case: MixedCycleCase,
) {
    assert_eq!(
        graph.bodies(),
        &[body_a.clone(), body_b.clone()],
        "{}",
        case.name
    );
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{}: {:?}",
        case.name,
        graph.gaps()
    );
    assert!(graph.gaps().is_empty(), "{}: {:?}", case.name, graph.gaps());
    assert!(graph.edges().is_empty(), "{}", case.name);
    assert!(graph.vertices().is_empty(), "{}", case.name);
    assert!(graph.loops().is_empty(), "{}", case.name);
    assert!(graph.rings().is_empty(), "{}", case.name);
    assert_eq!(graph.branches().len(), 6, "{}", case.name);
    assert_eq!(graph.curve_fragments().len(), 8, "{}", case.name);
    assert_eq!(graph.curve_endpoints().len(), 8, "{}", case.name);
    let [SectionPeriodicFaceEmbeddingEvidence::Certified(periodic)] =
        graph.periodic_face_embeddings()
    else {
        panic!(
            "{}: cylinder-side component embedding was not certified: {:?}",
            case.name,
            graph.periodic_face_embeddings()
        );
    };
    assert_eq!(periodic.source_loop_windings(), &[1, -1], "{}", case.name);
    assert_eq!(periodic.components().len(), 2, "{}", case.name);
    assert!(
        periodic.components().iter().all(|component| {
            component.fragments().len() == 4
                && component.winding() == 0
                && component.parent().is_none()
        }),
        "{}",
        case.name
    );
    for component in periodic.components() {
        for embedded in component.fragments() {
            let fragment = &graph.curve_fragments()[embedded.fragment()];
            let endpoint_ids = match fragment.span() {
                SectionCurveFragmentSpan::Arc { endpoints, .. } => {
                    endpoints.each_ref().map(|end| end.endpoint())
                }
                SectionCurveFragmentSpan::LineSegment { endpoints } => {
                    endpoints.each_ref().map(|end| end.endpoint())
                }
                SectionCurveFragmentSpan::Whole => {
                    panic!("{}: periodic embedding retained a whole carrier", case.name)
                }
                _ => panic!(
                    "{}: periodic embedding retained an unknown fragment",
                    case.name
                ),
            };
            for (end, endpoint_id) in endpoint_ids.iter().enumerate() {
                let scalar = &embedded.trim_scalars()[end];
                let interval = scalar.carrier_interval();
                assert_eq!(scalar.endpoint(), *endpoint_id, "{}", case.name);
                assert!(
                    scalar.carrier_parameter().is_finite()
                        && interval.lo() <= scalar.carrier_parameter()
                        && scalar.carrier_parameter() <= interval.hi(),
                    "{}",
                    case.name
                );
                assert!(
                    [scalar.point().x, scalar.point().y, scalar.point().z]
                        .into_iter()
                        .all(f64::is_finite),
                    "{}",
                    case.name
                );
                for axis in 0..2 {
                    assert!(
                        embedded.endpoints()[end][axis].lo() <= scalar.lifted_uv()[axis].lo()
                            && scalar.lifted_uv()[axis].hi()
                                <= embedded.endpoints()[end][axis].hi(),
                        "{}",
                        case.name
                    );
                }
            }
        }
    }
    assert_eq!(
        graph
            .curve_fragments()
            .iter()
            .filter(|fragment| fragment_kind(fragment) == FragmentKind::Arc)
            .count(),
        4,
        "{}",
        case.name
    );
    assert_eq!(
        graph
            .curve_fragments()
            .iter()
            .filter(|fragment| fragment_kind(fragment) == FragmentKind::Line)
            .count(),
        4,
        "{}",
        case.name
    );

    let block_slot = usize::from(case.swapped);
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    let operand_faces = [
        part.body(body_a.clone())
            .unwrap()
            .faces()
            .unwrap()
            .collect::<Vec<_>>(),
        part.body(body_b.clone())
            .unwrap()
            .faces()
            .unwrap()
            .collect::<Vec<_>>(),
    ];
    for branch in graph.branches() {
        assert!(
            operand_faces[0].contains(&branch.faces()[0]),
            "{}",
            case.name
        );
        assert!(
            operand_faces[1].contains(&branch.faces()[1]),
            "{}",
            case.name
        );
    }
    assert_components(graph, case);
    let occurrences = collect_endpoint_occurrences(
        &part,
        graph,
        block_slot,
        &fixture.before.block.edges,
        fixture.frame,
        case,
    );
    assert_shared_root_identity(graph, &occurrences, case);
}

#[test]
fn facade_exposes_deterministic_closed_mixed_cycles_in_both_operand_orders() {
    // x = +/-1 cuts x^2 + y^2 = (3/2)^2 at y = +/-sqrt(5)/2. The
    // four rulings and four cap arcs therefore form two alternating cycles.
    for case in CASES {
        let fixture = mixed_cycle_fixture(case);
        let (body_a, body_b) = if case.swapped {
            (fixture.cylinder.clone(), fixture.block.clone())
        } else {
            (fixture.block.clone(), fixture.cylinder.clone())
        };
        let request = || {
            fixture
                .session
                .part(fixture.part_id.clone())
                .unwrap()
                .section_bodies(SectionBodiesRequest::new(body_a.clone(), body_b.clone()))
                .unwrap()
                .into_result()
                .unwrap()
        };
        let graph = request();
        let repeated = request();
        assert_eq!(
            repeated, graph,
            "{}: repeated public query changed its exact payload",
            case.name
        );
        assert_graph_contract(&fixture, &graph, &body_a, &body_b, case);
        assert_eq!(
            source_signature(
                &fixture.session,
                &fixture.part_id,
                &fixture.block,
                &fixture.cylinder,
            ),
            fixture.before,
            "{}: read-only section query mutated its sources",
            case.name
        );
    }
}

#[test]
fn convex_one_loop_mixed_cycles_commit_full_valid_bounded_arc_results() {
    // A convex slab with x extent 2 and y extent 6 cuts the radius-3/2
    // cylinder at x = +/-1. Its two cap faces contribute four bounded arcs
    // and its two x faces contribute four rulings. Unlike the profile fixture
    // above, every planar source face has exactly one loop. World, translated,
    // and axis-permuted cases therefore reach the boundary-arrangement seam.
    // Full cylinder validity is invariant under all four placements, including
    // the all-nonzero oblique frame.
    for placement in [
        Placement::World,
        Placement::Translated,
        Placement::AxisPermuted,
        Placement::Oblique,
    ] {
        for swapped in [false, true] {
            let MixedCycleFixture {
                mut session,
                part_id,
                block,
                cylinder,
                frame: _,
                before,
            } = convex_mixed_cycle_fixture(placement);
            let cylinder_check = session
                .part(part_id.clone())
                .unwrap()
                .check_body(CheckBodyRequest::new(cylinder.clone(), CheckLevel::Full))
                .unwrap()
                .into_result()
                .unwrap();
            assert_eq!(
                cylinder_check.outcome(),
                CheckOutcome::Valid,
                "{placement:?}: {cylinder_check:?}"
            );

            let (left, right) = if swapped {
                (cylinder.clone(), block.clone())
            } else {
                (block.clone(), cylinder.clone())
            };
            let graph = session
                .part(part_id.clone())
                .unwrap()
                .section_bodies(SectionBodiesRequest::new(left.clone(), right.clone()))
                .unwrap()
                .into_result()
                .unwrap();
            assert_eq!(
                graph.completion(),
                SectionCompletion::Complete,
                "{placement:?} swapped={swapped}: {:?}",
                graph.gaps()
            );
            assert!(graph.gaps().is_empty());
            assert_eq!(graph.branches().len(), 6);
            assert_eq!(graph.curve_fragments().len(), 8);
            assert_eq!(graph.curve_endpoints().len(), 8);
            assert_eq!(graph.curve_components().len(), 2);
            assert!(
                graph
                    .curve_components()
                    .iter()
                    .all(SectionCurveComponent::closed)
            );

            let outcome = session
                .edit_part(part_id.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Intersect,
                    left,
                    right,
                ))
                .unwrap()
                .into_result()
                .unwrap();
            let created = match outcome {
                BooleanOutcome::Success(BooleanResult::Created(created)) => created,
                other => panic!("{placement:?} swapped={swapped}: {other:?}"),
            };
            assert_eq!(created.bodies().len(), 1);
            assert_eq!(created.reports().len(), 1);
            assert_eq!(
                created.reports()[0].report().outcome(),
                CheckOutcome::Valid,
                "{placement:?} swapped={swapped}: {:?}",
                created.reports()[0]
            );
            let result = created.bodies()[0].clone();
            let result_signature =
                body_signature(&session.part(part_id.clone()).unwrap(), result.clone());
            assert_eq!(result_signature.faces.len(), 6);
            assert_eq!(result_signature.edges.len(), 12);
            assert_eq!(result_signature.vertices.len(), 8);
            let repeated = session
                .part(part_id.clone())
                .unwrap()
                .check_body(CheckBodyRequest::new(result, CheckLevel::Full))
                .unwrap()
                .into_result()
                .unwrap();
            assert_eq!(
                repeated.outcome(),
                CheckOutcome::Valid,
                "{placement:?} swapped={swapped}: {repeated:?}"
            );
            let _xt = assert_mixed_deterministic_xt_and_fast_self_import(
                &mut session,
                &part_id,
                &created.bodies()[0],
            );

            let after = source_signature(&session, &part_id, &block, &cylinder);
            assert_eq!(after.block, before.block, "source block was mutated");
            assert_eq!(
                after.cylinder, before.cylinder,
                "source cylinder was mutated"
            );
            assert_eq!(after.body_count, before.body_count + 1);
            assert!(
                after
                    .geometry_counts
                    .iter()
                    .zip(before.geometry_counts)
                    .all(|(after, before)| *after >= before),
                "result realization unexpectedly removed source geometry"
            );
        }
    }
}

#[test]
fn cap_retaining_mixed_union_and_cylinder_subtract_commit_full_valid() {
    let strip_half_width = 1.0;
    let strip_area = 2.0
        * (strip_half_width * (RADIUS * RADIUS - strip_half_width * strip_half_width).sqrt()
            + RADIUS * RADIUS * STRIP_HALF_ANGLE);
    let intersection_volume = strip_area;
    let cylinder_volume = core::f64::consts::PI * RADIUS * RADIUS * CYLINDER_HEIGHT;
    let block_volume = 2.0 * 5.0;

    for placement in [
        Placement::World,
        Placement::Translated,
        Placement::AxisPermuted,
        Placement::Oblique,
    ] {
        for operation in [BooleanOperation::Unite, BooleanOperation::Subtract] {
            let MixedCycleFixture {
                mut session,
                part_id,
                block,
                cylinder,
                frame: _,
                before,
            } = cap_retaining_fixture(placement);
            let (left, right, expected_topology, expected_volume) = match operation {
                BooleanOperation::Unite => (
                    block.clone(),
                    cylinder.clone(),
                    (13, 26, 16),
                    block_volume + cylinder_volume - intersection_volume,
                ),
                BooleanOperation::Subtract => (
                    cylinder.clone(),
                    block.clone(),
                    (7, 14, 8),
                    cylinder_volume - intersection_volume,
                ),
                _ => unreachable!(),
            };
            let outcome = session
                .edit_part(part_id.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(operation, left, right))
                .unwrap()
                .into_result()
                .unwrap();
            let BooleanOutcome::Success(BooleanResult::Created(created)) = outcome else {
                panic!("cap-retaining {placement:?} {operation:?} did not commit: {outcome:?}")
            };
            assert_eq!(created.bodies().len(), 1, "{placement:?} {operation:?}");
            assert_eq!(created.reports().len(), 1, "{placement:?} {operation:?}");
            assert_eq!(
                created.reports()[0].report().outcome(),
                CheckOutcome::Valid,
                "{placement:?} {operation:?}"
            );

            let result = created.bodies()[0].clone();
            let signature = body_signature(&session.part(part_id.clone()).unwrap(), result.clone());
            assert_eq!(
                (
                    signature.faces.len(),
                    signature.edges.len(),
                    signature.vertices.len(),
                ),
                expected_topology,
                "{placement:?} {operation:?}"
            );
            let full = session
                .part(part_id.clone())
                .unwrap()
                .check_body(CheckBodyRequest::new(result.clone(), CheckLevel::Full))
                .unwrap()
                .into_result()
                .unwrap();
            assert_eq!(
                full.outcome(),
                CheckOutcome::Valid,
                "{placement:?} {operation:?}: {full:?}"
            );
            let mesh = session
                .part(part_id.clone())
                .unwrap()
                .tessellate_body(TessellateBodyRequest::new(
                    result.clone(),
                    TessOptions {
                        chord_tol: 1.0e-3,
                        max_edge_len: None,
                    },
                ))
                .unwrap()
                .into_result()
                .unwrap();
            let actual_volume = mesh_volume(mesh.positions(), mesh.triangles());
            assert!(
                (actual_volume - expected_volume).abs() <= expected_volume * 6.0e-4,
                "{placement:?} {operation:?}: mesh volume {actual_volume:.17e} differs from independent analytic value {expected_volume:.17e}"
            );
            let _xt =
                assert_mixed_deterministic_xt_and_fast_self_import(&mut session, &part_id, &result);

            let after = source_signature(&session, &part_id, &block, &cylinder);
            assert_eq!(after.block, before.block, "source block was mutated");
            assert_eq!(
                after.cylinder, before.cylinder,
                "source cylinder was mutated"
            );
            assert_eq!(after.body_count, before.body_count + 1);
        }
    }
}

#[test]
fn cap_retaining_mixed_realization_budget_is_exact_and_denial_is_failure_atomic() {
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

    for operation in [BooleanOperation::Unite, BooleanOperation::Subtract] {
        let operands = |fixture: &MixedCycleFixture| match operation {
            BooleanOperation::Unite => (fixture.block.clone(), fixture.cylinder.clone()),
            BooleanOperation::Subtract => (fixture.cylinder.clone(), fixture.block.clone()),
            _ => unreachable!(),
        };

        let mut baseline = cap_retaining_fixture(Placement::World);
        let (left, right) = operands(&baseline);
        let baseline_result = baseline
            .session
            .edit_part(baseline.part_id.clone())
            .unwrap()
            .boolean_bodies(BooleanBodiesRequest::new(operation, left, right))
            .unwrap();
        assert!(matches!(
            baseline_result.result().unwrap(),
            BooleanOutcome::Success(BooleanResult::Created(_))
        ));
        let usage = *baseline_result
            .report()
            .usage()
            .iter()
            .find(|usage| {
                usage.stage == BOOLEAN_POST_SELECTION_WORK && usage.resource == ResourceKind::Work
            })
            .unwrap();
        assert!(usage.consumed > 0);

        let mut admitted = cap_retaining_fixture(Placement::World);
        let (left, right) = operands(&admitted);
        let admitted_result = admitted
            .session
            .edit_part(admitted.part_id.clone())
            .unwrap()
            .boolean_bodies(
                BooleanBodiesRequest::new(operation, left, right)
                    .with_settings(settings_at(usage.consumed)),
            )
            .unwrap();
        assert!(matches!(
            admitted_result.into_result().unwrap(),
            BooleanOutcome::Success(BooleanResult::Created(_))
        ));

        let mut denied = cap_retaining_fixture(Placement::World);
        let denied_before = denied.before.clone();
        let (left, right) = operands(&denied);
        let denied_result = denied
            .session
            .edit_part(denied.part_id.clone())
            .unwrap()
            .boolean_bodies(
                BooleanBodiesRequest::new(operation, left, right)
                    .with_settings(settings_at(usage.consumed - 1)),
            )
            .unwrap();
        let expected = kernel::LimitSnapshot {
            allowed: usage.consumed - 1,
            ..usage
        };
        assert_eq!(denied_result.result().unwrap_err().limit(), Some(expected));
        assert_eq!(denied_result.report().limit_events(), &[expected]);
        assert_eq!(
            source_signature(
                &denied.session,
                &denied.part_id,
                &denied.block,
                &denied.cylinder,
            ),
            denied_before,
            "{operation:?} N-1 denial mutated a source or allocated a result"
        );
    }
}

#[test]
fn convex_three_patch_mixed_intersection_is_full_valid_and_deterministic() {
    for placement in [
        Placement::World,
        Placement::Translated,
        Placement::AxisPermuted,
        Placement::Oblique,
    ] {
        for swapped in [false, true] {
            let MixedCycleFixture {
                mut session,
                part_id,
                block,
                cylinder,
                frame: _,
                before,
            } = convex_three_patch_fixture(placement);
            let (left, right) = if swapped {
                (cylinder.clone(), block.clone())
            } else {
                (block.clone(), cylinder.clone())
            };
            let outcome = session
                .edit_part(part_id.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Intersect,
                    left,
                    right,
                ))
                .unwrap()
                .into_result()
                .unwrap();
            let created = match outcome {
                BooleanOutcome::Success(BooleanResult::Created(created)) => created,
                other => panic!("{placement:?} swapped={swapped}: {other:?}"),
            };
            assert_eq!(created.bodies().len(), 1);
            assert_eq!(created.reports().len(), 1);
            assert_eq!(
                created.reports()[0].report().outcome(),
                CheckOutcome::Valid,
                "{placement:?} swapped={swapped}: {:?}",
                created.reports()[0]
            );
            let result = created.bodies()[0].clone();
            let signature = body_signature(&session.part(part_id.clone()).unwrap(), result.clone());
            assert_eq!(signature.faces.len(), 8);
            assert_eq!(signature.edges.len(), 18);
            assert_eq!(signature.vertices.len(), 12);
            let full = session
                .part(part_id.clone())
                .unwrap()
                .check_body(CheckBodyRequest::new(result, CheckLevel::Full))
                .unwrap()
                .into_result()
                .unwrap();
            assert_eq!(
                full.outcome(),
                CheckOutcome::Valid,
                "{placement:?} swapped={swapped}: {full:?}"
            );
            let _xt = assert_mixed_deterministic_xt_and_fast_self_import(
                &mut session,
                &part_id,
                &created.bodies()[0],
            );

            let after = source_signature(&session, &part_id, &block, &cylinder);
            assert_eq!(after.block, before.block, "source prism was mutated");
            assert_eq!(
                after.cylinder, before.cylinder,
                "source cylinder was mutated"
            );
            assert_eq!(after.body_count, before.body_count + 1);
        }
    }
}

#[test]
fn convex_five_patch_mixed_intersection_is_full_valid_and_deterministic() {
    // Independently evaluated circle/polygon area for the literal profile in
    // `convex_five_patch_fixture`; the extrusion height is exactly one.
    const EXPECTED_VOLUME: f64 = 6.014_725_024_492_857;
    for placement in [
        Placement::World,
        Placement::Translated,
        Placement::AxisPermuted,
        Placement::Oblique,
    ] {
        for swapped in [false, true] {
            let MixedCycleFixture {
                mut session,
                part_id,
                block,
                cylinder,
                frame: _,
                before,
            } = convex_five_patch_fixture(placement);
            let (left, right) = if swapped {
                (cylinder.clone(), block.clone())
            } else {
                (block.clone(), cylinder.clone())
            };
            let outcome = session
                .edit_part(part_id.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(
                    BooleanOperation::Intersect,
                    left,
                    right,
                ))
                .unwrap()
                .into_result()
                .unwrap();
            let BooleanOutcome::Success(BooleanResult::Created(created)) = outcome else {
                panic!("five-patch {placement:?} swapped={swapped} did not commit: {outcome:?}")
            };
            assert_eq!(created.bodies().len(), 1);
            assert_eq!(created.reports().len(), 1);
            assert_eq!(created.reports()[0].report().outcome(), CheckOutcome::Valid);
            let result = created.bodies()[0].clone();
            let signature = body_signature(&session.part(part_id.clone()).unwrap(), result.clone());
            assert_eq!(
                (
                    signature.faces.len(),
                    signature.edges.len(),
                    signature.vertices.len(),
                ),
                (12, 30, 20),
                "{placement:?} swapped={swapped}",
            );
            let full = session
                .part(part_id.clone())
                .unwrap()
                .check_body(CheckBodyRequest::new(result.clone(), CheckLevel::Full))
                .unwrap()
                .into_result()
                .unwrap();
            assert_eq!(full.outcome(), CheckOutcome::Valid, "{full:?}");
            let mesh = session
                .part(part_id.clone())
                .unwrap()
                .tessellate_body(TessellateBodyRequest::new(
                    result.clone(),
                    TessOptions {
                        chord_tol: 1.0e-3,
                        max_edge_len: None,
                    },
                ))
                .unwrap()
                .into_result()
                .unwrap();
            let actual_volume = mesh_volume(mesh.positions(), mesh.triangles());
            assert!(
                (actual_volume - EXPECTED_VOLUME).abs() <= EXPECTED_VOLUME * 6.0e-4,
                "{placement:?} swapped={swapped}: mesh volume {actual_volume:.17e} differs from independent analytic value {EXPECTED_VOLUME:.17e}",
            );
            let _xt =
                assert_mixed_deterministic_xt_and_fast_self_import(&mut session, &part_id, &result);
            let after = source_signature(&session, &part_id, &block, &cylinder);
            assert_eq!(after.block, before.block, "source prism was mutated");
            assert_eq!(
                after.cylinder, before.cylinder,
                "source cylinder was mutated"
            );
            assert_eq!(after.body_count, before.body_count + 1);
        }
    }
}

#[test]
fn convex_five_patch_cap_retaining_operations_commit_under_default_policy() {
    const PROFILE_AREA: f64 = 6.086_761_704_674_135;
    const INTERSECTION_VOLUME: f64 = 6.014_725_024_492_857;
    const CYLINDER_VOLUME: f64 = 14.137_166_941_154_069;
    for placement in [
        Placement::World,
        Placement::Translated,
        Placement::AxisPermuted,
        Placement::Oblique,
    ] {
        for operation in [BooleanOperation::Unite, BooleanOperation::Subtract] {
            let MixedCycleFixture {
                mut session,
                part_id,
                block,
                cylinder,
                frame: _,
                before,
            } = convex_five_patch_fixture(placement);
            let (left, right, expected_topology, expected_volume) = match operation {
                BooleanOperation::Unite => (
                    block.clone(),
                    cylinder.clone(),
                    (23, 47, 30),
                    PROFILE_AREA + CYLINDER_VOLUME - INTERSECTION_VOLUME,
                ),
                BooleanOperation::Subtract => (
                    cylinder.clone(),
                    block.clone(),
                    (10, 32, 20),
                    CYLINDER_VOLUME - INTERSECTION_VOLUME,
                ),
                _ => unreachable!(),
            };
            let outcome = session
                .edit_part(part_id.clone())
                .unwrap()
                .boolean_bodies(BooleanBodiesRequest::new(operation, left, right))
                .unwrap()
                .into_result()
                .unwrap();
            let BooleanOutcome::Success(BooleanResult::Created(created)) = outcome else {
                panic!("five-patch {placement:?} {operation:?} did not commit: {outcome:?}")
            };
            assert_eq!(created.bodies().len(), 1);
            assert_eq!(created.reports().len(), 1);
            assert_eq!(created.reports()[0].report().outcome(), CheckOutcome::Valid);
            let result = created.bodies()[0].clone();
            let signature = body_signature(&session.part(part_id.clone()).unwrap(), result.clone());
            assert_eq!(
                (
                    signature.faces.len(),
                    signature.edges.len(),
                    signature.vertices.len(),
                ),
                expected_topology,
                "{placement:?} {operation:?}",
            );
            let full = session
                .part(part_id.clone())
                .unwrap()
                .check_body(CheckBodyRequest::new(result.clone(), CheckLevel::Full))
                .unwrap()
                .into_result()
                .unwrap();
            assert_eq!(full.outcome(), CheckOutcome::Valid, "{full:?}");
            let mesh = session
                .part(part_id.clone())
                .unwrap()
                .tessellate_body(TessellateBodyRequest::new(
                    result.clone(),
                    TessOptions {
                        chord_tol: 1.0e-3,
                        max_edge_len: None,
                    },
                ))
                .unwrap()
                .into_result()
                .unwrap();
            let actual_volume = mesh_volume(mesh.positions(), mesh.triangles());
            assert!(
                (actual_volume - expected_volume).abs() <= expected_volume * 1.0e-3,
                "{placement:?} {operation:?}: mesh volume {actual_volume:.17e} differs from independent analytic value {expected_volume:.17e}",
            );
            let _xt =
                assert_mixed_deterministic_xt_and_fast_self_import(&mut session, &part_id, &result);
            let after = source_signature(&session, &part_id, &block, &cylinder);
            assert_eq!(after.block, before.block, "source prism was mutated");
            assert_eq!(
                after.cylinder, before.cylinder,
                "source cylinder was mutated"
            );
            assert_eq!(after.body_count, before.body_count + 1);
        }
    }
}

#[test]
fn seam_crossing_five_patch_cylinder_subtract_is_full_valid_in_all_frames() {
    // Independently evaluated from each literal edge p_i -> p_{i+1} with
    // d_i = cross(p_i, p_{i+1}) / |p_{i+1} - p_i| and circular segment
    // C_i = r^2 acos(d_i/r) - d_i sqrt(r^2 - d_i^2). The overlap prism
    // height is one, so V(cylinder - profile) = pi r^2 + sum(C_i).
    const EXPECTED_VOLUME: f64 = 7.570_496_318_579_939;

    for placement in [
        Placement::World,
        Placement::Translated,
        Placement::AxisPermuted,
        Placement::Oblique,
    ] {
        let MixedCycleFixture {
            mut session,
            part_id,
            block,
            cylinder,
            frame: _,
            before,
        } = seam_crossing_five_patch_fixture(placement);
        let outcome = session
            .edit_part(part_id.clone())
            .unwrap()
            .boolean_bodies(BooleanBodiesRequest::new(
                BooleanOperation::Subtract,
                cylinder.clone(),
                block.clone(),
            ))
            .unwrap()
            .into_result()
            .unwrap();
        let BooleanOutcome::Success(BooleanResult::Created(created)) = outcome else {
            panic!("seam-crossing five-patch {placement:?} did not commit: {outcome:?}")
        };
        assert_eq!(created.bodies().len(), 1, "{placement:?}");
        assert_eq!(created.reports().len(), 1, "{placement:?}");
        assert_eq!(
            created.reports()[0].report().outcome(),
            CheckOutcome::Valid,
            "{placement:?}",
        );

        let result = created.bodies()[0].clone();
        let signature = body_signature(&session.part(part_id.clone()).unwrap(), result.clone());
        assert_eq!(
            (
                signature.faces.len(),
                signature.edges.len(),
                signature.vertices.len(),
            ),
            (10, 32, 20),
            "{placement:?}",
        );
        let full = session
            .part(part_id.clone())
            .unwrap()
            .check_body(CheckBodyRequest::new(result.clone(), CheckLevel::Full))
            .unwrap()
            .into_result()
            .unwrap();
        assert_eq!(
            full.outcome(),
            CheckOutcome::Valid,
            "{placement:?}: {full:?}"
        );

        let mesh = session
            .part(part_id.clone())
            .unwrap()
            .tessellate_body(TessellateBodyRequest::new(
                result.clone(),
                TessOptions {
                    chord_tol: 1.0e-3,
                    max_edge_len: None,
                },
            ))
            .unwrap()
            .into_result()
            .unwrap();
        let actual_volume = mesh_volume(mesh.positions(), mesh.triangles());
        assert!(
            (actual_volume - EXPECTED_VOLUME).abs() <= EXPECTED_VOLUME * 1.0e-3,
            "{placement:?}: mesh volume {actual_volume:.17e} differs from independent analytic value {EXPECTED_VOLUME:.17e}",
        );
        let _xt =
            assert_mixed_deterministic_xt_and_fast_self_import(&mut session, &part_id, &result);

        let after = source_signature(&session, &part_id, &block, &cylinder);
        assert_eq!(after.block, before.block, "source prism was mutated");
        assert_eq!(
            after.cylinder, before.cylinder,
            "source cylinder was mutated"
        );
        assert_eq!(after.body_count, before.body_count + 1);
    }
}

fn assert_nonconvex_star_intersection_target_once(
    expected_volume: f64,
) -> ([usize; 3], [usize; 3], Vec<u8>) {
    let MixedCycleFixture {
        mut session,
        part_id,
        block,
        cylinder,
        frame: _,
        before,
    } = nonconvex_star_five_patch_fixture(Placement::World);
    let request = || {
        session
            .part(part_id.clone())
            .unwrap()
            .section_bodies(SectionBodiesRequest::new(block.clone(), cylinder.clone()))
            .unwrap()
            .into_result()
            .unwrap()
    };
    let graph = request();
    assert_eq!(request(), graph, "repeated star section changed payload");
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{graph:#?}"
    );
    assert!(graph.gaps().is_empty(), "{graph:#?}");
    assert_eq!(graph.branches().len(), 12);
    assert_eq!(graph.curve_fragments().len(), 20);
    assert_eq!(graph.curve_endpoints().len(), 20);
    assert_eq!(graph.curve_components().len(), 5);
    assert_eq!(
        graph
            .curve_fragments()
            .iter()
            .filter(|fragment| fragment_kind(fragment) == FragmentKind::Arc)
            .count(),
        10,
    );
    assert_eq!(
        graph
            .curve_fragments()
            .iter()
            .filter(|fragment| fragment_kind(fragment) == FragmentKind::Line)
            .count(),
        10,
    );
    assert!(
        graph
            .curve_components()
            .iter()
            .all(|component| component.closed() && component.fragments().len() == 4)
    );
    assert_eq!(
        source_signature(&session, &part_id, &block, &cylinder),
        before,
        "read-only star section mutated a source",
    );

    let operation_outcome = session
        .edit_part(part_id.clone())
        .unwrap()
        .boolean_bodies(BooleanBodiesRequest::new(
            BooleanOperation::Intersect,
            block.clone(),
            cylinder.clone(),
        ))
        .unwrap();
    let mixed_profile_stage = kernel::StageId::new("ktopo.check.mixed-profile-prism-work").unwrap();
    assert!(
        operation_outcome
            .report()
            .usage()
            .contains(&kernel::LimitSnapshot {
                stage: mixed_profile_stage,
                resource: ResourceKind::Work,
                consumed: 2_851_200,
                allowed: 4_194_304,
            })
    );
    let outcome = operation_outcome.into_result().unwrap();
    let BooleanOutcome::Success(BooleanResult::Created(created)) = outcome else {
        panic!("non-convex star intersection did not commit: {outcome:?}")
    };
    assert_eq!(created.bodies().len(), 1);
    assert_eq!(created.reports().len(), 1);
    assert_eq!(created.reports()[0].report().level(), CheckLevel::Full);
    assert_eq!(
        created.reports()[0].report().outcome(),
        CheckOutcome::Valid,
        "{:?}",
        created.reports()[0],
    );
    let result = created.bodies()[0].clone();
    let topology = body_signature(&session.part(part_id.clone()).unwrap(), result.clone());
    assert_eq!(
        (
            topology.faces.len(),
            topology.edges.len(),
            topology.vertices.len(),
        ),
        (17, 45, 30),
    );
    let full = session
        .part(part_id.clone())
        .unwrap()
        .check_body(CheckBodyRequest::new(result.clone(), CheckLevel::Full))
        .unwrap()
        .into_result()
        .unwrap();
    assert_eq!(full.outcome(), CheckOutcome::Valid, "{full:?}");
    let mesh = session
        .part(part_id.clone())
        .unwrap()
        .tessellate_body(TessellateBodyRequest::new(
            result.clone(),
            TessOptions {
                chord_tol: 1.0e-3,
                max_edge_len: None,
            },
        ))
        .unwrap()
        .into_result()
        .unwrap();
    let actual_volume = mesh_volume(mesh.positions(), mesh.triangles());
    assert!(
        (actual_volume - expected_volume).abs() <= expected_volume * 1.0e-3,
        "mesh volume {actual_volume:.17e} differs from literal-derived value {expected_volume:.17e}",
    );
    let xt = assert_mixed_deterministic_xt_and_fast_self_import(&mut session, &part_id, &result);
    let after = source_signature(&session, &part_id, &block, &cylinder);
    assert_eq!(
        after.block, before.block,
        "committed intersection mutated the source star prism",
    );
    assert_eq!(
        after.cylinder, before.cylinder,
        "committed intersection mutated the source cylinder",
    );
    assert_eq!(after.body_count, before.body_count + 1);
    (
        [
            topology.faces.len(),
            topology.edges.len(),
            topology.vertices.len(),
        ],
        after.geometry_counts,
        xt,
    )
}

#[test]
fn nonconvex_star_section_and_intersection_commit_full_valid_deterministically() {
    const EXPECTED_VOLUME: f64 = 5.559_104_559_775_048;
    let literal_volume = literal_star_disk_intersection_area(&nonconvex_star_profile(), RADIUS)
        * (SLAB_HI - SLAB_LO);
    assert!(
        (literal_volume - EXPECTED_VOLUME).abs() <= 1.0e-12,
        "literal line/circle integration produced {literal_volume:.17e}",
    );

    let first = assert_nonconvex_star_intersection_target_once(literal_volume);
    let second = assert_nonconvex_star_intersection_target_once(literal_volume);
    assert_eq!(first.0, second.0, "rebuilt result topology counts changed");
    assert_eq!(first.1, second.1, "rebuilt result geometry counts changed");
    assert_eq!(first.2, second.2, "rebuilt result X_T changed");
}

#[test]
fn five_portal_shell_work_accepts_exact_n_and_refuses_n_minus_one_atomically() {
    let stage = kernel::StageId::new("ktopo.check.portal-cylinder-shell-work").unwrap();
    let settings_at = |allowed| {
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

    for (operation, expected_work) in [
        (BooleanOperation::Unite, 14_966_784),
        (BooleanOperation::Subtract, 1_095_237),
    ] {
        let operands = |fixture: &MixedCycleFixture| match operation {
            BooleanOperation::Unite => (fixture.block.clone(), fixture.cylinder.clone()),
            BooleanOperation::Subtract => (fixture.cylinder.clone(), fixture.block.clone()),
            _ => unreachable!(),
        };

        let mut baseline = convex_five_patch_fixture(Placement::World);
        let (left, right) = operands(&baseline);
        let baseline_result = baseline
            .session
            .edit_part(baseline.part_id.clone())
            .unwrap()
            .boolean_bodies(BooleanBodiesRequest::new(operation, left, right))
            .unwrap();
        assert!(matches!(
            baseline_result.result().unwrap(),
            BooleanOutcome::Success(BooleanResult::Created(_))
        ));
        let usage = *baseline_result
            .report()
            .usage()
            .iter()
            .find(|usage| usage.stage == stage && usage.resource == ResourceKind::Work)
            .unwrap();
        assert_eq!(usage.consumed, expected_work, "{operation:?}");

        let mut admitted = convex_five_patch_fixture(Placement::World);
        let (left, right) = operands(&admitted);
        let admitted_result = admitted
            .session
            .edit_part(admitted.part_id.clone())
            .unwrap()
            .boolean_bodies(
                BooleanBodiesRequest::new(operation, left, right)
                    .with_settings(settings_at(usage.consumed)),
            )
            .unwrap();
        assert!(matches!(
            admitted_result.into_result().unwrap(),
            BooleanOutcome::Success(BooleanResult::Created(_))
        ));

        let mut denied = convex_five_patch_fixture(Placement::World);
        let denied_before = denied.before.clone();
        let (left, right) = operands(&denied);
        let denied_result = denied
            .session
            .edit_part(denied.part_id.clone())
            .unwrap()
            .boolean_bodies(
                BooleanBodiesRequest::new(operation, left, right)
                    .with_settings(settings_at(usage.consumed - 1)),
            )
            .unwrap();
        let expected = kernel::LimitSnapshot {
            allowed: usage.consumed - 1,
            ..usage
        };
        assert_eq!(denied_result.result().unwrap_err().limit(), Some(expected));
        assert_eq!(denied_result.report().limit_events(), &[expected]);
        assert_eq!(
            source_signature(
                &denied.session,
                &denied.part_id,
                &denied.block,
                &denied.cylinder,
            ),
            denied_before,
            "{operation:?} N-1 refusal mutated a source or allocated a result",
        );
    }
}

#[test]
fn ordered_planar_minus_cylinder_commits_every_disconnected_profile_component() {
    let strip_half_width = HALF_BLOCK_X;
    let disk_inside_strip = 2.0
        * (strip_half_width * (RADIUS * RADIUS - strip_half_width * strip_half_width).sqrt()
            + RADIUS * RADIUS * STRIP_HALF_ANGLE);
    let rectangular_remainder_volume =
        (2.0 * HALF_BLOCK_X * 6.0 - disk_inside_strip) * (SLAB_HI - SLAB_LO);

    for placement in [
        Placement::World,
        Placement::Translated,
        Placement::AxisPermuted,
        Placement::Oblique,
    ] {
        assert_planar_minus_cylinder_components(
            convex_mixed_cycle_fixture(placement),
            2,
            (6, 12, 8),
            Some(rectangular_remainder_volume),
        );
        assert_planar_minus_cylinder_components(
            convex_three_patch_fixture(placement),
            3,
            (5, 9, 6),
            None,
        );
    }
}

#[test]
fn bounded_arc_realization_budget_is_exact_and_denial_is_failure_atomic() {
    let mut baseline_fixture = convex_mixed_cycle_fixture(Placement::World);
    let baseline = baseline_fixture
        .session
        .edit_part(baseline_fixture.part_id.clone())
        .unwrap()
        .boolean_bodies(BooleanBodiesRequest::new(
            BooleanOperation::Intersect,
            baseline_fixture.block.clone(),
            baseline_fixture.cylinder.clone(),
        ))
        .unwrap();
    assert!(matches!(
        baseline.result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));
    let usage = *baseline
        .report()
        .usage()
        .iter()
        .find(|usage| {
            usage.stage == BOOLEAN_POST_SELECTION_WORK && usage.resource == ResourceKind::Work
        })
        .unwrap();
    assert!(usage.consumed > 0);

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
    let mut admitted_fixture = convex_mixed_cycle_fixture(Placement::World);
    let admitted = admitted_fixture
        .session
        .edit_part(admitted_fixture.part_id.clone())
        .unwrap()
        .boolean_bodies(
            BooleanBodiesRequest::new(
                BooleanOperation::Intersect,
                admitted_fixture.block.clone(),
                admitted_fixture.cylinder.clone(),
            )
            .with_settings(settings_at(usage.consumed)),
        )
        .unwrap();
    assert!(matches!(
        admitted.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut denied_fixture = convex_mixed_cycle_fixture(Placement::World);
    let denied_before = denied_fixture.before.clone();
    let denied = denied_fixture
        .session
        .edit_part(denied_fixture.part_id.clone())
        .unwrap()
        .boolean_bodies(
            BooleanBodiesRequest::new(
                BooleanOperation::Intersect,
                denied_fixture.block.clone(),
                denied_fixture.cylinder.clone(),
            )
            .with_settings(settings_at(usage.consumed - 1)),
        )
        .unwrap();
    let expected = kernel::LimitSnapshot {
        allowed: usage.consumed - 1,
        ..usage
    };
    assert_eq!(denied.result().unwrap_err().limit(), Some(expected));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        source_signature(
            &denied_fixture.session,
            &denied_fixture.part_id,
            &denied_fixture.block,
            &denied_fixture.cylinder,
        ),
        denied_before,
        "post-selection budget denial mutated source topology or geometry"
    );
}

#[test]
fn disconnected_subtract_batch_denies_n_minus_one_before_any_component_allocates() {
    let mut baseline_fixture = convex_mixed_cycle_fixture(Placement::World);
    let baseline = baseline_fixture
        .session
        .edit_part(baseline_fixture.part_id.clone())
        .unwrap()
        .boolean_bodies(BooleanBodiesRequest::new(
            BooleanOperation::Subtract,
            baseline_fixture.block.clone(),
            baseline_fixture.cylinder.clone(),
        ))
        .unwrap();
    let BooleanOutcome::Success(BooleanResult::Created(created)) = baseline.result().unwrap()
    else {
        panic!("baseline disconnected subtraction did not commit")
    };
    assert_eq!(created.bodies().len(), 2);
    let usage = *baseline
        .report()
        .usage()
        .iter()
        .find(|usage| {
            usage.stage == BOOLEAN_POST_SELECTION_WORK && usage.resource == ResourceKind::Work
        })
        .unwrap();
    assert!(usage.consumed > 0);

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
    let mut admitted_fixture = convex_mixed_cycle_fixture(Placement::World);
    let admitted = admitted_fixture
        .session
        .edit_part(admitted_fixture.part_id.clone())
        .unwrap()
        .boolean_bodies(
            BooleanBodiesRequest::new(
                BooleanOperation::Subtract,
                admitted_fixture.block.clone(),
                admitted_fixture.cylinder.clone(),
            )
            .with_settings(settings_at(usage.consumed)),
        )
        .unwrap();
    assert!(matches!(
        admitted.into_result().unwrap(),
        BooleanOutcome::Success(BooleanResult::Created(_))
    ));

    let mut denied_fixture = convex_mixed_cycle_fixture(Placement::World);
    let denied_before = denied_fixture.before.clone();
    let denied = denied_fixture
        .session
        .edit_part(denied_fixture.part_id.clone())
        .unwrap()
        .boolean_bodies(
            BooleanBodiesRequest::new(
                BooleanOperation::Subtract,
                denied_fixture.block.clone(),
                denied_fixture.cylinder.clone(),
            )
            .with_settings(settings_at(usage.consumed - 1)),
        )
        .unwrap();
    let expected = kernel::LimitSnapshot {
        allowed: usage.consumed - 1,
        ..usage
    };
    assert_eq!(denied.result().unwrap_err().limit(), Some(expected));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        source_signature(
            &denied_fixture.session,
            &denied_fixture.part_id,
            &denied_fixture.block,
            &denied_fixture.cylinder,
        ),
        denied_before,
        "N-1 disconnected batch denial allocated a partial component"
    );
}
