//! Generate deterministic public-facade X_T Boolean oracle artifacts.

use std::error::Error as StdError;
use std::ffi::OsStr;
use std::fmt::Write as _;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

use kernel::{
    BlockRequest, BodyId, BodyKind, BooleanBodiesRequest, BooleanOperation, BooleanOutcome,
    BooleanResult, CheckBodyRequest, CheckLevel, CheckOutcome, CylinderRequest, ExportXtRequest,
    ExtrudeProfileRequest, Frame, ImportXtRequest, Kernel, PartId, Point2 as ProfilePoint2, Point3,
    Session, TessOptions, TessellateBodyRequest, Vec3,
};

type OracleResult<T> = Result<T, Box<dyn StdError>>;

const CONNECTED_UNITE: &str = "connected_unite.x_t";
const CONNECTED_SUBTRACT: &str = "connected_subtract.x_t";
const CONNECTED_INTERSECT: &str = "connected_intersect.x_t";
const DISJOINT_BODY_0: &str = "disjoint_unite_body_0.x_t";
const DISJOINT_BODY_1: &str = "disjoint_unite_body_1.x_t";
const CONTAINED_CAVITY: &str = "contained_subtract_cavity.x_t";
const BOUNDED_ARC_INTERSECT: &str = "bounded_arc_plane_cylinder_intersect.x_t";
const BOUNDED_ARC_SUBTRACT_BODY_0: &str = "bounded_arc_plane_cylinder_subtract_body_0.x_t";
const BOUNDED_ARC_SUBTRACT_BODY_1: &str = "bounded_arc_plane_cylinder_subtract_body_1.x_t";
const CAP_RETAINING_UNITE: &str = "cap_retaining_plane_cylinder_unite.x_t";
const CAP_RETAINING_CYLINDER_SUBTRACT: &str = "cap_retaining_cylinder_minus_plane.x_t";
const FIVE_PORTAL_UNITE: &str = "five_portal_plane_cylinder_unite.x_t";
const FIVE_PORTAL_CYLINDER_SUBTRACT: &str = "five_portal_cylinder_minus_plane.x_t";
const EXPECTED_FILES: [&str; 13] = [
    CONNECTED_UNITE,
    CONNECTED_SUBTRACT,
    CONNECTED_INTERSECT,
    DISJOINT_BODY_0,
    DISJOINT_BODY_1,
    CONTAINED_CAVITY,
    BOUNDED_ARC_INTERSECT,
    BOUNDED_ARC_SUBTRACT_BODY_0,
    BOUNDED_ARC_SUBTRACT_BODY_1,
    CAP_RETAINING_UNITE,
    CAP_RETAINING_CYLINDER_SUBTRACT,
    FIVE_PORTAL_UNITE,
    FIVE_PORTAL_CYLINDER_SUBTRACT,
];

const BOUNDED_ARC_RADIUS: f64 = 1.5;
const BOUNDED_ARC_HALF_STRIP_WIDTH: f64 = 1.0;
const BOUNDED_ARC_STRIP_HALF_LENGTH: f64 = 3.0;
const BOUNDED_ARC_SLAB_LO: f64 = 0.5;
const BOUNDED_ARC_SLAB_HI: f64 = 1.5;
const BOUNDED_ARC_CYLINDER_HEIGHT: f64 = 2.0;
/// Independently evaluated `asin(BOUNDED_ARC_HALF_STRIP_WIDTH / BOUNDED_ARC_RADIUS)` reference.
const BOUNDED_ARC_STRIP_HALF_ANGLE: f64 = 0.729_727_656_226_966_3;
const BOUNDED_ARC_RELATIVE_VOLUME_TOLERANCE: f64 = 5.0e-4;
const BOUNDED_ARC_CHORD_TOLERANCE: f64 = 1.0e-3;
const CAP_RETAINING_RELATIVE_VOLUME_TOLERANCE: f64 = 6.0e-4;
const CAP_RETAINING_CHORD_TOLERANCE: f64 = 1.0e-3;
/// Shoelace area of the exact nine-decimal profile literals below.
const FIVE_PORTAL_PROFILE_AREA: f64 = 6.086_761_704_674_135;
/// Independently evaluated disk/profile intersection area for those literals.
const FIVE_PORTAL_INTERSECTION_VOLUME: f64 = 6.014_725_024_492_857;
const FIVE_PORTAL_RELATIVE_VOLUME_TOLERANCE: f64 = 1.0e-3;

const SIN_047: f64 = 0.452_886_285_379_068_3;
const COS_047: f64 = 0.891_568_288_195_329;

#[derive(Debug, Clone, Copy)]
struct BlockSpec {
    center: [f64; 3],
    z: [f64; 3],
    x: [f64; 3],
    extents: [f64; 3],
}

impl BlockSpec {
    fn frame(self) -> OracleResult<Frame> {
        Ok(Frame::new(
            Point3::from_array(self.center),
            Vec3::from_array(self.z),
            Vec3::from_array(self.x),
        )?)
    }

    fn volume(self) -> f64 {
        self.extents.into_iter().product()
    }
}

const CONNECTED_LEFT: BlockSpec = BlockSpec {
    center: [-0.3, 0.2, -0.1],
    z: [0.0, 0.0, 1.0],
    x: [1.0, 0.0, 0.0],
    extents: [4.0, 3.5, 3.0],
};
const CONNECTED_RIGHT: BlockSpec = BlockSpec {
    center: [0.5, 0.1, 0.25],
    z: [0.0, 0.0, 1.0],
    x: [COS_047, SIN_047, 0.0],
    extents: [3.25, 2.75, 3.5],
};
const DISJOINT_LEFT: BlockSpec = BlockSpec {
    center: [-8.0, 1.0, 0.5],
    z: [0.0, 0.0, 1.0],
    x: [1.0, 0.0, 0.0],
    extents: [2.0, 1.5, 2.5],
};
const DISJOINT_RIGHT: BlockSpec = BlockSpec {
    center: [8.0, -1.0, -0.5],
    z: [0.0, 1.0, 0.0],
    x: [1.0, 0.0, 0.0],
    extents: [1.75, 2.25, 1.25],
};
const CONTAINED_OUTER: BlockSpec = BlockSpec {
    center: [0.0, 0.0, 0.0],
    z: [0.0, 0.0, 1.0],
    x: [1.0, 0.0, 0.0],
    extents: [6.0, 5.0, 4.0],
};
const CONTAINED_INNER: BlockSpec = BlockSpec {
    center: [0.25, -0.2, 0.1],
    z: [0.0, 0.0, 1.0],
    x: [1.0, 0.0, 0.0],
    extents: [1.5, 1.0, 0.8],
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct TopologyCounts {
    regions: usize,
    shells: usize,
    faces: usize,
    loops: usize,
    fins: usize,
    edges: usize,
    vertices: usize,
}

const DISJOINT_LEFT_COUNTS: TopologyCounts = TopologyCounts {
    regions: 2,
    shells: 1,
    faces: 10,
    loops: 10,
    fins: 40,
    edges: 20,
    vertices: 12,
};
const DISJOINT_RIGHT_COUNTS: TopologyCounts = DISJOINT_LEFT_COUNTS;
const CAVITY_COUNTS: TopologyCounts = TopologyCounts {
    regions: 3,
    shells: 2,
    faces: 60,
    loops: 60,
    fins: 240,
    edges: 120,
    vertices: 64,
};

const CONNECTED_UNITE_COUNTS: TopologyCounts = TopologyCounts {
    regions: 2,
    shells: 1,
    faces: 55,
    loops: 55,
    fins: 214,
    edges: 107,
    vertices: 54,
};
const CONNECTED_SUBTRACT_COUNTS: TopologyCounts = TopologyCounts {
    regions: 2,
    shells: 1,
    faces: 37,
    loops: 37,
    fins: 148,
    edges: 74,
    vertices: 39,
};
const CONNECTED_INTERSECT_COUNTS: TopologyCounts = TopologyCounts {
    regions: 2,
    shells: 1,
    faces: 9,
    loops: 9,
    fins: 42,
    edges: 21,
    vertices: 14,
};
const BOUNDED_ARC_INTERSECT_COUNTS: TopologyCounts = TopologyCounts {
    regions: 2,
    shells: 1,
    faces: 6,
    loops: 6,
    fins: 24,
    edges: 12,
    vertices: 8,
};
const BOUNDED_ARC_SUBTRACT_COMPONENT_COUNTS: TopologyCounts = TopologyCounts {
    // The public body view includes the solid and exterior regions.
    regions: 2,
    shells: 1,
    faces: 6,
    loops: 6,
    fins: 24,
    edges: 12,
    vertices: 8,
};
const CAP_RETAINING_UNITE_COUNTS: TopologyCounts = TopologyCounts {
    regions: 2,
    shells: 1,
    faces: 13,
    loops: 16,
    fins: 52,
    edges: 26,
    vertices: 16,
};
const CAP_RETAINING_CYLINDER_SUBTRACT_COUNTS: TopologyCounts = TopologyCounts {
    regions: 2,
    shells: 1,
    faces: 7,
    loops: 10,
    fins: 28,
    edges: 14,
    vertices: 8,
};
const FIVE_PORTAL_UNITE_COUNTS: TopologyCounts = TopologyCounts {
    regions: 2,
    shells: 1,
    faces: 23,
    loops: 29,
    fins: 94,
    edges: 47,
    vertices: 30,
};
const FIVE_PORTAL_CYLINDER_SUBTRACT_COUNTS: TopologyCounts = TopologyCounts {
    regions: 2,
    shells: 1,
    faces: 10,
    loops: 16,
    fins: 64,
    edges: 32,
    vertices: 20,
};

#[derive(Debug, Clone, Copy)]
struct MeshSummary {
    positions: usize,
    triangles: usize,
    volume: f64,
}

#[derive(Debug, Clone)]
struct Artifact {
    file: &'static str,
    body_kind: &'static str,
    counts: TopologyCounts,
    mesh: MeshSummary,
    bytes: Vec<u8>,
}

#[derive(Debug, Clone, Copy)]
struct Point2 {
    x: f64,
    y: f64,
}

impl Point2 {
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }

    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }

    fn scale(self, factor: f64) -> Self {
        Self {
            x: self.x * factor,
            y: self.y * factor,
        }
    }
}

fn main() -> OracleResult<()> {
    let output = exactly_one_output_directory()?;
    prepare_output_directory(&output)?;

    let first = build_bundle()?;
    let replay = build_bundle()?;
    require(
        first.len() == EXPECTED_FILES.len(),
        "oracle bundle size changed",
    )?;
    require(first.len() == replay.len(), "replay bundle size changed")?;
    for (artifact, repeated) in first.iter().zip(&replay) {
        require(
            artifact.file == repeated.file,
            "replay artifact order changed",
        )?;
        require(
            artifact.bytes == repeated.bytes,
            format!("{} was not byte-stable on replay", artifact.file),
        )?;
    }

    for artifact in &first {
        fs::write(output.join(artifact.file), &artifact.bytes)?;
    }
    fs::write(output.join("manifest.tsv"), manifest(&first))?;
    Ok(())
}

fn exactly_one_output_directory() -> OracleResult<PathBuf> {
    let mut arguments = std::env::args_os();
    let program = arguments
        .next()
        .unwrap_or_else(|| "boolean_xt_oracle".into());
    let Some(output) = arguments.next() else {
        return Err(failure(format!(
            "usage: {} OUTPUT_DIRECTORY",
            Path::new(&program).display()
        )));
    };
    if arguments.next().is_some() {
        return Err(failure(format!(
            "usage: {} OUTPUT_DIRECTORY (exactly one output directory is required)",
            Path::new(&program).display()
        )));
    }
    Ok(PathBuf::from(output))
}

fn prepare_output_directory(output: &Path) -> OracleResult<()> {
    if output.exists() {
        require(
            output.is_dir(),
            format!("output path is not a directory: {}", output.display()),
        )?;
    } else {
        fs::create_dir_all(output)?;
    }

    for entry in fs::read_dir(output)? {
        let entry = entry?;
        let path = entry.path();
        if path.extension() != Some(OsStr::new("x_t")) {
            continue;
        }
        let name = path.file_name().and_then(OsStr::to_str);
        let category = if name.is_some_and(|name| EXPECTED_FILES.contains(&name)) {
            "stale"
        } else {
            "unexpected"
        };
        return Err(failure(format!(
            "{category} X_T entry in output directory: {}",
            path.display()
        )));
    }
    Ok(())
}

fn build_bundle() -> OracleResult<Vec<Artifact>> {
    let overlap = connected_intersection_volume();
    let mut artifacts = Vec::with_capacity(EXPECTED_FILES.len());
    artifacts.push(build_single_result(
        CONNECTED_UNITE,
        CONNECTED_LEFT,
        CONNECTED_RIGHT,
        BooleanOperation::Unite,
        CONNECTED_LEFT.volume() + CONNECTED_RIGHT.volume() - overlap,
        CONNECTED_UNITE_COUNTS,
    )?);
    artifacts.push(build_single_result(
        CONNECTED_SUBTRACT,
        CONNECTED_LEFT,
        CONNECTED_RIGHT,
        BooleanOperation::Subtract,
        CONNECTED_LEFT.volume() - overlap,
        CONNECTED_SUBTRACT_COUNTS,
    )?);
    artifacts.push(build_single_result(
        CONNECTED_INTERSECT,
        CONNECTED_LEFT,
        CONNECTED_RIGHT,
        BooleanOperation::Intersect,
        overlap,
        CONNECTED_INTERSECT_COUNTS,
    )?);
    artifacts.extend(build_disjoint_union()?);
    artifacts.push(build_single_result(
        CONTAINED_CAVITY,
        CONTAINED_OUTER,
        CONTAINED_INNER,
        BooleanOperation::Subtract,
        CONTAINED_OUTER.volume() - CONTAINED_INNER.volume(),
        CAVITY_COUNTS,
    )?);
    artifacts.push(build_bounded_arc_intersection()?);
    artifacts.extend(build_bounded_arc_subtract()?);
    artifacts.push(build_cap_retaining_result(
        CAP_RETAINING_UNITE,
        BooleanOperation::Unite,
        cap_retaining_block_volume() + cap_retaining_cylinder_volume()
            - bounded_arc_disk_strip_volume(),
        CAP_RETAINING_UNITE_COUNTS,
    )?);
    artifacts.push(build_cap_retaining_result(
        CAP_RETAINING_CYLINDER_SUBTRACT,
        BooleanOperation::Subtract,
        cap_retaining_cylinder_volume() - bounded_arc_disk_strip_volume(),
        CAP_RETAINING_CYLINDER_SUBTRACT_COUNTS,
    )?);
    artifacts.push(build_five_portal_result(
        FIVE_PORTAL_UNITE,
        BooleanOperation::Unite,
        FIVE_PORTAL_PROFILE_AREA + cap_retaining_cylinder_volume()
            - FIVE_PORTAL_INTERSECTION_VOLUME,
        FIVE_PORTAL_UNITE_COUNTS,
    )?);
    artifacts.push(build_five_portal_result(
        FIVE_PORTAL_CYLINDER_SUBTRACT,
        BooleanOperation::Subtract,
        cap_retaining_cylinder_volume() - FIVE_PORTAL_INTERSECTION_VOLUME,
        FIVE_PORTAL_CYLINDER_SUBTRACT_COUNTS,
    )?);
    Ok(artifacts)
}

fn build_bounded_arc_intersection() -> OracleResult<Artifact> {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (block, cylinder) = create_bounded_arc_operands(&mut session, &part)?;
    let bodies = run_boolean(
        &mut session,
        &part,
        block.clone(),
        cylinder.clone(),
        BooleanOperation::Intersect,
    )?;
    require(
        bodies.len() == 1,
        format!(
            "{BOUNDED_ARC_INTERSECT} expected one result body, got {}",
            bodies.len()
        ),
    )?;
    assert_sources_retained(&session, &part, &block, &cylinder, 3)?;

    make_artifact_with_volume_tolerance(
        &mut session,
        &part,
        bodies[0].clone(),
        BOUNDED_ARC_INTERSECT,
        bounded_arc_disk_strip_volume(),
        BOUNDED_ARC_INTERSECT_COUNTS,
        BOUNDED_ARC_RELATIVE_VOLUME_TOLERANCE,
        BOUNDED_ARC_CHORD_TOLERANCE,
    )
}

fn build_bounded_arc_subtract() -> OracleResult<Vec<Artifact>> {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (block, cylinder) = create_bounded_arc_operands(&mut session, &part)?;
    let bodies = run_boolean(
        &mut session,
        &part,
        block.clone(),
        cylinder.clone(),
        BooleanOperation::Subtract,
    )?;
    require(
        bodies.len() == 2,
        format!(
            "bounded-arc planar subtraction expected two result bodies, got {}",
            bodies.len()
        ),
    )?;
    assert_sources_retained(&session, &part, &block, &cylinder, 4)?;

    let expected_component_volume =
        (bounded_arc_strip_block_volume() - bounded_arc_disk_strip_volume()) * 0.5;
    let first = make_artifact_with_volume_tolerance(
        &mut session,
        &part,
        bodies[0].clone(),
        BOUNDED_ARC_SUBTRACT_BODY_0,
        expected_component_volume,
        BOUNDED_ARC_SUBTRACT_COMPONENT_COUNTS,
        BOUNDED_ARC_RELATIVE_VOLUME_TOLERANCE,
        BOUNDED_ARC_CHORD_TOLERANCE,
    )?;
    let second = make_artifact_with_volume_tolerance(
        &mut session,
        &part,
        bodies[1].clone(),
        BOUNDED_ARC_SUBTRACT_BODY_1,
        expected_component_volume,
        BOUNDED_ARC_SUBTRACT_COMPONENT_COUNTS,
        BOUNDED_ARC_RELATIVE_VOLUME_TOLERANCE,
        BOUNDED_ARC_CHORD_TOLERANCE,
    )?;
    assert_sources_retained(&session, &part, &block, &cylinder, 4)?;
    Ok(vec![first, second])
}

fn build_cap_retaining_result(
    file: &'static str,
    operation: BooleanOperation,
    expected_volume: f64,
    expected_counts: TopologyCounts,
) -> OracleResult<Artifact> {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (block, cylinder) = create_cap_retaining_operands(&mut session, &part)?;
    let (left, right) = match operation {
        BooleanOperation::Unite => (block.clone(), cylinder.clone()),
        BooleanOperation::Subtract => (cylinder.clone(), block.clone()),
        _ => return Err(failure("cap-retaining oracle operation changed")),
    };
    let bodies = run_boolean(&mut session, &part, left, right, operation)?;
    require(
        bodies.len() == 1,
        format!("{file} expected one result body, got {}", bodies.len()),
    )?;
    assert_sources_retained(&session, &part, &block, &cylinder, 3)?;
    make_artifact_with_volume_tolerance(
        &mut session,
        &part,
        bodies[0].clone(),
        file,
        expected_volume,
        expected_counts,
        CAP_RETAINING_RELATIVE_VOLUME_TOLERANCE,
        CAP_RETAINING_CHORD_TOLERANCE,
    )
}

fn build_five_portal_result(
    file: &'static str,
    operation: BooleanOperation,
    expected_volume: f64,
    expected_counts: TopologyCounts,
) -> OracleResult<Artifact> {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (block, cylinder) = create_five_portal_operands(&mut session, &part)?;
    let (left, right) = match operation {
        BooleanOperation::Unite => (block.clone(), cylinder.clone()),
        BooleanOperation::Subtract => (cylinder.clone(), block.clone()),
        _ => return Err(failure("five-portal oracle operation changed")),
    };
    let bodies = run_boolean(&mut session, &part, left, right, operation)?;
    require(
        bodies.len() == 1,
        format!("{file} expected one result body, got {}", bodies.len()),
    )?;
    assert_sources_retained(&session, &part, &block, &cylinder, 3)?;
    make_artifact_with_volume_tolerance(
        &mut session,
        &part,
        bodies[0].clone(),
        file,
        expected_volume,
        expected_counts,
        FIVE_PORTAL_RELATIVE_VOLUME_TOLERANCE,
        CAP_RETAINING_CHORD_TOLERANCE,
    )
}

fn create_bounded_arc_operands(
    session: &mut Session,
    part: &PartId,
) -> OracleResult<(BodyId, BodyId)> {
    let mut edit = session.edit_part(part.clone())?;
    let frame = Frame::world();
    let block = edit
        .extrude_profile(ExtrudeProfileRequest::new(
            frame.with_origin(frame.point_at(0.0, 0.0, BOUNDED_ARC_SLAB_LO)),
            vec![
                ProfilePoint2::new(
                    -BOUNDED_ARC_HALF_STRIP_WIDTH,
                    -BOUNDED_ARC_STRIP_HALF_LENGTH,
                ),
                ProfilePoint2::new(BOUNDED_ARC_HALF_STRIP_WIDTH, -BOUNDED_ARC_STRIP_HALF_LENGTH),
                ProfilePoint2::new(BOUNDED_ARC_HALF_STRIP_WIDTH, BOUNDED_ARC_STRIP_HALF_LENGTH),
                ProfilePoint2::new(-BOUNDED_ARC_HALF_STRIP_WIDTH, BOUNDED_ARC_STRIP_HALF_LENGTH),
            ],
            Vec::new(),
            BOUNDED_ARC_SLAB_HI - BOUNDED_ARC_SLAB_LO,
        ))?
        .into_result()?
        .body();
    let cylinder = edit
        .create_cylinder(CylinderRequest::new(
            frame,
            BOUNDED_ARC_RADIUS,
            BOUNDED_ARC_CYLINDER_HEIGHT,
        ))?
        .into_result()?
        .body();
    Ok((block, cylinder))
}

fn create_cap_retaining_operands(
    session: &mut Session,
    part: &PartId,
) -> OracleResult<(BodyId, BodyId)> {
    let frame = Frame::world();
    let mut edit = session.edit_part(part.clone())?;
    let block = edit
        .create_block(BlockRequest::new(
            frame.with_origin(frame.point_at(0.0, 0.0, 1.0)),
            [2.0, 5.0, 1.0],
        ))?
        .into_result()?
        .body();
    let cylinder = edit
        .create_cylinder(CylinderRequest::new(
            frame,
            BOUNDED_ARC_RADIUS,
            BOUNDED_ARC_CYLINDER_HEIGHT,
        ))?
        .into_result()?
        .body();
    Ok((block, cylinder))
}

fn create_five_portal_operands(
    session: &mut Session,
    part: &PartId,
) -> OracleResult<(BodyId, BodyId)> {
    let frame = Frame::world();
    let mut edit = session.edit_part(part.clone())?;
    let block = edit
        .extrude_profile(ExtrudeProfileRequest::new(
            frame.with_origin(frame.point_at(0.0, 0.0, BOUNDED_ARC_SLAB_LO)),
            vec![
                ProfilePoint2::new(0.0, 1.6),
                ProfilePoint2::new(-1.521_690_426, 0.494_427_191),
                ProfilePoint2::new(-0.940_456_404, -1.294_427_191),
                ProfilePoint2::new(0.940_456_404, -1.294_427_191),
                ProfilePoint2::new(1.521_690_426, 0.494_427_191),
            ],
            Vec::new(),
            BOUNDED_ARC_SLAB_HI - BOUNDED_ARC_SLAB_LO,
        ))?
        .into_result()?
        .body();
    let cylinder = edit
        .create_cylinder(CylinderRequest::new(
            frame,
            BOUNDED_ARC_RADIUS,
            BOUNDED_ARC_CYLINDER_HEIGHT,
        ))?
        .into_result()?
        .body();
    Ok((block, cylinder))
}

fn bounded_arc_disk_strip_volume() -> f64 {
    let half_width = BOUNDED_ARC_HALF_STRIP_WIDTH;
    let radius = BOUNDED_ARC_RADIUS;
    let strip_area = 2.0
        * (half_width * (radius * radius - half_width * half_width).sqrt()
            + radius * radius * BOUNDED_ARC_STRIP_HALF_ANGLE);
    strip_area * (BOUNDED_ARC_SLAB_HI - BOUNDED_ARC_SLAB_LO)
}

fn bounded_arc_strip_block_volume() -> f64 {
    2.0 * BOUNDED_ARC_HALF_STRIP_WIDTH
        * 2.0
        * BOUNDED_ARC_STRIP_HALF_LENGTH
        * (BOUNDED_ARC_SLAB_HI - BOUNDED_ARC_SLAB_LO)
}

fn cap_retaining_block_volume() -> f64 {
    2.0 * 5.0
}

fn cap_retaining_cylinder_volume() -> f64 {
    core::f64::consts::PI * BOUNDED_ARC_RADIUS * BOUNDED_ARC_RADIUS * BOUNDED_ARC_CYLINDER_HEIGHT
}

fn build_single_result(
    file: &'static str,
    left: BlockSpec,
    right: BlockSpec,
    operation: BooleanOperation,
    expected_volume: f64,
    expected_counts: TopologyCounts,
) -> OracleResult<Artifact> {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (left_body, right_body) = create_operands(&mut session, &part, left, right)?;
    let bodies = run_boolean(
        &mut session,
        &part,
        left_body.clone(),
        right_body.clone(),
        operation,
    )?;
    require(
        bodies.len() == 1,
        format!("{file} expected one result body, got {}", bodies.len()),
    )?;
    assert_sources_retained(&session, &part, &left_body, &right_body, 3)?;
    make_artifact(
        &mut session,
        &part,
        bodies[0].clone(),
        file,
        expected_volume,
        expected_counts,
    )
}

fn build_disjoint_union() -> OracleResult<Vec<Artifact>> {
    let mut session = Kernel::new().create_session();
    let part = session.create_part();
    let (left, right) = create_operands(&mut session, &part, DISJOINT_LEFT, DISJOINT_RIGHT)?;
    let bodies = run_boolean(
        &mut session,
        &part,
        left.clone(),
        right.clone(),
        BooleanOperation::Unite,
    )?;
    require(
        bodies.len() == 2,
        format!("disjoint union expected two bodies, got {}", bodies.len()),
    )?;
    assert_sources_retained(&session, &part, &left, &right, 4)?;
    Ok(vec![
        make_artifact(
            &mut session,
            &part,
            bodies[0].clone(),
            DISJOINT_BODY_0,
            DISJOINT_LEFT.volume(),
            DISJOINT_LEFT_COUNTS,
        )?,
        make_artifact(
            &mut session,
            &part,
            bodies[1].clone(),
            DISJOINT_BODY_1,
            DISJOINT_RIGHT.volume(),
            DISJOINT_RIGHT_COUNTS,
        )?,
    ])
}

fn create_operands(
    session: &mut Session,
    part: &PartId,
    left: BlockSpec,
    right: BlockSpec,
) -> OracleResult<(BodyId, BodyId)> {
    let mut edit = session.edit_part(part.clone())?;
    let left_body = edit
        .create_block(BlockRequest::new(left.frame()?, left.extents))?
        .into_result()?
        .body();
    let right_body = edit
        .create_block(BlockRequest::new(right.frame()?, right.extents))?
        .into_result()?
        .body();
    Ok((left_body, right_body))
}

fn run_boolean(
    session: &mut Session,
    part: &PartId,
    left: BodyId,
    right: BodyId,
    operation: BooleanOperation,
) -> OracleResult<Vec<BodyId>> {
    let outcome = session
        .edit_part(part.clone())?
        .boolean_bodies(BooleanBodiesRequest::new(operation, left, right))?
        .into_result()?;
    let result = match outcome {
        BooleanOutcome::Success(result) => result,
        BooleanOutcome::Refused(refusal) => {
            return Err(failure(format!(
                "{operation:?} unexpectedly refused: {refusal:?}"
            )));
        }
        other => return Err(failure(format!("unexpected Boolean outcome: {other:?}"))),
    };
    let created = match result {
        BooleanResult::Created(created) => created,
        BooleanResult::ProvenEmpty => {
            return Err(failure(format!(
                "{operation:?} unexpectedly produced an empty result"
            )));
        }
        other => return Err(failure(format!("unexpected Boolean result: {other:?}"))),
    };
    require(
        created.bodies().len() == created.reports().len(),
        "Boolean Full reports were not aligned to result bodies",
    )?;
    for (body, report) in created.bodies().iter().zip(created.reports()) {
        require(
            report.body() == *body,
            "Boolean Full report body order changed",
        )?;
        require(
            report.report().level() == CheckLevel::Full
                && report.report().outcome() == CheckOutcome::Valid,
            "Boolean result was not Full-valid",
        )?;
    }
    Ok(created.bodies().to_vec())
}

fn assert_sources_retained(
    session: &Session,
    part: &PartId,
    left: &BodyId,
    right: &BodyId,
    expected_bodies: usize,
) -> OracleResult<()> {
    let part = session.part(part.clone())?;
    require(
        part.bodies().len() == expected_bodies,
        "Boolean changed the expected source/result body count",
    )?;
    require(
        part.body(left.clone())?.kind() == BodyKind::Solid
            && part.body(right.clone())?.kind() == BodyKind::Solid,
        "Boolean did not retain both source solids",
    )
}

fn make_artifact(
    session: &mut Session,
    part: &PartId,
    body: BodyId,
    file: &'static str,
    expected_volume: f64,
    expected_counts: TopologyCounts,
) -> OracleResult<Artifact> {
    make_artifact_with_volume_tolerance(
        session,
        part,
        body,
        file,
        expected_volume,
        expected_counts,
        2.0e-10,
        1.0e-6,
    )
}

#[allow(clippy::too_many_arguments)]
fn make_artifact_with_volume_tolerance(
    session: &mut Session,
    part: &PartId,
    body: BodyId,
    file: &'static str,
    expected_volume: f64,
    expected_counts: TopologyCounts,
    relative_volume_tolerance: f64,
    chord_tolerance: f64,
) -> OracleResult<Artifact> {
    let (body_kind, counts, mesh) = inspect_body(session, part, body.clone(), chord_tolerance)?;
    require(body_kind == "solid", format!("{file} was not a solid body"))?;
    require(
        counts == expected_counts,
        format!("{file} topology changed: expected {expected_counts:?}, got {counts:?}"),
    )?;
    assert_volume(
        file,
        mesh.volume,
        expected_volume,
        relative_volume_tolerance,
    )?;

    let bytes = session
        .part(part.clone())?
        .export_xt(ExportXtRequest::new(body))?
        .into_result()?
        .bytes()
        .to_vec();
    let imported_part = session.create_part();
    let imported = session
        .edit_part(imported_part.clone())?
        .import_xt(ImportXtRequest::new(&bytes))?
        .into_result()?;
    require(
        imported.skipped().is_empty(),
        format!("{file} local import unexpectedly skipped schema nodes"),
    )?;
    require(
        imported.bodies().len() == 1,
        format!("{file} local import did not reconstruct exactly one body"),
    )?;
    let imported_body = imported.bodies()[0].clone();
    let (imported_kind, imported_counts, imported_mesh) = inspect_body(
        session,
        &imported_part,
        imported_body.clone(),
        chord_tolerance,
    )?;
    require(
        imported_kind == body_kind,
        format!("{file} body kind changed on import"),
    )?;
    require(
        imported_counts == counts,
        format!("{file} topology counts changed on import"),
    )?;
    assert_volume(
        file,
        imported_mesh.volume,
        expected_volume,
        relative_volume_tolerance,
    )?;

    Ok(Artifact {
        file,
        body_kind,
        counts,
        mesh,
        bytes,
    })
}

fn inspect_body(
    session: &Session,
    part: &PartId,
    body: BodyId,
    chord_tolerance: f64,
) -> OracleResult<(&'static str, TopologyCounts, MeshSummary)> {
    let part = session.part(part.clone())?;
    let body_view = part.body(body.clone())?;
    let body_kind = body_kind(body_view.kind())?;
    let regions = body_view.regions().collect::<Vec<_>>();
    let shells = regions
        .iter()
        .map(|region| part.region(region.clone()).map(|view| view.shells().len()))
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .sum();
    let faces = body_view.faces()?.collect::<Vec<_>>();
    let loops = faces
        .iter()
        .map(|face| part.face(face.clone()).map(|view| view.loops().len()))
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .sum();
    let fins = faces
        .iter()
        .map(|face| {
            let face = part.face(face.clone())?;
            face.loops()
                .map(|loop_id| part.loop_(loop_id).map(|view| view.fins().len()))
                .collect::<Result<Vec<_>, _>>()
                .map(|counts| counts.into_iter().sum::<usize>())
        })
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .sum();
    let counts = TopologyCounts {
        regions: regions.len(),
        shells,
        faces: faces.len(),
        loops,
        fins,
        edges: body_view.edges()?.len(),
        vertices: body_view.vertices()?.len(),
    };

    let check = part
        .check_body(CheckBodyRequest::new(body.clone(), CheckLevel::Fast))?
        .into_result()?;
    require(
        check.outcome() == CheckOutcome::Valid,
        format!(
            "artifact body did not pass a public Fast check: outcome={:?}, faults={}, gaps={}",
            check.outcome(),
            check.faults().len(),
            check.gaps().len()
        ),
    )?;
    let mesh = part
        .tessellate_body(TessellateBodyRequest::new(
            body,
            TessOptions {
                chord_tol: chord_tolerance,
                max_edge_len: None,
            },
        ))?
        .into_result()?;
    Ok((
        body_kind,
        counts,
        MeshSummary {
            positions: mesh.positions().len(),
            triangles: mesh.triangles().len(),
            volume: mesh_volume(mesh.positions(), mesh.triangles()),
        },
    ))
}

fn body_kind(kind: BodyKind) -> OracleResult<&'static str> {
    match kind {
        BodyKind::Solid => Ok("solid"),
        BodyKind::Sheet => Ok("sheet"),
        BodyKind::Wire => Ok("wire"),
        other => Err(failure(format!("unsupported future body kind: {other:?}"))),
    }
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

fn assert_volume(
    file: &str,
    actual: f64,
    expected: f64,
    relative_tolerance: f64,
) -> OracleResult<()> {
    let tolerance = expected.abs().max(1.0) * relative_tolerance;
    require(
        actual.is_finite() && (actual - expected).abs() <= tolerance,
        format!(
            "{file} mesh volume {actual:.17e} differed from independent CSG volume {expected:.17e}"
        ),
    )
}

fn connected_intersection_volume() -> f64 {
    let left = horizontal_rectangle(CONNECTED_LEFT);
    let right = horizontal_rectangle(CONNECTED_RIGHT);
    let area = polygon_area(&clip_convex_polygon(left, &right));
    let left_z = (
        CONNECTED_LEFT.center[2] - CONNECTED_LEFT.extents[2] * 0.5,
        CONNECTED_LEFT.center[2] + CONNECTED_LEFT.extents[2] * 0.5,
    );
    let right_z = (
        CONNECTED_RIGHT.center[2] - CONNECTED_RIGHT.extents[2] * 0.5,
        CONNECTED_RIGHT.center[2] + CONNECTED_RIGHT.extents[2] * 0.5,
    );
    let height = left_z.1.min(right_z.1) - left_z.0.max(right_z.0);
    area * height.max(0.0)
}

fn horizontal_rectangle(spec: BlockSpec) -> Vec<Point2> {
    let center = Point2 {
        x: spec.center[0],
        y: spec.center[1],
    };
    let x = Point2 {
        x: spec.x[0] * spec.extents[0] * 0.5,
        y: spec.x[1] * spec.extents[0] * 0.5,
    };
    let y = Point2 {
        x: -spec.x[1] * spec.extents[1] * 0.5,
        y: spec.x[0] * spec.extents[1] * 0.5,
    };
    vec![
        center.sub(x).sub(y),
        center.add(x).sub(y),
        center.add(x).add(y),
        center.sub(x).add(y),
    ]
}

fn clip_convex_polygon(mut subject: Vec<Point2>, clip: &[Point2]) -> Vec<Point2> {
    for index in 0..clip.len() {
        let line_start = clip[index];
        let line_end = clip[(index + 1) % clip.len()];
        let edge = line_end.sub(line_start);
        let input = std::mem::take(&mut subject);
        let Some(mut previous) = input.last().copied() else {
            break;
        };
        let mut previous_inside = cross(edge, previous.sub(line_start)) >= 0.0;
        for current in input {
            let current_inside = cross(edge, current.sub(line_start)) >= 0.0;
            if current_inside != previous_inside {
                let direction = current.sub(previous);
                let denominator = cross(edge, direction);
                let parameter = -cross(edge, previous.sub(line_start)) / denominator;
                subject.push(previous.add(direction.scale(parameter)));
            }
            if current_inside {
                subject.push(current);
            }
            previous = current;
            previous_inside = current_inside;
        }
    }
    subject
}

fn polygon_area(polygon: &[Point2]) -> f64 {
    if polygon.len() < 3 {
        return 0.0;
    }
    polygon
        .iter()
        .zip(polygon.iter().cycle().skip(1))
        .take(polygon.len())
        .map(|(first, second)| first.x * second.y - first.y * second.x)
        .sum::<f64>()
        .abs()
        * 0.5
}

fn cross(first: Point2, second: Point2) -> f64 {
    first.x * second.y - first.y * second.x
}

fn manifest(artifacts: &[Artifact]) -> String {
    let mut output = String::from(
        "file\tbody_kind\tregions\tshells\tfaces\tloops\tfins\tedges\tvertices\tmesh_positions\tmesh_triangles\tvolume\tbytes\tfnv1a64\n",
    );
    for artifact in artifacts {
        writeln!(
            output,
            "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{:.17e}\t{}\t{:016x}",
            artifact.file,
            artifact.body_kind,
            artifact.counts.regions,
            artifact.counts.shells,
            artifact.counts.faces,
            artifact.counts.loops,
            artifact.counts.fins,
            artifact.counts.edges,
            artifact.counts.vertices,
            artifact.mesh.positions,
            artifact.mesh.triangles,
            artifact.mesh.volume,
            artifact.bytes.len(),
            fnv1a64(&artifact.bytes),
        )
        .expect("writing to a String cannot fail");
    }
    output
}

fn fnv1a64(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |hash, byte| {
        (hash ^ u64::from(*byte)).wrapping_mul(0x0000_0100_0000_01b3)
    })
}

fn require(condition: bool, message: impl Into<String>) -> OracleResult<()> {
    if condition {
        Ok(())
    } else {
        Err(failure(message))
    }
}

fn failure(message: impl Into<String>) -> Box<dyn StdError> {
    Box::new(io::Error::other(message.into()))
}
