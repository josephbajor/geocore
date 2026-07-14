//! CLI contract for the M3b external-oracle harness (`xt_oracle`).

use std::path::{Path, PathBuf};
use std::process::Command;

use kgeom::surface::Surface as _;

fn export_into(dir: &Path) {
    let output = Command::new(env!("CARGO_BIN_EXE_xt_oracle"))
        .arg("export")
        .arg(dir)
        .output()
        .expect("running xt_oracle export");
    assert!(
        output.status.success(),
        "export failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );
}

fn bundle_dir(name: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(name);
    if dir.exists() {
        std::fs::remove_dir_all(&dir).expect("clearing previous bundle dir");
    }
    dir
}

fn assert_close(actual: f64, expected: f64, tolerance: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{what}: expected {expected:.16e}, found {actual:.16e}"
    );
}

/// Pin the geometry that makes this fixture independent evidence rather than
/// another exactly planar B-surface that a host may canonicalize away.
fn assert_curved_nurbs_face_roundtrip(path: &Path) {
    let bytes = std::fs::read(path).expect("curved NURBS-face fixture");
    let mut store = ktopo::store::Store::new();
    let recon = kxt::import(&bytes, &mut store).expect("curved fixture self-import");
    assert_eq!(recon.bodies.len(), 1);
    let body = recon.bodies[0];
    assert_eq!(
        store.get(body).expect("body").kind,
        ktopo::entity::BodyKind::Solid
    );
    let faults = ktopo::check::check_body(&store, body).expect("curved fixture check");
    assert!(faults.is_empty(), "curved fixture faults: {faults:?}");

    let mut curved_faces = store
        .faces_of_body(body)
        .expect("body faces")
        .into_iter()
        .filter_map(|face_id| {
            let face = store.get(face_id).expect("face");
            match store.get(face.surface).expect("surface") {
                ktopo::geom::SurfaceGeom::Nurbs(surface) => Some((face_id, face, surface)),
                _ => None,
            }
        });
    let (_face_id, face, surface) = curved_faces.next().expect("one NURBS face");
    assert!(
        curved_faces.next().is_none(),
        "expected exactly one NURBS face"
    );
    assert_eq!(face.sense, ktopo::entity::Sense::Forward);
    let domain = face.domain.expect("bounded NURBS face domain");
    assert_eq!(domain.u, kgeom::param::ParamRange::new(0.0, 1.0));
    assert_eq!(domain.v, kgeom::param::ParamRange::new(0.0, 1.0));
    assert_eq!((surface.degree_u(), surface.degree_v()), (2, 2));
    assert_eq!(surface.net_size(), (3, 3));
    assert!(!surface.is_rational(), "fixture must remain polynomial");
    let expected_knots = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    assert_eq!(
        surface.knots(kgeom::surface::Dir::U).as_slice(),
        expected_knots
    );
    assert_eq!(
        surface.knots(kgeom::surface::Dir::V).as_slice(),
        expected_knots
    );

    // With u-row/v-column storage, indices 1, 3, 5, and 7 must be exact
    // boundary midpoints. The sole interior control is 0.04 m along the
    // outward surface normal, producing a 0.01 m evaluated center rise.
    let points = surface.points();
    for (mid, ends) in [(1, (0, 2)), (3, (0, 6)), (5, (2, 8)), (7, (6, 8))] {
        let expected = (points[ends.0] + points[ends.1]) * 0.5;
        assert!(
            points[mid].dist(expected) <= 1.0e-14,
            "control {mid} is not its boundary midpoint"
        );
    }
    let planar_center = (points[0] + points[2] + points[6] + points[8]) * 0.25;
    let normal = (points[6] - points[0])
        .cross(points[2] - points[0])
        .normalized()
        .expect("regular control-net plane");
    let control_offset = points[4] - planar_center;
    assert_close(
        control_offset.dot(normal),
        0.04,
        1.0e-14,
        "outward center-control offset",
    );
    assert!(
        (control_offset - normal * 0.04).norm() <= 1.0e-14,
        "center control has an in-plane displacement"
    );
    assert_close(
        (surface.eval([0.5, 0.5]) - planar_center).dot(normal),
        0.01,
        1.0e-14,
        "evaluated patch-center rise",
    );

    // Exact-edge FINs do not carry SP-curves in this writer slice. Prove from
    // the transmitted unit chart that each NURBS boundary still lifts onto
    // one exact straight topological edge; the source-chart pcurves are pinned
    // separately by the binary's unit test.
    assert_eq!(face.loops.len(), 1);
    let fins = &store.get(face.loops[0]).expect("curved face loop").fins;
    assert_eq!(fins.len(), 4);
    let unit_boundaries = [
        ([0.0, 0.0], [0.0, 1.0]),
        ([0.0, 1.0], [1.0, 1.0]),
        ([1.0, 1.0], [1.0, 0.0]),
        ([1.0, 0.0], [0.0, 0.0]),
    ];
    for (uv0, uv1) in unit_boundaries {
        let start_position = surface.eval(uv0);
        let end_position = surface.eval(uv1);
        let uvm = [0.5 * (uv0[0] + uv1[0]), 0.5 * (uv0[1] + uv1[1])];
        assert!(
            surface
                .eval(uvm)
                .dist((start_position + end_position) * 0.5)
                <= 1.0e-12
        );
        assert!(fins.iter().any(|&fin_id| {
            let edge = store
                .get(store.get(fin_id).expect("fin").edge)
                .expect("edge");
            let [Some(start), Some(end)] = edge.vertices else {
                return false;
            };
            let edge_start = store.vertex_position(start).expect("start position");
            let edge_end = store.vertex_position(end).expect("end position");
            (edge_start.dist(start_position) <= 1.0e-12 && edge_end.dist(end_position) <= 1.0e-12)
                || (edge_start.dist(end_position) <= 1.0e-12
                    && edge_end.dist(start_position) <= 1.0e-12)
        }));
    }

    let policy = kcore::operation::SessionPolicy::v1();
    let context =
        kcore::operation::OperationContext::new(&policy, kcore::tolerance::Tolerances::default())
            .expect("v1 tessellation context");
    let mesh = ktopo::btess::tessellate_body_with_context(
        &store,
        body,
        &ktopo::btess::TessOptions {
            chord_tol: 1.0e-3,
            max_edge_len: None,
        },
        &context,
    )
    .expect("valid tessellation policy")
    .into_result()
    .expect("curved fixture tessellation");
    let watertight = ktopo::btess::check_watertight(&mesh);
    assert!(
        watertight.is_empty(),
        "mesh is not watertight: {watertight:?}"
    );
    let exact_volume = 0.2 * 0.3 * 0.4 + 0.2 * 0.3 * 0.04 / 9.0;
    let mesh_volume = ktopo::btess::signed_volume(&mesh);
    assert!(
        (mesh_volume - exact_volume).abs() <= 1.0e-2 * exact_volume,
        "mesh volume {mesh_volume:.16e} is not close to exact {exact_volume:.16e}"
    );

    // Exercise a complete transmitted cycle once more. This intentionally
    // asserts structural/checker preservation, not host certification.
    let reexport = kxt::export_text(&store, body).expect("curved fixture re-export");
    let mut roundtripped = ktopo::store::Store::new();
    let second =
        kxt::import(reexport.as_bytes(), &mut roundtripped).expect("curved fixture second import");
    assert_eq!(second.bodies.len(), 1);
    let second_faults =
        ktopo::check::check_body(&roundtripped, second.bodies[0]).expect("second-roundtrip check");
    assert!(
        second_faults.is_empty(),
        "second-roundtrip faults: {second_faults:?}"
    );
}

/// The declared bundle contents. Growing this list is a writer-capability
/// event; shrinking it is a regression.
const EXPECTED_FILES: &[&str] = &[
    "solid_block.x_t",
    "solid_cylinder.x_t",
    "solid_cone.x_t",
    "solid_sphere.x_t",
    "solid_torus.x_t",
    "solid_block_nurbs_edge.x_t",
    "solid_block_nurbs_face.x_t",
    "solid_block_curved_nurbs_face.x_t",
    "solid_block_tolerant_edge.x_t",
    "sheet_cylinder_seam.x_t",
    "sheet_plane_polygon.x_t",
    "wire_polyline_open.x_t",
    "wire_polyline_closed.x_t",
    "acorn_point.x_t",
    "offset_plane.x_t",
];

#[test]
fn export_is_complete_deterministic_and_self_importable() {
    let first = bundle_dir("oracle_bundle_a");
    let second = bundle_dir("oracle_bundle_b");
    export_into(&first);
    export_into(&second);

    let mut produced: Vec<String> = std::fs::read_dir(&first)
        .expect("reading bundle dir")
        .map(|entry| entry.expect("dir entry").file_name().into_string().unwrap())
        .collect();
    produced.sort();
    let mut expected: Vec<String> = EXPECTED_FILES
        .iter()
        .map(|name| name.to_string())
        .chain(std::iter::once("manifest.tsv".to_string()))
        .collect();
    expected.sort();
    assert_eq!(produced, expected, "bundle file set changed");

    for name in &expected {
        let a = std::fs::read(first.join(name)).expect("first bundle file");
        let b = std::fs::read(second.join(name)).expect("second bundle file");
        assert_eq!(a, b, "{name} is not byte-deterministic across exports");
    }

    // The manifest declares one row per fixture in bundle order.
    let manifest = std::fs::read_to_string(first.join("manifest.tsv")).expect("manifest");
    let rows: Vec<&str> = manifest.lines().collect();
    assert_eq!(rows.len(), EXPECTED_FILES.len() + 1, "manifest row count");
    assert!(rows[0].starts_with("file\tbody_kind\tprobe\t"));

    // Every transport file re-imports checker-clean here. The generator
    // already enforces this before writing; the test pins it as a contract.
    for name in EXPECTED_FILES {
        let bytes = std::fs::read(first.join(name)).expect("bundle file");
        let mut store = ktopo::store::Store::new();
        let recon = kxt::import(&bytes, &mut store)
            .unwrap_or_else(|error| panic!("{name}: import failed: {error:?}"));
        assert_eq!(recon.bodies.len(), 1, "{name}: expected exactly one body");
        let faults = ktopo::check::check_body(&store, recon.bodies[0]).expect("check");
        assert!(faults.is_empty(), "{name}: checker faults: {faults:?}");
    }

    assert_curved_nurbs_face_roundtrip(&first.join("solid_block_curved_nurbs_face.x_t"));
}

#[test]
fn export_rejects_stale_transport_entries() {
    let dir = bundle_dir("oracle_bundle_stale_entry");
    std::fs::create_dir_all(&dir).expect("create bundle dir");
    std::fs::write(dir.join("obsolete.x_t"), b"stale").expect("write stale entry");
    let output = Command::new(env!("CARGO_BIN_EXE_xt_oracle"))
        .arg("export")
        .arg(&dir)
        .output()
        .expect("running xt_oracle export");
    assert!(!output.status.success(), "stale entry must fail export");
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(stderr.contains("stale or unexpected entries"), "{stderr}");
}

#[test]
fn compare_accepts_identity_and_rejects_a_different_body() {
    let dir = bundle_dir("oracle_bundle_compare");
    export_into(&dir);

    // Identity comparison passes for an exact solid and for the tolerant
    // SP-curve fixture (the hardest writer path).
    for name in [
        "solid_block.x_t",
        "solid_block_curved_nurbs_face.x_t",
        "solid_block_tolerant_edge.x_t",
    ] {
        let file = dir.join(name);
        let output = Command::new(env!("CARGO_BIN_EXE_xt_oracle"))
            .arg("compare")
            .arg(&file)
            .arg(&file)
            .output()
            .expect("running xt_oracle compare");
        assert!(
            output.status.success(),
            "{name}: identity compare failed:\n{}",
            String::from_utf8_lossy(&output.stdout),
        );
        let stdout = String::from_utf8_lossy(&output.stdout);
        assert!(stdout.contains("COMPARE OK"), "{name}: {stdout}");
    }

    // A genuinely different body must be flagged with exit code 1.
    let output = Command::new(env!("CARGO_BIN_EXE_xt_oracle"))
        .arg("compare")
        .arg(dir.join("solid_block.x_t"))
        .arg(dir.join("solid_sphere.x_t"))
        .output()
        .expect("running xt_oracle compare");
    assert_eq!(output.status.code(), Some(1), "mismatch must exit 1");
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("COMPARE FAILED"), "{stdout}");
    assert!(stdout.contains("FAIL  surface_classes"), "{stdout}");

    // An unreadable input is an operational error, not a mismatch.
    let output = Command::new(env!("CARGO_BIN_EXE_xt_oracle"))
        .arg("compare")
        .arg(dir.join("solid_block.x_t"))
        .arg(dir.join("does_not_exist.x_t"))
        .output()
        .expect("running xt_oracle compare");
    assert_eq!(output.status.code(), Some(2), "IO failure must exit 2");
}

#[test]
fn compare_reports_offset_as_a_stable_surface_class() {
    let fixture = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/offset_plane.x_t");
    let output = Command::new(env!("CARGO_BIN_EXE_xt_oracle"))
        .arg("compare")
        .arg(&fixture)
        .arg(&fixture)
        .output()
        .expect("running xt_oracle offset identity comparison");
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout.contains("PASS  surface_classes: offset:1"),
        "{stdout}"
    );
}
