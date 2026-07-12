//! External-oracle bundle generator and re-import comparator (M3b).
//!
//! Self-round-trip cannot certify interchange: a convention shared by this
//! repository's reader and writer (for example a transposed B-surface pole
//! order) round-trips cleanly and stays invisible. This tool feeds the
//! human-in-the-loop oracle procedure in `docs/oracle-loop.md`:
//!
//! - `xt_oracle export <dir>` deterministically generates one `.x_t` file per
//!   declared Tier 1 writer capability plus a `manifest.tsv` of expected
//!   topology counts, mass properties, checker outcomes, and content hashes.
//!   Every file is re-imported and re-checked locally before it is written, so
//!   a host is never handed a file this repository's own pipeline rejects.
//! - `xt_oracle compare <ours.x_t> <theirs.x_t>` diffs a licensed-host
//!   re-export against the original bundle file by body kind, entity counts,
//!   geometry-class histograms, entity tolerances, checker cleanliness,
//!   watertightness, and enclosed volume, and exits nonzero on any mismatch.

use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::path::Path;
use std::process::ExitCode;

use kcore::operation::{OperationContext, SessionPolicy};
use kcore::tolerance::{LINEAR_RESOLUTION, Tolerances};
use kgeom::curve2d::{Line2d, NurbsCurve2d};
use kgeom::frame::Frame;
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::param::ParamRange;
use kgeom::vec::{Point2, Point3, Vec3};
use ktopo::btess::{
    BodyMesh, TessOptions, check_watertight, signed_volume, tessellate_body_with_context,
};
use ktopo::check::{CheckLevel, CheckOutcome, check_body, check_body_report};
use ktopo::entity::{
    BodyId, BodyKind, Edge, EdgeId, FaceDomain, FaceId, Fin, FinPcurve, Loop, ParamMap1d, Region,
    Shell, Vertex,
};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::make;
use ktopo::store::Store;
use ktopo::tolerance::EntityTolerance;
use ktopo::transaction::AssemblyStore;

/// Chord tolerance shared by every mass-property measurement in the bundle.
const CHORD_TOL: f64 = 1e-3;
/// Relative drift allowed between two tessellated (mesh) volumes. XT does
/// not store edge parameter bounds or exact-fin chart metadata, so a
/// re-imported body legitimately triangulates differently at the same chord
/// tolerance; the bound is discretization-driven, not geometry-driven.
const MESH_VOLUME_REL_TOL: f64 = 2e-3;
/// Relative deficit allowed between a fixture's tessellated volume and its
/// closed-form volume (inscribed chords underestimate curved solids).
const EXACT_VOLUME_REL_TOL: f64 = 1e-2;
/// Absolute drift allowed when comparing entity-tolerance values.
const TOLERANCE_ABS_TOL: f64 = 1e-9;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.as_slice() {
        [cmd, dir] if cmd == "export" => match export_bundle(Path::new(dir)) {
            Ok(()) => ExitCode::SUCCESS,
            Err(error) => {
                eprintln!("xt_oracle export failed: {error}");
                ExitCode::from(2)
            }
        },
        [cmd, ours, theirs] if cmd == "compare" => {
            match compare_files(Path::new(ours), Path::new(theirs)) {
                Ok(true) => ExitCode::SUCCESS,
                Ok(false) => ExitCode::from(1),
                Err(error) => {
                    eprintln!("xt_oracle compare failed: {error}");
                    ExitCode::from(2)
                }
            }
        }
        _ => {
            eprintln!(
                "usage:\n  xt_oracle export <out_dir>\n  xt_oracle compare <ours.x_t> <theirs.x_t>"
            );
            ExitCode::from(2)
        }
    }
}

// --------------------------------------------------------------------------
// Fixtures
// --------------------------------------------------------------------------

/// The placement every fixture uses: a tilted, off-origin frame so host
/// import exercises general (non-axis-aligned) analytic geometry.
fn tilted() -> Frame {
    Frame::new(
        Point3::new(0.3, -1.2, 2.1),
        Vec3::new(1.0, 2.0, 3.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .expect("tilted oracle frame is valid")
}

struct Fixture {
    /// Bundle file stem; the writer appends `.x_t`.
    name: &'static str,
    /// What a host run of this file certifies.
    probe: &'static str,
    build: fn(&mut Store) -> BodyId,
    /// Closed-form enclosed volume for solids — the value a host's
    /// physical-properties tool should reproduce (near-)exactly.
    exact_volume: Option<f64>,
}

const PI: f64 = std::f64::consts::PI;
const BLOCK_VOLUME: f64 = 0.2 * 0.3 * 0.4;
const OFFSET_PLANE_BYTES: &[u8] = include_bytes!("../../tests/fixtures/offset_plane.x_t");

/// Bundle definition. Order is the manifest order; keep it stable.
const FIXTURES: &[Fixture] = &[
    Fixture {
        name: "solid_block",
        probe: "planes and lines; solid region/shell scaffold",
        build: |s| make::block(s, &tilted(), [0.2, 0.3, 0.4]).expect("block"),
        exact_volume: Some(BLOCK_VOLUME),
    },
    Fixture {
        name: "solid_cylinder",
        probe: "cylinder surface, circles, ring edges, cap sense",
        build: |s| make::cylinder(s, &tilted(), 0.13, 0.2).expect("cylinder"),
        exact_volume: Some(PI * 0.13 * 0.13 * 0.2),
    },
    Fixture {
        name: "solid_cone",
        probe: "cone surface with half-angle encoding; frustum caps",
        build: |s| make::cone(s, &tilted(), 0.15, 0.06, 0.2).expect("cone"),
        exact_volume: Some(PI * 0.2 * (0.15 * 0.15 + 0.15 * 0.06 + 0.06 * 0.06) / 3.0),
    },
    Fixture {
        name: "solid_sphere",
        probe: "sphere surface; zero-loop closed face",
        build: |s| make::sphere(s, &tilted(), 0.11).expect("sphere"),
        exact_volume: Some(4.0 / 3.0 * PI * 0.11 * 0.11 * 0.11),
    },
    Fixture {
        name: "solid_torus",
        probe: "torus surface; zero-loop closed face",
        build: |s| make::torus(s, &tilted(), 0.2, 0.07).expect("torus"),
        exact_volume: Some(2.0 * PI * PI * 0.2 * 0.07 * 0.07),
    },
    Fixture {
        name: "solid_block_nurbs_edge",
        probe: "B_CURVE edge geometry (NURBS_CURVE aux chain)",
        build: |s| {
            let body = make::block(s, &tilted(), [0.2, 0.3, 0.4]).expect("block");
            replace_edge_with_linear_nurbs(s, body);
            body
        },
        exact_volume: Some(BLOCK_VOLUME),
    },
    Fixture {
        name: "solid_block_nurbs_face",
        probe: "B_SURFACE face geometry — settles the provisional v-fastest pole ordering",
        build: |s| {
            let body = make::block(s, &tilted(), [0.2, 0.3, 0.4]).expect("block");
            replace_face_with_bilinear_nurbs(s, body);
            body
        },
        exact_volume: Some(BLOCK_VOLUME),
    },
    Fixture {
        name: "solid_block_tolerant_edge",
        probe: "curve-less tolerant edge as per-fin TRIMMED_CURVE/SP_CURVE/2D B_CURVE",
        build: |s| {
            let body = make::block(s, &tilted(), [0.2, 0.3, 0.4]).expect("block");
            make_first_edge_truly_tolerant(s, body);
            body
        },
        exact_volume: Some(BLOCK_VOLUME),
    },
    Fixture {
        name: "sheet_cylinder_seam",
        probe: "full-period cylindrical sheet; shared seam edge used twice by one face",
        build: |s| make::cylindrical_sheet(s, &tilted(), 0.13, 0.2).expect("cylindrical sheet"),
        exact_volume: None,
    },
    Fixture {
        name: "sheet_plane_polygon",
        probe: "concave planar polygon sheet; dummy boundary FINs",
        build: |s| {
            let polygon = [
                Point2::new(0.0, 0.0),
                Point2::new(0.4, 0.0),
                Point2::new(0.4, 0.15),
                Point2::new(0.15, 0.15),
                Point2::new(0.15, 0.35),
                Point2::new(0.0, 0.35),
            ];
            make::planar_sheet(s, &tilted(), &polygon).expect("planar sheet")
        },
        exact_volume: None,
    },
    Fixture {
        name: "wire_polyline_open",
        probe: "open wire body; dummy endpoint FINs",
        build: |s| {
            let points = [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.1, 0.0, 0.02),
                Point3::new(0.15, 0.08, 0.05),
                Point3::new(0.1, 0.16, 0.1),
            ];
            make::wire_polyline(s, &points, false).expect("open wire")
        },
        exact_volume: None,
    },
    Fixture {
        name: "wire_polyline_closed",
        probe: "closed wire body",
        build: |s| {
            let points = [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.12, 0.0, 0.0),
                Point3::new(0.06, 0.09, 0.06),
            ];
            make::wire_polyline(s, &points, true).expect("closed wire")
        },
        exact_volume: None,
    },
    Fixture {
        name: "acorn_point",
        probe: "acorn (single-vertex) body",
        build: |s| make::acorn(s, Point3::new(0.05, -0.02, 0.03)).expect("acorn"),
        exact_volume: None,
    },
];

// --------------------------------------------------------------------------
// Checked post-construction edits (same public-API path the writer tests use)
// --------------------------------------------------------------------------

fn edit_body<R>(
    store: &mut Store,
    body: BodyId,
    edit: impl FnOnce(&mut AssemblyStore<'_>) -> R,
) -> R {
    let mut transaction = store.transaction().expect("open transaction");
    let result = {
        let mut assembly = transaction.assembly();
        edit(&mut assembly)
    };
    transaction
        .commit_checked_body(body)
        .expect("checked commit of oracle fixture edit");
    result
}

fn first_bounded_edge(store: &Store, body: BodyId) -> EdgeId {
    store
        .edges_of_body(body)
        .expect("edges of body")
        .into_iter()
        .find(|&edge| {
            let edge = store.get(edge).expect("edge");
            edge.bounds.is_some() && edge.vertices[0].is_some() && edge.vertices[1].is_some()
        })
        .expect("body has a bounded edge")
}

fn first_plane_face(store: &Store, body: BodyId) -> FaceId {
    store
        .faces_of_body(body)
        .expect("faces of body")
        .into_iter()
        .find(|&face| {
            let surface = store.get(face).expect("face").surface;
            matches!(store.get(surface).expect("surface"), SurfaceGeom::Plane(_))
        })
        .expect("body has a planar face")
}

/// Swap one bounded straight edge's geometry for an exact degree-1 NURBS
/// curve so the bundle exercises the B_CURVE writer path. The replacement is
/// parameterized over the edge's existing bounds, so its 3D parameterization
/// matches the unit-speed line it replaces and every fin's edge-to-pcurve
/// map stays valid.
fn replace_edge_with_linear_nurbs(store: &mut Store, body: BodyId) {
    edit_body(store, body, |store| {
        let edge_id = first_bounded_edge(store, body);
        let edge = store.get(edge_id).expect("edge");
        let curve_id = edge.curve.expect("exact edge has a curve");
        let (t0, t1) = edge.bounds.expect("bounded edge");
        let start = store
            .vertex_position(edge.vertices[0].expect("start vertex"))
            .expect("start position");
        let end = store
            .vertex_position(edge.vertices[1].expect("end vertex"))
            .expect("end position");
        let nurbs =
            NurbsCurve::new(1, vec![t0, t0, t1, t1], vec![start, end], None).expect("linear NURBS");
        store
            .replace_curve(curve_id, CurveGeom::Nurbs(nurbs))
            .expect("curve replacement");
    });
}

/// Swap one planar face's surface for an exact bilinear NURBS patch over the
/// same plane so the bundle exercises the B_SURFACE writer path. This is the
/// fixture that lets a licensed host confirm or refute the provisional
/// v-fastest pole ordering.
fn replace_face_with_bilinear_nurbs(store: &mut Store, body: BodyId) {
    edit_body(store, body, |store| {
        let face_id = first_plane_face(store, body);
        let surface_id = store.get(face_id).expect("face").surface;
        let plane = match store.get(surface_id).expect("surface") {
            SurfaceGeom::Plane(plane) => *plane,
            _ => unreachable!("first_plane_face returned a non-plane"),
        };

        let mut u_bounds = [f64::INFINITY, f64::NEG_INFINITY];
        let mut v_bounds = [f64::INFINITY, f64::NEG_INFINITY];
        for &loop_id in &store.get(face_id).expect("face").loops {
            for &fin_id in &store.get(loop_id).expect("loop").fins {
                let edge = store
                    .get(store.get(fin_id).expect("fin").edge)
                    .expect("edge");
                for vertex in edge.vertices.into_iter().flatten() {
                    let local = plane
                        .frame()
                        .to_local(store.vertex_position(vertex).expect("vertex position"));
                    u_bounds[0] = u_bounds[0].min(local.x);
                    u_bounds[1] = u_bounds[1].max(local.x);
                    v_bounds[0] = v_bounds[0].min(local.y);
                    v_bounds[1] = v_bounds[1].max(local.y);
                }
            }
        }

        let points = vec![
            plane.frame().point_at(u_bounds[0], v_bounds[0], 0.0),
            plane.frame().point_at(u_bounds[0], v_bounds[1], 0.0),
            plane.frame().point_at(u_bounds[1], v_bounds[0], 0.0),
            plane.frame().point_at(u_bounds[1], v_bounds[1], 0.0),
        ];
        let surface = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            points,
            None,
        )
        .expect("bilinear NURBS patch");
        store
            .replace_surface(surface_id, SurfaceGeom::Nurbs(surface))
            .expect("surface replacement");
        store.get_mut(face_id).expect("face slot").domain =
            Some(FaceDomain::from_bounds(0.0, 1.0, 0.0, 1.0).expect("unit domain"));

        // The replacement surface uses normalized [0, 1]^2 parameters, so the
        // inherited plane-coordinate pcurves are rewritten as exact normalized
        // ones while keeping each fin's independent Curve2d identity.
        let du = u_bounds[1] - u_bounds[0];
        let dv = v_bounds[1] - v_bounds[0];
        let fin_ids: Vec<_> = store
            .get(face_id)
            .expect("face")
            .loops
            .iter()
            .flat_map(|&loop_id| store.get(loop_id).expect("loop").fins.iter().copied())
            .collect();
        for fin_id in fin_ids {
            let fin = store.get(fin_id).expect("fin");
            let edge = store.get(fin.edge).expect("edge");
            let [Some(start_id), Some(end_id)] = edge.vertices else {
                unreachable!("block face edges are vertex-bounded")
            };
            let Some((t0, t1)) = edge.bounds else {
                unreachable!("block face edges are bounded")
            };
            let to_uv = |point: Point3| {
                let local = plane.frame().to_local(point);
                Point2::new((local.x - u_bounds[0]) / du, (local.y - v_bounds[0]) / dv)
            };
            let start = to_uv(store.vertex_position(start_id).expect("start position"));
            let end = to_uv(store.vertex_position(end_id).expect("end position"));
            let uv_len = (end - start).norm();
            let pcurve_id = fin.pcurve.expect("block fins carry pcurves").curve();
            store
                .replace_pcurve(
                    pcurve_id,
                    Curve2dGeom::Line(Line2d::new(start, end - start).expect("uv line")),
                )
                .expect("pcurve replacement");
            let scale = uv_len / (t1 - t0);
            let map = ParamMap1d::affine(scale, -scale * t0).expect("affine map");
            store.get_mut(fin_id).expect("fin slot").pcurve = Some(
                FinPcurve::new(pcurve_id, ParamRange::new(0.0, uv_len), map).expect("fin pcurve"),
            );
        }
    });
}

/// Turn one bounded edge into a curve-less tolerant edge over the canonical
/// [0, 1] logical domain, with one rational 2D B-curve use whose stored curve
/// extends far beyond its active SP trim and one reversed-trim line use.
fn make_first_edge_truly_tolerant(store: &mut Store, body: BodyId) -> EdgeId {
    edit_body(store, body, |store| {
        let edge_id = first_bounded_edge(store, body);
        let edge = store.get(edge_id).expect("edge");
        let old_bounds = edge.bounds.expect("bounded edge");
        let fins = edge.fins.clone();
        for fin_id in &fins {
            let old = store.get(*fin_id).expect("fin").pcurve.expect("pcurve");
            let q0 = old.parameter_at_edge(old_bounds.0);
            let q1 = old.parameter_at_edge(old_bounds.1);
            store.get_mut(*fin_id).expect("fin slot").pcurve = Some(
                FinPcurve::new(
                    old.curve(),
                    old.range(),
                    ParamMap1d::affine(q1 - q0, q0).expect("affine map"),
                )
                .expect("fin pcurve"),
            );
        }

        let first = fins[0];
        let first_use = store.get(first).expect("fin").pcurve.expect("pcurve");
        let first_curve = store
            .get(first_use.curve())
            .expect("pcurve geom")
            .as_curve();
        let range = first_use.range();
        let extended_hi = range.lo + 10.0 * range.width();
        let endpoints = vec![first_curve.eval(range.lo), first_curve.eval(extended_hi)];
        store
            .replace_pcurve(
                first_use.curve(),
                Curve2dGeom::Nurbs(
                    NurbsCurve2d::new(
                        1,
                        vec![range.lo, range.lo, extended_hi, extended_hi],
                        endpoints,
                        Some(vec![2.0, 2.0]),
                    )
                    .expect("rational 2D B-curve"),
                ),
            )
            .expect("pcurve replacement");

        let second = fins[1];
        let second_use = store.get(second).expect("fin").pcurve.expect("pcurve");
        let Curve2dGeom::Line(line) = *store.get(second_use.curve()).expect("pcurve geom") else {
            unreachable!("block pcurve must be linear")
        };
        let range = second_use.range();
        store
            .replace_pcurve(
                second_use.curve(),
                Curve2dGeom::Line(
                    Line2d::new(
                        line.origin() + line.dir() * (range.lo + range.hi),
                        -line.dir(),
                    )
                    .expect("reversed uv line"),
                ),
            )
            .expect("pcurve replacement");
        let old_map = second_use.edge_to_pcurve();
        let reversed_map =
            ParamMap1d::affine(-old_map.scale(), range.lo + range.hi - old_map.offset())
                .expect("reversed map");
        store.get_mut(second).expect("fin slot").pcurve = Some(
            FinPcurve::new(second_use.curve(), second_use.range(), reversed_map)
                .expect("fin pcurve"),
        );

        let edge = store.get_mut(edge_id).expect("edge slot");
        edge.curve = None;
        edge.bounds = Some((0.0, 1.0));
        edge.tolerance = Some(
            EntityTolerance::operation(LINEAR_RESOLUTION, "oracle-bundle").expect("tolerance"),
        );
        edge_id
    })
}

// --------------------------------------------------------------------------
// Export
// --------------------------------------------------------------------------

struct Measured {
    kind: BodyKind,
    regions: usize,
    shells: usize,
    faces: usize,
    loops: usize,
    fins: usize,
    edges: usize,
    vertices: usize,
    /// Enclosed volume in m^3 for solids; `None` otherwise.
    volume: Option<f64>,
    /// `Some(true)` when a solid tessellation is watertight.
    watertight: Option<bool>,
    fast_faults: usize,
    full_outcome: CheckOutcome,
    full_gaps: usize,
    surface_classes: Vec<(String, usize)>,
    curve_classes: Vec<(String, usize)>,
    edge_tolerances: Vec<f64>,
    vertex_tolerances: Vec<f64>,
}

fn outcome_name(outcome: CheckOutcome) -> &'static str {
    match outcome {
        CheckOutcome::Valid => "valid",
        CheckOutcome::Invalid => "invalid",
        CheckOutcome::Indeterminate => "indeterminate",
    }
}

fn kind_name(kind: BodyKind) -> &'static str {
    match kind {
        BodyKind::Solid => "solid",
        BodyKind::Sheet => "sheet",
        BodyKind::Wire => "wire",
        BodyKind::Acorn => "acorn",
    }
}

fn surface_class(surface: &SurfaceGeom) -> &'static str {
    match surface {
        SurfaceGeom::Plane(_) => "plane",
        SurfaceGeom::Cylinder(_) => "cylinder",
        SurfaceGeom::Cone(_) => "cone",
        SurfaceGeom::Sphere(_) => "sphere",
        SurfaceGeom::Torus(_) => "torus",
        SurfaceGeom::Nurbs(_) => "nurbs",
        SurfaceGeom::Offset(_) => "offset",
        _ => "procedural",
    }
}

fn curve_class(curve: Option<&CurveGeom>) -> &'static str {
    match curve {
        Some(CurveGeom::Line(_)) => "line",
        Some(CurveGeom::Circle(_)) => "circle",
        Some(CurveGeom::Ellipse(_)) => "ellipse",
        Some(CurveGeom::Nurbs(_)) => "nurbs",
        Some(_) => "procedural",
        None => "tolerant",
    }
}

fn histogram(mut classes: Vec<&'static str>) -> Vec<(String, usize)> {
    classes.sort_unstable();
    let mut out: Vec<(String, usize)> = Vec::new();
    for class in classes {
        match out.last_mut() {
            Some((name, count)) if name == class => *count += 1,
            _ => out.push((class.to_string(), 1)),
        }
    }
    out
}

fn format_histogram(histogram: &[(String, usize)]) -> String {
    if histogram.is_empty() {
        return "-".to_string();
    }
    let mut out = String::new();
    for (i, (name, count)) in histogram.iter().enumerate() {
        if i > 0 {
            out.push(';');
        }
        let _ = write!(out, "{name}:{count}");
    }
    out
}

fn measure(store: &Store, body: BodyId) -> Result<Measured, String> {
    let kind = store
        .get(body)
        .map_err(|error| format!("body lookup: {error:?}"))?
        .kind;

    let fast_faults = check_body(store, body)
        .map_err(|error| format!("fast check: {error:?}"))?
        .len();
    let full = check_body_report(store, body, CheckLevel::Full)
        .map_err(|error| format!("full check: {error:?}"))?;

    let mut surface_classes = Vec::new();
    for face in store
        .faces_of_body(body)
        .map_err(|error| format!("faces: {error:?}"))?
    {
        let surface = store
            .get(store.get(face).map_err(|e| format!("face: {e:?}"))?.surface)
            .map_err(|error| format!("surface: {error:?}"))?;
        surface_classes.push(surface_class(surface));
    }

    let mut curve_classes = Vec::new();
    let mut edge_tolerances = Vec::new();
    for edge in store
        .edges_of_body(body)
        .map_err(|error| format!("edges: {error:?}"))?
    {
        let edge = store
            .get(edge)
            .map_err(|error| format!("edge: {error:?}"))?;
        let curve = match edge.curve {
            Some(curve) => Some(
                store
                    .get(curve)
                    .map_err(|error| format!("curve: {error:?}"))?,
            ),
            None => None,
        };
        curve_classes.push(curve_class(curve));
        if let Some(tolerance) = edge.tolerance {
            edge_tolerances.push(tolerance.value());
        }
    }

    let mut vertex_tolerances = Vec::new();
    for vertex in store
        .vertices_of_body(body)
        .map_err(|error| format!("vertices: {error:?}"))?
    {
        let vertex = store
            .get(vertex)
            .map_err(|error| format!("vertex: {error:?}"))?;
        if let Some(tolerance) = vertex.tolerance {
            vertex_tolerances.push(tolerance.value());
        }
    }
    edge_tolerances.sort_by(f64::total_cmp);
    vertex_tolerances.sort_by(f64::total_cmp);

    let (volume, watertight) = if kind == BodyKind::Solid {
        let policy = SessionPolicy::v1();
        let context = OperationContext::new(&policy, Tolerances::default())
            .expect("v1 oracle tessellation context is valid");
        let mesh: BodyMesh = tessellate_body_with_context(
            store,
            body,
            &TessOptions {
                chord_tol: CHORD_TOL,
                max_edge_len: None,
            },
            &context,
        )
        .expect("v1 body-tessellation policy is valid")
        .into_result()
        .map_err(|error| format!("tessellation: {error:?}"))?;
        (
            Some(signed_volume(&mesh)),
            Some(check_watertight(&mesh).is_empty()),
        )
    } else {
        (None, None)
    };

    Ok(Measured {
        kind,
        regions: store.count::<Region>(),
        shells: store.count::<Shell>(),
        faces: store.count::<ktopo::entity::Face>(),
        loops: store.count::<Loop>(),
        fins: store.count::<Fin>(),
        edges: store.count::<Edge>(),
        vertices: store.count::<Vertex>(),
        volume,
        watertight,
        fast_faults,
        full_outcome: full.outcome(),
        full_gaps: full.gaps.len(),
        surface_classes: histogram(surface_classes),
        curve_classes: histogram(curve_classes),
        edge_tolerances,
        vertex_tolerances,
    })
}

/// FNV-1a over the exact file bytes; the manifest identity for transport
/// integrity, matching the determinism suites' hash choice.
fn fnv64(bytes: &[u8]) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    for &byte in bytes {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}

fn append_manifest_row(
    manifest: &mut String,
    file_name: &str,
    probe: &str,
    exact_volume: Option<f64>,
    measured: &Measured,
    bytes: &[u8],
) {
    let volume_exact =
        exact_volume.map_or_else(|| "-".to_string(), |value| format!("{value:.12e}"));
    let volume_mesh = measured
        .volume
        .map_or_else(|| "-".to_string(), |value| format!("{value:.12e}"));
    let watertight = measured
        .watertight
        .map_or("-", |value| if value { "true" } else { "false" });
    let _ = writeln!(
        manifest,
        "{file_name}\t{kind}\t{probe}\t{regions}\t{shells}\t{faces}\t{loops}\t{fins}\
         \t{edges}\t{vertices}\t{volume_exact}\t{volume_mesh}\t{watertight}\
         \t{fast_faults}\t{full_outcome}\t{full_gaps}\t{bytes}\t{fnv:016x}",
        kind = kind_name(measured.kind),
        regions = measured.regions,
        shells = measured.shells,
        faces = measured.faces,
        loops = measured.loops,
        fins = measured.fins,
        edges = measured.edges,
        vertices = measured.vertices,
        fast_faults = measured.fast_faults,
        full_outcome = outcome_name(measured.full_outcome),
        full_gaps = measured.full_gaps,
        bytes = bytes.len(),
        fnv = fnv64(bytes),
    );
}

fn export_bundle(dir: &Path) -> Result<(), String> {
    std::fs::create_dir_all(dir).map_err(|error| format!("creating {}: {error}", dir.display()))?;

    let mut manifest = String::from(
        "file\tbody_kind\tprobe\tregions\tshells\tfaces\tloops\tfins\tedges\tvertices\
         \tvolume_exact_m3\tvolume_mesh_m3\twatertight\tfast_faults\tfull_outcome\
         \tfull_gaps\tbytes\tfnv64\n",
    );

    for fixture in FIXTURES {
        let mut store = Store::new();
        let body = (fixture.build)(&mut store);
        let measured = measure(&store, body)
            .map_err(|error| format!("{}: measuring source body: {error}", fixture.name))?;
        if measured.fast_faults != 0 {
            return Err(format!(
                "{}: source body is not checker-clean",
                fixture.name
            ));
        }
        if measured.watertight == Some(false) {
            return Err(format!("{}: source solid is not watertight", fixture.name));
        }
        match (fixture.exact_volume, measured.volume) {
            (Some(exact), Some(mesh)) => {
                if (exact - mesh).abs() > EXACT_VOLUME_REL_TOL * exact {
                    return Err(format!(
                        "{}: tessellated volume {mesh} is not within {EXACT_VOLUME_REL_TOL:e} \
                         of closed form {exact}",
                        fixture.name
                    ));
                }
            }
            (None, None) => {}
            (exact, mesh) => {
                return Err(format!(
                    "{}: exact/mesh volume availability disagrees: {exact:?} vs {mesh:?}",
                    fixture.name
                ));
            }
        }

        let text = kxt::export_text(&store, body)
            .map_err(|error| format!("{}: export: {error:?}", fixture.name))?;

        // Never hand a host a file our own pipeline rejects: re-import and
        // re-verify before writing anything.
        let mut imported = Store::new();
        let recon = kxt::import(text.as_bytes(), &mut imported)
            .map_err(|error| format!("{}: self re-import: {error:?}", fixture.name))?;
        if recon.bodies.len() != 1 {
            return Err(format!(
                "{}: self re-import produced {} bodies",
                fixture.name,
                recon.bodies.len()
            ));
        }
        let reimported = measure(&imported, recon.bodies[0])
            .map_err(|error| format!("{}: measuring re-import: {error}", fixture.name))?;
        if reimported.fast_faults != 0 {
            return Err(format!("{}: re-import is not checker-clean", fixture.name));
        }
        if let (Some(a), Some(b)) = (measured.volume, reimported.volume)
            && (a - b).abs() > MESH_VOLUME_REL_TOL * a.abs()
        {
            return Err(format!(
                "{}: self round-trip mesh volume drifted beyond the discretization \
                 bound: {a} vs {b}",
                fixture.name
            ));
        }

        let file_name = format!("{}.x_t", fixture.name);
        let path = dir.join(&file_name);
        std::fs::write(&path, text.as_bytes())
            .map_err(|error| format!("writing {}: {error}", path.display()))?;

        append_manifest_row(
            &mut manifest,
            &file_name,
            fixture.probe,
            fixture.exact_volume,
            &measured,
            text.as_bytes(),
        );
        println!("wrote {}", path.display());
    }

    // OFFSET_SURF is the first procedural writer capability. Keep its exact
    // host-certified canonical bytes in the authoritative bundle even though
    // its topology constructor is not part of the Tier-1 make fixture table.
    let mut offset_store = Store::new();
    let offset_recon = kxt::import(OFFSET_PLANE_BYTES, &mut offset_store)
        .map_err(|error| format!("offset_plane: self import: {error:?}"))?;
    if offset_recon.bodies.len() != 1 {
        return Err(format!(
            "offset_plane: self import produced {} bodies",
            offset_recon.bodies.len()
        ));
    }
    let offset_body = offset_recon.bodies[0];
    let offset_measured = measure(&offset_store, offset_body)
        .map_err(|error| format!("offset_plane: measuring source body: {error}"))?;
    if offset_measured.fast_faults != 0 {
        return Err("offset_plane: source body is not checker-clean".to_string());
    }
    let offset_text = kxt::export_text(&offset_store, offset_body)
        .map_err(|error| format!("offset_plane: export: {error:?}"))?;
    if offset_text.as_bytes() != OFFSET_PLANE_BYTES {
        return Err(
            "offset_plane: committed canonical fixture is not writer-byte-stable".to_string(),
        );
    }
    let offset_name = "offset_plane.x_t";
    let offset_path = dir.join(offset_name);
    std::fs::write(&offset_path, OFFSET_PLANE_BYTES)
        .map_err(|error| format!("writing {}: {error}", offset_path.display()))?;
    append_manifest_row(
        &mut manifest,
        offset_name,
        "OFFSET_SURF with basis GEOMETRIC_OWNER ring",
        None,
        &offset_measured,
        OFFSET_PLANE_BYTES,
    );
    println!("wrote {}", offset_path.display());

    let manifest_path = dir.join("manifest.tsv");
    std::fs::write(&manifest_path, manifest.as_bytes())
        .map_err(|error| format!("writing {}: {error}", manifest_path.display()))?;
    println!("wrote {}", manifest_path.display());
    let expected: BTreeSet<String> = FIXTURES
        .iter()
        .map(|fixture| format!("{}.x_t", fixture.name))
        .chain([offset_name.to_string(), "manifest.tsv".to_string()])
        .collect();
    let actual: BTreeSet<String> = std::fs::read_dir(dir)
        .map_err(|error| format!("reading {}: {error}", dir.display()))?
        .map(|entry| {
            entry
                .map_err(|error| format!("reading {} entry: {error}", dir.display()))
                .map(|entry| entry.file_name().to_string_lossy().into_owned())
        })
        .collect::<Result<_, _>>()?;
    if actual != expected {
        return Err(format!(
            "{} contains stale or unexpected entries: expected {expected:?}, found {actual:?}",
            dir.display()
        ));
    }
    println!(
        "bundle complete: {} fixtures; next step is docs/oracle-loop.md",
        FIXTURES.len() + 1
    );
    Ok(())
}

// --------------------------------------------------------------------------
// Compare
// --------------------------------------------------------------------------

fn load(path: &Path) -> Result<Measured, String> {
    let bytes =
        std::fs::read(path).map_err(|error| format!("reading {}: {error}", path.display()))?;
    let mut store = Store::new();
    let recon = kxt::import(&bytes, &mut store)
        .map_err(|error| format!("importing {}: {error:?}", path.display()))?;
    let skipped: usize = recon.skipped.iter().map(|&(_, count)| count).sum();
    if skipped > 0 {
        println!(
            "note: {} skipped {skipped} non-geometric nodes on import",
            path.display()
        );
    }
    if recon.bodies.len() != 1 {
        return Err(format!(
            "{} reconstructed {} bodies; the oracle bundle is one body per file",
            path.display(),
            recon.bodies.len()
        ));
    }
    measure(&store, recon.bodies[0])
        .map_err(|error| format!("measuring {}: {error}", path.display()))
}

fn tolerances_match(ours: &[f64], theirs: &[f64]) -> bool {
    ours.len() == theirs.len()
        && ours
            .iter()
            .zip(theirs)
            .all(|(a, b)| (a - b).abs() <= TOLERANCE_ABS_TOL)
}

fn format_tolerances(values: &[f64]) -> String {
    if values.is_empty() {
        return "-".to_string();
    }
    let mut out = String::new();
    for (i, value) in values.iter().enumerate() {
        if i > 0 {
            out.push(';');
        }
        let _ = write!(out, "{value:.3e}");
    }
    out
}

fn compare_files(ours_path: &Path, theirs_path: &Path) -> Result<bool, String> {
    let ours = load(ours_path)?;
    let theirs = load(theirs_path)?;
    let mut failures = 0usize;

    let mut check = |field: &str, ok: bool, ours: String, theirs: String| {
        if ok {
            println!("PASS  {field}: {ours}");
        } else {
            println!("FAIL  {field}: ours={ours} theirs={theirs}");
            failures += 1;
        }
    };

    check(
        "body_kind",
        ours.kind == theirs.kind,
        kind_name(ours.kind).to_string(),
        kind_name(theirs.kind).to_string(),
    );
    for (field, a, b) in [
        ("regions", ours.regions, theirs.regions),
        ("shells", ours.shells, theirs.shells),
        ("faces", ours.faces, theirs.faces),
        ("loops", ours.loops, theirs.loops),
        ("fins", ours.fins, theirs.fins),
        ("edges", ours.edges, theirs.edges),
        ("vertices", ours.vertices, theirs.vertices),
    ] {
        check(field, a == b, a.to_string(), b.to_string());
    }
    check(
        "surface_classes",
        ours.surface_classes == theirs.surface_classes,
        format_histogram(&ours.surface_classes),
        format_histogram(&theirs.surface_classes),
    );
    check(
        "curve_classes",
        ours.curve_classes == theirs.curve_classes,
        format_histogram(&ours.curve_classes),
        format_histogram(&theirs.curve_classes),
    );
    check(
        "edge_tolerances",
        tolerances_match(&ours.edge_tolerances, &theirs.edge_tolerances),
        format_tolerances(&ours.edge_tolerances),
        format_tolerances(&theirs.edge_tolerances),
    );
    check(
        "vertex_tolerances",
        tolerances_match(&ours.vertex_tolerances, &theirs.vertex_tolerances),
        format_tolerances(&ours.vertex_tolerances),
        format_tolerances(&theirs.vertex_tolerances),
    );
    check(
        "fast_checker",
        theirs.fast_faults == 0,
        format!("{} faults", ours.fast_faults),
        format!("{} faults", theirs.fast_faults),
    );
    match (ours.watertight, theirs.watertight) {
        (Some(a), Some(b)) => check("watertight", b, a.to_string(), b.to_string()),
        (None, None) => {}
        (a, b) => check("watertight", false, format!("{a:?}"), format!("{b:?}")),
    }
    match (ours.volume, theirs.volume) {
        (Some(a), Some(b)) => check(
            "volume_mesh_m3",
            (a - b).abs() <= MESH_VOLUME_REL_TOL * a.abs(),
            format!("{a:.12e}"),
            format!("{b:.12e}"),
        ),
        (None, None) => {}
        (a, b) => check("volume_mesh_m3", false, format!("{a:?}"), format!("{b:?}")),
    }
    println!(
        "info  full_checker: ours={}({} gaps) theirs={}({} gaps)",
        outcome_name(ours.full_outcome),
        ours.full_gaps,
        outcome_name(theirs.full_outcome),
        theirs.full_gaps,
    );

    if failures == 0 {
        println!("COMPARE OK");
        Ok(true)
    } else {
        println!("COMPARE FAILED: {failures} mismatched fields");
        Ok(false)
    }
}
