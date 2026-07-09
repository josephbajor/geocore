//! M3b writer round trips for self-authored primitives and supported imports.

use kgeom::curve::Curve;
use kgeom::frame::Frame;
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::vec::{Point3, Vec3};
use ktopo::btess::{TessOptions, check_watertight, tessellate_body};
use ktopo::check::check_body;
use ktopo::entity::{BodyId, BodyKind, Edge, EdgeId, Face, FaceId, Vertex};
use ktopo::geom::{CurveGeom, SurfaceGeom};
use ktopo::make;
use ktopo::store::Store;
use kxt::schema::code;

fn tilted() -> Frame {
    Frame::new(
        Point3::new(0.3, -1.2, 2.1),
        Vec3::new(1.0, 2.0, 3.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap()
}

fn fixture(name: &str) -> Vec<u8> {
    let path = format!("{}/tests/fixtures/{name}", env!("CARGO_MANIFEST_DIR"));
    std::fs::read(&path).unwrap_or_else(|error| panic!("reading fixture {path}: {error}"))
}

fn assert_roundtrip(store: &Store, body: BodyId) {
    let text = kxt::export_text(store, body).unwrap();
    assert_eq!(text, kxt::export_text(store, body).unwrap());
    let parsed = kxt::read_xt(text.as_bytes()).unwrap();
    assert_eq!(parsed.schema, "SCH_1300000_13006");
    assert_eq!(parsed.usfld_size, 0);

    let mut imported = Store::new();
    let recon = kxt::import(text.as_bytes(), &mut imported).unwrap();
    assert_eq!(recon.bodies.len(), 1);
    let imported_body = recon.bodies[0];
    let faults = check_body(&imported, imported_body).unwrap();
    assert!(faults.is_empty(), "round-trip checker faults: {faults:?}");
    assert_eq!(store.count::<Face>(), imported.count::<Face>());
    assert_eq!(store.count::<Edge>(), imported.count::<Edge>());
    assert_eq!(store.count::<Vertex>(), imported.count::<Vertex>());

    let mesh = tessellate_body(
        &imported,
        imported_body,
        &TessOptions {
            chord_tol: 1e-3,
            max_edge_len: None,
        },
    )
    .unwrap();
    assert!(check_watertight(&mesh).is_empty());
}

fn assert_checker_roundtrip(store: &Store, body: BodyId) -> (String, Store, BodyId) {
    let faults = check_body(store, body).unwrap();
    assert!(faults.is_empty(), "source checker faults: {faults:?}");

    let text = kxt::export_text(store, body).unwrap();
    assert_eq!(text, kxt::export_text(store, body).unwrap());
    let parsed = kxt::read_xt(text.as_bytes()).unwrap();
    assert_eq!(parsed.schema, "SCH_1300000_13006");

    let mut imported = Store::new();
    let recon = kxt::import(text.as_bytes(), &mut imported).unwrap();
    assert_eq!(recon.bodies.len(), 1);
    let imported_body = recon.bodies[0];
    let faults = check_body(&imported, imported_body).unwrap();
    assert!(faults.is_empty(), "round-trip checker faults: {faults:?}");
    assert_eq!(store.count::<Face>(), imported.count::<Face>());
    assert_eq!(store.count::<Edge>(), imported.count::<Edge>());
    assert_eq!(store.count::<Vertex>(), imported.count::<Vertex>());
    (text, imported, imported_body)
}

fn first_bounded_edge(store: &Store, body: BodyId) -> EdgeId {
    store
        .edges_of_body(body)
        .unwrap()
        .into_iter()
        .find(|&edge| {
            let edge = store.get(edge).unwrap();
            edge.bounds.is_some() && edge.vertices[0].is_some() && edge.vertices[1].is_some()
        })
        .unwrap()
}

fn first_plane_face(store: &Store, body: BodyId) -> FaceId {
    store
        .faces_of_body(body)
        .unwrap()
        .into_iter()
        .find(|&face| {
            let surface = store.get(face).unwrap().surface;
            matches!(store.get(surface).unwrap(), SurfaceGeom::Plane(_))
        })
        .unwrap()
}

fn replace_edge_with_linear_nurbs(store: &mut Store, body: BodyId) {
    let edge_id = first_bounded_edge(store, body);
    let edge = store.get(edge_id).unwrap();
    let curve_id = edge.curve.unwrap();
    let start = store.vertex_position(edge.vertices[0].unwrap()).unwrap();
    let end = store.vertex_position(edge.vertices[1].unwrap()).unwrap();
    let nurbs = NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![start, end], None).unwrap();
    *store.get_mut(curve_id).unwrap() = CurveGeom::Nurbs(nurbs);
    store.get_mut(edge_id).unwrap().bounds = Some((0.0, 1.0));
}

fn replace_face_with_bilinear_nurbs(store: &mut Store, body: BodyId) {
    let face_id = first_plane_face(store, body);
    let surface_id = store.get(face_id).unwrap().surface;
    let plane = match store.get(surface_id).unwrap() {
        SurfaceGeom::Plane(plane) => *plane,
        _ => unreachable!(),
    };

    let mut u_bounds = [f64::INFINITY, f64::NEG_INFINITY];
    let mut v_bounds = [f64::INFINITY, f64::NEG_INFINITY];
    for &loop_id in &store.get(face_id).unwrap().loops {
        for &fin_id in &store.get(loop_id).unwrap().fins {
            let edge = store.get(store.get(fin_id).unwrap().edge).unwrap();
            for vertex in edge.vertices.into_iter().flatten() {
                let local = plane
                    .frame()
                    .to_local(store.vertex_position(vertex).unwrap());
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
    .unwrap();
    *store.get_mut(surface_id).unwrap() = SurfaceGeom::Nurbs(surface);
}

#[test]
fn all_analytic_primitives_round_trip() {
    let frame = tilted();
    let constructors: [fn(&mut Store, &Frame) -> BodyId; 6] = [
        |store, frame| make::block(store, frame, [0.4, 0.3, 0.2]).unwrap(),
        |store, frame| make::cylinder(store, frame, 0.2, 0.5).unwrap(),
        |store, frame| make::cone(store, frame, 0.2, 0.35, 0.5).unwrap(),
        |store, frame| make::cone(store, frame, 0.35, 0.2, 0.5).unwrap(),
        |store, frame| make::sphere(store, frame, 0.25).unwrap(),
        |store, frame| make::torus(store, frame, 0.4, 0.1).unwrap(),
    ];
    for constructor in constructors {
        let mut store = Store::new();
        let body = constructor(&mut store, &frame);
        assert_roundtrip(&store, body);
    }
}

#[test]
fn nurbs_curve_edge_round_trips_as_b_curve() {
    let mut store = Store::new();
    let body = make::block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
    replace_edge_with_linear_nurbs(&mut store, body);

    let (text, imported, imported_body) = assert_checker_roundtrip(&store, body);
    let parsed = kxt::read_xt(text.as_bytes()).unwrap();
    assert!(parsed.nodes.values().any(|node| node.code == code::B_CURVE));
    assert!(
        parsed
            .nodes
            .values()
            .any(|node| node.code == code::NURBS_CURVE)
    );
    assert!(
        parsed
            .nodes
            .values()
            .any(|node| node.code == code::BSPLINE_VERTICES)
    );
    assert!(
        parsed
            .nodes
            .values()
            .any(|node| node.code == code::KNOT_MULT)
    );
    assert!(
        parsed
            .nodes
            .values()
            .any(|node| node.code == code::KNOT_SET)
    );
    let nurbs: Vec<_> = imported
        .iter::<CurveGeom>()
        .filter_map(|(_, curve)| match curve {
            CurveGeom::Nurbs(curve) => Some(curve),
            _ => None,
        })
        .collect();
    assert_eq!(nurbs.len(), 1);
    assert_eq!(nurbs[0].degree(), 1);
    assert_eq!(nurbs[0].param_range().lo, 0.0);
    assert_eq!(nurbs[0].param_range().hi, 1.0);
    assert_eq!(imported.edges_of_body(imported_body).unwrap().len(), 12);
}

#[test]
fn nurbs_surface_face_round_trips_as_b_surface() {
    let mut store = Store::new();
    let body = make::block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
    replace_face_with_bilinear_nurbs(&mut store, body);

    let (text, imported, imported_body) = assert_checker_roundtrip(&store, body);
    let parsed = kxt::read_xt(text.as_bytes()).unwrap();
    assert!(
        parsed
            .nodes
            .values()
            .any(|node| node.code == code::B_SURFACE)
    );
    assert!(
        parsed
            .nodes
            .values()
            .any(|node| node.code == code::NURBS_SURF)
    );
    assert!(
        parsed
            .nodes
            .values()
            .any(|node| node.code == code::BSPLINE_VERTICES)
    );
    assert!(
        parsed
            .nodes
            .values()
            .any(|node| node.code == code::KNOT_MULT)
    );
    assert!(
        parsed
            .nodes
            .values()
            .any(|node| node.code == code::KNOT_SET)
    );
    let nurbs: Vec<_> = imported
        .iter::<SurfaceGeom>()
        .filter_map(|(_, surface)| match surface {
            SurfaceGeom::Nurbs(surface) => Some(surface),
            _ => None,
        })
        .collect();
    assert_eq!(nurbs.len(), 1);
    assert_eq!(nurbs[0].degree_u(), 1);
    assert_eq!(nurbs[0].degree_v(), 1);
    assert_eq!(nurbs[0].net_size(), (2, 2));
    assert_eq!(imported.faces_of_body(imported_body).unwrap().len(), 6);
}

#[test]
fn real_world_sheet_disk_round_trips_through_writer() {
    let mut store = Store::new();
    let recon = kxt::import(&fixture("disk_nat.x_t"), &mut store).unwrap();
    assert_eq!(recon.bodies.len(), 1);
    let body = recon.bodies[0];
    assert_eq!(store.get(body).unwrap().kind, BodyKind::Sheet);

    let text = kxt::export_text(&store, body).unwrap();
    let parsed = kxt::read_xt(text.as_bytes()).unwrap();
    let body_node = parsed.node(1).unwrap();
    assert_eq!(
        parsed.field(body_node, "body_type").unwrap().as_int(),
        Some(3)
    );
    let mut imported = Store::new();
    let recon = kxt::import(text.as_bytes(), &mut imported).unwrap();
    assert_eq!(recon.bodies.len(), 1);
    let imported_body = recon.bodies[0];
    let faults = check_body(&imported, imported_body).unwrap();
    assert!(faults.is_empty(), "round-trip checker faults: {faults:?}");
    assert_eq!(imported.get(imported_body).unwrap().kind, BodyKind::Sheet);
    assert_eq!(imported.faces_of_body(imported_body).unwrap().len(), 1);
    let edges = imported.edges_of_body(imported_body).unwrap();
    assert_eq!(edges.len(), 1);
    assert_eq!(imported.get(edges[0]).unwrap().fins.len(), 1);
}
