//! M3b writer round trips for self-authored primitives and supported imports.

use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::curve2d::Line2d;
use kgeom::frame::Frame;
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::{Point2, Point3, Vec3};
use ktopo::btess::{TessOptions, check_watertight, tessellate_body};
use ktopo::check::check_body;
use ktopo::entity::{
    Body, BodyId, BodyKind, Edge, EdgeId, Face, FaceId, Fin, FinPcurve, Loop, ParamMap1d, Region,
    RegionKind, Sense, Shell, Vertex,
};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
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
    assert_eq!(store.count::<Point3>(), imported.count::<Point3>());

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
    assert_eq!(store.count::<Point3>(), imported.count::<Point3>());
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

    // The replacement surface uses normalized [0, 1]^2 parameters, so
    // replace the inherited plane-coordinate pcurves with exact normalized
    // ones while retaining each fin's independent Curve2d identity.
    let du = u_bounds[1] - u_bounds[0];
    let dv = v_bounds[1] - v_bounds[0];
    let fin_ids: Vec<_> = store
        .get(face_id)
        .unwrap()
        .loops
        .iter()
        .flat_map(|&loop_id| store.get(loop_id).unwrap().fins.iter().copied())
        .collect();
    for fin_id in fin_ids {
        let fin = store.get(fin_id).unwrap();
        let edge = store.get(fin.edge).unwrap();
        let [Some(start_id), Some(end_id)] = edge.vertices else {
            unreachable!()
        };
        let Some((t0, t1)) = edge.bounds else {
            unreachable!()
        };
        let to_uv = |point: Point3| {
            let local = plane.frame().to_local(point);
            Point2::new((local.x - u_bounds[0]) / du, (local.y - v_bounds[0]) / dv)
        };
        let start = to_uv(store.vertex_position(start_id).unwrap());
        let end = to_uv(store.vertex_position(end_id).unwrap());
        let uv_len = (end - start).norm();
        let pcurve_id = fin.pcurve.unwrap().curve();
        *store.get_mut(pcurve_id).unwrap() =
            Curve2dGeom::Line(Line2d::new(start, end - start).unwrap());
        let scale = uv_len / (t1 - t0);
        let map = ParamMap1d::affine(scale, -scale * t0).unwrap();
        store.get_mut(fin_id).unwrap().pcurve =
            Some(FinPcurve::new(pcurve_id, ParamRange::new(0.0, uv_len), map).unwrap());
    }
}

fn sheet_square(store: &mut Store) -> BodyId {
    let body = store.add(Body {
        kind: BodyKind::Sheet,
        regions: Vec::new(),
    });
    let region = store.add(Region {
        body,
        kind: RegionKind::Void,
        shells: Vec::new(),
    });
    store.get_mut(body).unwrap().regions.push(region);
    let shell = store.add(Shell {
        region,
        faces: Vec::new(),
        edges: Vec::new(),
        vertex: None,
    });
    store.get_mut(region).unwrap().shells.push(shell);

    let surface = store.add(SurfaceGeom::Plane(Plane::new(Frame::world())));
    let face = store.add(Face {
        shell,
        loops: Vec::new(),
        surface,
        sense: Sense::Forward,
    });
    store.get_mut(shell).unwrap().faces.push(face);
    let loop_id = store.add(Loop {
        face,
        fins: Vec::new(),
    });
    store.get_mut(face).unwrap().loops.push(loop_id);

    let corners = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    let vertices = corners.map(|point| {
        let point = store.add(point);
        store.add(Vertex {
            point,
            tolerance: None,
        })
    });
    for i in 0..corners.len() {
        let start = corners[i];
        let end = corners[(i + 1) % corners.len()];
        let curve = store.add(CurveGeom::Line(Line::new(start, end - start).unwrap()));
        let edge = store.add(Edge {
            curve: Some(curve),
            vertices: [Some(vertices[i]), Some(vertices[(i + 1) % vertices.len()])],
            bounds: Some((0.0, (end - start).norm())),
            fins: Vec::new(),
            tolerance: None,
        });
        let fin = store.add(Fin {
            parent: loop_id,
            edge,
            sense: Sense::Forward,
            pcurve: None,
        });
        store.get_mut(loop_id).unwrap().fins.push(fin);
        store.get_mut(edge).unwrap().fins.push(fin);
    }
    body
}

fn sheet_semicircle(store: &mut Store) -> BodyId {
    let body = store.add(Body {
        kind: BodyKind::Sheet,
        regions: Vec::new(),
    });
    let region = store.add(Region {
        body,
        kind: RegionKind::Void,
        shells: Vec::new(),
    });
    store.get_mut(body).unwrap().regions.push(region);
    let shell = store.add(Shell {
        region,
        faces: Vec::new(),
        edges: Vec::new(),
        vertex: None,
    });
    store.get_mut(region).unwrap().shells.push(shell);

    let surface = store.add(SurfaceGeom::Plane(Plane::new(Frame::world())));
    let face = store.add(Face {
        shell,
        loops: Vec::new(),
        surface,
        sense: Sense::Forward,
    });
    store.get_mut(shell).unwrap().faces.push(face);
    let loop_id = store.add(Loop {
        face,
        fins: Vec::new(),
    });
    store.get_mut(face).unwrap().loops.push(loop_id);

    let right = Point3::new(1.0, 0.0, 0.0);
    let left = Point3::new(-1.0, 0.0, 0.0);
    let vertices = [right, left].map(|point| {
        let point = store.add(point);
        store.add(Vertex {
            point,
            tolerance: None,
        })
    });

    let circle = store.add(CurveGeom::Circle(Circle::new(Frame::world(), 1.0).unwrap()));
    let arc = store.add(Edge {
        curve: Some(circle),
        vertices: [Some(vertices[0]), Some(vertices[1])],
        bounds: Some((0.0, core::f64::consts::PI)),
        fins: Vec::new(),
        tolerance: None,
    });
    let arc_fin = store.add(Fin {
        parent: loop_id,
        edge: arc,
        sense: Sense::Forward,
        pcurve: None,
    });
    store.get_mut(loop_id).unwrap().fins.push(arc_fin);
    store.get_mut(arc).unwrap().fins.push(arc_fin);

    let line = store.add(CurveGeom::Line(Line::new(left, right - left).unwrap()));
    let chord = store.add(Edge {
        curve: Some(line),
        vertices: [Some(vertices[1]), Some(vertices[0])],
        bounds: Some((0.0, (right - left).norm())),
        fins: Vec::new(),
        tolerance: None,
    });
    let chord_fin = store.add(Fin {
        parent: loop_id,
        edge: chord,
        sense: Sense::Forward,
        pcurve: None,
    });
    store.get_mut(loop_id).unwrap().fins.push(chord_fin);
    store.get_mut(chord).unwrap().fins.push(chord_fin);
    body
}

fn sheet_two_faces_shared_surface(store: &mut Store) -> BodyId {
    let body = store.add(Body {
        kind: BodyKind::Sheet,
        regions: Vec::new(),
    });
    let region = store.add(Region {
        body,
        kind: RegionKind::Void,
        shells: Vec::new(),
    });
    store.get_mut(body).unwrap().regions.push(region);
    let shell = store.add(Shell {
        region,
        faces: Vec::new(),
        edges: Vec::new(),
        vertex: None,
    });
    store.get_mut(region).unwrap().shells.push(shell);

    let surface = store.add(SurfaceGeom::Plane(Plane::new(Frame::world())));
    for x0 in [0.0, 2.0] {
        let face = store.add(Face {
            shell,
            loops: Vec::new(),
            surface,
            sense: Sense::Forward,
        });
        store.get_mut(shell).unwrap().faces.push(face);
        let loop_id = store.add(Loop {
            face,
            fins: Vec::new(),
        });
        store.get_mut(face).unwrap().loops.push(loop_id);

        let corners = [
            Point3::new(x0, 0.0, 0.0),
            Point3::new(x0 + 1.0, 0.0, 0.0),
            Point3::new(x0 + 1.0, 1.0, 0.0),
            Point3::new(x0, 1.0, 0.0),
        ];
        let vertices = corners.map(|point| {
            let point = store.add(point);
            store.add(Vertex {
                point,
                tolerance: None,
            })
        });
        for i in 0..corners.len() {
            let start = corners[i];
            let end = corners[(i + 1) % corners.len()];
            let curve = store.add(CurveGeom::Line(Line::new(start, end - start).unwrap()));
            let edge = store.add(Edge {
                curve: Some(curve),
                vertices: [Some(vertices[i]), Some(vertices[(i + 1) % vertices.len()])],
                bounds: Some((0.0, (end - start).norm())),
                fins: Vec::new(),
                tolerance: None,
            });
            let fin = store.add(Fin {
                parent: loop_id,
                edge,
                sense: Sense::Forward,
                pcurve: None,
            });
            store.get_mut(loop_id).unwrap().fins.push(fin);
            store.get_mut(edge).unwrap().fins.push(fin);
        }
    }
    body
}

fn wire_line(store: &mut Store) -> BodyId {
    let body = store.add(Body {
        kind: BodyKind::Wire,
        regions: Vec::new(),
    });
    let region = store.add(Region {
        body,
        kind: RegionKind::Void,
        shells: Vec::new(),
    });
    store.get_mut(body).unwrap().regions.push(region);
    let shell = store.add(Shell {
        region,
        faces: Vec::new(),
        edges: Vec::new(),
        vertex: None,
    });
    store.get_mut(region).unwrap().shells.push(shell);

    let start = Point3::new(0.0, 0.0, 0.0);
    let end = Point3::new(1.25, 0.0, 0.0);
    let vertices = [start, end].map(|point| {
        let point = store.add(point);
        store.add(Vertex {
            point,
            tolerance: None,
        })
    });
    let curve = store.add(CurveGeom::Line(Line::new(start, end - start).unwrap()));
    let edge = store.add(Edge {
        curve: Some(curve),
        vertices: [Some(vertices[0]), Some(vertices[1])],
        bounds: Some((0.0, (end - start).norm())),
        fins: Vec::new(),
        tolerance: None,
    });
    store.get_mut(shell).unwrap().edges.push(edge);
    body
}

fn wire_shared_line_segments(store: &mut Store) -> BodyId {
    let body = store.add(Body {
        kind: BodyKind::Wire,
        regions: Vec::new(),
    });
    let region = store.add(Region {
        body,
        kind: RegionKind::Void,
        shells: Vec::new(),
    });
    store.get_mut(body).unwrap().regions.push(region);
    let shell = store.add(Shell {
        region,
        faces: Vec::new(),
        edges: Vec::new(),
        vertex: None,
    });
    store.get_mut(region).unwrap().shells.push(shell);

    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    ];
    let vertices = points.map(|point| {
        let point = store.add(point);
        store.add(Vertex {
            point,
            tolerance: None,
        })
    });
    let curve = store.add(CurveGeom::Line(
        Line::new(points[0], Vec3::new(1.0, 0.0, 0.0)).unwrap(),
    ));
    for i in 0..2 {
        let edge = store.add(Edge {
            curve: Some(curve),
            vertices: [Some(vertices[i]), Some(vertices[i + 1])],
            bounds: Some((i as f64, i as f64 + 1.0)),
            fins: Vec::new(),
            tolerance: None,
        });
        store.get_mut(shell).unwrap().edges.push(edge);
    }
    body
}

fn wire_shared_point_vertices(store: &mut Store) -> BodyId {
    let body = store.add(Body {
        kind: BodyKind::Wire,
        regions: Vec::new(),
    });
    let region = store.add(Region {
        body,
        kind: RegionKind::Void,
        shells: Vec::new(),
    });
    store.get_mut(body).unwrap().regions.push(region);
    let shell = store.add(Shell {
        region,
        faces: Vec::new(),
        edges: Vec::new(),
        vertex: None,
    });
    store.get_mut(region).unwrap().shells.push(shell);

    let coords = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
    ];
    let points = coords.map(|point| store.add(point));
    let vertices = [
        store.add(Vertex {
            point: points[0],
            tolerance: None,
        }),
        store.add(Vertex {
            point: points[1],
            tolerance: None,
        }),
        store.add(Vertex {
            point: points[1],
            tolerance: None,
        }),
        store.add(Vertex {
            point: points[2],
            tolerance: None,
        }),
    ];

    let segments = [
        (coords[0], coords[1], vertices[0], vertices[1]),
        (coords[1], coords[2], vertices[2], vertices[3]),
    ];
    for (start, end, start_vertex, end_vertex) in segments {
        let curve = store.add(CurveGeom::Line(Line::new(start, end - start).unwrap()));
        let edge = store.add(Edge {
            curve: Some(curve),
            vertices: [Some(start_vertex), Some(end_vertex)],
            bounds: Some((0.0, (end - start).norm())),
            fins: Vec::new(),
            tolerance: None,
        });
        store.get_mut(shell).unwrap().edges.push(edge);
    }
    body
}

fn wire_ellipse_arc(store: &mut Store) -> BodyId {
    let body = store.add(Body {
        kind: BodyKind::Wire,
        regions: Vec::new(),
    });
    let region = store.add(Region {
        body,
        kind: RegionKind::Void,
        shells: Vec::new(),
    });
    store.get_mut(body).unwrap().regions.push(region);
    let shell = store.add(Shell {
        region,
        faces: Vec::new(),
        edges: Vec::new(),
        vertex: None,
    });
    store.get_mut(region).unwrap().shells.push(shell);

    let start = Point3::new(2.0, 0.0, 0.0);
    let end = Point3::new(0.0, 1.0, 0.0);
    let vertices = [start, end].map(|point| {
        let point = store.add(point);
        store.add(Vertex {
            point,
            tolerance: None,
        })
    });
    let curve = store.add(CurveGeom::Ellipse(
        Ellipse::new(Frame::world(), 2.0, 1.0).unwrap(),
    ));
    let edge = store.add(Edge {
        curve: Some(curve),
        vertices: [Some(vertices[0]), Some(vertices[1])],
        bounds: Some((0.0, core::f64::consts::FRAC_PI_2)),
        fins: Vec::new(),
        tolerance: None,
    });
    store.get_mut(shell).unwrap().edges.push(edge);
    body
}

fn acorn_point(store: &mut Store) -> BodyId {
    let body = store.add(Body {
        kind: BodyKind::Acorn,
        regions: Vec::new(),
    });
    let region = store.add(Region {
        body,
        kind: RegionKind::Void,
        shells: Vec::new(),
    });
    store.get_mut(body).unwrap().regions.push(region);

    let point = store.add(Point3::new(0.25, -0.5, 1.5));
    let vertex = store.add(Vertex {
        point,
        tolerance: None,
    });
    let shell = store.add(Shell {
        region,
        faces: Vec::new(),
        edges: Vec::new(),
        vertex: Some(vertex),
    });
    store.get_mut(region).unwrap().shells.push(shell);
    body
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
fn exact_tolerant_edge_and_vertex_round_trip() {
    let mut store = Store::new();
    let body = make::block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
    let edge = first_bounded_edge(&store, body);
    let vertex = store.get(edge).unwrap().vertices[0].unwrap();
    store.get_mut(edge).unwrap().tolerance = Some(LINEAR_RESOLUTION * 10.0);
    store.get_mut(vertex).unwrap().tolerance = Some(LINEAR_RESOLUTION * 20.0);

    let (text, imported, imported_body) = assert_checker_roundtrip(&store, body);
    let parsed = kxt::read_xt(text.as_bytes()).unwrap();
    assert!(
        parsed
            .nodes
            .values()
            .filter(|node| node.code == code::EDGE)
            .any(|node| matches!(parsed.field(node, "tolerance").unwrap().as_f64(), Some(t) if t == LINEAR_RESOLUTION * 10.0))
    );
    assert!(
        parsed
            .nodes
            .values()
            .filter(|node| node.code == code::VERTEX)
            .any(|node| matches!(parsed.field(node, "tolerance").unwrap().as_f64(), Some(t) if t == LINEAR_RESOLUTION * 20.0))
    );
    assert!(
        imported
            .edges_of_body(imported_body)
            .unwrap()
            .into_iter()
            .any(|edge| imported.get(edge).unwrap().tolerance == Some(LINEAR_RESOLUTION * 10.0))
    );
    assert!(
        imported
            .vertices_of_body(imported_body)
            .unwrap()
            .into_iter()
            .any(|vertex| imported.get(vertex).unwrap().tolerance == Some(LINEAR_RESOLUTION * 20.0))
    );
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
            .any(|node| node.code == code::TRIMMED_CURVE)
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
fn sheet_square_boundary_edges_round_trip_with_dummy_fins() {
    let mut store = Store::new();
    let body = sheet_square(&mut store);

    let (text, imported, imported_body) = assert_checker_roundtrip(&store, body);
    let parsed = kxt::read_xt(text.as_bytes()).unwrap();
    let fin_nodes = parsed
        .nodes
        .values()
        .filter(|node| node.code == code::FIN)
        .count();
    assert_eq!(fin_nodes, 8, "four real FINs plus four dummy FINs");
    assert_eq!(imported.get(imported_body).unwrap().kind, BodyKind::Sheet);
    assert_eq!(imported.faces_of_body(imported_body).unwrap().len(), 1);
    let edges = imported.edges_of_body(imported_body).unwrap();
    assert_eq!(edges.len(), 4);
    for edge in edges {
        let edge = imported.get(edge).unwrap();
        assert_eq!(edge.fins.len(), 1);
        assert!(edge.vertices[0].is_some());
        assert!(edge.vertices[1].is_some());
        assert!(edge.bounds.is_some());
    }
}

#[test]
fn sheet_semicircle_arc_round_trips() {
    let mut store = Store::new();
    let body = sheet_semicircle(&mut store);

    let (text, imported, imported_body) = assert_checker_roundtrip(&store, body);
    let parsed = kxt::read_xt(text.as_bytes()).unwrap();
    assert!(parsed.nodes.values().any(|node| node.code == code::CIRCLE));
    assert!(
        parsed
            .nodes
            .values()
            .any(|node| node.code == code::TRIMMED_CURVE)
    );
    let edges = imported.edges_of_body(imported_body).unwrap();
    assert_eq!(edges.len(), 2);
    let arc = edges
        .into_iter()
        .find(|&edge| {
            let curve = imported.get(edge).unwrap().curve.unwrap();
            matches!(imported.get(curve).unwrap(), CurveGeom::Circle(_))
        })
        .expect("round-tripped circle edge");
    let arc = imported.get(arc).unwrap();
    assert_eq!(arc.fins.len(), 1);
    assert!(arc.vertices[0].is_some());
    assert!(arc.vertices[1].is_some());
    let (lo, hi) = arc.bounds.unwrap();
    assert!((lo - 0.0).abs() < 1e-12);
    assert!((hi - core::f64::consts::PI).abs() < 1e-12);
}

#[test]
fn sheet_faces_can_share_a_surface_node() {
    let mut store = Store::new();
    let body = sheet_two_faces_shared_surface(&mut store);

    let (text, imported, imported_body) = assert_checker_roundtrip(&store, body);
    let parsed = kxt::read_xt(text.as_bytes()).unwrap();
    let plane_nodes = parsed
        .nodes
        .values()
        .filter(|node| node.code == code::PLANE)
        .count();
    assert_eq!(plane_nodes, 1, "shared plane should be emitted once");
    assert_eq!(imported.faces_of_body(imported_body).unwrap().len(), 2);
    assert_eq!(
        imported
            .iter::<SurfaceGeom>()
            .filter(|(_, surface)| matches!(surface, SurfaceGeom::Plane(_)))
            .count(),
        1
    );
}

#[test]
fn wire_line_round_trips_with_dummy_fins() {
    let mut store = Store::new();
    let body = wire_line(&mut store);

    let (text, imported, imported_body) = assert_checker_roundtrip(&store, body);
    let parsed = kxt::read_xt(text.as_bytes()).unwrap();
    let body_node = parsed.node(1).unwrap();
    assert_eq!(
        parsed.field(body_node, "body_type").unwrap().as_int(),
        Some(2)
    );
    let fin_nodes = parsed
        .nodes
        .values()
        .filter(|node| node.code == code::FIN)
        .count();
    assert_eq!(fin_nodes, 2, "wire edge start/end dummy FINs");
    assert_eq!(imported.get(imported_body).unwrap().kind, BodyKind::Wire);
    let edges = imported.edges_of_body(imported_body).unwrap();
    assert_eq!(edges.len(), 1);
    let edge = imported.get(edges[0]).unwrap();
    assert!(edge.fins.is_empty());
    assert!(edge.vertices[0].is_some());
    assert!(edge.vertices[1].is_some());
    assert!(edge.bounds.is_some());
}

#[test]
fn wire_edges_can_share_a_basis_curve() {
    let mut store = Store::new();
    let body = wire_shared_line_segments(&mut store);

    let (text, imported, imported_body) = assert_checker_roundtrip(&store, body);
    let parsed = kxt::read_xt(text.as_bytes()).unwrap();
    let line_nodes = parsed
        .nodes
        .values()
        .filter(|node| node.code == code::LINE)
        .count();
    let trim_nodes = parsed
        .nodes
        .values()
        .filter(|node| node.code == code::TRIMMED_CURVE)
        .count();
    assert_eq!(line_nodes, 1, "shared basis line should be emitted once");
    assert_eq!(trim_nodes, 2, "each bounded edge gets its own trim");
    assert_eq!(imported.edges_of_body(imported_body).unwrap().len(), 2);
    assert_eq!(
        imported
            .iter::<CurveGeom>()
            .filter(|(_, curve)| matches!(curve, CurveGeom::Line(_)))
            .count(),
        1
    );
}

#[test]
fn wire_vertices_can_share_a_point_node() {
    let mut store = Store::new();
    let body = wire_shared_point_vertices(&mut store);
    assert_eq!(store.count::<Vertex>(), 4);
    assert_eq!(store.count::<Point3>(), 3);

    let (text, imported, imported_body) = assert_checker_roundtrip(&store, body);
    let parsed = kxt::read_xt(text.as_bytes()).unwrap();
    let point_nodes = parsed
        .nodes
        .values()
        .filter(|node| node.code == code::POINT)
        .count();
    assert_eq!(point_nodes, 3, "shared point should be emitted once");
    assert_eq!(imported.edges_of_body(imported_body).unwrap().len(), 2);
    assert_eq!(imported.count::<Vertex>(), 4);
    assert_eq!(imported.count::<Point3>(), 3);
}

#[test]
fn wire_ellipse_arc_round_trips() {
    let mut store = Store::new();
    let body = wire_ellipse_arc(&mut store);

    let (text, imported, imported_body) = assert_checker_roundtrip(&store, body);
    let parsed = kxt::read_xt(text.as_bytes()).unwrap();
    assert!(parsed.nodes.values().any(|node| node.code == code::ELLIPSE));
    assert!(
        parsed
            .nodes
            .values()
            .any(|node| node.code == code::TRIMMED_CURVE)
    );
    let edges = imported.edges_of_body(imported_body).unwrap();
    assert_eq!(edges.len(), 1);
    let edge = imported.get(edges[0]).unwrap();
    let curve = edge.curve.unwrap();
    assert!(matches!(
        imported.get(curve).unwrap(),
        CurveGeom::Ellipse(_)
    ));
    assert!(edge.fins.is_empty());
    assert!(edge.vertices[0].is_some());
    assert!(edge.vertices[1].is_some());
    let (lo, hi) = edge.bounds.unwrap();
    assert!((lo - 0.0).abs() < 1e-12);
    assert!((hi - core::f64::consts::FRAC_PI_2).abs() < 1e-12);
}

#[test]
fn acorn_point_round_trips() {
    let mut store = Store::new();
    let body = acorn_point(&mut store);

    let (text, imported, imported_body) = assert_checker_roundtrip(&store, body);
    let parsed = kxt::read_xt(text.as_bytes()).unwrap();
    let body_node = parsed.node(1).unwrap();
    assert_eq!(
        parsed.field(body_node, "body_type").unwrap().as_int(),
        Some(2)
    );
    assert_eq!(imported.get(imported_body).unwrap().kind, BodyKind::Acorn);
    assert!(imported.faces_of_body(imported_body).unwrap().is_empty());
    assert!(imported.edges_of_body(imported_body).unwrap().is_empty());
    let vertices = imported.vertices_of_body(imported_body).unwrap();
    assert_eq!(vertices.len(), 1);
    let point = imported.vertex_position(vertices[0]).unwrap();
    assert_eq!(point, Point3::new(0.25, -0.5, 1.5));
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
