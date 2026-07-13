//! M3b writer round trips for self-authored primitives and supported imports.

use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::curve2d::{Line2d, NurbsCurve2d};
use kgeom::frame::Frame;
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Surface};
use kgeom::vec::{Point2, Point3, Vec2, Vec3};
use ktopo::btess::{TessOptions, check_watertight, tessellate_body};
use ktopo::check::check_body;
use ktopo::entity::{
    Body, BodyId, BodyKind, Edge, EdgeId, Face, FaceId, Fin, FinPcurve, Loop, ParamMap1d, Region,
    RegionKind, Sense, Shell, Vertex,
};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::make;
use ktopo::store::Store;
use ktopo::tolerance::{EntityTolerance, ToleranceOrigin};
use ktopo::transaction::AssemblyStore;
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
    assert_eq!(parsed.schema, "SCH_2700142_26105_13006");
    assert_eq!(parsed.usfld_size, 1);

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
    assert_eq!(parsed.schema, "SCH_2700142_26105_13006");

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

fn edit_body<R>(
    store: &mut Store,
    body: BodyId,
    edit: impl FnOnce(&mut AssemblyStore<'_>) -> R,
) -> R {
    let mut transaction = store.transaction().unwrap();
    let result = {
        let mut assembly = transaction.assembly();
        edit(&mut assembly)
    };
    transaction.commit_checked_body(body).unwrap();
    result
}

fn assemble_body(
    store: &mut Store,
    assemble: impl FnOnce(&mut AssemblyStore<'_>) -> BodyId,
) -> BodyId {
    let mut transaction = store.transaction().unwrap();
    let body = {
        let mut assembly = transaction.assembly();
        assemble(&mut assembly)
    };
    transaction.commit_checked_body(body).unwrap();
    body
}

fn replace_edge_with_linear_nurbs(store: &mut Store, body: BodyId) {
    edit_body(store, body, |store| {
        let edge_id = first_bounded_edge(store, body);
        let edge = store.get(edge_id).unwrap();
        let curve_id = edge.curve.unwrap();
        let start = store.vertex_position(edge.vertices[0].unwrap()).unwrap();
        let end = store.vertex_position(edge.vertices[1].unwrap()).unwrap();
        let nurbs = NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![start, end], None).unwrap();
        store
            .replace_curve(curve_id, CurveGeom::Nurbs(nurbs))
            .unwrap();
        store.get_mut(edge_id).unwrap().bounds = Some((0.0, 1.0));
    });
}

fn make_first_edge_truly_tolerant(store: &mut Store, body: BodyId) -> EdgeId {
    edit_body(store, body, |store| {
        let edge_id = first_bounded_edge(store, body);
        let edge = store.get(edge_id).unwrap();
        let old_bounds = edge.bounds.unwrap();
        let fins = edge.fins.clone();
        for fin_id in &fins {
            let old = store.get(*fin_id).unwrap().pcurve.unwrap();
            let q0 = old.parameter_at_edge(old_bounds.0);
            let q1 = old.parameter_at_edge(old_bounds.1);
            store.get_mut(*fin_id).unwrap().pcurve = Some(
                FinPcurve::new(
                    old.curve(),
                    old.range(),
                    ParamMap1d::affine(q1 - q0, q0).unwrap(),
                )
                .unwrap(),
            );
        }

        // Exercise rational 2D B-curve emission whose stored curve extends
        // well beyond the active SP-curve trim. Import/domain reconstruction
        // must use the exact active NURBS subrange rather than its global hull.
        let first = fins[0];
        let first_use = store.get(first).unwrap().pcurve.unwrap();
        let first_curve = store.get(first_use.curve()).unwrap().as_curve();
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
                    .unwrap(),
                ),
            )
            .unwrap();

        // Exercise decreasing SP-curve trim parameters on the other use while
        // preserving exactly the same lifted geometry.
        let second = fins[1];
        let second_use = store.get(second).unwrap().pcurve.unwrap();
        let Curve2dGeom::Line(line) = *store.get(second_use.curve()).unwrap() else {
            panic!("block pcurve must be linear");
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
                    .unwrap(),
                ),
            )
            .unwrap();
        let old_map = second_use.edge_to_pcurve();
        let reversed_map =
            ParamMap1d::affine(-old_map.scale(), range.lo + range.hi - old_map.offset()).unwrap();
        store.get_mut(second).unwrap().pcurve =
            Some(FinPcurve::new(second_use.curve(), second_use.range(), reversed_map).unwrap());

        let edge = store.get_mut(edge_id).unwrap();
        edge.curve = None;
        edge.bounds = Some((0.0, 1.0));
        edge.tolerance =
            Some(EntityTolerance::operation(LINEAR_RESOLUTION, "writer-test").unwrap());
        edge_id
    })
}

fn replace_face_with_bilinear_nurbs(store: &mut Store, body: BodyId) {
    edit_body(store, body, |store| {
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
        store
            .replace_surface(surface_id, SurfaceGeom::Nurbs(surface))
            .unwrap();
        store.get_mut(face_id).unwrap().domain =
            Some(ktopo::entity::FaceDomain::from_bounds(0.0, 1.0, 0.0, 1.0).unwrap());

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
            store
                .replace_pcurve(
                    pcurve_id,
                    Curve2dGeom::Line(Line2d::new(start, end - start).unwrap()),
                )
                .unwrap();
            let scale = uv_len / (t1 - t0);
            let map = ParamMap1d::affine(scale, -scale * t0).unwrap();
            store.get_mut(fin_id).unwrap().pcurve =
                Some(FinPcurve::new(pcurve_id, ParamRange::new(0.0, uv_len), map).unwrap());
        }
    });
}

fn replace_sheet_patch_with_periodic_rational_nurbs(store: &mut Store, body: BodyId) {
    let face_id = store.faces_of_body(body).unwrap()[0];
    let surface_id = store.get(face_id).unwrap().surface;
    let loop_id = store.get(face_id).unwrap().loops[0];
    let fins = store.get(loop_id).unwrap().fins.clone();
    let edges: Vec<_> = fins
        .iter()
        .map(|&fin| store.get(fin).unwrap().edge)
        .collect();
    let curves: Vec<_> = edges
        .iter()
        .map(|&edge| store.get(edge).unwrap().curve.unwrap())
        .collect();
    let vertices = [
        store.get(edges[0]).unwrap().vertices[0].unwrap(),
        store.get(edges[0]).unwrap().vertices[1].unwrap(),
        store.get(edges[1]).unwrap().vertices[1].unwrap(),
        store.get(edges[2]).unwrap().vertices[1].unwrap(),
    ];

    edit_body(store, body, |store| {
        let xy = [
            (Point3::new(1.0, 0.0, 0.0), 1.0),
            (Point3::new(2.0, 1.0, 0.0), 0.75),
            (Point3::new(0.4, -0.6, 0.0), 1.25),
            (Point3::new(1.0, 0.0, 0.0), 1.0),
        ];
        let knots_u = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let mut points = Vec::new();
        let mut weights = Vec::new();
        for (point, weight) in xy {
            for z in [0.0, 2.0] {
                points.push(Point3::new(point.x, point.y, z));
                weights.push(weight);
            }
        }
        let surface = NurbsSurface::new(
            3,
            1,
            knots_u.clone(),
            vec![0.0, 0.0, 2.0, 2.0],
            points,
            Some(weights),
        )
        .unwrap()
        .with_certified_periodicity([true, false], 0.0)
        .unwrap();
        let (u0, u1) = (0.15, 0.65);
        let positions = [
            surface.eval([u0, 0.0]),
            surface.eval([u1, 0.0]),
            surface.eval([u1, 2.0]),
            surface.eval([u0, 2.0]),
        ];
        for (vertex, position) in vertices.into_iter().zip(positions) {
            let point = store.get(vertex).unwrap().point;
            *store.get_mut(point).unwrap() = position;
        }
        store
            .replace_surface(surface_id, SurfaceGeom::Nurbs(surface.clone()))
            .unwrap();
        store.get_mut(face_id).unwrap().domain =
            Some(ktopo::entity::FaceDomain::from_bounds(u0, u1, 0.0, 2.0).unwrap());

        let iso_curve = |z: f64| {
            NurbsCurve::new(
                3,
                knots_u.clone(),
                xy.into_iter()
                    .map(|(point, _)| Point3::new(point.x, point.y, z))
                    .collect(),
                Some(xy.into_iter().map(|(_, weight)| weight).collect()),
            )
            .unwrap()
            .restricted_to(ParamRange::new(u0, u1))
            .unwrap()
        };
        store
            .replace_curve(curves[0], CurveGeom::Nurbs(iso_curve(0.0)))
            .unwrap();
        store
            .replace_curve(
                curves[1],
                CurveGeom::Line(Line::new(positions[1], positions[2] - positions[1]).unwrap()),
            )
            .unwrap();
        store
            .replace_curve(curves[2], CurveGeom::Nurbs(iso_curve(2.0)))
            .unwrap();
        store
            .replace_curve(
                curves[3],
                CurveGeom::Line(Line::new(positions[0], positions[3] - positions[0]).unwrap()),
            )
            .unwrap();
        for (edge, bounds) in
            edges
                .iter()
                .copied()
                .zip([(u0, u1), (0.0, 2.0), (u0, u1), (0.0, 2.0)])
        {
            store.get_mut(edge).unwrap().bounds = Some(bounds);
        }
        store.get_mut(edges[2]).unwrap().vertices = [Some(vertices[3]), Some(vertices[2])];
        store.get_mut(edges[3]).unwrap().vertices = [Some(vertices[0]), Some(vertices[3])];
        store.get_mut(fins[2]).unwrap().sense = Sense::Reversed;
        store.get_mut(fins[3]).unwrap().sense = Sense::Reversed;

        for (fin, origin, direction, range) in [
            (
                fins[0],
                Point2::new(0.0, 0.0),
                Vec2::new(1.0, 0.0),
                ParamRange::new(u0, u1),
            ),
            (
                fins[1],
                Point2::new(u1, 0.0),
                Vec2::new(0.0, 1.0),
                ParamRange::new(0.0, 2.0),
            ),
            (
                fins[2],
                Point2::new(0.0, 2.0),
                Vec2::new(1.0, 0.0),
                ParamRange::new(u0, u1),
            ),
            (
                fins[3],
                Point2::new(u0, 0.0),
                Vec2::new(0.0, 1.0),
                ParamRange::new(0.0, 2.0),
            ),
        ] {
            let curve = store.get(fin).unwrap().pcurve.unwrap().curve();
            store
                .replace_pcurve(
                    curve,
                    Curve2dGeom::Line(Line2d::new(origin, direction).unwrap()),
                )
                .unwrap();
            store.get_mut(fin).unwrap().pcurve =
                Some(FinPcurve::new(curve, range, ParamMap1d::identity()).unwrap());
        }
        let faults = check_body(store, body).unwrap();
        assert!(faults.is_empty(), "periodic patch edit faults: {faults:?}");
    });
}

fn sheet_square(store: &mut Store) -> BodyId {
    assemble_body(store, |store| {
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

        let surface = store
            .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        let face = store.add(Face {
            shell,
            loops: Vec::new(),
            surface,
            sense: Sense::Forward,
            domain: None,
            tolerance: None,
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
            let curve = store
                .insert_curve(CurveGeom::Line(Line::new(start, end - start).unwrap()))
                .unwrap();
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
    })
}

fn sheet_semicircle(store: &mut Store) -> BodyId {
    assemble_body(store, |store| {
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

        let surface = store
            .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        let face = store.add(Face {
            shell,
            loops: Vec::new(),
            surface,
            sense: Sense::Forward,
            domain: None,
            tolerance: None,
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

        let circle = store
            .insert_curve(CurveGeom::Circle(Circle::new(Frame::world(), 1.0).unwrap()))
            .unwrap();
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

        let line = store
            .insert_curve(CurveGeom::Line(Line::new(left, right - left).unwrap()))
            .unwrap();
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
    })
}

fn sheet_two_faces_shared_surface(store: &mut Store) -> BodyId {
    assemble_body(store, |store| {
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

        let surface = store
            .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        for x0 in [0.0, 2.0] {
            let face = store.add(Face {
                shell,
                loops: Vec::new(),
                surface,
                sense: Sense::Forward,
                domain: None,
                tolerance: None,
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
                let curve = store
                    .insert_curve(CurveGeom::Line(Line::new(start, end - start).unwrap()))
                    .unwrap();
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
    })
}

fn wire_line(store: &mut Store) -> BodyId {
    assemble_body(store, |store| {
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
        let curve = store
            .insert_curve(CurveGeom::Line(Line::new(start, end - start).unwrap()))
            .unwrap();
        let edge = store.add(Edge {
            curve: Some(curve),
            vertices: [Some(vertices[0]), Some(vertices[1])],
            bounds: Some((0.0, (end - start).norm())),
            fins: Vec::new(),
            tolerance: None,
        });
        store.get_mut(shell).unwrap().edges.push(edge);
        body
    })
}

fn wire_shared_line_segments(store: &mut Store) -> BodyId {
    assemble_body(store, |store| {
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
        let curve = store
            .insert_curve(CurveGeom::Line(
                Line::new(points[0], Vec3::new(1.0, 0.0, 0.0)).unwrap(),
            ))
            .unwrap();
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
    })
}

fn wire_shared_point_vertices(store: &mut Store) -> BodyId {
    assemble_body(store, |store| {
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
            let curve = store
                .insert_curve(CurveGeom::Line(Line::new(start, end - start).unwrap()))
                .unwrap();
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
    })
}

fn wire_ellipse_arc(store: &mut Store) -> BodyId {
    assemble_body(store, |store| {
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
        let curve = store
            .insert_curve(CurveGeom::Ellipse(
                Ellipse::new(Frame::world(), 2.0, 1.0).unwrap(),
            ))
            .unwrap();
        let edge = store.add(Edge {
            curve: Some(curve),
            vertices: [Some(vertices[0]), Some(vertices[1])],
            bounds: Some((0.0, core::f64::consts::FRAC_PI_2)),
            fins: Vec::new(),
            tolerance: None,
        });
        store.get_mut(shell).unwrap().edges.push(edge);
        body
    })
}

fn acorn_point(store: &mut Store) -> BodyId {
    assemble_body(store, |store| {
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
    })
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
fn checked_non_solid_builders_round_trip() {
    let mut sheet_store = Store::new();
    let sheet = make::planar_sheet(
        &mut sheet_store,
        &tilted(),
        &[
            Point2::new(0.0, 0.0),
            Point2::new(0.0, 1.0),
            Point2::new(0.6, 0.4),
            Point2::new(1.2, 1.0),
            Point2::new(1.2, 0.0),
        ],
    )
    .unwrap();
    assert!(
        sheet_store
            .iter::<Fin>()
            .all(|(_, fin)| fin.pcurve.is_some())
    );
    let (_, imported_sheet, imported_sheet_body) = assert_checker_roundtrip(&sheet_store, sheet);
    assert_eq!(
        imported_sheet.get(imported_sheet_body).unwrap().kind,
        BodyKind::Sheet
    );
    let mesh = tessellate_body(
        &imported_sheet,
        imported_sheet_body,
        &TessOptions {
            chord_tol: 1e-3,
            max_edge_len: Some(0.2),
        },
    )
    .unwrap();
    assert!(!mesh.triangles.is_empty());

    let mut wire_store = Store::new();
    let wire = make::wire_polyline(
        &mut wire_store,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.5, 0.8, 0.2),
            Point3::new(1.0, 0.0, 0.0),
        ],
        false,
    )
    .unwrap();
    let (_, imported_wire, imported_wire_body) = assert_checker_roundtrip(&wire_store, wire);
    assert_eq!(
        imported_wire.get(imported_wire_body).unwrap().kind,
        BodyKind::Wire
    );
    assert_eq!(
        imported_wire
            .edges_of_body(imported_wire_body)
            .unwrap()
            .len(),
        2
    );

    let mut acorn_store = Store::new();
    let position = Point3::new(0.25, -0.5, 1.5);
    let acorn = make::acorn(&mut acorn_store, position).unwrap();
    let (_, imported_acorn, imported_acorn_body) = assert_checker_roundtrip(&acorn_store, acorn);
    assert_eq!(
        imported_acorn.get(imported_acorn_body).unwrap().kind,
        BodyKind::Acorn
    );
    assert_eq!(
        imported_acorn
            .vertex_position(
                imported_acorn
                    .vertices_of_body(imported_acorn_body)
                    .unwrap()[0]
            )
            .unwrap(),
        position
    );
}

#[test]
fn cylindrical_sheet_seam_topology_round_trips() {
    let mut store = Store::new();
    let body = make::cylindrical_sheet(&mut store, &tilted(), 1.25, 2.5).unwrap();
    let (_text, imported, imported_body) = assert_checker_roundtrip(&store, body);
    assert_eq!(imported.faces_of_body(imported_body).unwrap().len(), 1);
    assert_eq!(imported.edges_of_body(imported_body).unwrap().len(), 3);
    let seam = imported
        .edges_of_body(imported_body)
        .unwrap()
        .into_iter()
        .find(|&edge| imported.get(edge).unwrap().fins.len() == 2)
        .unwrap();
    let seam_faces: Vec<_> = imported
        .get(seam)
        .unwrap()
        .fins
        .iter()
        .map(|&fin| {
            let loop_id = imported.get(fin).unwrap().parent;
            imported.get(loop_id).unwrap().face
        })
        .collect();
    assert_eq!(seam_faces[0], seam_faces[1]);

    let mesh = tessellate_body(
        &imported,
        imported_body,
        &TessOptions {
            chord_tol: 1e-3,
            max_edge_len: Some(0.25),
        },
    )
    .unwrap();
    assert!(!mesh.triangles.is_empty());
}

#[test]
fn non_null_face_tolerance_is_rejected_by_schema_13006_writer() {
    let mut store = Store::new();
    let body = make::block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
    let face = store.faces_of_body(body).unwrap()[0];
    edit_body(&mut store, body, |store| {
        store.get_mut(face).unwrap().tolerance =
            Some(EntityTolerance::operation(LINEAR_RESOLUTION, "writer-test").unwrap());
    });
    let error = kxt::export_text(&store, body).unwrap_err();
    assert_eq!(error.capability(), Some(kxt::XtCapability::FaceTolerances));
}

#[test]
fn exact_tolerant_edge_and_vertex_round_trip() {
    let mut store = Store::new();
    let body = make::block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
    let edge = first_bounded_edge(&store, body);
    let vertex = store.get(edge).unwrap().vertices[0].unwrap();
    edit_body(&mut store, body, |store| {
        store.get_mut(edge).unwrap().tolerance =
            Some(EntityTolerance::operation(LINEAR_RESOLUTION * 10.0, "writer-test").unwrap());
        store.get_mut(vertex).unwrap().tolerance =
            Some(EntityTolerance::operation(LINEAR_RESOLUTION * 20.0, "writer-test").unwrap());
    });

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
            .any(|edge| imported
                .get(edge)
                .unwrap()
                .tolerance
                .is_some_and(|tolerance| {
                    tolerance.value() == LINEAR_RESOLUTION * 10.0
                        && tolerance.origin() == ToleranceOrigin::ImportedXt
                }))
    );
    assert!(
        imported
            .vertices_of_body(imported_body)
            .unwrap()
            .into_iter()
            .any(|vertex| imported
                .get(vertex)
                .unwrap()
                .tolerance
                .is_some_and(|tolerance| {
                    tolerance.value() == LINEAR_RESOLUTION * 20.0
                        && tolerance.origin() == ToleranceOrigin::ImportedXt
                }))
    );
}

#[test]
fn curve_less_tolerant_edge_round_trips_through_trimmed_sp_curves() {
    let mut store = Store::new();
    let body = make::block(&mut store, &Frame::world(), [2.0, 3.0, 4.0]).unwrap();
    let tolerant = make_first_edge_truly_tolerant(&mut store, body);
    assert!(check_body(&store, body).unwrap().is_empty());

    let (text, imported, imported_body) = assert_checker_roundtrip(&store, body);
    let parsed = kxt::read_xt(text.as_bytes()).unwrap();
    let sp_curves: Vec<_> = parsed
        .nodes
        .values()
        .filter(|node| node.code == code::SP_CURVE)
        .collect();
    assert_eq!(sp_curves.len(), 2);
    assert!(
        sp_curves
            .iter()
            .any(|node| { parsed.field(node, "sense").and_then(kxt::Value::as_char) == Some('-') })
    );
    let null_curve_edges = parsed
        .nodes
        .values()
        .filter(|node| {
            node.code == code::EDGE
                && parsed.field(node, "curve").and_then(kxt::Value::as_ptr) == Some(0)
        })
        .count();
    assert_eq!(null_curve_edges, 1);

    // All directly/indirectly attached curve geometry belongs to the
    // body's boundary-curve chain. The embedded 2D B-curves are the one
    // specified exception: ownerless, unchained, and reached only via SP.
    let body_node = parsed.nodes.get(&1).unwrap();
    let mut curve_index = parsed
        .field(body_node, "boundary_curve")
        .and_then(kxt::Value::as_ptr)
        .unwrap();
    let mut previous = 0;
    let mut boundary_codes = Vec::new();
    while curve_index != 0 {
        let node = parsed.nodes.get(&curve_index).unwrap();
        assert_eq!(
            parsed.field(node, "previous").and_then(kxt::Value::as_ptr),
            Some(previous)
        );
        boundary_codes.push(node.code);
        previous = curve_index;
        curve_index = parsed
            .field(node, "next")
            .and_then(kxt::Value::as_ptr)
            .unwrap();
    }
    // 11 exact line edges reference their LINE directly (no trims), plus
    // the tolerant edge's two per-fin TRIMMED_CURVE + SP_CURVE pairs.
    assert_eq!(boundary_codes.len(), 15);
    assert_eq!(
        boundary_codes
            .iter()
            .filter(|&&node_code| node_code == code::SP_CURVE)
            .count(),
        2
    );
    assert!(!boundary_codes.contains(&code::B_CURVE));
    for bcurve in parsed
        .nodes
        .values()
        .filter(|node| node.code == code::B_CURVE)
    {
        assert_eq!(
            parsed.field(bcurve, "owner").and_then(kxt::Value::as_ptr),
            Some(0)
        );
        assert_eq!(
            parsed.field(bcurve, "next").and_then(kxt::Value::as_ptr),
            Some(0)
        );
    }
    // Only the tolerant fins carry GEOMETRIC_OWNER nodes (one on each
    // trimmed SP-curve, one on each supporting surface); exact edges no
    // longer contribute any.
    assert_eq!(
        parsed
            .nodes
            .values()
            .filter(|node| node.code == code::GEOMETRIC_OWNER)
            .count(),
        4
    );
    assert!(sp_curves.iter().all(|node| {
        parsed
            .field(node, "geometric_owner")
            .and_then(kxt::Value::as_ptr)
            .is_some_and(|owner| owner != 0)
    }));

    let imported_edge = imported
        .edges_of_body(imported_body)
        .unwrap()
        .into_iter()
        .find(|&edge| imported.get(edge).unwrap().curve.is_none())
        .unwrap();
    let edge = imported.get(imported_edge).unwrap();
    assert_eq!(edge.bounds, Some((0.0, 1.0)));
    let tolerance = edge.tolerance.unwrap();
    assert_eq!(tolerance.value(), LINEAR_RESOLUTION);
    assert_eq!(tolerance.origin(), ToleranceOrigin::ImportedXt);
    assert_eq!(edge.fins.len(), 2);
    assert!(
        edge.fins
            .iter()
            .all(|&fin| imported.get(fin).unwrap().pcurve.is_some())
    );
    assert!(edge.fins.iter().any(|&fin| {
        imported.get(fin).unwrap().pcurve.unwrap().sense() == ktopo::entity::Sense::Reversed
    }));
    assert!(edge.fins.iter().any(|&fin| {
        let curve = imported.get(fin).unwrap().pcurve.unwrap().curve();
        matches!(imported.get(curve).unwrap(), Curve2dGeom::Nurbs(n) if n.weights().is_some())
    }));
    assert!(edge.fins.iter().all(|&fin| {
        let loop_id = imported.get(fin).unwrap().parent;
        let face = imported.get(loop_id).unwrap().face;
        imported.get(face).unwrap().domain.is_some()
    }));

    let mesh = tessellate_body(
        &imported,
        imported_body,
        &TessOptions {
            chord_tol: 1e-3,
            max_edge_len: Some(0.2),
        },
    )
    .unwrap();
    assert!(check_watertight(&mesh).is_empty());
    assert!(
        mesh.edge_polylines
            .iter()
            .find(|(edge, _)| *edge == imported_edge)
            .unwrap()
            .1
            .len()
            > 2
    );

    // Keep the source handle observably in use: it must be the only
    // curve-less edge before and after interchange.
    assert!(store.get(tolerant).unwrap().curve.is_none());
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
    // The bounded NURBS edge references its B_CURVE directly; parameter
    // bounds are recovered from the vertices on import.
    assert!(
        !parsed
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
fn periodic_rational_nurbs_sheet_round_trips_with_closed_flags() {
    let mut store = Store::new();
    let body = make::planar_sheet(
        &mut store,
        &Frame::world(),
        &[
            Point2::new(0.15, 0.0),
            Point2::new(0.65, 0.0),
            Point2::new(0.65, 2.0),
            Point2::new(0.15, 2.0),
        ],
    )
    .unwrap();
    replace_sheet_patch_with_periodic_rational_nurbs(&mut store, body);

    let (text, imported, _) = assert_checker_roundtrip(&store, body);
    let parsed = kxt::read_xt(text.as_bytes()).unwrap();
    let nurbs_node = parsed
        .nodes
        .values()
        .find(|node| node.code == code::NURBS_SURF)
        .unwrap();
    for name in ["u_periodic", "u_closed"] {
        assert_eq!(
            parsed.field(nurbs_node, name),
            Some(&kxt::Value::Logical(true))
        );
    }
    for name in ["v_periodic", "v_closed"] {
        assert_eq!(
            parsed.field(nurbs_node, name),
            Some(&kxt::Value::Logical(false))
        );
    }
    assert_eq!(
        parsed.field(nurbs_node, "rational"),
        Some(&kxt::Value::Logical(true))
    );

    let surface = imported
        .iter::<SurfaceGeom>()
        .find_map(|(_, surface)| match surface {
            SurfaceGeom::Nurbs(surface) => Some(surface),
            _ => None,
        })
        .unwrap();
    assert!(surface.is_rational());
    assert_eq!(surface.periodicity(), [Some(1.0), None]);
    for uv in [[0.0, 0.4], [0.23, 1.3], [1.0, 1.7]] {
        assert!(surface.eval(uv).dist(surface.eval([uv[0] + 1.0, uv[1]])) < 1.0e-13);
    }
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
    // Exact bounded edges reference their basis curve directly, as real
    // exact-modeling files do; no TRIMMED_CURVE wrapper is emitted.
    assert!(
        !parsed
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
    assert_eq!(
        trim_nodes, 0,
        "bounded edges reference the shared basis curve directly"
    );
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
    // The bounded arc references the ELLIPSE directly; its sub-range is
    // recovered from the vertices on import.
    assert!(
        !parsed
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

/// The writer's embedded-schema output must match real V27 Parasolid byte
/// for byte where they describe the same thing: the flag-sequence schema
/// key and the BODY edit script are extracted from `disk_nat.x_t` (written
/// by Parasolid itself) and compared against a freshly exported block.
/// Plain pre-embedded-schema text is rejected by production Parasolid
/// hosts, so this framing is load-bearing for interchange.
#[test]
fn writer_embedded_schema_matches_real_v27_output() {
    fn stream(text: &str) -> String {
        let body = text
            .split_once("**END_OF_HEADER")
            .expect("header terminator")
            .1;
        let body = body.split_once('\n').expect("header line end").1;
        body.replace(['\n', '\r'], "")
    }
    fn body_script(stream: &str, after: &str) -> String {
        let start = stream.find(after).expect("BODY first occurrence") + after.len();
        let end = stream[start..].find("dZ").expect("edit script end") + 2;
        stream[start..start + end].to_string()
    }

    let real = stream(&String::from_utf8(fixture("disk_nat.x_t")).unwrap());
    let real_script = body_script(&real, "SCH_2700142_26105_13006196 1 12 ");

    let mut store = Store::new();
    let body = make::block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
    let text = kxt::export_text(&store, body).unwrap();
    let ours = stream(&text);
    // Same schema key, max-node-type count, and user-field size.
    let ours_script = body_script(&ours, "SCH_2700142_26105_13006196 1 12 ");
    assert_eq!(
        ours_script, real_script,
        "BODY edit script drifted from real V27 output"
    );

    // The parsed layout must be the full 30-field V26105 BODY.
    let parsed = kxt::read_xt(text.as_bytes()).unwrap();
    let def = &parsed.defs[&12];
    assert_eq!(def.fields.len(), 30);
    assert_eq!(def.fields[13].name, "owner");
    assert_eq!(def.fields[23].name, "boundary_mesh");
    assert_eq!(def.fields[29].name, "lowest_node_id");
}
