//! Deterministic XT text emission for self-authored analytic solids.
//!
//! This first M3b slice intentionally writes the fixed base schema 13006.
//! Unsupported model classes fail explicitly rather than being approximated.

use crate::error::{Result, XtError};
use crate::parse::Value;
use crate::schema::{base_schema, code};
use kcore::arena::Handle;
use kcore::math;
use kcore::tolerance::{LINEAR_RESOLUTION, check_in_size_box};
use kgeom::vec::Vec3;
use ktopo::check::check_body;
use ktopo::entity::{
    BodyId, BodyKind, CurveId, EdgeId, FaceId, FinId, LoopId, PointId, RegionKind, Sense,
    SurfaceId, VertexId,
};
use ktopo::geom::{CurveGeom, SurfaceGeom};
use ktopo::store::Store;

const VERSION: &str = ": TRANSMIT FILE created by modeller version 1300000";
const SCHEMA: &str = "SCH_1300000_13006";

struct OutNode {
    code: u16,
    index: u32,
    values: Vec<Value>,
}

struct Plan {
    faces: Vec<(FaceId, u32)>,
    loops: Vec<(LoopId, u32)>,
    fins: Vec<(FinId, u32)>,
    edges: Vec<(EdgeId, u32)>,
    vertices: Vec<(VertexId, u32)>,
    surfaces: Vec<(SurfaceId, u32)>,
    curves: Vec<(CurveId, u32)>,
    points: Vec<(PointId, u32)>,
    void_shell_id: u32,
    max_id: u32,
}

/// Export one checker-clean analytic solid as deterministic schema-13006
/// text XT. The current writer supports all self-authored primitive bodies.
pub fn export_text(store: &Store, body: BodyId) -> Result<String> {
    let plan = Plan::build(store, body)?;
    let nodes = plan.nodes(store)?;
    serialize(&nodes)
}

impl Plan {
    fn build(store: &Store, body: BodyId) -> Result<Self> {
        if !check_body(store, body)?.is_empty() {
            return Err(XtError::InvalidModel {
                what: "body checker reported faults",
            });
        }
        let b = store.get(body)?;
        if b.kind != BodyKind::Solid {
            return Err(XtError::Unsupported {
                what: "text writing currently supports solid bodies only",
            });
        }
        if b.regions.len() != 2 {
            return Err(XtError::Unsupported {
                what: "text writing requires one void and one solid region",
            });
        }
        let void_region = b.regions[0];
        let solid_region = b.regions[1];
        let vr = store.get(void_region)?;
        let sr = store.get(solid_region)?;
        if vr.kind != RegionKind::Void || !vr.shells.is_empty() || sr.kind != RegionKind::Solid {
            return Err(XtError::Unsupported {
                what: "text writing requires the standard solid region scaffold",
            });
        }
        let [shell] = sr.shells.as_slice() else {
            return Err(XtError::Unsupported {
                what: "text writing requires exactly one solid shell",
            });
        };
        let shell = *shell;
        let sh = store.get(shell)?;
        if sh.faces.is_empty() || !sh.edges.is_empty() || sh.vertex.is_some() {
            return Err(XtError::Unsupported {
                what: "wireframe, acorn, and empty shells are not writable yet",
            });
        }

        let face_handles = store.faces_of_body(body)?;
        let edge_handles = store.edges_of_body(body)?;
        let vertex_handles = store.vertices_of_body(body)?;
        let mut loop_handles = Vec::new();
        let mut fin_handles = Vec::new();
        let mut surface_handles = Vec::new();
        for &face in &face_handles {
            let f = store.get(face)?;
            push_unique(
                &mut surface_handles,
                f.surface,
                "surface shared by multiple faces",
            )?;
            for &lp in &f.loops {
                if store.get(lp)?.fins.is_empty() {
                    return Err(XtError::InvalidModel { what: "empty loop" });
                }
                loop_handles.push(lp);
                fin_handles.extend_from_slice(&store.get(lp)?.fins);
            }
        }
        let mut curve_handles = Vec::new();
        for &edge in &edge_handles {
            let e = store.get(edge)?;
            if e.tolerance.is_some() {
                return Err(XtError::Unsupported {
                    what: "tolerant edges",
                });
            }
            if e.fins.len() != 2 {
                return Err(XtError::Unsupported {
                    what: "solid edges must have exactly two fins",
                });
            }
            let curve = e.curve.ok_or(XtError::Unsupported {
                what: "curve-less edges",
            })?;
            push_unique(&mut curve_handles, curve, "curve shared by multiple edges")?;
            match (store.get(curve)?, e.vertices, e.bounds) {
                (CurveGeom::Line(_), [Some(_), Some(_)], Some(_)) => {}
                (CurveGeom::Circle(_), [None, None], None) => {}
                (CurveGeom::Ellipse(_), [None, None], None) => {}
                (CurveGeom::Nurbs(_), _, _) => {
                    return Err(XtError::Unsupported {
                        what: "NURBS curves",
                    });
                }
                _ => {
                    return Err(XtError::Unsupported {
                        what: "unsupported edge/curve topology",
                    });
                }
            }
        }
        let mut point_handles = Vec::new();
        for &vertex in &vertex_handles {
            let v = store.get(vertex)?;
            if v.tolerance.is_some() {
                return Err(XtError::Unsupported {
                    what: "tolerant vertices",
                });
            }
            check_in_size_box(store.get(v.point)?.to_array())?;
            push_unique(
                &mut point_handles,
                v.point,
                "point shared by multiple vertices",
            )?;
        }
        for &surface in &surface_handles {
            match store.get(surface)? {
                SurfaceGeom::Nurbs(_) => {
                    return Err(XtError::Unsupported {
                        what: "NURBS surfaces",
                    });
                }
                SurfaceGeom::Plane(s) => check_in_size_box(s.frame().origin().to_array())?,
                SurfaceGeom::Cylinder(s) => check_in_size_box(s.frame().origin().to_array())?,
                SurfaceGeom::Cone(s) => check_in_size_box(s.frame().origin().to_array())?,
                SurfaceGeom::Sphere(s) => check_in_size_box(s.frame().origin().to_array())?,
                SurfaceGeom::Torus(s) => check_in_size_box(s.frame().origin().to_array())?,
            }
        }

        let mut next = 6u32; // body, two regions, real shell, synthetic void shell
        let faces = assign(&face_handles, &mut next);
        let loops = assign(&loop_handles, &mut next);
        let fins = assign(&fin_handles, &mut next);
        let edges = assign(&edge_handles, &mut next);
        let vertices = assign(&vertex_handles, &mut next);
        let surfaces = assign(&surface_handles, &mut next);
        let curves = assign(&curve_handles, &mut next);
        let points = assign(&point_handles, &mut next);
        Ok(Plan {
            faces,
            loops,
            fins,
            edges,
            vertices,
            surfaces,
            curves,
            points,
            void_shell_id: 5,
            max_id: next - 1,
        })
    }

    fn nodes(&self, store: &Store) -> Result<Vec<OutNode>> {
        let mut out = Vec::with_capacity(self.max_id as usize);
        out.push(OutNode {
            code: code::BODY,
            index: 1,
            values: vec![
                int(self.max_id),
                ptr(0),
                ptr(0),
                ptr(0),
                ptr(0),
                ptr(0),
                ptr(0),
                Value::Double(1000.0),
                Value::Double(LINEAR_RESOLUTION),
                ptr(0),
                ptr(0),
                ptr(0),
                Value::Int(1),
                ptr(0),
                Value::Int(1),
                Value::Int(1),
                ptr(4),
                ptr(first_id(&self.surfaces)),
                ptr(first_id(&self.curves)),
                ptr(first_id(&self.points)),
                ptr(2),
                ptr(first_id(&self.edges)),
                ptr(first_id(&self.vertices)),
            ],
        });
        out.push(OutNode {
            code: code::REGION,
            index: 2,
            values: vec![
                int(2),
                ptr(0),
                ptr(1),
                ptr(3),
                ptr(0),
                ptr(self.void_shell_id),
                Value::Char('V'),
            ],
        });
        out.push(OutNode {
            code: code::REGION,
            index: 3,
            values: vec![
                int(3),
                ptr(0),
                ptr(1),
                ptr(0),
                ptr(2),
                ptr(4),
                Value::Char('S'),
            ],
        });
        out.push(OutNode {
            code: code::SHELL,
            index: 4,
            values: vec![
                int(4),
                ptr(0),
                ptr(1),
                ptr(0),
                ptr(first_id(&self.faces)),
                ptr(0),
                ptr(0),
                ptr(3),
                ptr(0),
            ],
        });
        out.push(OutNode {
            code: code::SHELL,
            index: self.void_shell_id,
            values: vec![
                int(self.void_shell_id),
                ptr(0),
                ptr(0),
                ptr(0),
                ptr(0),
                ptr(0),
                ptr(0),
                ptr(2),
                ptr(first_id(&self.faces)),
            ],
        });

        for (position, &(face_id, index)) in self.faces.iter().enumerate() {
            let face = store.get(face_id)?;
            let first_loop = face.loops.first().map_or(0, |&lp| id_of(&self.loops, lp));
            let next = adjacent(&self.faces, position, 1);
            let previous = adjacent(&self.faces, position, -1);
            out.push(OutNode {
                code: code::FACE,
                index,
                values: vec![
                    int(index),
                    ptr(0),
                    Value::Null,
                    ptr(next),
                    ptr(previous),
                    ptr(first_loop),
                    ptr(4),
                    ptr(id_of(&self.surfaces, face.surface)),
                    sense(face.sense),
                    ptr(0),
                    ptr(0),
                    ptr(next),
                    ptr(previous),
                    ptr(self.void_shell_id),
                ],
            });
        }
        for &(loop_id, index) in &self.loops {
            let lp = store.get(loop_id)?;
            let position = store
                .get(lp.face)?
                .loops
                .iter()
                .position(|&candidate| candidate == loop_id)
                .ok_or(XtError::InvalidModel {
                    what: "loop missing from face",
                })?;
            let face_loops = &store.get(lp.face)?.loops;
            let next = face_loops
                .get(position + 1)
                .map_or(0, |&v| id_of(&self.loops, v));
            out.push(OutNode {
                code: code::LOOP,
                index,
                values: vec![
                    int(index),
                    ptr(0),
                    ptr(id_of(&self.fins, lp.fins[0])),
                    ptr(id_of(&self.faces, lp.face)),
                    ptr(next),
                ],
            });
        }
        for &(fin_id, index) in &self.fins {
            let fin = store.get(fin_id)?;
            let lp = store.get(fin.parent)?;
            let position =
                lp.fins
                    .iter()
                    .position(|&v| v == fin_id)
                    .ok_or(XtError::InvalidModel {
                        what: "fin missing from loop",
                    })?;
            let forward = lp.fins[(position + 1) % lp.fins.len()];
            let backward = lp.fins[(position + lp.fins.len() - 1) % lp.fins.len()];
            let edge = store.get(fin.edge)?;
            let other =
                edge.fins
                    .iter()
                    .copied()
                    .find(|&v| v != fin_id)
                    .ok_or(XtError::InvalidModel {
                        what: "edge has no paired fin",
                    })?;
            let head = store.fin_head(fin_id)?;
            out.push(OutNode {
                code: code::FIN,
                index,
                values: vec![
                    ptr(0),
                    ptr(id_of(&self.loops, fin.parent)),
                    ptr(id_of(&self.fins, forward)),
                    ptr(id_of(&self.fins, backward)),
                    ptr(head.map_or(0, |v| id_of(&self.vertices, v))),
                    ptr(id_of(&self.fins, other)),
                    ptr(id_of(&self.edges, fin.edge)),
                    ptr(0),
                    ptr(head
                        .and_then(|v| self.next_fin_at_vertex(store, v, fin_id))
                        .map_or(0, |v| id_of(&self.fins, v))),
                    sense(fin.sense),
                ],
            });
        }
        for (position, &(edge_id, index)) in self.edges.iter().enumerate() {
            let edge = store.get(edge_id)?;
            out.push(OutNode {
                code: code::EDGE,
                index,
                values: vec![
                    int(index),
                    ptr(0),
                    Value::Null,
                    ptr(id_of(&self.fins, edge.fins[0])),
                    ptr(adjacent(&self.edges, position, -1)),
                    ptr(adjacent(&self.edges, position, 1)),
                    ptr(id_of(&self.curves, edge.curve.expect("validated curve"))),
                    ptr(0),
                    ptr(0),
                    ptr(1),
                ],
            });
        }
        for (position, &(vertex_id, index)) in self.vertices.iter().enumerate() {
            let vertex = store.get(vertex_id)?;
            let first_fin = self
                .fins
                .iter()
                .find_map(|&(fin, id)| {
                    (store.fin_head(fin).ok().flatten() == Some(vertex_id)).then_some(id)
                })
                .unwrap_or(0);
            out.push(OutNode {
                code: code::VERTEX,
                index,
                values: vec![
                    int(index),
                    ptr(0),
                    ptr(first_fin),
                    ptr(adjacent(&self.vertices, position, -1)),
                    ptr(adjacent(&self.vertices, position, 1)),
                    ptr(id_of(&self.points, vertex.point)),
                    Value::Null,
                    ptr(1),
                ],
            });
        }
        for (position, &(surface_id, index)) in self.surfaces.iter().enumerate() {
            let face = self
                .faces
                .iter()
                .find_map(|&(face, _)| {
                    (store.get(face).ok()?.surface == surface_id).then_some(face)
                })
                .expect("validated surface owner");
            let common = geom_common(
                index,
                id_of(&self.faces, face),
                adjacent(&self.surfaces, position, 1),
                adjacent(&self.surfaces, position, -1),
            );
            out.push(surface_node(store.get(surface_id)?, index, common));
        }
        for (position, &(curve_id, index)) in self.curves.iter().enumerate() {
            let edge = self
                .edges
                .iter()
                .find_map(|&(edge, _)| {
                    (store.get(edge).ok()?.curve == Some(curve_id)).then_some(edge)
                })
                .expect("validated curve owner");
            let common = geom_common(
                index,
                id_of(&self.edges, edge),
                adjacent(&self.curves, position, 1),
                adjacent(&self.curves, position, -1),
            );
            out.push(curve_node(store.get(curve_id)?, index, common)?);
        }
        for (position, &(point_id, index)) in self.points.iter().enumerate() {
            out.push(OutNode {
                code: code::POINT,
                index,
                values: vec![
                    int(index),
                    ptr(0),
                    ptr(1),
                    ptr(adjacent(&self.points, position, 1)),
                    ptr(adjacent(&self.points, position, -1)),
                    vector(*store.get(point_id)?),
                ],
            });
        }
        out.sort_by_key(|node| node.index);
        Ok(out)
    }

    fn next_fin_at_vertex(&self, store: &Store, vertex: VertexId, fin: FinId) -> Option<FinId> {
        let mut found = false;
        for &(candidate, _) in &self.fins {
            if store.fin_head(candidate).ok().flatten() != Some(vertex) {
                continue;
            }
            if found {
                return Some(candidate);
            }
            found = candidate == fin;
        }
        None
    }
}

fn surface_node(surface: &SurfaceGeom, index: u32, mut values: Vec<Value>) -> OutNode {
    let code = match surface {
        SurfaceGeom::Plane(s) => {
            values.extend([
                vector(s.frame().origin()),
                vector(s.frame().z()),
                vector(s.frame().x()),
            ]);
            code::PLANE
        }
        SurfaceGeom::Cylinder(s) => {
            values.extend([
                vector(s.frame().origin()),
                vector(s.frame().z()),
                Value::Double(s.radius()),
                vector(s.frame().x()),
            ]);
            code::CYLINDER
        }
        SurfaceGeom::Cone(s) => {
            let (sin, cos) = math::sincos(s.half_angle());
            values.extend([
                vector(s.frame().origin()),
                vector(-s.frame().z()),
                Value::Double(s.radius()),
                Value::Double(sin),
                Value::Double(cos),
                vector(s.frame().x()),
            ]);
            code::CONE
        }
        SurfaceGeom::Sphere(s) => {
            values.extend([
                vector(s.frame().origin()),
                Value::Double(s.radius()),
                vector(s.frame().z()),
                vector(s.frame().x()),
            ]);
            code::SPHERE
        }
        SurfaceGeom::Torus(s) => {
            values.extend([
                vector(s.frame().origin()),
                vector(s.frame().z()),
                Value::Double(s.major_radius()),
                Value::Double(s.minor_radius()),
                vector(s.frame().x()),
            ]);
            code::TORUS
        }
        SurfaceGeom::Nurbs(_) => unreachable!("rejected during planning"),
    };
    OutNode {
        code,
        index,
        values,
    }
}

fn curve_node(curve: &CurveGeom, index: u32, mut values: Vec<Value>) -> Result<OutNode> {
    let code = match curve {
        CurveGeom::Line(c) => {
            values.extend([vector(c.origin()), vector(c.dir())]);
            code::LINE
        }
        CurveGeom::Circle(c) => {
            values.extend([
                vector(c.frame().origin()),
                vector(c.frame().z()),
                vector(c.frame().x()),
                Value::Double(c.radius()),
            ]);
            code::CIRCLE
        }
        CurveGeom::Ellipse(c) => {
            values.extend([
                vector(c.frame().origin()),
                vector(c.frame().z()),
                vector(c.frame().x()),
                Value::Double(c.major_radius()),
                Value::Double(c.minor_radius()),
            ]);
            code::ELLIPSE
        }
        CurveGeom::Nurbs(_) => {
            return Err(XtError::Unsupported {
                what: "NURBS curves",
            });
        }
    };
    Ok(OutNode {
        code,
        index,
        values,
    })
}

fn geom_common(index: u32, owner: u32, next: u32, previous: u32) -> Vec<Value> {
    vec![
        int(index),
        ptr(0),
        ptr(owner),
        ptr(next),
        ptr(previous),
        ptr(0),
        Value::Char('+'),
    ]
}

fn serialize(nodes: &[OutNode]) -> Result<String> {
    let defs = base_schema();
    let mut data = format!(
        "T{} {}{} {} 0 ",
        VERSION.len(),
        VERSION,
        SCHEMA.len(),
        SCHEMA
    );
    for node in nodes {
        let def = defs
            .iter()
            .find(|def| def.code == node.code)
            .ok_or(XtError::InvalidModel {
                what: "writer selected an unknown node type",
            })?;
        if def.fields.len() != node.values.len() {
            return Err(XtError::InvalidModel {
                what: "writer produced the wrong field count",
            });
        }
        data.push_str(&format!("{} {} ", node.code, node.index));
        for value in &node.values {
            write_value(&mut data, value)?;
        }
    }
    data.push_str("1 0 ");
    let mut wrapped = String::new();
    for chunk in data.as_bytes().chunks(79) {
        wrapped.push_str(core::str::from_utf8(chunk).expect("writer emits ASCII"));
        wrapped.push('\n');
    }
    Ok(format!(
        "**ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz**************************\n\
         **PARASOLID !\"#$%&'()*+,-./:;<=>?@[\\]^_`{{|}}~0123456789**************************\n\
         **PART1;MC=none;APPL=cad_prototype-kxt;SITE=none;USER=none;FORMAT=text;GUISE=transmit;\n\
         **PART2;SCH={SCHEMA};USFLD_SIZE=0;\n\
         **PART3;\n\
         **END_OF_HEADER*****************************************************************\n{wrapped}"
    ))
}

fn write_value(out: &mut String, value: &Value) -> Result<()> {
    match value {
        Value::Null => out.push('?'),
        Value::Int(v) => out.push_str(&format!("{v} ")),
        Value::Double(v) => {
            if !v.is_finite() {
                return Err(XtError::InvalidModel {
                    what: "non-finite numeric value",
                });
            }
            out.push_str(&format!("{v} "));
        }
        Value::Char(v) => out.push(*v),
        Value::Logical(v) => out.push(if *v { 'T' } else { 'F' }),
        Value::Ptr(v) => out.push_str(&format!("{v} ")),
        Value::Vector(Some(v)) => {
            for value in v {
                if !value.is_finite() {
                    return Err(XtError::InvalidModel {
                        what: "non-finite vector value",
                    });
                }
                out.push_str(&format!("{value} "));
            }
        }
        _ => {
            return Err(XtError::InvalidModel {
                what: "unsupported writer value",
            });
        }
    }
    Ok(())
}

fn assign<T>(handles: &[Handle<T>], next: &mut u32) -> Vec<(Handle<T>, u32)> {
    handles
        .iter()
        .map(|&handle| {
            let id = *next;
            *next += 1;
            (handle, id)
        })
        .collect()
}

fn push_unique<T>(values: &mut Vec<Handle<T>>, value: Handle<T>, what: &'static str) -> Result<()> {
    if values.contains(&value) {
        return Err(XtError::Unsupported { what });
    }
    values.push(value);
    Ok(())
}

fn id_of<T>(values: &[(Handle<T>, u32)], handle: Handle<T>) -> u32 {
    values
        .iter()
        .find_map(|&(candidate, id)| (candidate == handle).then_some(id))
        .expect("planned handle")
}

fn first_id<T>(values: &[(Handle<T>, u32)]) -> u32 {
    values.first().map_or(0, |&(_, id)| id)
}

fn adjacent<T>(values: &[(Handle<T>, u32)], position: usize, direction: i8) -> u32 {
    match direction {
        -1 if position > 0 => values[position - 1].1,
        1 if position + 1 < values.len() => values[position + 1].1,
        _ => 0,
    }
}

fn int(value: u32) -> Value {
    Value::Int(i64::from(value))
}

fn ptr(value: u32) -> Value {
    Value::Ptr(value)
}

fn sense(value: Sense) -> Value {
    Value::Char(if value.is_forward() { '+' } else { '-' })
}

fn vector(value: Vec3) -> Value {
    Value::Vector(Some(value.to_array()))
}
