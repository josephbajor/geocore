//! Deterministic XT text emission for checker-clean bodies.
//!
//! This first M3b slice intentionally writes the fixed base schema 13006.
//! Unsupported model classes fail explicitly rather than being approximated.

use crate::error::{Result, XtCapability, XtError};
use crate::parse::Value;
use crate::schema::{base_schema, code};
use kcore::arena::Handle;
use kcore::math;
use kcore::tolerance::{LINEAR_RESOLUTION, check_in_size_box};
use kgeom::curve::Curve;
use kgeom::curve2d::{Curve2d, NurbsCurve2d};
use kgeom::nurbs::{KnotVector, NurbsCurve, NurbsSurface};
use kgeom::surface::{Dir, Surface};
use kgeom::vec::{Point3, Vec3};
use ktopo::check::check_body;
use ktopo::entity::{
    BodyId, BodyKind, CurveId, Edge, EdgeId, FaceId, FinId, FinPcurve, LoopId, PointId, RegionKind,
    Sense, SurfaceId, VertexId,
};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::store::Store;
use ktopo::tolerance::EntityTolerance;

const VERSION: &str = ": TRANSMIT FILE created by modeller version 1300000";
const SCHEMA: &str = "SCH_1300000_13006";

struct OutNode {
    code: u16,
    index: u32,
    values: Vec<Value>,
}

struct Plan {
    body_kind: BodyKind,
    faces: Vec<(FaceId, u32)>,
    loops: Vec<(LoopId, u32)>,
    fins: Vec<(FinId, u32)>,
    dummy_fins: Vec<(DummyFin, u32)>,
    edges: Vec<(EdgeId, u32)>,
    vertices: Vec<(VertexId, u32)>,
    surfaces: Vec<(SurfaceId, u32)>,
    trimmed_curves: Vec<(EdgeId, u32)>,
    trimmed_curve_owners: Vec<(EdgeId, u32)>,
    curves: Vec<(CurveId, u32)>,
    fin_pcurves: Vec<FinPcurvePlan>,
    points: Vec<(PointId, u32)>,
    shell_id: u32,
    void_shell_id: u32,
    first_aux_id: u32,
    max_id: u32,
}

#[derive(Debug, Clone, Copy)]
struct FinPcurvePlan {
    fin: FinId,
    trimmed: u32,
    sp_curve: u32,
    b_curve: u32,
    sp_geometric_owner: u32,
    surface_geometric_owner: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct DummyFin {
    edge: EdgeId,
    role: DummyFinRole,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DummyFinRole {
    SheetBoundary,
    WireEnd,
    WireStart,
}

#[derive(Debug, Clone, Copy)]
struct Scaffold {
    shell_id: u32,
    void_shell_id: u32,
}

#[derive(Debug, Clone, Copy)]
struct CurveAuxIds {
    nurbs: u32,
    poles: u32,
    knot_mult: u32,
    knots: u32,
}

#[derive(Debug, Clone, Copy)]
struct SurfaceAuxIds {
    nurbs: u32,
    poles: u32,
    u_knot_mult: u32,
    v_knot_mult: u32,
    u_knots: u32,
    v_knots: u32,
}

/// Export one checker-clean solid, supported sheet body, supported wire body,
/// or acorn body as deterministic schema-13006 text XT. The writer supports
/// self-authored analytic bodies plus non-periodic B-spline/NURBS curves and
/// surfaces.
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
        let scaffold = Scaffold::validate(store, b)?;

        let face_handles = store.faces_of_body(body)?;
        let edge_handles = store.edges_of_body(body)?;
        let vertex_handles = store.vertices_of_body(body)?;
        let mut loop_handles = Vec::new();
        let mut fin_handles = Vec::new();
        let mut surface_handles = Vec::new();
        for &face in &face_handles {
            let f = store.get(face)?;
            if f.tolerance.is_some() {
                return Err(XtError::Unsupported {
                    capability: XtCapability::FaceTolerances,
                    what: "schema-13006 FACE tolerance is required to be null",
                });
            }
            push_interned(&mut surface_handles, f.surface);
            for &lp in &f.loops {
                if store.get(lp)?.fins.is_empty() {
                    return Err(XtError::InvalidModel { what: "empty loop" });
                }
                loop_handles.push(lp);
                fin_handles.extend_from_slice(&store.get(lp)?.fins);
            }
        }
        let mut curve_handles = Vec::new();
        let mut trimmed_curve_edges = Vec::new();
        let mut fin_pcurve_handles = Vec::new();
        let mut dummy_fin_specs = Vec::new();
        for &edge in &edge_handles {
            let e = store.get(edge)?;
            validate_edge_fin_count(e, b.kind)?;
            push_dummy_fins(&mut dummy_fin_specs, edge, e, b.kind);
            match e.curve {
                Some(curve) => {
                    if e.bounds.is_some() {
                        trimmed_curve_edges.push(edge);
                    }
                    push_interned(&mut curve_handles, curve);
                    match (store.get(curve)?, e.vertices, e.bounds) {
                        (CurveGeom::Line(_), [Some(_), Some(_)], Some(_)) => {}
                        (CurveGeom::Circle(_), [None, None], None) => {}
                        (CurveGeom::Circle(_), [Some(_), Some(_)], Some(_)) => {}
                        (CurveGeom::Ellipse(_), [None, None], None) => {}
                        (CurveGeom::Ellipse(_), [Some(_), Some(_)], Some(_)) => {}
                        (CurveGeom::Nurbs(n), [Some(_), Some(_)], Some(_)) => {
                            validate_nurbs_curve(n)?;
                        }
                        _ => {
                            return Err(XtError::Unsupported {
                                capability: XtCapability::WriterEdgeTopology,
                                what: "unsupported edge/curve topology",
                            });
                        }
                    }
                }
                None => {
                    if e.fins.is_empty() {
                        return Err(XtError::Unsupported {
                            capability: XtCapability::TolerantWireEdges,
                            what: "curve-less tolerant wire edges",
                        });
                    }
                    for &fin in &e.fins {
                        pcurve_nurbs(store, fin)?;
                        fin_pcurve_handles.push(fin);
                    }
                }
            }
        }
        let mut point_handles = Vec::new();
        for &vertex in &vertex_handles {
            let v = store.get(vertex)?;
            check_in_size_box(store.get(v.point)?.to_array())?;
            push_interned(&mut point_handles, v.point);
        }
        for &surface in &surface_handles {
            match store.get(surface)? {
                SurfaceGeom::Nurbs(s) => validate_nurbs_surface(s)?,
                SurfaceGeom::Plane(s) => check_in_size_box(s.frame().origin().to_array())?,
                SurfaceGeom::Cylinder(s) => check_in_size_box(s.frame().origin().to_array())?,
                SurfaceGeom::Cone(s) => check_in_size_box(s.frame().origin().to_array())?,
                SurfaceGeom::Sphere(s) => check_in_size_box(s.frame().origin().to_array())?,
                SurfaceGeom::Torus(s) => check_in_size_box(s.frame().origin().to_array())?,
            }
        }

        let mut next = scaffold.first_entity_id();
        let faces = assign(&face_handles, &mut next);
        let loops = assign(&loop_handles, &mut next);
        let fins = assign(&fin_handles, &mut next);
        let dummy_fins = assign_dummy_fins(&dummy_fin_specs, &mut next);
        let edges = assign(&edge_handles, &mut next);
        let vertices = assign(&vertex_handles, &mut next);
        let surfaces = assign(&surface_handles, &mut next);
        let trimmed_curves = assign(&trimmed_curve_edges, &mut next);
        let curves = assign(&curve_handles, &mut next);
        let fin_pcurves = assign_fin_pcurves(&fin_pcurve_handles, &mut next);
        let trimmed_curve_owners = assign(&trimmed_curve_edges, &mut next);
        let points = assign(&point_handles, &mut next);
        let first_aux_id = next;
        next += aux_node_count(store, &surface_handles, &curve_handles)?;
        next += 4 * u32::try_from(fin_pcurves.len()).map_err(|_| XtError::InvalidModel {
            what: "too many tolerant fin pcurves",
        })?;
        Ok(Plan {
            body_kind: b.kind,
            faces,
            loops,
            fins,
            dummy_fins,
            edges,
            vertices,
            surfaces,
            trimmed_curves,
            trimmed_curve_owners,
            curves,
            fin_pcurves,
            points,
            shell_id: scaffold.shell_id,
            void_shell_id: scaffold.void_shell_id,
            first_aux_id,
            max_id: next - 1,
        })
    }

    fn nodes(&self, store: &Store) -> Result<Vec<OutNode>> {
        let mut out = Vec::with_capacity(self.max_id as usize);
        let mut next_aux = self.first_aux_id;
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
                Value::Int(body_type(self.body_kind)),
                Value::Int(1),
                ptr(self.shell_id),
                ptr(first_id(&self.surfaces)),
                ptr(self.first_curve_id()),
                ptr(first_id(&self.points)),
                ptr(2),
                ptr(first_id(&self.edges)),
                ptr(first_id(&self.vertices)),
            ],
        });
        match self.body_kind {
            BodyKind::Solid => self.push_solid_scaffold_nodes(&mut out),
            BodyKind::Sheet => self.push_sheet_scaffold_nodes(&mut out),
            BodyKind::Wire => self.push_wire_scaffold_nodes(&mut out),
            BodyKind::Acorn => self.push_acorn_scaffold_nodes(&mut out),
        }

        for (position, &(face_id, index)) in self.faces.iter().enumerate() {
            let face = store.get(face_id)?;
            let first_loop = face.loops.first().map_or(0, |&lp| id_of(&self.loops, lp));
            let next = adjacent(&self.faces, position, 1);
            let previous = adjacent(&self.faces, position, -1);
            let surface_faces: Vec<_> = self
                .faces
                .iter()
                .copied()
                .filter(|&(candidate, _)| {
                    store
                        .get(candidate)
                        .is_ok_and(|f| f.surface == face.surface)
                })
                .collect();
            let surface_position = surface_faces
                .iter()
                .position(|&(candidate, _)| candidate == face_id)
                .expect("validated surface owner chain");
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
                    ptr(self.shell_id),
                    ptr(id_of(&self.surfaces, face.surface)),
                    sense(face.sense),
                    ptr(adjacent(&surface_faces, surface_position, 1)),
                    ptr(adjacent(&surface_faces, surface_position, -1)),
                    ptr(if self.body_kind == BodyKind::Solid {
                        next
                    } else {
                        0
                    }),
                    ptr(if self.body_kind == BodyKind::Solid {
                        previous
                    } else {
                        0
                    }),
                    ptr(if self.body_kind == BodyKind::Solid {
                        self.void_shell_id
                    } else {
                        0
                    }),
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
            let other = edge
                .fins
                .iter()
                .copied()
                .find(|&v| v != fin_id)
                .map(|fin| id_of(&self.fins, fin))
                .or_else(|| self.dummy_fin_id(fin.edge, DummyFinRole::SheetBoundary));
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
                    ptr(other.unwrap_or(0)),
                    ptr(id_of(&self.edges, fin.edge)),
                    ptr(self.fin_pcurve_id(fin_id).unwrap_or(0)),
                    ptr(head
                        .and_then(|v| self.next_fin_at_vertex(store, v, fin_id))
                        .map_or(0, |v| id_of(&self.fins, v))),
                    sense(fin.sense),
                ],
            });
        }
        for &(dummy, index) in &self.dummy_fins {
            let edge_id = dummy.edge;
            let edge = store.get(edge_id)?;
            let (vertex, other, fin_sense) = match dummy.role {
                DummyFinRole::SheetBoundary => {
                    let [actual_fin] = edge.fins.as_slice() else {
                        return Err(XtError::InvalidModel {
                            what: "sheet boundary dummy FIN edge must have exactly one real fin",
                        });
                    };
                    let actual_fin = *actual_fin;
                    let actual = store.get(actual_fin)?;
                    let tail = store.fin_tail(actual_fin)?.ok_or(XtError::InvalidModel {
                        what: "sheet boundary dummy FIN edge has no tail vertex",
                    })?;
                    (tail, id_of(&self.fins, actual_fin), actual.sense.flipped())
                }
                DummyFinRole::WireEnd => {
                    let vertex = edge.vertices[1].ok_or(XtError::InvalidModel {
                        what: "wire dummy FIN edge has no end vertex",
                    })?;
                    let other = self.dummy_fin_id(edge_id, DummyFinRole::WireStart).ok_or(
                        XtError::InvalidModel {
                            what: "wire dummy FIN pair is incomplete",
                        },
                    )?;
                    (vertex, other, Sense::Forward)
                }
                DummyFinRole::WireStart => {
                    let vertex = edge.vertices[0].ok_or(XtError::InvalidModel {
                        what: "wire dummy FIN edge has no start vertex",
                    })?;
                    let other = self.dummy_fin_id(edge_id, DummyFinRole::WireEnd).ok_or(
                        XtError::InvalidModel {
                            what: "wire dummy FIN pair is incomplete",
                        },
                    )?;
                    (vertex, other, Sense::Reversed)
                }
            };
            out.push(OutNode {
                code: code::FIN,
                index,
                values: vec![
                    ptr(0),
                    ptr(0),
                    ptr(0),
                    ptr(0),
                    ptr(id_of(&self.vertices, vertex)),
                    ptr(other),
                    ptr(id_of(&self.edges, edge_id)),
                    ptr(0),
                    ptr(0),
                    sense(fin_sense),
                ],
            });
        }
        for (position, &(edge_id, index)) in self.edges.iter().enumerate() {
            let edge = store.get(edge_id)?;
            let first_fin = edge.fins.first().map_or_else(
                || {
                    self.dummy_fin_id(edge_id, DummyFinRole::WireEnd)
                        .unwrap_or(0)
                },
                |&fin| id_of(&self.fins, fin),
            );
            out.push(OutNode {
                code: code::EDGE,
                index,
                values: vec![
                    int(index),
                    ptr(0),
                    optional_double(edge.tolerance.map(EntityTolerance::value)),
                    ptr(first_fin),
                    ptr(adjacent(&self.edges, position, -1)),
                    ptr(adjacent(&self.edges, position, 1)),
                    ptr(match edge.curve {
                        Some(curve) => self
                            .trimmed_curve_id(edge_id)
                            .unwrap_or_else(|| id_of(&self.curves, curve)),
                        None => 0,
                    }),
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
                    optional_double(vertex.tolerance.map(EntityTolerance::value)),
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
            let mut common = geom_common(
                index,
                id_of(&self.faces, face),
                adjacent(&self.surfaces, position, 1),
                adjacent(&self.surfaces, position, -1),
            );
            common[5] = ptr(self.surface_geometric_owner(store, surface_id).unwrap_or(0));
            let aux = if matches!(store.get(surface_id)?, SurfaceGeom::Nurbs(_)) {
                let ids = SurfaceAuxIds::allocate(&mut next_aux);
                push_surface_aux_nodes(&mut out, store.get(surface_id)?, ids)?;
                Some(ids)
            } else {
                None
            };
            out.push(surface_node(store.get(surface_id)?, index, common, aux));
        }
        for (position, &(edge_id, index)) in self.trimmed_curves.iter().enumerate() {
            let common = geom_common(
                index,
                id_of(&self.edges, edge_id),
                self.adjacent_curve_node(position, 1),
                self.adjacent_curve_node(position, -1),
            );
            out.push(self.trimmed_curve_node(store, edge_id, index, common)?);
        }
        for (position, &(curve_id, index)) in self.curves.iter().enumerate() {
            let direct_owner = self.edges.iter().find_map(|&(edge, id)| {
                let attached_directly = store.get(edge).ok()?.curve == Some(curve_id)
                    && self.trimmed_curve_id(edge).is_none();
                attached_directly.then_some(id)
            });
            let chain_position = self.trimmed_curves.len() + position;
            let mut common = geom_common(
                index,
                direct_owner.unwrap_or(1),
                self.adjacent_curve_node(chain_position, 1),
                self.adjacent_curve_node(chain_position, -1),
            );
            common[5] = ptr(self.curve_geometric_owner(store, curve_id).unwrap_or(0));
            let aux = if matches!(store.get(curve_id)?, CurveGeom::Nurbs(_)) {
                let ids = CurveAuxIds::allocate(&mut next_aux);
                push_curve_aux_nodes(&mut out, store.get(curve_id)?, ids)?;
                Some(ids)
            } else {
                None
            };
            out.push(curve_node(store.get(curve_id)?, index, common, aux)?);
        }
        for plan in &self.fin_pcurves {
            self.push_fin_pcurve_nodes(store, *plan, &mut next_aux, &mut out)?;
        }
        for &(edge_id, index) in &self.trimmed_curve_owners {
            let curve = store
                .get(edge_id)?
                .curve
                .expect("trimmed curve has validated basis");
            let ring = self.curve_geometric_owners(store, curve);
            let position = ring
                .iter()
                .position(|&(edge, _)| edge == edge_id)
                .expect("planned geometric owner");
            out.push(OutNode {
                code: code::GEOMETRIC_OWNER,
                index,
                values: vec![
                    ptr(self.trimmed_curve_id(edge_id).expect("planned trim")),
                    ptr(ring[(position + 1) % ring.len()].1),
                    ptr(ring[(position + ring.len() - 1) % ring.len()].1),
                    ptr(id_of(&self.curves, curve)),
                ],
            });
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
        debug_assert_eq!(next_aux, self.max_id + 1);
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

    fn dummy_fin_id(&self, edge: EdgeId, role: DummyFinRole) -> Option<u32> {
        self.dummy_fins.iter().find_map(|&(candidate, id)| {
            (candidate.edge == edge && candidate.role == role).then_some(id)
        })
    }

    fn trimmed_curve_id(&self, edge: EdgeId) -> Option<u32> {
        self.trimmed_curves
            .iter()
            .find_map(|&(candidate, id)| (candidate == edge).then_some(id))
    }

    fn fin_pcurve_id(&self, fin: FinId) -> Option<u32> {
        self.fin_pcurves
            .iter()
            .find_map(|plan| (plan.fin == fin).then_some(plan.trimmed))
    }

    fn curve_geometric_owners(&self, store: &Store, curve: CurveId) -> Vec<(EdgeId, u32)> {
        self.trimmed_curve_owners
            .iter()
            .copied()
            .filter(|&(edge, _)| store.get(edge).is_ok_and(|e| e.curve == Some(curve)))
            .collect()
    }

    fn curve_geometric_owner(&self, store: &Store, curve: CurveId) -> Option<u32> {
        self.curve_geometric_owners(store, curve)
            .first()
            .map(|&(_, id)| id)
    }

    fn surface_geometric_owners(&self, store: &Store, surface: SurfaceId) -> Vec<(FinId, u32)> {
        self.fin_pcurves
            .iter()
            .filter_map(|plan| {
                let fin = store.get(plan.fin).ok()?;
                let lp = store.get(fin.parent).ok()?;
                let face = store.get(lp.face).ok()?;
                (face.surface == surface).then_some((plan.fin, plan.surface_geometric_owner))
            })
            .collect()
    }

    fn surface_geometric_owner(&self, store: &Store, surface: SurfaceId) -> Option<u32> {
        self.surface_geometric_owners(store, surface)
            .first()
            .map(|&(_, id)| id)
    }

    fn push_fin_pcurve_nodes(
        &self,
        store: &Store,
        plan: FinPcurvePlan,
        next_aux: &mut u32,
        out: &mut Vec<OutNode>,
    ) -> Result<()> {
        let fin = store.get(plan.fin)?;
        let use_ = fin.pcurve.ok_or(XtError::InvalidModel {
            what: "curve-less tolerant edge fin has no pcurve",
        })?;
        let edge = store.get(fin.edge)?;
        let (t0, t1) = edge.bounds.ok_or(XtError::InvalidModel {
            what: "curve-less tolerant edge has no logical bounds",
        })?;
        let q0 = use_.parameter_at_edge(t0);
        let q1 = use_.parameter_at_edge(t1);
        let lp = store.get(fin.parent)?;
        let face = store.get(lp.face)?;
        let surface = store.get(face.surface)?.as_surface();
        let pcurve = store.get(use_.curve())?.as_curve();
        let uv0 = pcurve.eval(q0);
        let uv1 = pcurve.eval(q1);
        let point0 = surface.eval([uv0.x, uv0.y]);
        let point1 = surface.eval([uv1.x, uv1.y]);

        let position = self
            .fin_pcurves
            .iter()
            .position(|candidate| candidate.fin == plan.fin)
            .expect("planned fin pcurve");
        let chain_position = self.trimmed_curves.len() + self.curves.len() + position * 2;
        let mut trimmed_values = geom_common(
            plan.trimmed,
            id_of(&self.fins, plan.fin),
            self.adjacent_curve_node(chain_position, 1),
            self.adjacent_curve_node(chain_position, -1),
        );
        trimmed_values.extend([
            ptr(plan.sp_curve),
            vector(point0),
            vector(point1),
            Value::Double(q0),
            Value::Double(q1),
        ]);
        out.push(OutNode {
            code: code::TRIMMED_CURVE,
            index: plan.trimmed,
            values: trimmed_values,
        });

        let mut sp_values = geom_common(
            plan.sp_curve,
            1,
            self.adjacent_curve_node(chain_position + 1, 1),
            self.adjacent_curve_node(chain_position + 1, -1),
        );
        sp_values[5] = ptr(plan.sp_geometric_owner);
        sp_values[6] = Value::Char(if q1 > q0 { '+' } else { '-' });
        sp_values.extend([
            ptr(id_of(&self.surfaces, face.surface)),
            ptr(plan.b_curve),
            ptr(0),
            Value::Null,
        ]);
        out.push(OutNode {
            code: code::SP_CURVE,
            index: plan.sp_curve,
            values: sp_values,
        });

        let aux = CurveAuxIds::allocate(next_aux);
        let nurbs = pcurve_nurbs(store, plan.fin)?;
        push_pcurve_aux_nodes(out, &nurbs, aux);
        let mut bcurve_values = geom_common(plan.b_curve, 0, 0, 0);
        bcurve_values.extend([ptr(aux.nurbs), ptr(0)]);
        out.push(OutNode {
            code: code::B_CURVE,
            index: plan.b_curve,
            values: bcurve_values,
        });

        out.push(OutNode {
            code: code::GEOMETRIC_OWNER,
            index: plan.sp_geometric_owner,
            values: vec![
                ptr(plan.trimmed),
                ptr(plan.sp_geometric_owner),
                ptr(plan.sp_geometric_owner),
                ptr(plan.sp_curve),
            ],
        });
        let surface_owners = self.surface_geometric_owners(store, face.surface);
        let surface_position = surface_owners
            .iter()
            .position(|&(fin, _)| fin == plan.fin)
            .expect("planned surface geometric owner");
        out.push(OutNode {
            code: code::GEOMETRIC_OWNER,
            index: plan.surface_geometric_owner,
            values: vec![
                ptr(plan.sp_curve),
                ptr(surface_owners[(surface_position + 1) % surface_owners.len()].1),
                ptr(surface_owners
                    [(surface_position + surface_owners.len() - 1) % surface_owners.len()]
                .1),
                ptr(id_of(&self.surfaces, face.surface)),
            ],
        });
        Ok(())
    }

    fn first_curve_id(&self) -> u32 {
        self.trimmed_curves.first().map_or_else(
            || {
                self.curves.first().map_or_else(
                    || self.fin_pcurves.first().map_or(0, |plan| plan.trimmed),
                    |&(_, id)| id,
                )
            },
            |&(_, id)| id,
        )
    }

    fn curve_node_id(&self, position: usize) -> Option<u32> {
        if position < self.trimmed_curves.len() {
            return Some(self.trimmed_curves[position].1);
        }
        self.curves
            .get(position.checked_sub(self.trimmed_curves.len())?)
            .map(|&(_, id)| id)
            .or_else(|| {
                let offset = position.checked_sub(self.trimmed_curves.len() + self.curves.len())?;
                let plan = self.fin_pcurves.get(offset / 2)?;
                Some(if offset % 2 == 0 {
                    plan.trimmed
                } else {
                    plan.sp_curve
                })
            })
    }

    fn adjacent_curve_node(&self, position: usize, direction: i8) -> u32 {
        match direction {
            -1 if position > 0 => self.curve_node_id(position - 1).unwrap_or(0),
            1 => self.curve_node_id(position + 1).unwrap_or(0),
            _ => 0,
        }
    }

    fn trimmed_curve_node(
        &self,
        store: &Store,
        edge_id: EdgeId,
        index: u32,
        mut values: Vec<Value>,
    ) -> Result<OutNode> {
        let edge = store.get(edge_id)?;
        let (t0, t1) = edge.bounds.ok_or(XtError::InvalidModel {
            what: "trimmed curve edge has no parameter bounds",
        })?;
        let start = edge.vertices[0].ok_or(XtError::InvalidModel {
            what: "trimmed curve edge has no start vertex",
        })?;
        let end = edge.vertices[1].ok_or(XtError::InvalidModel {
            what: "trimmed curve edge has no end vertex",
        })?;
        values.extend([
            ptr(id_of(&self.curves, edge.curve.expect("validated curve"))),
            vector(store.vertex_position(start)?),
            vector(store.vertex_position(end)?),
            Value::Double(t0),
            Value::Double(t1),
        ]);
        Ok(OutNode {
            code: code::TRIMMED_CURVE,
            index,
            values,
        })
    }

    fn push_solid_scaffold_nodes(&self, out: &mut Vec<OutNode>) {
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
                ptr(self.shell_id),
                Value::Char('S'),
            ],
        });
        out.push(OutNode {
            code: code::SHELL,
            index: self.shell_id,
            values: vec![
                int(self.shell_id),
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
    }

    fn push_sheet_scaffold_nodes(&self, out: &mut Vec<OutNode>) {
        out.push(OutNode {
            code: code::REGION,
            index: 2,
            values: vec![
                int(2),
                ptr(0),
                ptr(1),
                ptr(0),
                ptr(0),
                ptr(self.shell_id),
                Value::Char('V'),
            ],
        });
        out.push(OutNode {
            code: code::SHELL,
            index: self.shell_id,
            values: vec![
                int(self.shell_id),
                ptr(0),
                ptr(1),
                ptr(0),
                ptr(first_id(&self.faces)),
                ptr(0),
                ptr(0),
                ptr(2),
                ptr(0),
            ],
        });
    }

    fn push_wire_scaffold_nodes(&self, out: &mut Vec<OutNode>) {
        out.push(OutNode {
            code: code::REGION,
            index: 2,
            values: vec![
                int(2),
                ptr(0),
                ptr(1),
                ptr(0),
                ptr(0),
                ptr(self.shell_id),
                Value::Char('V'),
            ],
        });
        out.push(OutNode {
            code: code::SHELL,
            index: self.shell_id,
            values: vec![
                int(self.shell_id),
                ptr(0),
                ptr(1),
                ptr(0),
                ptr(0),
                ptr(first_id(&self.edges)),
                ptr(0),
                ptr(2),
                ptr(0),
            ],
        });
    }

    fn push_acorn_scaffold_nodes(&self, out: &mut Vec<OutNode>) {
        out.push(OutNode {
            code: code::REGION,
            index: 2,
            values: vec![
                int(2),
                ptr(0),
                ptr(1),
                ptr(0),
                ptr(0),
                ptr(self.shell_id),
                Value::Char('V'),
            ],
        });
        out.push(OutNode {
            code: code::SHELL,
            index: self.shell_id,
            values: vec![
                int(self.shell_id),
                ptr(0),
                ptr(1),
                ptr(0),
                ptr(0),
                ptr(0),
                ptr(first_id(&self.vertices)),
                ptr(2),
                ptr(0),
            ],
        });
    }
}

impl Scaffold {
    fn validate(store: &Store, body: &ktopo::entity::Body) -> Result<Self> {
        match body.kind {
            BodyKind::Solid => Self::solid(store, body),
            BodyKind::Sheet => Self::sheet(store, body),
            BodyKind::Wire => Self::wire(store, body),
            BodyKind::Acorn => Self::acorn(store, body),
        }
    }

    fn solid(store: &Store, body: &ktopo::entity::Body) -> Result<Self> {
        if body.regions.len() != 2 {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "solid text writing requires one void and one solid region",
            });
        }
        let void_region = body.regions[0];
        let solid_region = body.regions[1];
        let vr = store.get(void_region)?;
        let sr = store.get(solid_region)?;
        if vr.kind != RegionKind::Void || !vr.shells.is_empty() || sr.kind != RegionKind::Solid {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "solid text writing requires the standard solid region scaffold",
            });
        }
        let [shell] = sr.shells.as_slice() else {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "solid text writing requires exactly one solid shell",
            });
        };
        let sh = store.get(*shell)?;
        if sh.faces.is_empty() || !sh.edges.is_empty() || sh.vertex.is_some() {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "wireframe, acorn, and empty shells are not writable yet",
            });
        }
        Ok(Scaffold {
            shell_id: 4,
            void_shell_id: 5,
        })
    }

    fn sheet(store: &Store, body: &ktopo::entity::Body) -> Result<Self> {
        let [void_region] = body.regions.as_slice() else {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "sheet text writing requires one void region",
            });
        };
        let region = store.get(*void_region)?;
        let [shell] = region.shells.as_slice() else {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "sheet text writing requires exactly one shell",
            });
        };
        let shell = store.get(*shell)?;
        if region.kind != RegionKind::Void
            || shell.faces.is_empty()
            || !shell.edges.is_empty()
            || shell.vertex.is_some()
        {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "sheet text writing requires one non-empty face shell in the void region",
            });
        }
        Ok(Scaffold {
            shell_id: 3,
            void_shell_id: 0,
        })
    }

    fn wire(store: &Store, body: &ktopo::entity::Body) -> Result<Self> {
        let [void_region] = body.regions.as_slice() else {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "wire text writing requires one void region",
            });
        };
        let region = store.get(*void_region)?;
        let [shell] = region.shells.as_slice() else {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "wire text writing requires exactly one shell",
            });
        };
        let shell = store.get(*shell)?;
        if region.kind != RegionKind::Void
            || !shell.faces.is_empty()
            || shell.edges.is_empty()
            || shell.vertex.is_some()
        {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "wire text writing requires one non-empty edge shell in the void region",
            });
        }
        Ok(Scaffold {
            shell_id: 3,
            void_shell_id: 0,
        })
    }

    fn acorn(store: &Store, body: &ktopo::entity::Body) -> Result<Self> {
        let [void_region] = body.regions.as_slice() else {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "acorn text writing requires one void region",
            });
        };
        let region = store.get(*void_region)?;
        let [shell] = region.shells.as_slice() else {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "acorn text writing requires exactly one shell",
            });
        };
        let shell = store.get(*shell)?;
        if region.kind != RegionKind::Void
            || !shell.faces.is_empty()
            || !shell.edges.is_empty()
            || shell.vertex.is_none()
        {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "acorn text writing requires one vertex-only shell in the void region",
            });
        }
        Ok(Scaffold {
            shell_id: 3,
            void_shell_id: 0,
        })
    }

    fn first_entity_id(self) -> u32 {
        if self.void_shell_id == 0 { 4 } else { 6 }
    }
}

impl CurveAuxIds {
    fn allocate(next: &mut u32) -> Self {
        let ids = CurveAuxIds {
            nurbs: *next,
            poles: *next + 1,
            knot_mult: *next + 2,
            knots: *next + 3,
        };
        *next += 4;
        ids
    }
}

impl SurfaceAuxIds {
    fn allocate(next: &mut u32) -> Self {
        let ids = SurfaceAuxIds {
            nurbs: *next,
            poles: *next + 1,
            u_knot_mult: *next + 2,
            v_knot_mult: *next + 3,
            u_knots: *next + 4,
            v_knots: *next + 5,
        };
        *next += 6;
        ids
    }
}

fn body_type(kind: BodyKind) -> i64 {
    match kind {
        BodyKind::Solid => 1,
        BodyKind::Sheet => 3,
        BodyKind::Wire => 2,
        BodyKind::Acorn => 2,
    }
}

fn validate_edge_fin_count(edge: &Edge, kind: BodyKind) -> Result<()> {
    match kind {
        BodyKind::Solid if edge.fins.len() != 2 => {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterEdgeTopology,
                what: "solid edges must have exactly two fins",
            });
        }
        BodyKind::Sheet if edge.fins.len() == 2 => {}
        BodyKind::Sheet
            if edge.fins.len() == 1 && edge.vertices == [None, None] && edge.bounds.is_none() => {}
        BodyKind::Sheet
            if edge.fins.len() == 1
                && edge.vertices[0].is_some()
                && edge.vertices[1].is_some()
                && edge.bounds.is_some() => {}
        BodyKind::Sheet => {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterEdgeTopology,
                what: "unsupported sheet edge fin topology",
            });
        }
        BodyKind::Wire if !edge.fins.is_empty() => {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterEdgeTopology,
                what: "wire edges must not have real fins",
            });
        }
        BodyKind::Acorn => unreachable!("rejected during planning"),
        _ => {}
    }
    Ok(())
}

fn push_dummy_fins(out: &mut Vec<DummyFin>, edge_id: EdgeId, edge: &Edge, kind: BodyKind) {
    match kind {
        BodyKind::Sheet
            if edge.fins.len() == 1
                && edge.vertices[0].is_some()
                && edge.vertices[1].is_some()
                && edge.bounds.is_some() =>
        {
            out.push(DummyFin {
                edge: edge_id,
                role: DummyFinRole::SheetBoundary,
            });
        }
        BodyKind::Wire
            if edge.fins.is_empty()
                && edge.vertices[0].is_some()
                && edge.vertices[1].is_some()
                && edge.bounds.is_some() =>
        {
            out.push(DummyFin {
                edge: edge_id,
                role: DummyFinRole::WireEnd,
            });
            out.push(DummyFin {
                edge: edge_id,
                role: DummyFinRole::WireStart,
            });
        }
        _ => {}
    }
}

fn surface_node(
    surface: &SurfaceGeom,
    index: u32,
    mut values: Vec<Value>,
    aux: Option<SurfaceAuxIds>,
) -> OutNode {
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
        SurfaceGeom::Nurbs(_) => {
            let aux = aux.expect("planned NURBS surface auxiliaries");
            values.extend([ptr(aux.nurbs), ptr(0)]);
            code::B_SURFACE
        }
    };
    OutNode {
        code,
        index,
        values,
    }
}

fn curve_node(
    curve: &CurveGeom,
    index: u32,
    mut values: Vec<Value>,
    aux: Option<CurveAuxIds>,
) -> Result<OutNode> {
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
            let aux = aux.expect("planned NURBS curve auxiliaries");
            values.extend([ptr(aux.nurbs), ptr(0)]);
            code::B_CURVE
        }
    };
    Ok(OutNode {
        code,
        index,
        values,
    })
}

fn push_curve_aux_nodes(out: &mut Vec<OutNode>, curve: &CurveGeom, ids: CurveAuxIds) -> Result<()> {
    let CurveGeom::Nurbs(curve) = curve else {
        return Ok(());
    };
    let (knots, multiplicities) = compressed_knots(curve.knots());
    let rational = curve.is_rational();
    let vertex_dim = if rational { 4 } else { 3 };
    out.push(OutNode {
        code: code::NURBS_CURVE,
        index: ids.nurbs,
        values: vec![
            Value::Int(curve.degree() as i64),
            Value::Int(curve.points().len() as i64),
            Value::Int(vertex_dim),
            Value::Int(knots.len() as i64),
            Value::Int(0),
            Value::Logical(false),
            Value::Logical(false),
            Value::Logical(rational),
            Value::Int(0),
            ptr(ids.poles),
            ptr(ids.knot_mult),
            ptr(ids.knots),
        ],
    });
    out.push(bspline_vertices_node(ids.poles, flatten_curve_poles(curve)));
    out.push(int_values_node(ids.knot_mult, &multiplicities));
    out.push(knot_values_node(ids.knots, knots));
    Ok(())
}

fn push_pcurve_aux_nodes(out: &mut Vec<OutNode>, curve: &NurbsCurve2d, ids: CurveAuxIds) {
    let (knots, multiplicities) = compressed_knots(curve.knots());
    let rational = curve.weights().is_some();
    let vertex_dim = if rational { 3 } else { 2 };
    out.push(OutNode {
        code: code::NURBS_CURVE,
        index: ids.nurbs,
        values: vec![
            Value::Int(curve.degree() as i64),
            Value::Int(curve.points().len() as i64),
            Value::Int(vertex_dim),
            Value::Int(knots.len() as i64),
            Value::Int(0),
            Value::Logical(false),
            Value::Logical(false),
            Value::Logical(rational),
            Value::Int(0),
            ptr(ids.poles),
            ptr(ids.knot_mult),
            ptr(ids.knots),
        ],
    });
    out.push(bspline_vertices_node(
        ids.poles,
        flatten_pcurve_poles(curve),
    ));
    out.push(int_values_node(ids.knot_mult, &multiplicities));
    out.push(knot_values_node(ids.knots, knots));
}

fn push_surface_aux_nodes(
    out: &mut Vec<OutNode>,
    surface: &SurfaceGeom,
    ids: SurfaceAuxIds,
) -> Result<()> {
    let SurfaceGeom::Nurbs(surface) = surface else {
        return Ok(());
    };
    let (u_knots, u_multiplicities) = compressed_knots(surface.knots(Dir::U));
    let (v_knots, v_multiplicities) = compressed_knots(surface.knots(Dir::V));
    let rational = surface.is_rational();
    let vertex_dim = if rational { 4 } else { 3 };
    let (nu, nv) = surface.net_size();
    out.push(OutNode {
        code: code::NURBS_SURF,
        index: ids.nurbs,
        values: vec![
            Value::Logical(false),
            Value::Logical(false),
            Value::Int(surface.degree_u() as i64),
            Value::Int(surface.degree_v() as i64),
            Value::Int(nu as i64),
            Value::Int(nv as i64),
            Value::Int(0),
            Value::Int(0),
            Value::Int(u_knots.len() as i64),
            Value::Int(v_knots.len() as i64),
            Value::Logical(rational),
            Value::Logical(false),
            Value::Logical(false),
            Value::Int(0),
            Value::Int(vertex_dim),
            ptr(ids.poles),
            ptr(ids.u_knot_mult),
            ptr(ids.v_knot_mult),
            ptr(ids.u_knots),
            ptr(ids.v_knots),
        ],
    });
    out.push(bspline_vertices_node(
        ids.poles,
        flatten_surface_poles(surface),
    ));
    out.push(int_values_node(ids.u_knot_mult, &u_multiplicities));
    out.push(int_values_node(ids.v_knot_mult, &v_multiplicities));
    out.push(knot_values_node(ids.u_knots, u_knots));
    out.push(knot_values_node(ids.v_knots, v_knots));
    Ok(())
}

fn aux_node_count(store: &Store, surfaces: &[SurfaceId], curves: &[CurveId]) -> Result<u32> {
    let mut count = 0u32;
    for &surface in surfaces {
        if matches!(store.get(surface)?, SurfaceGeom::Nurbs(_)) {
            count += 6;
        }
    }
    for &curve in curves {
        if matches!(store.get(curve)?, CurveGeom::Nurbs(_)) {
            count += 4;
        }
    }
    Ok(count)
}

fn validate_nurbs_curve(curve: &NurbsCurve) -> Result<()> {
    if curve.periodicity().is_some() {
        return Err(XtError::Unsupported {
            capability: XtCapability::PeriodicNurbsCurves,
            what: "periodic NURBS curves",
        });
    }
    for &point in curve.points() {
        check_in_size_box(point.to_array())?;
    }
    Ok(())
}

fn validate_nurbs_surface(surface: &NurbsSurface) -> Result<()> {
    if surface.periodicity() != [None, None] {
        return Err(XtError::Unsupported {
            capability: XtCapability::PeriodicNurbsSurfaces,
            what: "periodic NURBS surfaces",
        });
    }
    for &point in surface.points() {
        check_in_size_box(point.to_array())?;
    }
    Ok(())
}

fn compressed_knots(knots: &KnotVector) -> (Vec<f64>, Vec<i64>) {
    let raw = knots.as_slice();
    let mut values = Vec::new();
    let mut multiplicities = Vec::new();
    let mut i = 0;
    while i < raw.len() {
        let value = raw[i];
        let mut j = i + 1;
        while j < raw.len() && raw[j] == value {
            j += 1;
        }
        values.push(value);
        multiplicities.push((j - i) as i64);
        i = j;
    }
    (values, multiplicities)
}

fn flatten_curve_poles(curve: &NurbsCurve) -> Vec<f64> {
    flatten_poles(curve.points(), curve.weights())
}

fn flatten_pcurve_poles(curve: &NurbsCurve2d) -> Vec<f64> {
    let mut values =
        Vec::with_capacity(curve.points().len() * if curve.weights().is_some() { 3 } else { 2 });
    match curve.weights() {
        Some(weights) => {
            for (&point, &weight) in curve.points().iter().zip(weights) {
                values.extend([point.x * weight, point.y * weight, weight]);
            }
        }
        None => {
            for point in curve.points() {
                values.extend([point.x, point.y]);
            }
        }
    }
    values
}

fn pcurve_nurbs(store: &Store, fin_id: FinId) -> Result<NurbsCurve2d> {
    let use_: FinPcurve = store.get(fin_id)?.pcurve.ok_or(XtError::InvalidModel {
        what: "curve-less tolerant edge fin has no pcurve",
    })?;
    if !use_.chart().is_identity() || use_.seam().is_some() {
        return Err(XtError::Unsupported {
            capability: XtCapability::PeriodicPcurves,
            what: "explicit periodic chart or seam metadata on tolerant fin pcurves",
        });
    }
    match store.get(use_.curve())? {
        Curve2dGeom::Line(line) => {
            let range = use_.range();
            NurbsCurve2d::new(
                1,
                vec![range.lo, range.lo, range.hi, range.hi],
                vec![line.eval(range.lo), line.eval(range.hi)],
                None,
            )
            .map_err(XtError::Kernel)
        }
        Curve2dGeom::Nurbs(curve) => Ok(curve.clone()),
        Curve2dGeom::Circle(_) => Err(XtError::Unsupported {
            capability: XtCapability::CircularPcurves,
            what: "circular pcurves on curve-less tolerant edges",
        }),
    }
}

fn flatten_surface_poles(surface: &NurbsSurface) -> Vec<f64> {
    flatten_poles(surface.points(), surface.weights())
}

fn flatten_poles(points: &[Point3], weights: Option<&[f64]>) -> Vec<f64> {
    let mut values = Vec::with_capacity(points.len() * if weights.is_some() { 4 } else { 3 });
    match weights {
        Some(weights) => {
            for (&point, &weight) in points.iter().zip(weights) {
                values.extend([point.x * weight, point.y * weight, point.z * weight, weight]);
            }
        }
        None => {
            for point in points {
                values.extend([point.x, point.y, point.z]);
            }
        }
    }
    values
}

fn bspline_vertices_node(index: u32, values: Vec<f64>) -> OutNode {
    OutNode {
        code: code::BSPLINE_VERTICES,
        index,
        values: vec![Value::Arr(values.into_iter().map(Value::Double).collect())],
    }
}

fn knot_values_node(index: u32, values: Vec<f64>) -> OutNode {
    OutNode {
        code: code::KNOT_SET,
        index,
        values: vec![Value::Arr(values.into_iter().map(Value::Double).collect())],
    }
}

fn int_values_node(index: u32, values: &[i64]) -> OutNode {
    OutNode {
        code: code::KNOT_MULT,
        index,
        values: vec![Value::Arr(values.iter().copied().map(Value::Int).collect())],
    }
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
        if def.is_variable() {
            let variable_len = node
                .values
                .last()
                .and_then(|value| match value {
                    Value::Arr(values) => Some(values.len()),
                    Value::Str(value) => Some(value.len()),
                    _ => None,
                })
                .ok_or(XtError::InvalidModel {
                    what: "writer produced a variable node without an array value",
                })?;
            data.push_str(&format!("{} {} {} ", node.code, variable_len, node.index));
        } else {
            data.push_str(&format!("{} {} ", node.code, node.index));
        }
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
        Value::Arr(values) => {
            for value in values {
                write_value(out, value)?;
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

fn assign_dummy_fins(dummy_fins: &[DummyFin], next: &mut u32) -> Vec<(DummyFin, u32)> {
    dummy_fins
        .iter()
        .map(|&dummy| {
            let id = *next;
            *next += 1;
            (dummy, id)
        })
        .collect()
}

fn assign_fin_pcurves(fins: &[FinId], next: &mut u32) -> Vec<FinPcurvePlan> {
    fins.iter()
        .map(|&fin| {
            let plan = FinPcurvePlan {
                fin,
                trimmed: *next,
                sp_curve: *next + 1,
                b_curve: *next + 2,
                sp_geometric_owner: *next + 3,
                surface_geometric_owner: *next + 4,
            };
            *next += 5;
            plan
        })
        .collect()
}

fn push_interned<T>(values: &mut Vec<Handle<T>>, value: Handle<T>) {
    if !values.contains(&value) {
        values.push(value);
    }
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

fn optional_double(value: Option<f64>) -> Value {
    value.map_or(Value::Null, Value::Double)
}

fn sense(value: Sense) -> Value {
    Value::Char(if value.is_forward() { '+' } else { '-' })
}

fn vector(value: Vec3) -> Value {
    Value::Vector(Some(value.to_array()))
}
