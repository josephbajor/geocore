//! Deterministic XT text emission for checker-clean bodies.
//!
//! Output is modern embedded-schema text (`SCH_2700142_26105_13006`) whose
//! node layouts follow base schema 13006 plus the V26105 BODY/REGION edits.
//! Exact bounded edges reference their basis curve directly (bounds are
//! implied by the vertices, as in every real exact-modeling corpus file);
//! TRIMMED_CURVE/GEOMETRIC_OWNER chains are emitted only for tolerant
//! fin SP-curves. Unsupported model classes fail explicitly rather than
//! being approximated.

use crate::error::{Result, XtCapability, XtError};
use crate::parse::Value;
use crate::schema::{base_schema, code};
use kcore::arena::Handle;
use kcore::math;
use kcore::tolerance::{LINEAR_RESOLUTION, Tolerances, check_in_size_box};
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::curve2d::{Curve2d, NurbsCurve2d};
use kgeom::nurbs::{KnotVector, NurbsCurve, NurbsSurface};
use kgeom::surface::{Dir, Surface};
use kgeom::vec::{Point3, Vec3};
use kgraph::{EvalLimits, SurfaceDerivativeOrder, VerifiedIntersectionCarrier};
use ktopo::check::check_body;
use ktopo::entity::{
    BodyId, BodyKind, CurveId, Edge, EdgeId, FaceId, FinId, FinPcurve, LoopId, PointId, RegionId,
    RegionKind, Sense, ShellId, SurfaceId, VertexId,
};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::store::Store;
use ktopo::tolerance::EntityTolerance;
use std::collections::BTreeSet;

const VERSION: &str = ": TRANSMIT FILE created by modeller version 2700142";
/// Declared file schema (PART2 header form, without the base suffix).
///
/// The writer emits the modern *embedded-schema* transmit form: the flag
/// sequence declares `SCH_2700142_26105_13006` and every node type carries
/// its layout at first occurrence, so a receiving Parasolid needs no
/// external schema files. Plain pre-embedded-schema text (previously
/// `SCH_1300000_13006`) is rejected by at least one production Parasolid
/// host (Onshape: "Invalid or corrupt input file"), first observed through
/// the oracle loop on 2026-07-11.
const SCHEMA: &str = "SCH_2700142_26105";
/// Flag-sequence schema key: file schema plus the `_13006` base suffix that
/// marks the embedded-schema mechanism.
const SCHEMA_KEY: &str = "SCH_2700142_26105_13006";
/// Maximum node-type count in the flag sequence, as real V27 files declare.
const MAX_NODE_TYPES: u16 = 196;
/// BODY first-occurrence embedded description, byte-for-byte as real V27
/// Parasolid emits it: copy base fields 1-13, delete-and-reinsert `owner`
/// (its pointer class changed), copy base fields 15-23, then append the
/// seven V26105 bookkeeping fields. Extracted from
/// `tests/fixtures/disk_nat.x_t`; a test pins it against that file.
const BODY_EDIT_SCRIPT: &str = "30 CCCCCCCCCCCCCDI5 owner1040 0 CCCCCCCCCA13 \
                                boundary_mesh1006 0 A16 index_map_offset0 0 1 dA9 index_map82 0 \
                                A17 node_id_index_map82 0 A20 schema_embedding_map82 0 A5 \
                                child12 0 A14 lowest_node_id0 0 1 dZ";
/// The V26105 BODY layout appends these fields to the base-13006 layout:
/// boundary_mesh, index_map_offset, index_map, node_id_index_map,
/// schema_embedding_map, child, lowest_node_id.
const BODY_APPENDED_FIELDS: usize = 7;
/// REGION first-occurrence embedded description, byte-for-byte as real V27
/// Parasolid emits it: copy the seven base fields, append `owner`.
const REGION_EDIT_SCRIPT: &str = "8 CCCCCCCA5 owner12 0 Z";
/// The V26105 REGION layout appends one field: owner.
const REGION_APPENDED_FIELDS: usize = 1;

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
    curves: Vec<(CurveId, u32)>,
    fin_pcurves: Vec<FinPcurvePlan>,
    points: Vec<(PointId, u32)>,
    scaffold: Scaffold,
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

#[derive(Debug, Clone)]
struct Scaffold {
    shell_id: u32,
    solid: Option<SolidScaffold>,
}

#[derive(Debug, Clone)]
struct SolidScaffold {
    regions: Vec<(RegionId, u32)>,
    solid_region: RegionId,
    shell_pairs: Vec<SolidShellPair>,
}

#[derive(Debug, Clone, Copy)]
struct SolidShellPair {
    shell: ShellId,
    void_region: RegionId,
    back_id: u32,
    front_id: u32,
}

#[derive(Debug, Clone, Copy)]
struct CurveAuxIds {
    nurbs: u32,
    data: u32,
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

/// Native X_T curve geometry retained by one supported graph descriptor.
///
/// Verified analytic intersections deliberately borrow their immutable
/// certificate carrier rather than reconstructing it from endpoints or
/// source surfaces. This preserves the exact carrier bits while keeping all
/// other procedural families on the typed refusal path.
#[derive(Debug, Clone, Copy)]
enum NativeCurve<'curve> {
    Line(Line),
    Circle(Circle),
    Ellipse(Ellipse),
    Nurbs(&'curve NurbsCurve),
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
            if f.tolerance().is_some() {
                return Err(XtError::Unsupported {
                    capability: XtCapability::FaceTolerances,
                    what: "schema-13006 FACE tolerance is required to be null",
                });
            }
            plan_surface_dependencies(store, f.surface(), &mut surface_handles)?;
            for &lp in f.loops() {
                if store.get(lp)?.fins().is_empty() {
                    return Err(XtError::InvalidModel { what: "empty loop" });
                }
                loop_handles.push(lp);
                fin_handles.extend_from_slice(store.get(lp)?.fins());
            }
        }
        let mut curve_handles = Vec::new();
        let mut fin_pcurve_handles = Vec::new();
        let mut dummy_fin_specs = Vec::new();
        for &edge in &edge_handles {
            let e = store.get(edge)?;
            validate_edge_fin_count(e, b.kind())?;
            push_dummy_fins(&mut dummy_fin_specs, edge, e, b.kind());
            match e.curve() {
                Some(curve) => {
                    push_interned(&mut curve_handles, curve);
                    let curve = store.get(curve)?;
                    if let CurveGeom::Intersection(descriptor) = curve {
                        validate_verified_intersection_edge(descriptor, e)?;
                    }
                    match (native_curve(curve)?, e.vertices(), e.bounds()) {
                        (NativeCurve::Line(_), [Some(_), Some(_)], Some(_)) => {}
                        (NativeCurve::Circle(_), [None, None], None) => {}
                        (NativeCurve::Circle(_), [Some(_), Some(_)], Some(_)) => {}
                        (NativeCurve::Ellipse(_), [None, None], None) => {}
                        (NativeCurve::Ellipse(_), [Some(_), Some(_)], Some(_)) => {}
                        (NativeCurve::Nurbs(n), [Some(_), Some(_)], Some(_)) => {
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
                    if e.fins().is_empty() {
                        return Err(XtError::Unsupported {
                            capability: XtCapability::TolerantWireEdges,
                            what: "curve-less tolerant wire edges",
                        });
                    }
                    for &fin in e.fins() {
                        pcurve_nurbs(store, fin)?;
                        fin_pcurve_handles.push(fin);
                    }
                }
            }
        }
        // Procedural faces cannot reconstruct exact-edge incidence from the
        // 3D basis alone. Preserve every authored pcurve as a trimmed
        // SP_CURVE, in the already-deterministic fin traversal order.
        for &fin in &fin_handles {
            let fin_value = store.get(fin)?;
            let face = store.get(fin_value.parent())?.face();
            if matches!(
                store.get(store.get(face)?.surface())?,
                SurfaceGeom::Offset(_)
            ) {
                if fin_value.pcurve().is_none() {
                    return Err(XtError::InvalidModel {
                        what: "offset-face fin requires an authored pcurve for X_T export",
                    });
                }
                pcurve_nurbs(store, fin)?;
                push_interned(&mut fin_pcurve_handles, fin);
            }
        }
        let mut point_handles = Vec::new();
        for &vertex in &vertex_handles {
            let v = store.get(vertex)?;
            check_in_size_box(store.get(v.point())?.to_array())?;
            push_interned(&mut point_handles, v.point());
        }
        for &surface in &surface_handles {
            match store.get(surface)? {
                SurfaceGeom::Nurbs(s) => validate_nurbs_surface(s)?,
                SurfaceGeom::Plane(s) => check_in_size_box(s.frame().origin().to_array())?,
                SurfaceGeom::Cylinder(s) => check_in_size_box(s.frame().origin().to_array())?,
                SurfaceGeom::Cone(s) => check_in_size_box(s.frame().origin().to_array())?,
                SurfaceGeom::Sphere(s) => check_in_size_box(s.frame().origin().to_array())?,
                SurfaceGeom::Torus(s) => check_in_size_box(s.frame().origin().to_array())?,
                SurfaceGeom::Offset(offset) => {
                    if !offset.signed_distance().is_finite()
                        || offset.signed_distance().abs() <= LINEAR_RESOLUTION
                    {
                        return Err(XtError::InvalidModel {
                            what: "offset distance must be finite and exceed linear resolution",
                        });
                    }
                    if matches!(store.get(offset.basis())?, SurfaceGeom::Offset(_)) {
                        return Err(XtError::Unsupported {
                            capability: XtCapability::NestedOffsetExport,
                            what: "nested offset-surface export",
                        });
                    }
                }
                _ => {
                    return Err(XtError::Unsupported {
                        capability: XtCapability::ProceduralSurfaces,
                        what: "unimplemented geometry-graph surface class",
                    });
                }
            }
        }
        reject_shared_offset_bases(store, &surface_handles)?;
        reject_direct_faces_on_offset_bases(store, &face_handles)?;

        let mut next = scaffold.first_entity_id();
        let faces = assign(&face_handles, &mut next);
        let loops = assign(&loop_handles, &mut next);
        let fins = assign(&fin_handles, &mut next);
        let dummy_fins = assign_dummy_fins(&dummy_fin_specs, &mut next);
        let edges = assign(&edge_handles, &mut next);
        let vertices = assign(&vertex_handles, &mut next);
        let surfaces = assign(&surface_handles, &mut next);
        let curves = assign(&curve_handles, &mut next);
        let fin_pcurves = assign_fin_pcurves(&fin_pcurve_handles, &mut next);
        let points = assign(&point_handles, &mut next);
        let first_aux_id = next;
        next += aux_node_count(store, &surface_handles, &curve_handles)?;
        next += 5 * u32::try_from(fin_pcurves.len()).map_err(|_| XtError::InvalidModel {
            what: "too many tolerant fin pcurves",
        })?;
        Ok(Plan {
            body_kind: b.kind(),
            faces,
            loops,
            fins,
            dummy_fins,
            edges,
            vertices,
            surfaces,
            curves,
            fin_pcurves,
            points,
            scaffold,
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
                ptr(self.scaffold.shell_id),
                ptr(first_id(&self.surfaces)),
                ptr(self.first_curve_id()),
                ptr(first_id(&self.points)),
                ptr(2),
                ptr(first_id(&self.edges)),
                ptr(first_id(&self.vertices)),
                // V26105 appended fields (see BODY_EDIT_SCRIPT): null
                // boundary_mesh, index_map_offset, index_map,
                // node_id_index_map, schema_embedding_map, child,
                // lowest_node_id — real Parasolid writes them as zeros.
                ptr(0),
                Value::Int(0),
                ptr(0),
                ptr(0),
                ptr(0),
                ptr(0),
                Value::Int(0),
            ],
        });
        match self.body_kind {
            BodyKind::Solid => self.push_solid_scaffold_nodes(store, &mut out)?,
            BodyKind::Sheet => self.push_sheet_scaffold_nodes(&mut out),
            BodyKind::Wire => self.push_wire_scaffold_nodes(&mut out),
            BodyKind::Acorn => self.push_acorn_scaffold_nodes(&mut out),
        }

        for (position, &(face_id, index)) in self.faces.iter().enumerate() {
            let face = store.get(face_id)?;
            let first_loop = face.loops().first().map_or(0, |&lp| id_of(&self.loops, lp));
            let (shell_id, front_shell_id, next, previous) =
                self.face_scaffold(store, face_id, position)?;
            let surface_faces: Vec<_> = self
                .faces
                .iter()
                .copied()
                .filter(|&(candidate, _)| {
                    store
                        .get(candidate)
                        .is_ok_and(|f| f.surface() == face.surface())
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
                    ptr(shell_id),
                    ptr(id_of(&self.surfaces, face.surface())),
                    sense(face.sense()),
                    ptr(owner_ring_adjacent(&surface_faces, surface_position, 1)),
                    ptr(owner_ring_adjacent(&surface_faces, surface_position, -1)),
                    // Solid faces front the void-region shell; sheet faces
                    // bound their own shell from both sides and front it
                    // too (disk_nat.x_t). The front chain mirrors the face
                    // chain either way.
                    ptr(next),
                    ptr(previous),
                    ptr(front_shell_id),
                ],
            });
        }
        for &(loop_id, index) in &self.loops {
            let lp = store.get(loop_id)?;
            let position = store
                .get(lp.face())?
                .loops()
                .iter()
                .position(|&candidate| candidate == loop_id)
                .ok_or(XtError::InvalidModel {
                    what: "loop missing from face",
                })?;
            let face_loops = store.get(lp.face())?.loops();
            let next = face_loops
                .get(position + 1)
                .map_or(0, |&v| id_of(&self.loops, v));
            out.push(OutNode {
                code: code::LOOP,
                index,
                values: vec![
                    int(index),
                    ptr(0),
                    ptr(id_of(&self.fins, lp.fins()[0])),
                    ptr(id_of(&self.faces, lp.face())),
                    ptr(next),
                ],
            });
        }
        for &(fin_id, index) in &self.fins {
            let fin = store.get(fin_id)?;
            let lp = store.get(fin.parent())?;
            let position =
                lp.fins()
                    .iter()
                    .position(|&v| v == fin_id)
                    .ok_or(XtError::InvalidModel {
                        what: "fin missing from loop",
                    })?;
            let forward = lp.fins()[(position + 1) % lp.fins().len()];
            let backward = lp.fins()[(position + lp.fins().len() - 1) % lp.fins().len()];
            let edge = store.get(fin.edge())?;
            let other = edge
                .fins()
                .iter()
                .copied()
                .find(|&v| v != fin_id)
                .map(|fin| id_of(&self.fins, fin))
                .or_else(|| self.dummy_fin_id(fin.edge(), DummyFinRole::SheetBoundary));
            let head = store.fin_head(fin_id)?;
            out.push(OutNode {
                code: code::FIN,
                index,
                values: vec![
                    ptr(0),
                    ptr(id_of(&self.loops, fin.parent())),
                    ptr(id_of(&self.fins, forward)),
                    ptr(id_of(&self.fins, backward)),
                    ptr(head.map_or(0, |v| id_of(&self.vertices, v))),
                    ptr(other.unwrap_or(0)),
                    ptr(id_of(&self.edges, fin.edge())),
                    ptr(self.fin_pcurve_id(fin_id).unwrap_or(0)),
                    ptr(match head {
                        Some(v) => self
                            .next_fin_at_vertex(store, v, fin_id)
                            .map(|f| id_of(&self.fins, f))
                            .or_else(|| self.dummy_fins_at_vertex(store, v).first().copied())
                            .unwrap_or(0),
                        None => 0,
                    }),
                    sense(fin.sense()),
                ],
            });
        }
        for &(dummy, index) in &self.dummy_fins {
            let edge_id = dummy.edge;
            let edge = store.get(edge_id)?;
            let (vertex, other, fin_sense) = match dummy.role {
                DummyFinRole::SheetBoundary => {
                    let [actual_fin] = edge.fins() else {
                        return Err(XtError::InvalidModel {
                            what: "sheet boundary dummy FIN edge must have exactly one real fin",
                        });
                    };
                    let actual_fin = *actual_fin;
                    let actual = store.get(actual_fin)?;
                    let tail = store.fin_tail(actual_fin)?.ok_or(XtError::InvalidModel {
                        what: "sheet boundary dummy FIN edge has no tail vertex",
                    })?;
                    (
                        tail,
                        id_of(&self.fins, actual_fin),
                        actual.sense().flipped(),
                    )
                }
                DummyFinRole::WireEnd => {
                    let vertex = edge.vertices()[1].ok_or(XtError::InvalidModel {
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
                    let vertex = edge.vertices()[0].ok_or(XtError::InvalidModel {
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
            let next_at_vertex = {
                let peers = self.dummy_fins_at_vertex(store, vertex);
                let position = peers.iter().position(|&id| id == index);
                position
                    .and_then(|p| peers.get(p + 1))
                    .copied()
                    .unwrap_or(0)
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
                    ptr(next_at_vertex),
                    sense(fin_sense),
                ],
            });
        }
        for (position, &(edge_id, index)) in self.edges.iter().enumerate() {
            let edge = store.get(edge_id)?;
            // Every real corpus file points EDGE.fin at the positive-sense
            // fin (exemplar.x_t 262/262, cyl.x_t, plate.x_t, disk_nat.x_t);
            // the reversed partner is reachable through FIN.other. For a
            // sheet boundary edge whose single real fin is reversed, the
            // positive fin is the dummy partner.
            let positive_fin = edge
                .fins()
                .iter()
                .copied()
                .find(|&fin| store.get(fin).is_ok_and(|f| f.sense() == Sense::Forward));
            let first_fin = match (positive_fin, edge.fins().first()) {
                (Some(fin), _) => id_of(&self.fins, fin),
                (None, Some(&fin)) => self
                    .dummy_fin_id(edge_id, DummyFinRole::SheetBoundary)
                    .unwrap_or_else(|| id_of(&self.fins, fin)),
                (None, None) => self
                    .dummy_fin_id(edge_id, DummyFinRole::WireEnd)
                    .unwrap_or(0),
            };
            out.push(OutNode {
                code: code::EDGE,
                index,
                values: vec![
                    int(index),
                    ptr(0),
                    optional_double(edge.tolerance().map(EntityTolerance::value)),
                    ptr(first_fin),
                    ptr(adjacent(&self.edges, position, -1)),
                    ptr(adjacent(&self.edges, position, 1)),
                    // Real files reference the basis curve directly for
                    // exact bounded edges (block.x_t, plate.x_t); parameter
                    // bounds are implied by the vertices. TRIMMED_CURVE
                    // wrapping stays reserved for tolerant fin SP-curves.
                    ptr(match edge.curve() {
                        Some(curve) => id_of(&self.curves, curve),
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
                .or_else(|| self.dummy_fins_at_vertex(store, vertex_id).first().copied())
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
                    ptr(id_of(&self.points, vertex.point())),
                    optional_double(vertex.tolerance().map(EntityTolerance::value)),
                    ptr(1),
                ],
            });
        }
        // Every real corpus OFFSET_SURF registers in its basis surface's
        // geometric-owner ring (exemplar.x_t, 44/44: a self-ring
        // GEOMETRIC_OWNER owned by the offset with shared_geometry = basis,
        // and the basis's geometric_owner backlink pointing at it); Onshape
        // rejects the offset body as corrupt without it.
        let mut offset_basis_owners = Vec::new();
        for &(surface_id, index) in &self.surfaces {
            if let SurfaceGeom::Offset(offset) = store.get(surface_id)? {
                let owner = next_aux;
                next_aux += 1;
                offset_basis_owners.push((offset.basis(), owner, index));
            }
        }
        for (position, &(surface_id, index)) in self.surfaces.iter().enumerate() {
            let face = self.faces.iter().find_map(|&(face, _)| {
                (store.get(face).ok()?.surface() == surface_id).then_some(face)
            });
            let mut common = geom_common(
                index,
                face.map_or(1, |face| id_of(&self.faces, face)),
                adjacent(&self.surfaces, position, 1),
                adjacent(&self.surfaces, position, -1),
            );
            common[5] = ptr(offset_basis_owners
                .iter()
                .find(|&&(basis, _, _)| basis == surface_id)
                .map(|&(_, owner, _)| owner)
                .or_else(|| self.surface_geometric_owner(store, surface_id))
                .unwrap_or(0));
            let aux = if matches!(store.get(surface_id)?, SurfaceGeom::Nurbs(_)) {
                let ids = SurfaceAuxIds::allocate(&mut next_aux);
                push_surface_aux_nodes(&mut out, store.get(surface_id)?, ids)?;
                Some(ids)
            } else {
                None
            };
            out.push(surface_node(
                store.get(surface_id)?,
                index,
                common,
                aux,
                &self.surfaces,
            )?);
            if let Some(&(basis, owner, _)) = offset_basis_owners
                .iter()
                .find(|&&(_, _, offset)| offset == index)
            {
                out.push(OutNode {
                    code: code::GEOMETRIC_OWNER,
                    index: owner,
                    values: vec![
                        ptr(index),
                        ptr(owner),
                        ptr(owner),
                        ptr(id_of(&self.surfaces, basis)),
                    ],
                });
            }
        }
        for (position, &(curve_id, index)) in self.curves.iter().enumerate() {
            let direct_owner = self.edges.iter().find_map(|&(edge, id)| {
                (store.get(edge).ok()?.curve() == Some(curve_id)).then_some(id)
            });
            let common = geom_common(
                index,
                direct_owner.unwrap_or(1),
                self.adjacent_curve_node(position, 1),
                self.adjacent_curve_node(position, -1),
            );
            let curve = native_curve(store.get(curve_id)?)?;
            let aux = if matches!(curve, NativeCurve::Nurbs(_)) {
                let ids = CurveAuxIds::allocate(&mut next_aux);
                push_curve_aux_nodes(&mut out, curve, ids)?;
                Some(ids)
            } else {
                None
            };
            out.push(curve_node(curve, index, common, aux)?);
        }
        for plan in &self.fin_pcurves {
            self.push_fin_pcurve_nodes(store, *plan, &mut next_aux, &mut out)?;
        }
        for (position, &(point_id, index)) in self.points.iter().enumerate() {
            // Real files own each POINT by its VERTEX (plate.x_t and every
            // modern corpus file); only ancient base-13006 output owned
            // points by the body.
            let owner = self
                .vertices
                .iter()
                .find_map(|&(vertex, id)| {
                    (store.get(vertex).ok()?.point() == point_id).then_some(id)
                })
                .unwrap_or(1);
            out.push(OutNode {
                code: code::POINT,
                index,
                values: vec![
                    int(index),
                    ptr(0),
                    ptr(owner),
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

    /// The vertex a dummy fin heads at: the tail of the real fin for a
    /// sheet boundary, the corresponding edge end for wire fins.
    fn dummy_fin_vertex(&self, store: &Store, dummy: DummyFin) -> Option<VertexId> {
        let edge = store.get(dummy.edge).ok()?;
        match dummy.role {
            DummyFinRole::SheetBoundary => {
                let &[actual_fin] = edge.fins() else {
                    return None;
                };
                store.fin_tail(actual_fin).ok().flatten()
            }
            DummyFinRole::WireEnd => edge.vertices()[1],
            DummyFinRole::WireStart => edge.vertices()[0],
        }
    }

    /// Node ids of the dummy fins heading at `vertex`, in emission order.
    /// Parasolid requires every fin claiming a vertex to be reachable from
    /// the vertex's fin chain, dummies included (host-verified: Onshape
    /// rejects sheets and wires whose loop-less fins are unlisted).
    fn dummy_fins_at_vertex(&self, store: &Store, vertex: VertexId) -> Vec<u32> {
        self.dummy_fins
            .iter()
            .filter(|&&(dummy, _)| self.dummy_fin_vertex(store, dummy) == Some(vertex))
            .map(|&(_, id)| id)
            .collect()
    }

    fn dummy_fin_id(&self, edge: EdgeId, role: DummyFinRole) -> Option<u32> {
        self.dummy_fins.iter().find_map(|&(candidate, id)| {
            (candidate.edge == edge && candidate.role == role).then_some(id)
        })
    }

    fn fin_pcurve_id(&self, fin: FinId) -> Option<u32> {
        self.fin_pcurves
            .iter()
            .find_map(|plan| (plan.fin == fin).then_some(plan.trimmed))
    }

    fn surface_geometric_owners(&self, store: &Store, surface: SurfaceId) -> Vec<(FinId, u32)> {
        self.fin_pcurves
            .iter()
            .filter_map(|plan| {
                let fin = store.get(plan.fin).ok()?;
                let lp = store.get(fin.parent()).ok()?;
                let face = store.get(lp.face()).ok()?;
                (face.surface() == surface).then_some((plan.fin, plan.surface_geometric_owner))
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
        let use_ = fin.pcurve().ok_or(XtError::InvalidModel {
            what: "curve-less tolerant edge fin has no pcurve",
        })?;
        let edge = store.get(fin.edge())?;
        let (t0, t1) = edge.bounds().ok_or(XtError::InvalidModel {
            what: "curve-less tolerant edge has no logical bounds",
        })?;
        let q0 = use_.parameter_at_edge(t0);
        let q1 = use_.parameter_at_edge(t1);
        let lp = store.get(fin.parent())?;
        let face = store.get(lp.face())?;
        let pcurve = store.get(use_.curve())?.as_curve();
        let uv0 = pcurve.eval(q0);
        let uv1 = pcurve.eval(q1);
        let mut eval = store.eval_context(EvalLimits::default(), Tolerances::default());
        let point0 = eval
            .eval_surface(
                face.surface(),
                [uv0.x, uv0.y],
                SurfaceDerivativeOrder::Position,
            )
            .map_err(XtError::Evaluation)?
            .p;
        let point1 = eval
            .eval_surface(
                face.surface(),
                [uv1.x, uv1.y],
                SurfaceDerivativeOrder::Position,
            )
            .map_err(XtError::Evaluation)?
            .p;

        let position = self
            .fin_pcurves
            .iter()
            .position(|candidate| candidate.fin == plan.fin)
            .expect("planned fin pcurve");
        let chain_position = self.curves.len() + position * 2;
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
            ptr(id_of(&self.surfaces, face.surface())),
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
        bcurve_values.extend([ptr(aux.nurbs), ptr(aux.data)]);
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
        let surface_owners = self.surface_geometric_owners(store, face.surface());
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
                ptr(id_of(&self.surfaces, face.surface())),
            ],
        });
        Ok(())
    }

    fn first_curve_id(&self) -> u32 {
        self.curves.first().map_or_else(
            || self.fin_pcurves.first().map_or(0, |plan| plan.trimmed),
            |&(_, id)| id,
        )
    }

    fn curve_node_id(&self, position: usize) -> Option<u32> {
        self.curves.get(position).map(|&(_, id)| id).or_else(|| {
            let offset = position.checked_sub(self.curves.len())?;
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

    fn face_scaffold(
        &self,
        store: &Store,
        face: FaceId,
        global_position: usize,
    ) -> Result<(u32, u32, u32, u32)> {
        let Some(solid) = &self.scaffold.solid else {
            let next = adjacent(&self.faces, global_position, 1);
            let previous = adjacent(&self.faces, global_position, -1);
            let front_shell = match self.body_kind {
                BodyKind::Sheet => self.scaffold.shell_id,
                _ => 0,
            };
            return Ok((self.scaffold.shell_id, front_shell, next, previous));
        };
        let pair = solid
            .shell_pairs
            .iter()
            .find(|pair| {
                store
                    .get(pair.shell)
                    .is_ok_and(|shell| shell.faces().contains(&face))
            })
            .ok_or(XtError::InvalidModel {
                what: "solid face is missing from its planned shell",
            })?;
        let shell = store.get(pair.shell)?;
        let position = shell
            .faces()
            .iter()
            .position(|&candidate| candidate == face)
            .ok_or(XtError::InvalidModel {
                what: "solid face is missing from its planned shell",
            })?;
        let next = shell
            .faces()
            .get(position + 1)
            .map_or(0, |&face| id_of(&self.faces, face));
        let previous = position
            .checked_sub(1)
            .and_then(|position| shell.faces().get(position))
            .map_or(0, |&face| id_of(&self.faces, face));
        Ok((pair.back_id, pair.front_id, next, previous))
    }

    fn push_solid_scaffold_nodes(&self, store: &Store, out: &mut Vec<OutNode>) -> Result<()> {
        let solid = self
            .scaffold
            .solid
            .as_ref()
            .expect("solid bodies have a planned solid scaffold");
        for (position, &(region, index)) in solid.regions.iter().enumerate() {
            let region_value = store.get(region)?;
            let shell = if region == solid.solid_region {
                solid.shell_pairs.first().map_or(0, |pair| pair.back_id)
            } else {
                solid
                    .shell_pairs
                    .iter()
                    .find(|pair| pair.void_region == region)
                    .map_or(0, |pair| pair.front_id)
            };
            out.push(OutNode {
                code: code::REGION,
                index,
                values: vec![
                    int(index),
                    ptr(0),
                    ptr(1),
                    ptr(adjacent(&solid.regions, position, 1)),
                    ptr(adjacent(&solid.regions, position, -1)),
                    ptr(shell),
                    Value::Char(match region_value.kind() {
                        RegionKind::Solid => 'S',
                        RegionKind::Void => 'V',
                    }),
                    // V26105 appended field (see REGION_EDIT_SCRIPT): null owner.
                    ptr(0),
                ],
            });
        }
        let solid_region_id = id_of(&solid.regions, solid.solid_region);
        for (position, pair) in solid.shell_pairs.iter().enumerate() {
            out.push(OutNode {
                code: code::SHELL,
                index: pair.back_id,
                values: vec![
                    int(pair.back_id),
                    ptr(0),
                    ptr(1),
                    ptr(solid
                        .shell_pairs
                        .get(position + 1)
                        .map_or(0, |pair| pair.back_id)),
                    ptr(first_mapped_id(&self.faces, store.get(pair.shell)?.faces())),
                    ptr(0),
                    ptr(0),
                    ptr(solid_region_id),
                    ptr(0),
                ],
            });
        }
        for pair in &solid.shell_pairs {
            out.push(OutNode {
                code: code::SHELL,
                index: pair.front_id,
                values: vec![
                    int(pair.front_id),
                    ptr(0),
                    ptr(0),
                    ptr(0),
                    ptr(0),
                    ptr(0),
                    ptr(0),
                    ptr(id_of(&solid.regions, pair.void_region)),
                    ptr(first_mapped_id(&self.faces, store.get(pair.shell)?.faces())),
                ],
            });
        }
        Ok(())
    }

    fn push_sheet_scaffold_nodes(&self, out: &mut Vec<OutNode>) {
        let shell_id = self.scaffold.shell_id;
        out.push(OutNode {
            code: code::REGION,
            index: 2,
            values: vec![
                int(2),
                ptr(0),
                ptr(1),
                ptr(0),
                ptr(0),
                ptr(shell_id),
                Value::Char('V'),
                // V26105 appended field (see REGION_EDIT_SCRIPT): null owner.
                ptr(0),
            ],
        });
        out.push(OutNode {
            code: code::SHELL,
            index: shell_id,
            values: vec![
                int(shell_id),
                ptr(0),
                ptr(1),
                ptr(0),
                ptr(first_id(&self.faces)),
                ptr(0),
                ptr(0),
                ptr(2),
                // A sheet's faces bound their void-region shell from both
                // sides, so they are also its front faces (disk_nat.x_t).
                ptr(first_id(&self.faces)),
            ],
        });
    }

    fn push_wire_scaffold_nodes(&self, out: &mut Vec<OutNode>) {
        let shell_id = self.scaffold.shell_id;
        out.push(OutNode {
            code: code::REGION,
            index: 2,
            values: vec![
                int(2),
                ptr(0),
                ptr(1),
                ptr(0),
                ptr(0),
                ptr(shell_id),
                Value::Char('V'),
                // V26105 appended field (see REGION_EDIT_SCRIPT): null owner.
                ptr(0),
            ],
        });
        out.push(OutNode {
            code: code::SHELL,
            index: shell_id,
            values: vec![
                int(shell_id),
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
        let shell_id = self.scaffold.shell_id;
        out.push(OutNode {
            code: code::REGION,
            index: 2,
            values: vec![
                int(2),
                ptr(0),
                ptr(1),
                ptr(0),
                ptr(0),
                ptr(shell_id),
                Value::Char('V'),
                // V26105 appended field (see REGION_EDIT_SCRIPT): null owner.
                ptr(0),
            ],
        });
        out.push(OutNode {
            code: code::SHELL,
            index: shell_id,
            values: vec![
                int(shell_id),
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
        match body.kind() {
            BodyKind::Solid => Self::solid(store, body),
            BodyKind::Sheet => Self::sheet(store, body),
            BodyKind::Wire => Self::wire(store, body),
            BodyKind::Acorn => Self::acorn(store, body),
        }
    }

    fn solid(store: &Store, body: &ktopo::entity::Body) -> Result<Self> {
        if body.regions().len() < 2 {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "solid text writing requires exterior void and solid regions",
            });
        }
        let void_regions: Vec<_> = core::iter::once(body.regions()[0])
            .chain(body.regions()[2..].iter().copied())
            .collect();
        let solid_region = body.regions()[1];
        let sr = store.get(solid_region)?;
        let mut void_layout_is_valid = true;
        for &region in &void_regions {
            let region = store.get(region)?;
            void_layout_is_valid &= region.kind() == RegionKind::Void && region.shells().is_empty();
        }
        if sr.kind() != RegionKind::Solid || !void_layout_is_valid {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "solid text writing requires one solid region after an empty exterior void, followed by empty finite voids",
            });
        }
        if sr.shells().len() != void_regions.len() {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "solid text writing requires one retained shell per void region",
            });
        }
        for &shell in sr.shells() {
            let shell_value = store.get(shell)?;
            if shell_value.faces().is_empty()
                || !shell_value.edges().is_empty()
                || shell_value.vertex().is_some()
            {
                return Err(XtError::Unsupported {
                    capability: XtCapability::WriterBodyTopology,
                    what: "solid text writing requires non-empty face-only retained shells",
                });
            }
        }

        let mut next = 2;
        let regions = assign(body.regions(), &mut next);
        let back_shells = assign(sr.shells(), &mut next);
        let front_shells = assign(sr.shells(), &mut next);
        let shell_pairs = back_shells
            .into_iter()
            .zip(front_shells)
            .zip(void_regions)
            .map(
                |(((shell, back_id), (front_shell, front_id)), void_region)| {
                    debug_assert_eq!(shell, front_shell);
                    SolidShellPair {
                        shell,
                        void_region,
                        back_id,
                        front_id,
                    }
                },
            )
            .collect::<Vec<_>>();
        let shell_id = shell_pairs
            .first()
            .expect("validated solid has a retained shell")
            .back_id;
        Ok(Scaffold {
            shell_id,
            solid: Some(SolidScaffold {
                regions,
                solid_region,
                shell_pairs,
            }),
        })
    }

    fn sheet(store: &Store, body: &ktopo::entity::Body) -> Result<Self> {
        let [void_region] = body.regions() else {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "sheet text writing requires one void region",
            });
        };
        let region = store.get(*void_region)?;
        let [shell] = region.shells() else {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "sheet text writing requires exactly one shell",
            });
        };
        let shell = store.get(*shell)?;
        if region.kind() != RegionKind::Void
            || shell.faces().is_empty()
            || !shell.edges().is_empty()
            || shell.vertex().is_some()
        {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "sheet text writing requires one non-empty face shell in the void region",
            });
        }
        Ok(Scaffold {
            shell_id: 3,
            solid: None,
        })
    }

    fn wire(store: &Store, body: &ktopo::entity::Body) -> Result<Self> {
        let [void_region] = body.regions() else {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "wire text writing requires one void region",
            });
        };
        let region = store.get(*void_region)?;
        let [shell] = region.shells() else {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "wire text writing requires exactly one shell",
            });
        };
        let shell = store.get(*shell)?;
        if region.kind() != RegionKind::Void
            || !shell.faces().is_empty()
            || shell.edges().is_empty()
            || shell.vertex().is_some()
        {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "wire text writing requires one non-empty edge shell in the void region",
            });
        }
        Ok(Scaffold {
            shell_id: 3,
            solid: None,
        })
    }

    fn acorn(store: &Store, body: &ktopo::entity::Body) -> Result<Self> {
        let [void_region] = body.regions() else {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "acorn text writing requires one void region",
            });
        };
        let region = store.get(*void_region)?;
        let [shell] = region.shells() else {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "acorn text writing requires exactly one shell",
            });
        };
        let shell = store.get(*shell)?;
        if region.kind() != RegionKind::Void
            || !shell.faces().is_empty()
            || !shell.edges().is_empty()
            || shell.vertex().is_none()
        {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterBodyTopology,
                what: "acorn text writing requires one vertex-only shell in the void region",
            });
        }
        Ok(Scaffold {
            shell_id: 3,
            solid: None,
        })
    }

    fn first_entity_id(&self) -> u32 {
        self.solid.as_ref().map_or(4, |solid| {
            solid
                .shell_pairs
                .last()
                .expect("validated solid has a retained shell")
                .front_id
                + 1
        })
    }
}

impl CurveAuxIds {
    fn allocate(next: &mut u32) -> Self {
        let ids = CurveAuxIds {
            nurbs: *next,
            data: *next + 1,
            poles: *next + 2,
            knot_mult: *next + 3,
            knots: *next + 4,
        };
        *next += 5;
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
        BodyKind::Solid if edge.fins().len() != 2 => {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterEdgeTopology,
                what: "solid edges must have exactly two fins",
            });
        }
        BodyKind::Sheet if edge.fins().len() == 2 => {}
        BodyKind::Sheet
            if edge.fins().len() == 1
                && edge.vertices() == [None, None]
                && edge.bounds().is_none() => {}
        BodyKind::Sheet
            if edge.fins().len() == 1
                && edge.vertices()[0].is_some()
                && edge.vertices()[1].is_some()
                && edge.bounds().is_some() => {}
        BodyKind::Sheet => {
            return Err(XtError::Unsupported {
                capability: XtCapability::WriterEdgeTopology,
                what: "unsupported sheet edge fin topology",
            });
        }
        BodyKind::Wire if !edge.fins().is_empty() => {
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
            if edge.fins().len() == 1
                && edge.vertices()[0].is_some()
                && edge.vertices()[1].is_some()
                && edge.bounds().is_some() =>
        {
            out.push(DummyFin {
                edge: edge_id,
                role: DummyFinRole::SheetBoundary,
            });
        }
        BodyKind::Wire
            if edge.fins().is_empty()
                && edge.vertices()[0].is_some()
                && edge.vertices()[1].is_some()
                && edge.bounds().is_some() =>
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
    surfaces: &[(SurfaceId, u32)],
) -> Result<OutNode> {
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
        SurfaceGeom::Offset(offset) => {
            values.extend([
                Value::Char('U'),
                Value::Logical(false),
                ptr(id_of(surfaces, offset.basis())),
                Value::Double(offset.signed_distance()),
                Value::Null,
            ]);
            code::OFFSET_SURF
        }
        _ => {
            return Err(XtError::Unsupported {
                capability: XtCapability::ProceduralSurfaces,
                what: "unimplemented geometry-graph surface class",
            });
        }
    };
    Ok(OutNode {
        code,
        index,
        values,
    })
}

/// Append `surface` after its direct dependencies. Face iteration supplies
/// deterministic root order; descriptor field order supplies dependency order.
fn plan_surface_dependencies(
    store: &Store,
    surface: SurfaceId,
    planned: &mut Vec<SurfaceId>,
) -> Result<()> {
    if planned.contains(&surface) {
        return Ok(());
    }
    if let SurfaceGeom::Offset(offset) = store.get(surface)? {
        if matches!(store.get(offset.basis())?, SurfaceGeom::Offset(_)) {
            return Err(XtError::Unsupported {
                capability: XtCapability::NestedOffsetExport,
                what: "nested offset-surface export",
            });
        }
        plan_surface_dependencies(store, offset.basis(), planned)?;
    }
    planned.push(surface);
    Ok(())
}

/// A basis that also carries a face directly would need its geometric-owner
/// ring merged across the offset's entry and that face's SP-curve entries;
/// no real corpus file exercises that shape, so exporting it is refused
/// rather than guessed.
fn reject_direct_faces_on_offset_bases(store: &Store, faces: &[FaceId]) -> Result<()> {
    let mut bases = Vec::new();
    for &face in faces {
        if let SurfaceGeom::Offset(offset) = store.get(store.get(face)?.surface())? {
            bases.push(offset.basis());
        }
    }
    for &face in faces {
        if bases.contains(&store.get(face)?.surface()) {
            return Err(XtError::Unsupported {
                capability: XtCapability::SharedOffsetBasisExport,
                what: "a face sits directly on another face's offset basis",
            });
        }
    }
    Ok(())
}

fn reject_shared_offset_bases(store: &Store, surfaces: &[SurfaceId]) -> Result<()> {
    let mut bases = Vec::<SurfaceId>::new();
    for &surface in surfaces {
        if let SurfaceGeom::Offset(offset) = store.get(surface)? {
            if bases.contains(&offset.basis()) {
                return Err(XtError::Unsupported {
                    capability: XtCapability::SharedOffsetBasisExport,
                    what: "multiple offset surfaces share one basis surface",
                });
            }
            bases.push(offset.basis());
        }
    }
    Ok(())
}

fn curve_node(
    curve: NativeCurve<'_>,
    index: u32,
    mut values: Vec<Value>,
    aux: Option<CurveAuxIds>,
) -> Result<OutNode> {
    let code = match curve {
        NativeCurve::Line(c) => {
            values.extend([vector(c.origin()), vector(c.dir())]);
            code::LINE
        }
        NativeCurve::Circle(c) => {
            values.extend([
                vector(c.frame().origin()),
                vector(c.frame().z()),
                vector(c.frame().x()),
                Value::Double(c.radius()),
            ]);
            code::CIRCLE
        }
        NativeCurve::Ellipse(c) => {
            values.extend([
                vector(c.frame().origin()),
                vector(c.frame().z()),
                vector(c.frame().x()),
                Value::Double(c.major_radius()),
                Value::Double(c.minor_radius()),
            ]);
            code::ELLIPSE
        }
        NativeCurve::Nurbs(_) => {
            let aux = aux.expect("planned NURBS curve auxiliaries");
            values.extend([ptr(aux.nurbs), ptr(aux.data)]);
            code::B_CURVE
        }
    };
    Ok(OutNode {
        code,
        index,
        values,
    })
}

fn push_curve_aux_nodes(
    out: &mut Vec<OutNode>,
    curve: NativeCurve<'_>,
    ids: CurveAuxIds,
) -> Result<()> {
    let NativeCurve::Nurbs(curve) = curve else {
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
            // Every real corpus NURBS_CURVE (exemplar.x_t, 301/301)
            // declares knot_type 5 (bespoke) and curve_form 1
            // (unspecified); Onshape rejects 0 in either as corrupt.
            Value::Int(5),
            Value::Logical(false),
            Value::Logical(false),
            Value::Logical(rational),
            Value::Int(1),
            ptr(ids.poles),
            ptr(ids.knot_mult),
            ptr(ids.knots),
        ],
    });
    out.push(curve_data_node(ids.data));
    out.push(bspline_vertices_node(ids.poles, flatten_curve_poles(curve)));
    out.push(int_values_node(ids.knot_mult, &multiplicities));
    out.push(knot_values_node(ids.knots, knots));
    Ok(())
}

/// Every real corpus B_CURVE carries a CURVE_DATA companion declaring
/// self_int 1 (checked, no self-intersection) and no analytic form.
fn curve_data_node(index: u32) -> OutNode {
    OutNode {
        code: code::CURVE_DATA,
        index,
        values: vec![Value::Int(1), ptr(0)],
    }
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
            // knot_type 5 / curve_form 1, as in every real corpus file.
            Value::Int(5),
            Value::Logical(false),
            Value::Logical(false),
            Value::Logical(rational),
            Value::Int(1),
            ptr(ids.poles),
            ptr(ids.knot_mult),
            ptr(ids.knots),
        ],
    });
    out.push(curve_data_node(ids.data));
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
    let periodic = surface.periodicity().map(|period| period.is_some());
    let vertex_dim = if rational { 4 } else { 3 };
    let (nu, nv) = surface.net_size();
    out.push(OutNode {
        code: code::NURBS_SURF,
        index: ids.nurbs,
        values: vec![
            Value::Logical(periodic[0]),
            Value::Logical(periodic[1]),
            Value::Int(surface.degree_u() as i64),
            Value::Int(surface.degree_v() as i64),
            Value::Int(nu as i64),
            Value::Int(nv as i64),
            Value::Int(0),
            Value::Int(0),
            Value::Int(u_knots.len() as i64),
            Value::Int(v_knots.len() as i64),
            Value::Logical(rational),
            Value::Logical(periodic[0]),
            Value::Logical(periodic[1]),
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
        match store.get(surface)? {
            SurfaceGeom::Nurbs(_) => count += 6,
            // One GEOMETRIC_OWNER ring entry ties each offset to its basis.
            SurfaceGeom::Offset(_) => count += 1,
            _ => {}
        }
    }
    for &curve in curves {
        if matches!(native_curve(store.get(curve)?)?, NativeCurve::Nurbs(_)) {
            count += 5;
        }
    }
    Ok(count)
}

fn native_curve(curve: &CurveGeom) -> Result<NativeCurve<'_>> {
    match curve {
        CurveGeom::Line(curve) => Ok(NativeCurve::Line(*curve)),
        CurveGeom::Circle(curve) => Ok(NativeCurve::Circle(*curve)),
        CurveGeom::Ellipse(curve) => Ok(NativeCurve::Ellipse(*curve)),
        CurveGeom::Nurbs(curve) => Ok(NativeCurve::Nurbs(curve)),
        CurveGeom::Intersection(descriptor) => match descriptor.carrier() {
            VerifiedIntersectionCarrier::Line(carrier) => Ok(NativeCurve::Line(carrier)),
            VerifiedIntersectionCarrier::Circle(carrier) => Ok(NativeCurve::Circle(carrier)),
        },
        _ => Err(XtError::Unsupported {
            capability: XtCapability::ProceduralCurves,
            what: "verified or procedural curve has no supported native X_T carrier",
        }),
    }
}

fn validate_verified_intersection_edge(
    descriptor: &kgraph::VerifiedIntersectionCurveDescriptor,
    edge: &Edge,
) -> Result<()> {
    if edge.tolerance().is_some() {
        return Err(XtError::Unsupported {
            capability: XtCapability::WriterEdgeTopology,
            what: "verified analytic intersection export requires an exact edge",
        });
    }
    let ([Some(_), Some(_)], Some((lo, hi))) = (edge.vertices(), edge.bounds()) else {
        return Err(XtError::Unsupported {
            capability: XtCapability::WriterEdgeTopology,
            what: "verified analytic intersection export requires a bounded edge",
        });
    };
    let certified = descriptor.carrier_range();
    if !lo.is_finite() || !hi.is_finite() || lo >= hi || lo < certified.lo || hi > certified.hi {
        return Err(XtError::InvalidModel {
            what: "verified intersection edge bounds exceed its certified carrier range",
        });
    }
    Ok(())
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
    let use_: FinPcurve = store.get(fin_id)?.pcurve().ok_or(XtError::InvalidModel {
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
        _ => Err(XtError::Unsupported {
            capability: XtCapability::ProceduralCurves,
            what: "unimplemented geometry-graph pcurve class",
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
    // Embedded-schema flag sequence: version string, schema key with base
    // suffix, maximum node-type count, user-field size 0. The key is
    // length-prefixed, so the count follows it without a separator —
    // exactly as real Parasolid writes it.
    // User-field size 1, as every observed real Parasolid file declares;
    // each node's data is followed by one zero user-field int.
    let mut data = format!(
        "T{} {}{} {}{} 1 ",
        VERSION.len(),
        VERSION,
        SCHEMA_KEY.len(),
        SCHEMA_KEY,
        MAX_NODE_TYPES
    );
    let mut described: BTreeSet<u16> = BTreeSet::new();
    for node in nodes {
        let def = defs
            .iter()
            .find(|def| def.code == node.code)
            .ok_or(XtError::InvalidModel {
                what: "writer selected an unknown node type",
            })?;
        let expected_fields = if node.code == code::BODY {
            def.fields.len() + BODY_APPENDED_FIELDS
        } else if node.code == code::REGION {
            def.fields.len() + REGION_APPENDED_FIELDS
        } else {
            def.fields.len()
        };
        if expected_fields != node.values.len() {
            return Err(XtError::InvalidModel {
                what: "writer produced the wrong field count",
            });
        }
        // First occurrence of a node type carries its embedded layout:
        // BODY and REGION differ from base 13006 (verified against every
        // real fixture in the corpus) and get the real edit scripts; every
        // other emitted type is byte 255, "identical to base".
        let marker = if described.insert(node.code) {
            if node.code == code::BODY {
                BODY_EDIT_SCRIPT
            } else if node.code == code::REGION {
                REGION_EDIT_SCRIPT
            } else {
                "255 "
            }
        } else {
            ""
        };
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
            data.push_str(&format!(
                "{} {marker}{} {} ",
                node.code, variable_len, node.index
            ));
        } else {
            data.push_str(&format!("{} {marker}{} ", node.code, node.index));
        }
        for value in &node.values {
            write_value(&mut data, value)?;
        }
        // One zero user-field per node (USFLD_SIZE=1).
        data.push_str("0 ");
    }
    data.push_str("1 0 ");
    // Text lines are 80-column records, and a record must never END with a
    // space: real Parasolid readers right-trim each line before splicing
    // the stream back together, so a separator space at a line end is
    // eaten and the adjacent tokens merge. Real writers therefore move a
    // space that would land on column 80 to the start of the next line
    // (their files mix 80-character lines with 79-character lines whose
    // successor starts with a space — never a line ending in a space).
    // Tokens themselves may split across records; only trailing spaces are
    // unsafe. Established empirically through the oracle loop: re-wrapping
    // a host-accepted file at a fixed 80 columns (leaving separator spaces
    // at line ends) makes the host reject it as corrupt.
    let bytes = data.as_bytes();
    let mut wrapped = String::new();
    let mut start = 0;
    while start < bytes.len() {
        let width = (bytes.len() - start).min(80);
        let mut cut = width;
        if start + width < bytes.len() {
            while cut > 1 && bytes[start + cut - 1] == b' ' {
                cut -= 1;
            }
        }
        wrapped.push_str(
            core::str::from_utf8(&bytes[start..start + cut]).expect("writer emits ASCII"),
        );
        wrapped.push('\n');
        start += cut;
    }
    // Header layout mirrors Parasolid-authored transmit files field for
    // field: one keyword per line, every record within 80 columns, and the
    // full PART1 identity set present with deterministic placeholder values
    // (real Parasolid accepts and itself emits `unknown` here).
    Ok(format!(
        "**ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz**************************\n\
         **PARASOLID !\"#$%&'()*+,-./:;<=>?@[\\]^_`{{|}}~0123456789**************************\n\
         **PART1;\n\
         MC=unknown;\n\
         MC_MODEL=unknown;\n\
         MC_ID=unknown;\n\
         OS=unknown;\n\
         OS_RELEASE=unknown;\n\
         FRU=unknown;\n\
         APPL=cad_prototype-kxt;\n\
         SITE=unknown;\n\
         USER=unknown;\n\
         FORMAT=text;\n\
         GUISE=transmit;\n\
         KEY=unknown;\n\
         FILE=unknown;\n\
         DATE=unknown;\n\
         **PART2;\n\
         SCH={SCHEMA};\n\
         USFLD_SIZE=1;\n\
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

fn first_mapped_id<T>(values: &[(Handle<T>, u32)], handles: &[Handle<T>]) -> u32 {
    handles.first().map_or(0, |&handle| id_of(values, handle))
}

fn adjacent<T>(values: &[(Handle<T>, u32)], position: usize, direction: i8) -> u32 {
    match direction {
        -1 if position > 0 => values[position - 1].1,
        1 if position + 1 < values.len() => values[position + 1].1,
        _ => 0,
    }
}

/// Parasolid ownership lists are singleton-null or circular doubly linked rings.
fn owner_ring_adjacent<T>(values: &[(Handle<T>, u32)], position: usize, direction: i8) -> u32 {
    if values.len() < 2 {
        return 0;
    }
    match direction {
        -1 => values[(position + values.len() - 1) % values.len()].1,
        1 => values[(position + 1) % values.len()].1,
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
