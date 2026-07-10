//! Reconstruction: from a parsed [`XtFile`] node graph to `ktopo` bodies.
//!
//! Mapping notes (XT → kernel), all Tier 0:
//!
//! - XT stores the edge/curve orientation on the **curve** (`sense`).
//!   Our model has no edge sense — an edge always runs along its curve's
//!   natural direction — so a `-` curve sense is absorbed by swapping the
//!   edge's vertices and flipping every fin sense on that edge. This is
//!   exact: no geometry is modified.
//! - The face normal in XT is the natural surface normal iff
//!   `face.sense == surface.sense`; that combination becomes our
//!   `Face::sense`.
//! - XT shells list *back-faces* (normal out of the owning region) and
//!   *front-faces* separately. Our model attaches each face to the shell
//!   it is a back-face of, which matches our outward-normal convention.
//!   Shells left with no content (e.g. the void exterior shell of a
//!   solid, which only lists front-faces) are dropped.
//! - Exact-edge parameter bounds are not stored in XT; they are recovered by
//!   closed-form inversion of the vertex positions on analytic curves
//!   (arc length on lines, `atan2` angles on circles/ellipses) and by
//!   projection on B-curves. A tolerant edge with `EDGE.curve = null` gets
//!   the canonical logical domain `[0, 1]`; each trimmed SP-curve on its
//!   fins supplies the correspondence to a 2D B-curve. Ring edges (no
//!   vertices) get `bounds: None` and must have exact curve geometry.
//! - Geometry conventions transfer exactly: XT and kernel
//!   parameterizations coincide for plane/cylinder/sphere/torus/circle/
//!   ellipse/line. Cones differ (XT measures `v` against the axis with a
//!   `tan α` taper; ours is slant-parameterized): the cone is rebuilt
//!   geometrically — same point set, different `(u, v)`.
//! - Attributes, groups, transforms, and construction geometry are parsed
//!   but not reconstructed (recorded in [`Reconstruction::skipped`]).

use crate::error::{Result, XtError};
use crate::parse::{Node, Value, XtFile};
use crate::schema::code;
use kcore::math;
use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::curve2d::NurbsCurve2d;
use kgeom::frame::Frame;
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::param::{ParamRange, wrap_periodic};
use kgeom::surface::{Cone, Cylinder, Plane, Sphere, Torus};
use kgeom::vec::{Point2, Point3, Vec3};
use ktopo::entity::{
    Body, BodyId, BodyKind, Curve2dId, CurveId, Edge, EdgeId, Face, FaceId, Fin, FinPcurve, Loop,
    ParamMap1d, Region, RegionId, RegionKind, Sense, Shell, ShellId, SurfaceId, Vertex, VertexId,
};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::store::Store;
use ktopo::transaction::{Journal, MutationKind};
use std::collections::BTreeMap;

/// Everything produced by reconstructing one transmit file.
#[derive(Debug)]
pub struct Reconstruction {
    /// The bodies created in the store, in file order.
    pub bodies: Vec<BodyId>,
    /// Node types that were present but intentionally not reconstructed
    /// (attributes, groups, …), with occurrence counts.
    pub skipped: Vec<(u16, usize)>,
    /// Transaction journal for every entity created or changed by this
    /// reconstruction. Import currently emits raw mutations; file-node
    /// provenance becomes semantic lineage when interchange provenance IDs
    /// are introduced.
    pub journal: Journal,
}

/// Reconstruct every body in the file into `store`.
pub fn reconstruct(file: &XtFile, store: &mut Store) -> Result<Reconstruction> {
    let mut transaction = store.transaction()?;
    let mut reconstruction = reconstruct_into(file, transaction.store_mut())?;
    reconstruction.journal = transaction.commit()?;
    debug_assert!(
        reconstruction
            .journal
            .mutations()
            .iter()
            .all(|mutation| mutation.kind != MutationKind::Deleted)
    );
    Ok(reconstruction)
}

/// Reconstruct inside the caller's active copy-on-write transaction.
fn reconstruct_into(file: &XtFile, store: &mut Store) -> Result<Reconstruction> {
    let root = xnode(file, 1)?;
    let mut body_indices = Vec::new();
    match root.code {
        code::BODY => body_indices.push(1u32),
        code::POINTER_LIS_BLOCK => {
            // An array-of-parts file: entries point to the parts.
            let mut block_idx = 1u32;
            while block_idx != 0 {
                let block = xnode(file, block_idx)?;
                for v in entries(file, block)? {
                    if xnode(file, v)?.code == code::BODY {
                        body_indices.push(v);
                    } else {
                        return Err(XtError::Unsupported {
                            what: "non-body parts (assemblies) in array-of-parts files",
                        });
                    }
                }
                block_idx = ptr(file, block, "next_block")?;
            }
        }
        code::ASSEMBLY => {
            return Err(XtError::Unsupported {
                what: "assembly transmit files (Tier-0 reads body files)",
            });
        }
        code::WORLD => {
            return Err(XtError::Unsupported {
                what: "partition transmit files (Tier-0 reads body files)",
            });
        }
        _ => {
            return Err(XtError::BadField {
                index: 1,
                what: "root node is not a body, part list, assembly, or world",
            });
        }
    }

    let mut recon = Recon {
        file,
        store,
        curves: BTreeMap::new(),
        pcurves: BTreeMap::new(),
        surfaces: BTreeMap::new(),
        points: BTreeMap::new(),
        vertices: BTreeMap::new(),
        edges: BTreeMap::new(),
    };
    let bodies = body_indices
        .iter()
        .map(|&b| recon.body(b))
        .collect::<Result<Vec<_>>>()?;

    // Count node types we deliberately did not reconstruct.
    let mut skipped: BTreeMap<u16, usize> = BTreeMap::new();
    for node in file.nodes.values() {
        let deliberately_skipped = matches!(
            node.code,
            code::ATTRIBUTE
                | code::ATTRIB_DEF
                | code::ATT_DEF_ID
                | code::INT_VALUES
                | code::REAL_VALUES
                | code::CHAR_VALUES
                | code::POINT_VALUES
                | code::VECTOR_VALUES
                | code::AXIS_VALUES
                | code::TAG_VALUES
                | code::DIRECTION_VALUES
                | code::UNICODE_VALUES
                | code::FIELD_NAMES
                | code::GROUP
                | code::MEMBER_OF_GROUP
                | code::LIST
                | code::POINTER_LIS_BLOCK
                | code::TRANSFORM
                | code::GEOMETRIC_OWNER
                | code::KEY
        ) || file.foreign_codes.contains(&node.code);
        if deliberately_skipped {
            *skipped.entry(node.code).or_insert(0) += 1;
        }
    }
    Ok(Reconstruction {
        bodies,
        skipped: skipped.into_iter().collect(),
        journal: Journal::default(),
    })
}

// ------------------------------------------------------- field helpers --

fn xnode(file: &XtFile, index: u32) -> Result<&Node> {
    file.node(index).ok_or(XtError::MissingNode { index })
}

fn field<'a>(file: &'a XtFile, node: &'a Node, name: &'static str) -> Result<&'a Value> {
    file.field(node, name).ok_or(XtError::BadField {
        index: 0,
        what: name,
    })
}

fn ptr(file: &XtFile, node: &Node, name: &'static str) -> Result<u32> {
    field(file, node, name)?.as_ptr().ok_or(XtError::BadField {
        index: 0,
        what: "expected a pointer field",
    })
}

fn ch(file: &XtFile, node: &Node, name: &'static str) -> Result<char> {
    field(file, node, name)?.as_char().ok_or(XtError::BadField {
        index: 0,
        what: "expected a char field",
    })
}

fn f64_of(file: &XtFile, node: &Node, name: &'static str) -> Result<f64> {
    field(file, node, name)?.as_f64().ok_or(XtError::BadField {
        index: 0,
        what: "expected a numeric field",
    })
}

fn logical_of(file: &XtFile, node: &Node, name: &'static str) -> Result<bool> {
    match field(file, node, name)? {
        Value::Logical(b) => Ok(*b),
        _ => Err(XtError::BadField {
            index: 0,
            what: "expected a logical field",
        }),
    }
}

fn vector(file: &XtFile, node: &Node, name: &'static str) -> Result<Vec3> {
    let v = field(file, node, name)?
        .as_vector()
        .ok_or(XtError::BadField {
            index: 0,
            what: "expected a non-null vector field",
        })?;
    Ok(Vec3::new(v[0], v[1], v[2]))
}

/// Optional tolerance: null double → `None`.
fn tolerance(file: &XtFile, node: &Node) -> Result<Option<f64>> {
    Ok(match field(file, node, "tolerance")? {
        Value::Null => None,
        v => Some(v.as_f64().ok_or(XtError::BadField {
            index: 0,
            what: "tolerance is not numeric",
        })?),
    })
}

fn entries(file: &XtFile, block: &Node) -> Result<Vec<u32>> {
    match field(file, block, "entries")? {
        Value::Arr(vs) => Ok(vs
            .iter()
            .filter_map(Value::as_ptr)
            .filter(|&p| p != 0)
            .collect()),
        _ => Err(XtError::BadField {
            index: 0,
            what: "entries is not an array",
        }),
    }
}

fn in_size_box(p: Vec3) -> Result<Vec3> {
    for c in [p.x, p.y, p.z] {
        if !c.is_finite() || c.abs() > 500.0 {
            return Err(XtError::OutsideSizeBox { value: c });
        }
    }
    Ok(p)
}

// ---------------------------------------------------------------- body --

struct Recon<'a> {
    file: &'a XtFile,
    store: &'a mut Store,
    /// XT curve index → (kernel curve, XT sense was `-`).
    curves: BTreeMap<u32, (CurveId, bool)>,
    /// XT 2D B-curve index → kernel pcurve geometry.
    pcurves: BTreeMap<u32, Curve2dId>,
    /// XT surface index → (kernel surface, XT sense char).
    surfaces: BTreeMap<u32, (SurfaceId, char)>,
    /// XT point index → kernel point.
    points: BTreeMap<u32, ktopo::entity::PointId>,
    vertices: BTreeMap<u32, VertexId>,
    /// XT edge index → (kernel edge, fins must flip: curve sense was `-`).
    edges: BTreeMap<u32, (EdgeId, bool)>,
}

impl Recon<'_> {
    fn body(&mut self, body_idx: u32) -> Result<BodyId> {
        let file = self.file;
        let body_node = xnode(file, body_idx)?;
        if body_node.code != code::BODY {
            return Err(XtError::BadField {
                index: body_idx,
                what: "referenced part is not a BODY node",
            });
        }
        let kind = match field(file, body_node, "body_type")?.as_int() {
            Some(1) => BodyKind::Solid,
            Some(2) => BodyKind::Wire, // acorn detection below
            Some(3) => BodyKind::Sheet,
            _ => {
                return Err(XtError::Unsupported {
                    what: "general bodies (body_type 6)",
                });
            }
        };
        let body = self.store.add(Body {
            kind,
            regions: Vec::new(),
        });

        // Region chain; the head is the infinite (exterior) region.
        let mut region_idx = ptr(file, body_node, "region")?;
        let mut is_acorn = false;
        while region_idx != 0 {
            let region_node = xnode(file, region_idx)?;
            let region_kind = match ch(file, region_node, "type")? {
                'S' => RegionKind::Solid,
                'V' => RegionKind::Void,
                _ => {
                    return Err(XtError::BadField {
                        index: region_idx,
                        what: "region type is not S/V",
                    });
                }
            };
            let next_region = ptr(file, region_node, "next")?;
            let first_shell = ptr(file, region_node, "shell")?;
            let region = self.store.add(Region {
                body,
                kind: region_kind,
                shells: Vec::new(),
            });
            self.store.get_mut(body)?.regions.push(region);

            let mut shell_idx = first_shell;
            while shell_idx != 0 {
                let next_shell = ptr(file, xnode(file, shell_idx)?, "next")?;
                if let Some(acorn) = self.shell(region, shell_idx)? {
                    is_acorn |= acorn;
                }
                shell_idx = next_shell;
            }
            region_idx = next_region;
        }
        if self.store.get(body)?.regions.is_empty() {
            return Err(XtError::BadField {
                index: body_idx,
                what: "body has no regions",
            });
        }
        if is_acorn {
            self.store.get_mut(body)?.kind = BodyKind::Acorn;
        }
        Ok(body)
    }

    /// Reconstruct one shell; returns `Some(is_acorn)`, or `None` if the
    /// shell was dropped as empty (a solid's void-exterior shell).
    fn shell(&mut self, region: RegionId, shell_idx: u32) -> Result<Option<bool>> {
        let file = self.file;
        let shell_node = xnode(file, shell_idx)?;
        let first_face = ptr(file, shell_node, "face")?;
        let first_edge = ptr(file, shell_node, "edge")?;
        let vertex_idx = ptr(file, shell_node, "vertex")?;

        let shell = self.store.add(Shell {
            region,
            faces: Vec::new(),
            edges: Vec::new(),
            vertex: None,
        });

        // Back-faces: the faces whose normal points out of this shell's
        // region — exactly our convention.
        let mut face_idx = first_face;
        while face_idx != 0 {
            let next = ptr(file, xnode(file, face_idx)?, "next")?;
            self.face(shell, face_idx)?;
            face_idx = next;
        }
        // Wireframe edges.
        let mut edge_idx = first_edge;
        while edge_idx != 0 {
            let next = ptr(file, xnode(file, edge_idx)?, "next")?;
            let (edge, _) = self.edge(edge_idx)?;
            self.store.get_mut(shell)?.edges.push(edge);
            edge_idx = next;
        }
        // Acorn vertex.
        let mut acorn = false;
        if vertex_idx != 0 {
            let v = self.vertex(vertex_idx)?;
            self.store.get_mut(shell)?.vertex = Some(v);
            acorn = true;
        }

        let s = self.store.get(shell)?;
        if s.faces.is_empty() && s.edges.is_empty() && s.vertex.is_none() {
            // The void-exterior shell of a solid lists only front-faces;
            // it carries no information our model keeps. Nothing points
            // to it yet, so it can simply be removed.
            self.store.remove(shell)?;
            return Ok(None);
        }
        self.store.get_mut(region)?.shells.push(shell);
        Ok(Some(acorn))
    }

    fn face(&mut self, shell: ShellId, face_idx: u32) -> Result<FaceId> {
        let file = self.file;
        let face_node = xnode(file, face_idx)?;
        let surface_idx = ptr(file, face_node, "surface")?;
        if surface_idx == 0 {
            return Err(XtError::Unsupported {
                what: "faces without surface geometry",
            });
        }
        let first_loop = ptr(file, face_node, "loop")?;
        let xt_face_sense = ch(file, face_node, "sense")?;

        let (surface, surf_sense) = self.surface(surface_idx)?;
        // Face normal == natural surface normal iff the two senses agree.
        let sense = if xt_face_sense == surf_sense {
            Sense::Forward
        } else {
            Sense::Reversed
        };
        let face = self.store.add(Face {
            shell,
            loops: Vec::new(),
            surface,
            sense,
        });
        self.store.get_mut(shell)?.faces.push(face);

        let mut loop_idx = first_loop;
        while loop_idx != 0 {
            let next = ptr(file, xnode(file, loop_idx)?, "next")?;
            self.lp(face, loop_idx)?;
            loop_idx = next;
        }
        Ok(face)
    }

    fn lp(&mut self, face: FaceId, loop_idx: u32) -> Result<()> {
        let file = self.file;
        let loop_node = xnode(file, loop_idx)?;
        let first_fin = ptr(file, loop_node, "fin")?;
        if first_fin == 0 {
            return Err(XtError::Unsupported {
                what: "isolated loops (single-vertex loops)",
            });
        }
        let lp = self.store.add(Loop {
            face,
            fins: Vec::new(),
        });
        self.store.get_mut(face)?.loops.push(lp);

        // Walk the fin ring via forward pointers.
        let mut fin_idx = first_fin;
        let mut fins = Vec::new();
        loop {
            let fin_node = xnode(file, fin_idx)?;
            let edge_idx = ptr(file, fin_node, "edge")?;
            if edge_idx == 0 {
                return Err(XtError::BadField {
                    index: fin_idx,
                    what: "loop fin has no edge",
                });
            }
            let xt_sense = ch(file, fin_node, "sense")?;
            let forward = ptr(file, fin_node, "forward")?;

            let (edge, flip) = self.edge(edge_idx)?;
            let mut sense = match xt_sense {
                '+' => Sense::Forward,
                '-' => Sense::Reversed,
                _ => {
                    return Err(XtError::BadField {
                        index: fin_idx,
                        what: "fin sense is not +/-",
                    });
                }
            };
            if flip {
                sense = sense.flipped();
            }
            let pcurve = self.fin_pcurve(fin_idx, face, edge)?;
            let fin = self.store.add(Fin {
                parent: lp,
                edge,
                sense,
                pcurve,
            });
            self.store.get_mut(edge)?.fins.push(fin);
            fins.push(fin);

            if forward == first_fin || forward == 0 {
                break;
            }
            fin_idx = forward;
            if fins.len() > 1_000_000 {
                return Err(XtError::BadField {
                    index: loop_idx,
                    what: "fin ring does not close",
                });
            }
        }
        self.store.get_mut(lp)?.fins = fins;
        Ok(())
    }

    fn vertex(&mut self, vertex_idx: u32) -> Result<VertexId> {
        if let Some(&v) = self.vertices.get(&vertex_idx) {
            return Ok(v);
        }
        let file = self.file;
        let vertex_node = xnode(file, vertex_idx)?;
        let point_idx = ptr(file, vertex_node, "point")?;
        let point = if let Some(&point) = self.points.get(&point_idx) {
            point
        } else {
            let point_node = xnode(file, point_idx)?;
            let p = in_size_box(vector(file, point_node, "pvec")?)?;
            let point = self.store.add(p);
            self.points.insert(point_idx, point);
            point
        };
        let tol = tolerance(file, vertex_node)?;
        let v = self.store.add(Vertex {
            point,
            tolerance: tol,
        });
        self.vertices.insert(vertex_idx, v);
        Ok(v)
    }

    fn edge(&mut self, edge_idx: u32) -> Result<(EdgeId, bool)> {
        if let Some(&e) = self.edges.get(&edge_idx) {
            return Ok(e);
        }
        let file = self.file;
        let edge_node = xnode(file, edge_idx)?;
        let curve_idx = ptr(file, edge_node, "curve")?;
        let (curve, curve_reversed, trim) = if curve_idx == 0 {
            (None, false, None)
        } else {
            // Trimmed curves carry their bounds; plain curves get inverted.
            let curve_node = xnode(file, curve_idx)?;
            let (geom_curve_idx, trim) = if curve_node.code == code::TRIMMED_CURVE {
                let basis = ptr(file, curve_node, "basis_curve")?;
                let p1 = f64_of(file, curve_node, "parm_1")?;
                let p2 = f64_of(file, curve_node, "parm_2")?;
                (basis, Some((p1, p2)))
            } else {
                (curve_idx, None)
            };
            let (curve, reversed) = self.curve(geom_curve_idx)?;
            (Some(curve), reversed, trim)
        };

        // Vertices via the edge's fin ring: a `+` fin's forward vertex is
        // the edge end, a `-` fin's is the edge start (dummy fins exist
        // precisely to make both reachable).
        let mut start_idx = 0u32;
        let mut end_idx = 0u32;
        let head_fin_idx = ptr(file, edge_node, "fin")?;
        let mut f_idx = head_fin_idx;
        let mut hops = 0;
        while f_idx != 0 {
            let fin_node = xnode(file, f_idx)?;
            let v = ptr(file, fin_node, "vertex")?;
            match ch(file, fin_node, "sense")? {
                '+' if end_idx == 0 => end_idx = v,
                '-' if start_idx == 0 => start_idx = v,
                _ => {}
            }
            f_idx = ptr(file, fin_node, "other")?;
            if f_idx == head_fin_idx {
                break;
            }
            hops += 1;
            if hops > 10_000 {
                return Err(XtError::BadField {
                    index: edge_idx,
                    what: "fin ring around edge does not close",
                });
            }
        }
        let mut start = if start_idx != 0 {
            Some(self.vertex(start_idx)?)
        } else {
            None
        };
        let mut end = if end_idx != 0 {
            Some(self.vertex(end_idx)?)
        } else {
            None
        };
        // A reversed XT curve sense flips the edge into curve direction.
        if curve_reversed {
            core::mem::swap(&mut start, &mut end);
        }
        if start.is_some() != end.is_some() {
            return Err(XtError::BadField {
                index: edge_idx,
                what: "edge has exactly one vertex",
            });
        }

        let bounds = match (start, end) {
            (None, None) => None,
            (Some(s), Some(e)) => {
                let sp = self.store.vertex_position(s)?;
                let ep = self.store.vertex_position(e)?;
                match curve {
                    Some(curve) => {
                        let curve_geom = self.store.get(curve)?;
                        Some(edge_bounds(curve_geom, sp, ep, trim, curve_reversed).ok_or(
                            XtError::BadField {
                                index: edge_idx,
                                what: "could not recover edge parameter bounds on its curve",
                            },
                        )?)
                    }
                    None => Some((0.0, 1.0)),
                }
            }
            _ => unreachable!("checked above"),
        };

        let tol = tolerance(file, edge_node)?;
        if curve.is_none() && tol.is_none() {
            return Err(XtError::BadField {
                index: edge_idx,
                what: "curve-less edge has no tolerance",
            });
        }
        if curve.is_none() && bounds.is_none() {
            return Err(XtError::Unsupported {
                what: "curve-less tolerant ring edges",
            });
        }
        let e = self.store.add(Edge {
            curve,
            vertices: [start, end],
            bounds,
            fins: Vec::new(),
            tolerance: tol,
        });
        self.edges.insert(edge_idx, (e, curve_reversed));
        Ok((e, curve_reversed))
    }

    /// Reconstruct the trimmed SP-curve attached to one real FIN.
    fn fin_pcurve(
        &mut self,
        fin_idx: u32,
        face: FaceId,
        edge: EdgeId,
    ) -> Result<Option<FinPcurve>> {
        let file = self.file;
        let fin_node = xnode(file, fin_idx)?;
        let trim_idx = ptr(file, fin_node, "curve")?;
        if trim_idx == 0 {
            if self.store.get(edge)?.curve.is_none() {
                return Err(XtError::BadField {
                    index: fin_idx,
                    what: "curve-less tolerant edge fin has no SP-curve",
                });
            }
            return Ok(None);
        }
        let trim = xnode(file, trim_idx)?;
        if trim.code != code::TRIMMED_CURVE {
            return Err(XtError::BadField {
                index: trim_idx,
                what: "FIN curve is not a TRIMMED_CURVE",
            });
        }
        if ch(file, trim, "sense")? != '+' {
            return Err(XtError::BadField {
                index: trim_idx,
                what: "FIN TRIMMED_CURVE sense must be positive",
            });
        }
        let sp_idx = ptr(file, trim, "basis_curve")?;
        let sp = xnode(file, sp_idx)?;
        if sp.code != code::SP_CURVE {
            return Err(XtError::BadField {
                index: sp_idx,
                what: "FIN TRIMMED_CURVE basis is not an SP_CURVE",
            });
        }
        let sp_surface = ptr(file, sp, "surface")?;
        let (surface, _) = self.surface(sp_surface)?;
        if surface != self.store.get(face)?.surface {
            return Err(XtError::BadField {
                index: sp_idx,
                what: "SP_CURVE surface is not the FIN's face surface",
            });
        }
        let bcurve_idx = ptr(file, sp, "b_curve")?;
        let pcurve = self.pcurve_b_curve(bcurve_idx)?;
        let p1 = f64_of(file, trim, "parm_1")?;
        let p2 = f64_of(file, trim, "parm_2")?;
        let sp_forward = match ch(file, sp, "sense")? {
            '+' => true,
            '-' => false,
            _ => {
                return Err(XtError::BadField {
                    index: sp_idx,
                    what: "SP_CURVE sense is not +/-",
                });
            }
        };
        if !(p1.is_finite() && p2.is_finite() && p1 != p2 && ((p2 > p1) == sp_forward)) {
            return Err(XtError::BadField {
                index: trim_idx,
                what: "SP-curve trim parameters disagree with basis sense",
            });
        }
        let (t0, t1) = self.store.get(edge)?.bounds.ok_or(XtError::BadField {
            index: fin_idx,
            what: "FIN SP-curve is attached to an unbounded edge",
        })?;
        let scale = (p2 - p1) / (t1 - t0);
        let map = ParamMap1d::affine(scale, p1 - scale * t0).map_err(XtError::Kernel)?;
        let curve = self.store.get(pcurve)?.as_curve();
        let natural = curve.param_range();
        if !natural.contains(p1) || !natural.contains(p2) {
            return Err(XtError::BadField {
                index: trim_idx,
                what: "SP-curve trim parameters lie outside the 2D B-curve domain",
            });
        }
        let surface = self.store.get(surface)?.as_surface();
        let trim_point_1 = vector(file, trim, "point_1")?;
        let trim_point_2 = vector(file, trim, "point_2")?;
        let uv1 = curve.eval(p1);
        let uv2 = curve.eval(p2);
        let tolerance = self
            .store
            .get(edge)?
            .tolerance
            .unwrap_or(LINEAR_RESOLUTION)
            .max(LINEAR_RESOLUTION);
        if surface.eval([uv1.x, uv1.y]).dist(trim_point_1) > tolerance
            || surface.eval([uv2.x, uv2.y]).dist(trim_point_2) > tolerance
        {
            return Err(XtError::BadField {
                index: trim_idx,
                what: "TRIMMED_CURVE points do not match its SP-curve parameters",
            });
        }
        let use_ = FinPcurve::new(pcurve, ParamRange::new(p1.min(p2), p1.max(p2)), map)
            .map_err(XtError::Kernel)?;
        Ok(Some(use_))
    }

    fn pcurve_b_curve(&mut self, curve_idx: u32) -> Result<Curve2dId> {
        if let Some(&curve) = self.pcurves.get(&curve_idx) {
            return Ok(curve);
        }
        let node = xnode(self.file, curve_idx)?;
        if node.code != code::B_CURVE {
            return Err(XtError::BadField {
                index: curve_idx,
                what: "SP_CURVE parameter geometry is not a B_CURVE",
            });
        }
        let curve = self.b_curve_2d(curve_idx, node)?;
        let id = self.store.add(Curve2dGeom::Nurbs(curve));
        self.pcurves.insert(curve_idx, id);
        Ok(id)
    }

    fn b_curve_2d(&mut self, curve_idx: u32, node: &Node) -> Result<NurbsCurve2d> {
        let file = self.file;
        let nurbs_idx = ptr(file, node, "nurbs")?;
        let n = xnode(file, nurbs_idx)?;
        let degree = f64_of(file, n, "degree")? as usize;
        let n_vertices = f64_of(file, n, "n_vertices")? as usize;
        let vertex_dim = f64_of(file, n, "vertex_dim")? as usize;
        if logical_of(file, n, "periodic")? {
            return Err(XtError::Unsupported {
                what: "periodic 2D B-curves",
            });
        }
        let rational = logical_of(file, n, "rational")?;
        let knots = self.knot_vector(ptr(file, n, "knots")?, ptr(file, n, "knot_mult")?)?;
        let raw = self.doubles(ptr(file, n, "bspline_vertices")?, "vertices")?;
        if raw.len() != n_vertices * vertex_dim {
            return Err(XtError::BadField {
                index: curve_idx,
                what: "2D bspline vertex array length mismatch",
            });
        }
        let (points, weights) = split_poles_2d(&raw, vertex_dim, rational)?;
        NurbsCurve2d::new(degree, knots, points, weights).map_err(XtError::Kernel)
    }

    /// Convert an XT curve node to kernel geometry. Returns the curve and
    /// whether the XT sense was `-` (reversed against its
    /// parameterization).
    fn curve(&mut self, curve_idx: u32) -> Result<(CurveId, bool)> {
        if let Some(&(c, r)) = self.curves.get(&curve_idx) {
            return Ok((c, r));
        }
        let file = self.file;
        let node = xnode(file, curve_idx)?;
        let reversed = ch(file, node, "sense")? == '-';
        let geom: CurveGeom = match node.code {
            code::LINE => {
                let origin = in_size_box(vector(file, node, "pvec")?)?;
                let dir = vector(file, node, "direction")?;
                Line::new(origin, dir).map_err(XtError::Kernel)?.into()
            }
            code::CIRCLE => {
                let frame = frame_from(file, node, "centre", "normal", "x_axis")?;
                let radius = f64_of(file, node, "radius")?;
                Circle::new(frame, radius).map_err(XtError::Kernel)?.into()
            }
            code::ELLIPSE => {
                let frame = frame_from(file, node, "centre", "normal", "x_axis")?;
                let major = f64_of(file, node, "major_radius")?;
                let minor = f64_of(file, node, "minor_radius")?;
                Ellipse::new(frame, major, minor)
                    .map_err(XtError::Kernel)?
                    .into()
            }
            code::B_CURVE => self.b_curve(curve_idx, node)?.into(),
            code::INTERSECTION | code::SP_CURVE | code::PE_CURVE => {
                return Err(XtError::Unsupported {
                    what: "procedural curves (intersection/SP/foreign) — Tier 2",
                });
            }
            _ => {
                return Err(XtError::BadField {
                    index: curve_idx,
                    what: "node referenced as a curve is not a curve",
                });
            }
        };
        let c = self.store.add(geom);
        self.curves.insert(curve_idx, (c, reversed));
        Ok((c, reversed))
    }

    fn b_curve(&mut self, curve_idx: u32, node: &Node) -> Result<NurbsCurve> {
        let file = self.file;
        let nurbs_idx = ptr(file, node, "nurbs")?;
        let n = xnode(file, nurbs_idx)?;
        let degree = f64_of(file, n, "degree")? as usize;
        let n_vertices = f64_of(file, n, "n_vertices")? as usize;
        let vertex_dim = f64_of(file, n, "vertex_dim")? as usize;
        if logical_of(file, n, "periodic")? {
            return Err(XtError::Unsupported {
                what: "periodic B-curves (kernel periodic NURBS lands at M3)",
            });
        }
        let rational = logical_of(file, n, "rational")?;
        let knots = self.knot_vector(ptr(file, n, "knots")?, ptr(file, n, "knot_mult")?)?;
        let raw = self.doubles(ptr(file, n, "bspline_vertices")?, "vertices")?;
        if raw.len() != n_vertices * vertex_dim {
            return Err(XtError::BadField {
                index: curve_idx,
                what: "bspline vertex array length mismatch",
            });
        }
        let (points, weights) = split_poles(&raw, vertex_dim, rational)?;
        for p in &points {
            in_size_box(*p)?;
        }
        NurbsCurve::new(degree, knots, points, weights).map_err(XtError::Kernel)
    }

    fn surface(&mut self, surface_idx: u32) -> Result<(SurfaceId, char)> {
        if let Some(&(s, sense)) = self.surfaces.get(&surface_idx) {
            return Ok((s, sense));
        }
        let file = self.file;
        let node = xnode(file, surface_idx)?;
        let sense = ch(file, node, "sense")?;
        let geom: SurfaceGeom = match node.code {
            code::PLANE => {
                let frame = frame_from(file, node, "pvec", "normal", "x_axis")?;
                Plane::new(frame).into()
            }
            code::CYLINDER => {
                let frame = frame_from(file, node, "pvec", "axis", "x_axis")?;
                let radius = f64_of(file, node, "radius")?;
                Cylinder::new(frame, radius)
                    .map_err(XtError::Kernel)?
                    .into()
            }
            code::CONE => cone_from(file, node)?.into(),
            code::SPHERE => {
                let frame = frame_from(file, node, "centre", "axis", "x_axis")?;
                let radius = f64_of(file, node, "radius")?;
                Sphere::new(frame, radius).map_err(XtError::Kernel)?.into()
            }
            code::TORUS => {
                let frame = frame_from(file, node, "centre", "axis", "x_axis")?;
                let major = f64_of(file, node, "major_radius")?;
                let minor = f64_of(file, node, "minor_radius")?;
                if major <= minor {
                    return Err(XtError::Unsupported {
                        what: "self-intersecting (apple/lemon) tori",
                    });
                }
                Torus::new(frame, major, minor)
                    .map_err(XtError::Kernel)?
                    .into()
            }
            code::B_SURFACE => self.b_surface(surface_idx, node)?.into(),
            code::SWEPT_SURF
            | code::SPUN_SURF
            | code::OFFSET_SURF
            | code::BLENDED_EDGE
            | code::BLEND_BOUND
            | code::PE_SURF => {
                return Err(XtError::Unsupported {
                    what: "procedural surfaces (swept/spun/offset/blend/foreign) — Tier 2",
                });
            }
            _ => {
                return Err(XtError::BadField {
                    index: surface_idx,
                    what: "node referenced as a surface is not a surface",
                });
            }
        };
        let s = self.store.add(geom);
        self.surfaces.insert(surface_idx, (s, sense));
        Ok((s, sense))
    }

    fn b_surface(&mut self, surface_idx: u32, node: &Node) -> Result<NurbsSurface> {
        let file = self.file;
        let nurbs_idx = ptr(file, node, "nurbs")?;
        let n = xnode(file, nurbs_idx)?;
        if logical_of(file, n, "u_periodic")? || logical_of(file, n, "v_periodic")? {
            return Err(XtError::Unsupported {
                what: "periodic B-surfaces (kernel periodic NURBS lands at M3)",
            });
        }
        let rational = logical_of(file, n, "rational")?;
        let u_degree = f64_of(file, n, "u_degree")? as usize;
        let v_degree = f64_of(file, n, "v_degree")? as usize;
        let n_u = f64_of(file, n, "n_u_vertices")? as usize;
        let n_v = f64_of(file, n, "n_v_vertices")? as usize;
        let vertex_dim = f64_of(file, n, "vertex_dim")? as usize;
        let u_knots = self.knot_vector(ptr(file, n, "u_knots")?, ptr(file, n, "u_knot_mult")?)?;
        let v_knots = self.knot_vector(ptr(file, n, "v_knots")?, ptr(file, n, "v_knot_mult")?)?;
        let raw = self.doubles(ptr(file, n, "bspline_vertices")?, "vertices")?;
        if raw.len() != n_u * n_v * vertex_dim {
            return Err(XtError::BadField {
                index: surface_idx,
                what: "bspline vertex array length mismatch",
            });
        }
        // Pole ordering: assumed v-fastest (matching the kernel's
        // `i*nv + j` layout). Provisional — to be re-verified against a
        // real-world B-surface part during M3b round-trip testing.
        let (points, weights) = split_poles(&raw, vertex_dim, rational)?;
        for p in &points {
            in_size_box(*p)?;
        }
        NurbsSurface::new(u_degree, v_degree, u_knots, v_knots, points, weights)
            .map_err(XtError::Kernel)
    }

    /// Expand an XT (distinct knots, multiplicities) pair into the full
    /// knot vector.
    fn knot_vector(&mut self, knots_idx: u32, mult_idx: u32) -> Result<Vec<f64>> {
        let knots = self.doubles(knots_idx, "knots")?;
        let mult_node = xnode(self.file, mult_idx)?;
        let mults: Vec<i64> = match field(self.file, mult_node, "mult")? {
            Value::Arr(vs) => vs.iter().filter_map(Value::as_int).collect(),
            _ => {
                return Err(XtError::BadField {
                    index: mult_idx,
                    what: "knot multiplicities are not an array",
                });
            }
        };
        if mults.len() != knots.len() {
            return Err(XtError::BadField {
                index: mult_idx,
                what: "knot and multiplicity arrays differ in length",
            });
        }
        let mut out = Vec::new();
        for (k, m) in knots.iter().zip(&mults) {
            for _ in 0..*m {
                out.push(*k);
            }
        }
        Ok(out)
    }

    fn doubles(&self, idx: u32, name: &'static str) -> Result<Vec<f64>> {
        let node = xnode(self.file, idx)?;
        match field(self.file, node, name)? {
            Value::Arr(vs) => vs
                .iter()
                .map(|v| {
                    v.as_f64().ok_or(XtError::BadField {
                        index: idx,
                        what: "non-numeric value in double array",
                    })
                })
                .collect(),
            _ => Err(XtError::BadField {
                index: idx,
                what: "expected a double array",
            }),
        }
    }
}

/// Build a kernel frame from XT origin/axis/x-axis fields.
fn frame_from(
    file: &XtFile,
    node: &Node,
    origin: &'static str,
    z: &'static str,
    x: &'static str,
) -> Result<Frame> {
    let o = in_size_box(vector(file, node, origin)?)?;
    let zv = vector(file, node, z)?;
    let xv = vector(file, node, x)?;
    Frame::new(o, zv, xv).map_err(XtError::Kernel)
}

/// XT cone → kernel cone. XT: `R(u,v) = P − vA + (X cos u + Y sin u)
/// (r + v tan α)` — the axis points away from the half in use — so the
/// kernel frame takes `z = −A`, giving the same point set under our
/// slant parameterization.
fn cone_from(file: &XtFile, node: &Node) -> Result<Cone> {
    let pvec = in_size_box(vector(file, node, "pvec")?)?;
    let axis = vector(file, node, "axis")?;
    let x_axis = vector(file, node, "x_axis")?;
    let radius = f64_of(file, node, "radius")?;
    let sin_a = f64_of(file, node, "sin_half_angle")?;
    let cos_a = f64_of(file, node, "cos_half_angle")?;
    let half_angle = math::atan2(sin_a, cos_a);
    let frame = Frame::new(pvec, -axis, x_axis).map_err(XtError::Kernel)?;
    Cone::new(frame, radius, half_angle).map_err(XtError::Kernel)
}

/// Split a flat XT pole array into points and optional weights.
/// Rational poles are stored premultiplied (`x·w, y·w, z·w, w`).
fn split_poles(raw: &[f64], dim: usize, rational: bool) -> Result<(Vec<Point3>, Option<Vec<f64>>)> {
    let expected_dim = if rational { 4 } else { 3 };
    if dim != expected_dim {
        return Err(XtError::Unsupported {
            what: "B-geometry with vertex dimension other than 3 (or 4 rational)",
        });
    }
    let mut points = Vec::new();
    let mut weights = Vec::new();
    for pole in raw.chunks_exact(dim) {
        if rational {
            let w = pole[3];
            if w <= 0.0 {
                return Err(XtError::BadField {
                    index: 0,
                    what: "non-positive rational weight",
                });
            }
            points.push(Point3::new(pole[0] / w, pole[1] / w, pole[2] / w));
            weights.push(w);
        } else {
            points.push(Point3::new(pole[0], pole[1], pole[2]));
        }
    }
    Ok((points, if rational { Some(weights) } else { None }))
}

/// Split a flat XT 2D pole array. Rational poles are premultiplied
/// (`u·w, v·w, w`).
fn split_poles_2d(
    raw: &[f64],
    dim: usize,
    rational: bool,
) -> Result<(Vec<Point2>, Option<Vec<f64>>)> {
    let expected_dim = if rational { 3 } else { 2 };
    if dim != expected_dim {
        return Err(XtError::Unsupported {
            what: "2D B-geometry with vertex dimension other than 2 (or 3 rational)",
        });
    }
    let mut points = Vec::new();
    let mut weights = Vec::new();
    for pole in raw.chunks_exact(dim) {
        if rational {
            let w = pole[2];
            if w <= 0.0 {
                return Err(XtError::BadField {
                    index: 0,
                    what: "non-positive 2D rational weight",
                });
            }
            points.push(Point2::new(pole[0] / w, pole[1] / w));
            weights.push(w);
        } else {
            points.push(Point2::new(pole[0], pole[1]));
        }
    }
    Ok((points, if rational { Some(weights) } else { None }))
}

/// Recover the parameter interval of an edge from its endpoint positions
/// on the (natural-direction) curve. `trim` short-circuits with the
/// parameters stored in a trimmed curve, oriented to increase along the
/// natural direction.
fn edge_bounds(
    curve: &CurveGeom,
    start: Point3,
    end: Point3,
    trim: Option<(f64, f64)>,
    curve_reversed: bool,
) -> Option<(f64, f64)> {
    if let Some((p1, p2)) = trim {
        // With a '+' basis sense parm_2 > parm_1; with '-' they come
        // reversed (the edge flip already swapped the vertices).
        let (lo, hi) = if curve_reversed { (p2, p1) } else { (p1, p2) };
        return (hi > lo).then_some((lo, hi));
    }
    let tau = core::f64::consts::TAU;
    match curve {
        CurveGeom::Line(line) => {
            let t0 = (start - line.origin()).dot(line.dir());
            let t1 = (end - line.origin()).dot(line.dir());
            (t1 > t0).then_some((t0, t1))
        }
        CurveGeom::Circle(c) => {
            let f = c.frame();
            let angle = |p: Point3| {
                let l = f.to_local(p);
                wrap_periodic(math::atan2(l.y, l.x), 0.0, tau)
            };
            Some(unwrap_interval(angle(start), angle(end), tau))
        }
        CurveGeom::Ellipse(e) => {
            let f = e.frame();
            let angle = |p: Point3| {
                let l = f.to_local(p);
                wrap_periodic(
                    math::atan2(l.y / e.minor_radius(), l.x / e.major_radius()),
                    0.0,
                    tau,
                )
            };
            Some(unwrap_interval(angle(start), angle(end), tau))
        }
        CurveGeom::Nurbs(n) => {
            let range = n.param_range();
            let t0 = kgeom::project::project_to_curve(n, start, range)?.t;
            let t1 = kgeom::project::project_to_curve(n, end, range)?.t;
            (t1 > t0).then_some((t0, t1))
        }
    }
}

/// Make `(t0, t1)` increasing on a periodic curve, unwrapping past the
/// seam; coincident endpoints mean a full-period closed edge.
fn unwrap_interval(t0: f64, t1: f64, period: f64) -> (f64, f64) {
    if t1 > t0 { (t0, t1) } else { (t0, t1 + period) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unwrap_interval_handles_seam_and_closure() {
        let tau = core::f64::consts::TAU;
        assert_eq!(unwrap_interval(1.0, 2.0, tau), (1.0, 2.0));
        let (a, b) = unwrap_interval(5.0, 1.0, tau);
        assert_eq!(a, 5.0);
        assert!((b - (1.0 + tau)).abs() < 1e-15);
        // Coincident endpoints: full period.
        let (a, b) = unwrap_interval(0.5, 0.5, tau);
        assert_eq!(a, 0.5);
        assert!((b - (0.5 + tau)).abs() < 1e-15);
    }

    #[test]
    fn split_poles_unweights_rational_data() {
        let raw = [2.0, 4.0, 6.0, 2.0, 1.0, 0.0, 0.0, 1.0];
        let (pts, w) = split_poles(&raw, 4, true).unwrap();
        assert_eq!(pts[0], Point3::new(1.0, 2.0, 3.0));
        assert_eq!(w.unwrap(), vec![2.0, 1.0]);
        assert!(split_poles(&raw, 2, false).is_err());
    }

    #[test]
    fn split_poles_2d_unweights_rational_data() {
        let raw = [2.0, 4.0, 2.0, 1.0, 0.0, 1.0];
        let (points, weights) = split_poles_2d(&raw, 3, true).unwrap();
        assert_eq!(points, vec![Point2::new(1.0, 2.0), Point2::new(1.0, 0.0)]);
        assert_eq!(weights.unwrap(), vec![2.0, 1.0]);
        assert!(split_poles_2d(&raw, 2, true).is_err());
    }
}
