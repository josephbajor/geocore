//! Whole-body tessellation: one watertight mesh per body.
//!
//! The crack-elimination contract (spec §L2, M2 exit criterion):
//!
//! 1. Every topological **edge is discretized exactly once** into a 3D
//!    polyline (chordal refinement of its curve), producing shared mesh
//!    vertices.
//! 2. Each face builds its UV trim loops from those *frozen* edge
//!    polylines by evaluating the fin's explicit pcurve at every retained
//!    edge parameter. Legacy topology without pcurves falls back to surface
//!    inversion. Periodic surfaces are seam-cut; zero-loop closed faces get
//!    full-period rectangles with seam and pole rows welded by index.
//! 3. Faces tessellate with [`kgeom::tess`] (frozen boundary), and the
//!    body mesh is assembled by **index mapping** — never by positional
//!    welding.
//!
//! Result: across any two adjacent faces the shared edge contributes the
//! same vertex indices to both, so the mesh is watertight by construction;
//! [`check_watertight`] verifies every interior mesh edge is used by
//! exactly two triangles with opposite orientation. Triangles are oriented
//! outward (away from material).
//!
//! Crack-prevention rule: edge polylines are refined until every segment
//! satisfies `kgeom::tess`'s own boundary criterion — the surface point at
//! the UV midpoint within tolerance of the 3D chord — against **every**
//! adjacent face's surface, using a safety margin ([`MARGIN`]) so kgeom's
//! re-measurement (bitwise identical on ordinary loops, ulp-perturbed on
//! period-welded copies) can never decide to insert a boundary vertex. A
//! boundary-count mismatch after face tessellation is therefore a kernel
//! bug, reported as an error, never silently accepted as a crack.
//!
//! Closed faces (sphere, torus) are split at half-periods into 2 / 4
//! rectangular patches so that no single patch's UV domain contains two
//! boundary points welded to the same mesh vertex (except sphere pole
//! rows, which intentionally collapse to one vertex; the triangles that
//! degenerate under that collapse are dropped). Spherical polar caps — a
//! face bounded by a single loop winding the `u` period once, as produced
//! by real-world XT cut spheres — reuse the same machinery: the contained
//! pole (chosen by the loop's material side) stands in for the missing
//! second boundary, and the domain splits into two patches at an existing
//! chain sample near the half period. A distinct bipolar case handles
//! meridional loops that pass through both sphere poles: it splits the loop
//! into two frozen pole-to-pole sides and welds the parameter-singular pole
//! rows by global mesh identity.

use crate::entity::{
    BodyId, Edge, EdgeId, FaceDomain, FaceId, FinPcurve, Sense, SurfaceId, VertexId,
};
use crate::geom::{Curve2dGeom, SurfaceGeom};
use crate::store::Store;
use kcore::error::Error;
use kcore::operation::{
    AccountingMode, BudgetPlan, DiagnosticCode, DiagnosticKind, ExecutionPolicy, LimitSnapshot,
    LimitSpec, NumericalPolicy, OperationContext, OperationOutcome, OperationPolicyError,
    OperationScope, PolicyVersion, ResourceKind, SessionPolicy, SessionPrecision, StageId,
};
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::surface::Surface;
use kgeom::surface_point::{invert_surface_point_in_scope, normalize_surface_uv};
pub use kgeom::tess::TessOptions;
use kgeom::tess::{
    FACE_TESSELLATION_BOUNDARY_DEPTH, FACE_TESSELLATION_BOUNDARY_DEPTH_LIMIT,
    FACE_TESSELLATION_BOUNDARY_SPLIT_LIMIT, FACE_TESSELLATION_BOUNDARY_SPLITS,
    FACE_TESSELLATION_MESH_TRIANGLE_LIMIT, FACE_TESSELLATION_MESH_TRIANGLES,
    FACE_TESSELLATION_MESH_VERTEX_LIMIT, FACE_TESSELLATION_MESH_VERTICES,
    FACE_TESSELLATION_REFINEMENT_PASS_LIMIT, FACE_TESSELLATION_REFINEMENT_PASSES,
    FaceTessellationBudgetProfile, TrimLoop, TrimmedSurface, tessellate_in_sequential_ledger,
};
use kgeom::vec::{Point3, Vec2};
use kgraph::{EvalContext, EvalResult};
mod error;
mod offset;
mod policy;

pub use error::{
    EVALUATION_FAILED, OFFSET_PERIODIC_WINDING, PROCEDURAL_LEAF_ALGORITHM,
    REGULARITY_INDETERMINATE, SURFACE_REGULARITY_PROOF, TessellationError, TessellationResult,
    UNSUPPORTED_TESSELLATION,
};
use offset::{eval_surface_point, face_case_planar_offset, surface_periodicity};
use policy::validate_body_tessellation_budget;
pub use policy::{
    BODY_TESSELLATION_EDGE_DEPTH, BODY_TESSELLATION_EDGE_DEPTH_LIMIT,
    BODY_TESSELLATION_EDGE_DEPTH_LIMIT_REACHED, BODY_TESSELLATION_EDGE_SPLIT_LIMIT_REACHED,
    BODY_TESSELLATION_EDGE_SPLITS, BODY_TESSELLATION_ISO_ARC_DEPTH,
    BODY_TESSELLATION_ISO_ARC_DEPTH_LIMIT, BODY_TESSELLATION_ISO_ARC_DEPTH_LIMIT_REACHED,
    BODY_TESSELLATION_ISO_ARC_SPLIT_LIMIT_REACHED, BODY_TESSELLATION_ISO_ARC_SPLITS,
    BODY_TESSELLATION_MESH_VERTEX_LIMIT, BODY_TESSELLATION_MESH_VERTEX_LIMIT_REACHED,
    BODY_TESSELLATION_MESH_VERTICES, BODY_TESSELLATION_SPLIT_LIMIT,
    BODY_TESSELLATION_TOTAL_WORK_LIMIT_REACHED, BodyTessellationBudgetProfile,
};

type Result<T> = TessellationResult<T>;

/// A watertight tessellation of one body.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyMesh {
    /// Vertex positions in model space.
    pub positions: Vec<Point3>,
    /// Triangles as vertex-index triples, oriented outward (counter-
    /// clockwise seen from outside the material).
    pub triangles: Vec<[u32; 3]>,
    /// Per face, the range of `triangles` it produced, in
    /// [`Store::faces_of_body`] order.
    pub face_ranges: Vec<(FaceId, core::ops::Range<usize>)>,
    /// Per topological edge, its polyline as vertex indices, in
    /// [`Store::edges_of_body`] order (closed polylines repeat the first
    /// index last).
    pub edge_polylines: Vec<(EdgeId, Vec<u32>)>,
}

impl BodyMesh {
    /// Serialize as Wavefront OBJ (positions + triangles; 1-indexed).
    pub fn to_obj(&self) -> String {
        let mut out = String::new();
        for p in &self.positions {
            out.push_str(&format!("v {:?} {:?} {:?}\n", p.x, p.y, p.z));
        }
        for t in &self.triangles {
            out.push_str(&format!("f {} {} {}\n", t[0] + 1, t[1] + 1, t[2] + 1));
        }
        out
    }
}

/// Signed volume of a triangle mesh via the divergence theorem
/// (`Σ det(a, b, c) / 6`). Positive for a closed mesh with outward
/// orientation; exact for the mesh itself.
pub fn signed_volume(mesh: &BodyMesh) -> f64 {
    let mut vol = 0.0;
    for t in &mesh.triangles {
        let [a, b, c] = t.map(|i| mesh.positions[i as usize]);
        vol += a.dot(b.cross(c)) / 6.0;
    }
    vol
}

/// Verify closed-solid watertightness. Returns a list of human-readable
/// problems; empty means the mesh is watertight: every undirected triangle
/// edge is used by exactly two triangles with opposite directed
/// orientation, no triangle is degenerate, and every vertex is referenced.
pub fn check_watertight(mesh: &BodyMesh) -> Vec<String> {
    use std::collections::BTreeMap;
    let mut problems = Vec::new();
    let mut directed: BTreeMap<(u32, u32), usize> = BTreeMap::new();
    let mut undirected: BTreeMap<(u32, u32), i64> = BTreeMap::new();
    let mut referenced = vec![false; mesh.positions.len()];
    for (ti, t) in mesh.triangles.iter().enumerate() {
        let [a, b, c] = *t;
        if a == b || b == c || c == a {
            problems.push(format!("triangle {ti} is degenerate: {t:?}"));
            continue;
        }
        for &i in t {
            match referenced.get_mut(i as usize) {
                Some(r) => *r = true,
                None => problems.push(format!("triangle {ti} references vertex {i} out of range")),
            }
        }
        for (i, j) in [(a, b), (b, c), (c, a)] {
            *directed.entry((i, j)).or_insert(0) += 1;
            let key = if i < j { (i, j) } else { (j, i) };
            *undirected.entry(key).or_insert(0) += 1;
        }
    }
    for (e, n) in &directed {
        if *n > 1 {
            problems.push(format!("directed edge {e:?} used {n} times"));
        }
    }
    for (e, n) in &undirected {
        if *n != 2 {
            problems.push(format!("undirected edge {e:?} used {n} times (want 2)"));
        }
    }
    for (i, r) in referenced.iter().enumerate() {
        if !r {
            problems.push(format!("vertex {i} is not referenced by any triangle"));
        }
    }
    problems
}

/// Safety factor between this module's refinement tolerance and the
/// tolerance kgeom re-measures against, so ulp-level differences on
/// period-welded boundary copies can never trigger a boundary insertion.
const MARGIN: f64 = 0.9;
/// Recursion cap for edge / iso-arc refinement (2^16 segments).
const MAX_DEPTH: usize = 16;

/// Refinement tolerances, margin-scaled from the caller's [`TessOptions`].
#[derive(Clone, Copy)]
struct Ctx {
    tol: f64,
    max_len: f64,
}

/// Growing global vertex pool.
struct MeshAcc {
    positions: Vec<Point3>,
}

impl MeshAcc {
    fn push(&mut self, p: Point3, work: &mut BodyTessellationWork<'_, '_, '_>) -> Result<u32> {
        if u32::try_from(self.positions.len()).is_err() {
            return work.reject_physical_mesh_vertex(self.positions.len());
        }
        work.charge_mesh_vertex()?;
        let i = mesh_vertex_index(self.positions.len())?;
        self.positions.push(p);
        Ok(i)
    }
    fn pos(&self, gid: u32) -> Point3 {
        self.positions[gid as usize]
    }
}

/// Shared deterministic accounting for one whole-body tessellation.
struct BodyTessellationWork<'scope, 'context, 'session> {
    scope: &'scope mut OperationScope<'context, 'session>,
}

impl BodyTessellationWork<'_, '_, '_> {
    fn charge_split(
        &mut self,
        stage: StageId,
        diagnostic: DiagnosticCode,
        message: &'static str,
    ) -> Result<()> {
        let result = self
            .scope
            .ledger_mut()
            .charge_resource(stage, ResourceKind::Work, 1);
        self.direct_limit(result, diagnostic, message)
    }

    fn charge_mesh_vertex(&mut self) -> Result<()> {
        let result = self.scope.ledger_mut().charge_resource(
            BODY_TESSELLATION_MESH_VERTICES,
            ResourceKind::Items,
            1,
        );
        self.direct_limit(
            result,
            BODY_TESSELLATION_MESH_VERTEX_LIMIT_REACHED,
            "whole-body tessellation mesh vertex limit reached",
        )
    }

    fn reject_physical_mesh_vertex(&mut self, current_items: usize) -> Result<u32> {
        let snapshot = LimitSnapshot {
            stage: BODY_TESSELLATION_MESH_VERTICES,
            resource: ResourceKind::Items,
            consumed: u64::try_from(current_items)
                .unwrap_or(u64::MAX)
                .saturating_add(1),
            allowed: BODY_TESSELLATION_MESH_VERTEX_LIMIT,
        };
        let configured = self
            .scope
            .ledger()
            .snapshots()
            .into_iter()
            .find(|entry| {
                entry.stage == BODY_TESSELLATION_MESH_VERTICES
                    && entry.resource == ResourceKind::Items
            })
            .expect("validated body profile retains mesh vertex accounting");
        if configured.allowed <= BODY_TESSELLATION_MESH_VERTEX_LIMIT {
            let result = self.scope.ledger_mut().charge_resource(
                BODY_TESSELLATION_MESH_VERTICES,
                ResourceKind::Items,
                1,
            );
            let _ = self.direct_limit(
                result,
                BODY_TESSELLATION_MESH_VERTEX_LIMIT_REACHED,
                "whole-body tessellation mesh vertex limit reached",
            );
        } else {
            // Raising policy cannot enlarge the u32 mesh index address space.
            // The physical rejection remains atomic and is diagnosed against
            // the canonical format ceiling. It is intentionally not a ledger
            // limit event: the configured ledger did not reject the unit.
            self.scope.diagnose(
                snapshot.stage,
                BODY_TESSELLATION_MESH_VERTEX_LIMIT_REACHED,
                DiagnosticKind::LimitReached(snapshot),
                "whole-body tessellation mesh vertex format limit reached",
            );
        }
        Err(Error::ResourceLimit { snapshot }.into())
    }

    fn observe_depth(
        &mut self,
        stage: StageId,
        value: u64,
        diagnostic: kcore::operation::DiagnosticCode,
        message: &'static str,
    ) -> Result<()> {
        let local_allowed = if stage == BODY_TESSELLATION_EDGE_DEPTH {
            BODY_TESSELLATION_EDGE_DEPTH_LIMIT
        } else {
            debug_assert_eq!(stage, BODY_TESSELLATION_ISO_ARC_DEPTH);
            BODY_TESSELLATION_ISO_ARC_DEPTH_LIMIT
        };
        let plan = BudgetPlan::new([LimitSpec::new(
            stage,
            ResourceKind::Depth,
            AccountingMode::HighWater,
            local_allowed,
        )])
        .expect("built-in body refinement depth plan is valid");
        let result = self
            .scope
            .ledger_mut()
            .sequential(plan)
            .and_then(|mut ledger| ledger.observe(stage, ResourceKind::Depth, value));
        self.direct_limit(result, diagnostic, message)
    }

    fn direct_limit(
        &mut self,
        result: core::result::Result<(), OperationPolicyError>,
        diagnostic: kcore::operation::DiagnosticCode,
        message: &'static str,
    ) -> Result<()> {
        if let Err(OperationPolicyError::LimitReached(snapshot)) = result {
            let (diagnostic, message) = if snapshot.stage == kcore::operation::TOTAL_WORK_STAGE {
                (
                    BODY_TESSELLATION_TOTAL_WORK_LIMIT_REACHED,
                    "whole-body tessellation total work limit reached",
                )
            } else {
                (diagnostic, message)
            };
            self.scope.diagnose(
                snapshot.stage,
                diagnostic,
                DiagnosticKind::LimitReached(snapshot),
                message,
            );
            return Err(Error::ResourceLimit { snapshot }.into());
        }
        result.map_err(Error::from).map_err(Into::into)
    }

    fn graph_query<T>(
        &mut self,
        store: &Store,
        query: impl FnOnce(&mut EvalContext<'_>) -> EvalResult<T>,
    ) -> Result<T> {
        match crate::graph_work::query_sequential(self.scope, store, query) {
            Err(OperationPolicyError::LimitReached(snapshot)) => {
                Err(Error::ResourceLimit { snapshot }.into())
            }
            Err(error) => Err(Error::from(error).into()),
            Ok(lower) => lower.map_err(Into::into),
        }
    }
}

/// A refinement decision may already own the midpoint evaluation required to
/// establish curvature error. Length-forced splits deliberately defer that
/// evaluation until depth and split/root admission have both succeeded.
enum SplitDecision<T> {
    Keep,
    Split { evaluated_midpoint: Option<T> },
}

fn mesh_vertex_index(current_items: usize) -> Result<u32> {
    u32::try_from(current_items).map_err(|_| {
        Error::ResourceLimit {
            snapshot: LimitSnapshot {
                stage: BODY_TESSELLATION_MESH_VERTICES,
                resource: ResourceKind::Items,
                consumed: u64::try_from(current_items)
                    .unwrap_or(u64::MAX)
                    .saturating_add(1),
                allowed: BODY_TESSELLATION_MESH_VERTEX_LIMIT,
            },
        }
        .into()
    })
}

fn next_refinement_depth(
    depth: usize,
    stage: StageId,
    diagnostic: kcore::operation::DiagnosticCode,
    message: &'static str,
    work: &mut BodyTessellationWork<'_, '_, '_>,
) -> Result<usize> {
    let next = depth.saturating_add(1);
    work.observe_depth(
        stage,
        u64::try_from(next).unwrap_or(u64::MAX),
        diagnostic,
        message,
    )?;
    Ok(next)
}

/// Distance from `p` to the 3D segment `[a, b]`.
fn point_seg_dist(p: Point3, a: Point3, b: Point3) -> f64 {
    let ab = b - a;
    let len_sq = ab.norm_sq();
    if len_sq == 0.0 {
        return p.dist(a);
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    p.dist(a + ab * t)
}

fn require_leaf_surface(surface: &SurfaceGeom) -> Result<&dyn Surface> {
    surface
        .as_leaf_surface()
        .ok_or(TessellationError::Unsupported {
            capability: PROCEDURAL_LEAF_ALGORITHM,
        })
}

/// Invert a point known to lie on the surface to UV coordinates, with
/// periodic parameters wrapped into the surface's base range.
fn invert_uv(
    sg: &SurfaceGeom,
    p: Point3,
    work: &mut BodyTessellationWork<'_, '_, '_>,
) -> Result<Vec2> {
    let surface = require_leaf_surface(sg)?;
    let mapped = invert_surface_point_in_scope(surface, p, work.scope)
        .map_err(TessellationError::SurfacePoint)?;
    let uv = normalize_surface_uv(surface, mapped.uv);
    Ok(Vec2::new(uv[0], uv[1]))
}

/// Shift `uv` by whole periods so it lands nearest `prev`.
fn unwrap_near(mut uv: Vec2, prev: Vec2, periods: [Option<f64>; 2]) -> Vec2 {
    if let Some(p) = periods[0] {
        uv.x -= p * ((uv.x - prev.x) / p).round();
    }
    if let Some(p) = periods[1] {
        uv.y -= p * ((uv.y - prev.y) / p).round();
    }
    uv
}

/// One face's parameter-space use of an edge during shared refinement.
struct FaceUse<'a> {
    store: &'a Store,
    surface_id: SurfaceId,
    surface: &'a SurfaceGeom,
    pcurve: Option<(&'a Curve2dGeom, FinPcurve)>,
}

impl FaceUse<'_> {
    fn uv_at(
        &self,
        edge_parameter: f64,
        point: Point3,
        work: &mut BodyTessellationWork<'_, '_, '_>,
    ) -> Result<Vec2> {
        match self.pcurve {
            Some((geometry, use_)) => Ok(use_.evaluate_uv(
                geometry.as_curve(),
                edge_parameter,
                surface_periodicity(self.store, self.surface_id, work)?,
            )?),
            None => invert_uv(self.surface, point, work),
        }
    }
}

/// Edge-polyline refinement: split until the curve chord criterion *and*
/// kgeom's boundary criterion against every adjacent face use hold with
/// margin. Explicit pcurves preserve seam branches; legacy uses invert.
struct CurveRefine<'a> {
    curve: Option<&'a dyn Curve>,
    face_uses: Vec<FaceUse<'a>>,
    ctx: Ctx,
}

impl CurveRefine<'_> {
    fn point_at(
        &self,
        edge_parameter: f64,
        work: &mut BodyTessellationWork<'_, '_, '_>,
    ) -> Result<Point3> {
        if let Some(curve) = self.curve {
            return Ok(curve.eval(edge_parameter));
        }
        if self.face_uses.is_empty() {
            return Err(Error::InvalidGeometry {
                reason: "curve-less tolerant edge has no adjacent face pcurves",
            }
            .into());
        }
        let mut xyz = [0.0; 3];
        for face_use in &self.face_uses {
            let uv = face_use.uv_at(
                edge_parameter,
                Point3::new(f64::NAN, f64::NAN, f64::NAN),
                work,
            )?;
            let p = eval_surface_point(face_use.store, face_use.surface_id, uv, work)?;
            xyz[0] += p.x;
            xyz[1] += p.y;
            xyz[2] += p.z;
        }
        let n = self.face_uses.len() as f64;
        Ok(Point3::new(xyz[0] / n, xyz[1] / n, xyz[2] / n))
    }

    fn split_decision(
        &self,
        a: (f64, Point3),
        b: (f64, Point3),
        work: &mut BodyTessellationWork<'_, '_, '_>,
    ) -> Result<SplitDecision<Point3>> {
        if a.1.dist(b.1) > self.ctx.max_len {
            return Ok(SplitDecision::Split {
                evaluated_midpoint: None,
            });
        }
        let mid = self.point_at((a.0 + b.0) / 2.0, work)?;
        if point_seg_dist(mid, a.1, b.1) > self.ctx.tol {
            return Ok(SplitDecision::Split {
                evaluated_midpoint: Some(mid),
            });
        }
        for face_use in &self.face_uses {
            let ua = face_use.uv_at(a.0, a.1, work)?;
            let ub = unwrap_near(
                face_use.uv_at(b.0, b.1, work)?,
                ua,
                surface_periodicity(face_use.store, face_use.surface_id, work)?,
            );
            let um = (ua + ub) / 2.0;
            let q = eval_surface_point(face_use.store, face_use.surface_id, um, work)?;
            if point_seg_dist(q, a.1, b.1) > self.ctx.tol {
                return Ok(SplitDecision::Split {
                    evaluated_midpoint: Some(mid),
                });
            }
        }
        Ok(SplitDecision::Keep)
    }

    /// Append the interior refinement points of `(a, b)` (exclusive).
    fn refine(
        &self,
        a: (f64, Point3),
        b: (f64, Point3),
        depth: usize,
        out: &mut Vec<(f64, Point3)>,
        work: &mut BodyTessellationWork<'_, '_, '_>,
    ) -> Result<()> {
        let evaluated_midpoint = match self.split_decision(a, b, work)? {
            SplitDecision::Keep => return Ok(()),
            SplitDecision::Split { evaluated_midpoint } => evaluated_midpoint,
        };
        let next_depth = next_refinement_depth(
            depth,
            BODY_TESSELLATION_EDGE_DEPTH,
            BODY_TESSELLATION_EDGE_DEPTH_LIMIT_REACHED,
            "whole-body exact-edge refinement depth limit reached",
            work,
        )?;
        work.charge_split(
            BODY_TESSELLATION_EDGE_SPLITS,
            BODY_TESSELLATION_EDGE_SPLIT_LIMIT_REACHED,
            "whole-body exact-edge refinement split limit reached",
        )?;
        let tm = (a.0 + b.0) / 2.0;
        let m = (
            tm,
            match evaluated_midpoint {
                Some(midpoint) => midpoint,
                None => self.point_at(tm, work)?,
            },
        );
        self.refine(a, m, next_depth, out, work)?;
        out.push(m);
        self.refine(m, b, next_depth, out, work)
    }
}

/// Refine a straight UV segment on `s` until kgeom's boundary criterion
/// holds with margin; appends interior `(uv, position)` points, exclusive.
fn refine_uv_seg(
    s: &dyn Surface,
    a: (Vec2, Point3),
    b: (Vec2, Point3),
    ctx: Ctx,
    depth: usize,
    out: &mut Vec<(Vec2, Point3)>,
    work: &mut BodyTessellationWork<'_, '_, '_>,
) -> Result<()> {
    let mid_uv = (a.0 + b.0) / 2.0;
    let evaluated_midpoint = if a.1.dist(b.1) > ctx.max_len {
        None
    } else {
        let midpoint = s.eval([mid_uv.x, mid_uv.y]);
        if point_seg_dist(midpoint, a.1, b.1) <= ctx.tol {
            return Ok(());
        }
        Some(midpoint)
    };
    let next_depth = next_refinement_depth(
        depth,
        BODY_TESSELLATION_ISO_ARC_DEPTH,
        BODY_TESSELLATION_ISO_ARC_DEPTH_LIMIT_REACHED,
        "whole-body iso-arc refinement depth limit reached",
        work,
    )?;
    work.charge_split(
        BODY_TESSELLATION_ISO_ARC_SPLITS,
        BODY_TESSELLATION_ISO_ARC_SPLIT_LIMIT_REACHED,
        "whole-body iso-arc refinement split limit reached",
    )?;
    let mid_p = match evaluated_midpoint {
        Some(midpoint) => midpoint,
        None => s.eval([mid_uv.x, mid_uv.y]),
    };
    let m = (mid_uv, mid_p);
    refine_uv_seg(s, a, m, ctx, next_depth, out, work)?;
    out.push(m);
    refine_uv_seg(s, m, b, ctx, next_depth, out, work)
}

/// An iso/seam arc: UV points with their global vertex ids, endpoints
/// included.
type Arc = Vec<(Vec2, u32)>;

/// Build an arc between two existing global vertices by refining the
/// straight UV segment; interior points become fresh global vertices.
fn iso_arc(
    s: &dyn Surface,
    a: (Vec2, u32),
    b: (Vec2, u32),
    acc: &mut MeshAcc,
    ctx: Ctx,
    work: &mut BodyTessellationWork<'_, '_, '_>,
) -> Result<Arc> {
    let mut interior = Vec::new();
    refine_uv_seg(
        s,
        (a.0, acc.pos(a.1)),
        (b.0, acc.pos(b.1)),
        ctx,
        0,
        &mut interior,
        work,
    )?;
    let mut arc = Vec::with_capacity(interior.len() + 2);
    arc.push(a);
    for (uv, p) in interior {
        let gid = acc.push(p, work)?;
        arc.push((uv, gid));
    }
    arc.push(b);
    Ok(arc)
}

/// One trim loop expressed in the face surface's UV space: global vertex
/// ids, parallel unwrapped UV points, the unwrapped image of the first
/// point continued past the last (loop closure), and the periodic winding
/// counts of the traversal.
struct UvChain {
    ids: Vec<u32>,
    uvs: Vec<Vec2>,
    close_uv: Vec2,
    winding: [i64; 2],
}

/// One retained point of a shared edge polyline. The edge parameter is kept
/// even when a closed edge repeats the first global vertex at its end: the
/// two parameters can map to different branches of a periodic pcurve.
#[derive(Debug, Clone, Copy)]
struct EdgeSample {
    parameter: f64,
    vertex: u32,
}

/// One edge's frozen shared polyline in increasing edge parameter.
struct EdgeLine {
    edge: EdgeId,
    samples: Vec<EdgeSample>,
}

/// The discretized edges of a body, parallel to [`Store::edges_of_body`].
type EdgeLines = Vec<EdgeLine>;

fn find_eline(elines: &EdgeLines, edge: EdgeId) -> Result<&EdgeLine> {
    elines
        .iter()
        .find(|line| line.edge == edge)
        .ok_or_else(|| Error::StaleHandle.into())
}

/// Discretize one edge into global mesh vertices. Returns the vertex-index
/// polyline; closed polylines (ring edges and closed one-vertex edges)
/// repeat the first index last.
fn discretize_edge(
    store: &Store,
    edge: EdgeId,
    vgids: &[(VertexId, u32)],
    acc: &mut MeshAcc,
    ctx: Ctx,
    work: &mut BodyTessellationWork<'_, '_, '_>,
) -> Result<EdgeLine> {
    let e: &Edge = store.get(edge)?;
    let curve = match e.curve {
        Some(curve_id) => Some(store.get(curve_id)?.as_curve()),
        None if e.tolerance.is_some() => None,
        None => {
            return Err(Error::InvalidGeometry {
                reason: "edge has neither curve geometry nor a tolerance",
            }
            .into());
        }
    };

    // Parameter interval: explicit bounds, or one full period for a ring.
    let (t0, t1) = match e.bounds {
        Some((a, b)) => {
            if !(a.is_finite() && b.is_finite() && a < b) {
                return Err(Error::InvalidGeometry {
                    reason: "edge bounds are not a finite increasing interval",
                }
                .into());
            }
            (a, b)
        }
        None => {
            let c = curve.ok_or(Error::InvalidGeometry {
                reason: "curve-less tolerant ring edges are unsupported",
            })?;
            let p = c.periodicity().ok_or(Error::InvalidGeometry {
                reason: "ring edge on a non-periodic curve",
            })?;
            let lo = c.param_range().lo;
            (lo, lo + p)
        }
    };

    // Endpoint vertices anchor the polyline to shared global ids; a ring
    // edge gets a fresh anchor at its parameter start.
    let vgid = |v: VertexId| -> Result<u32> {
        vgids
            .iter()
            .find(|(id, _)| *id == v)
            .map(|&(_, g)| g)
            .ok_or_else(|| Error::StaleHandle.into())
    };
    let (g_start, g_end, closed) = match e.vertices {
        [Some(v0), Some(v1)] => (vgid(v0)?, vgid(v1)?, v0 == v1),
        [None, None] => {
            let c = curve.ok_or(Error::InvalidGeometry {
                reason: "curve-less tolerant ring edges are unsupported",
            })?;
            let g = acc.push(c.eval(t0), work)?;
            (g, g, true)
        }
        _ => {
            return Err(Error::InvalidGeometry {
                reason: "edge has exactly one vertex",
            }
            .into());
        }
    };

    // Adjacent face uses in deterministic fin order. They are deliberately
    // not deduplicated by surface: two fins on the same periodic surface can
    // carry different pcurve branches.
    let mut face_uses = Vec::with_capacity(e.fins.len());
    for &fin_id in &e.fins {
        let fin = store.get(fin_id)?;
        let lp = fin.parent;
        let face = store.get(store.get(lp)?.face)?;
        let pcurve = match fin.pcurve {
            Some(use_) => Some((store.get(use_.curve())?, use_)),
            None if curve.is_some() => None,
            None => {
                return Err(Error::InvalidGeometry {
                    reason: "curve-less tolerant edge fin has no pcurve",
                }
                .into());
            }
        };
        face_uses.push(FaceUse {
            store,
            surface_id: face.surface,
            surface: store.get(face.surface)?,
            pcurve,
        });
    }
    let refine = CurveRefine {
        curve,
        face_uses,
        ctx,
    };

    // Seed: closed polylines start from quarter points (their full-span
    // chord is degenerate); open ones from the single endpoint chord.
    let mut seed: Vec<(f64, Point3)> = Vec::new();
    seed.push((t0, acc.pos(g_start)));
    if closed {
        for k in 1..4 {
            let t = t0 + (t1 - t0) * f64::from(k) / 4.0;
            seed.push((t, refine.point_at(t, work)?));
        }
    }
    seed.push((t1, acc.pos(g_end)));

    let mut samples = vec![EdgeSample {
        parameter: t0,
        vertex: g_start,
    }];
    for w in seed.windows(2) {
        let mut interior = Vec::new();
        refine.refine(w[0], w[1], 0, &mut interior, work)?;
        for (parameter, point) in interior {
            samples.push(EdgeSample {
                parameter,
                vertex: acc.push(point, work)?,
            });
        }
        // Segment end: a seed interior point gets a fresh vertex; the
        // final endpoint reuses its anchor id.
        if w[1].0 < t1 {
            samples.push(EdgeSample {
                parameter: w[1].0,
                vertex: acc.push(w[1].1, work)?,
            });
        }
    }
    samples.push(EdgeSample {
        parameter: t1,
        vertex: g_end,
    });
    Ok(EdgeLine { edge, samples })
}

#[derive(Clone, Copy)]
struct RawUvSample {
    vertex: u32,
    uv: Vec2,
}

struct RawUvChain {
    samples: Vec<RawUvSample>,
    close: RawUvSample,
    declared_winding: Option<[i64; 2]>,
}

#[derive(Clone, Copy)]
struct FaceChart<'a> {
    surface_id: SurfaceId,
    surface: &'a SurfaceGeom,
}

fn fin_sample_uv(
    store: &Store,
    surface_id: SurfaceId,
    sg: &SurfaceGeom,
    acc: &MeshAcc,
    fin: &crate::entity::Fin,
    sample: EdgeSample,
    work: &mut BodyTessellationWork<'_, '_, '_>,
) -> Result<Vec2> {
    match fin.pcurve {
        Some(use_) => {
            let curve = store.get(use_.curve())?.as_curve();
            Ok(use_.evaluate_uv(
                curve,
                sample.parameter,
                surface_periodicity(store, surface_id, work)?,
            )?)
        }
        None => invert_uv(sg, acc.pos(sample.vertex), work),
    }
}

/// Assemble the oriented `(global vertex, raw UV)` chain of one loop by
/// concatenating its fins' edge samples. Each fin contributes all but its
/// final endpoint; that endpoint is retained separately so a periodic
/// pcurve can close at a different UV branch over the same global vertex.
/// When `reverse` is set the whole traversal is flipped.
fn loop_chain(
    store: &Store,
    elines: &EdgeLines,
    chart: FaceChart<'_>,
    acc: &MeshAcc,
    lp: crate::entity::LoopId,
    reverse: bool,
    work: &mut BodyTessellationWork<'_, '_, '_>,
) -> Result<RawUvChain> {
    let fins = &store.get(lp)?.fins;
    if fins.is_empty() {
        return Err(Error::InvalidGeometry {
            reason: "loop has no fins",
        }
        .into());
    }
    let mut chain = Vec::new();
    let mut close = None;
    let declared_winding = if fins.len() == 1 {
        let fin = store.get(fins[0])?;
        fin.pcurve
            .and_then(|use_| use_.closure_winding())
            .map(|winding| {
                let sign = if fin.sense.is_forward() != reverse {
                    1_i64
                } else {
                    -1_i64
                };
                [i64::from(winding[0]) * sign, i64::from(winding[1]) * sign]
            })
    } else {
        None
    };
    let ordered: Vec<_> = if reverse {
        fins.iter().rev().copied().collect()
    } else {
        fins.to_vec()
    };
    for fin_id in ordered {
        let fin = store.get(fin_id)?;
        let line = find_eline(elines, fin.edge)?;
        if line.samples.len() < 2 {
            return Err(Error::InvalidGeometry {
                reason: "edge polyline has fewer than two parameter samples",
            }
            .into());
        }
        let forward = fin.sense.is_forward() != reverse;
        if forward {
            for &sample in &line.samples[..line.samples.len() - 1] {
                chain.push(RawUvSample {
                    vertex: sample.vertex,
                    uv: fin_sample_uv(
                        store,
                        chart.surface_id,
                        chart.surface,
                        acc,
                        fin,
                        sample,
                        work,
                    )?,
                });
            }
            let sample = *line.samples.last().expect("at least two samples");
            close = Some(RawUvSample {
                vertex: sample.vertex,
                uv: fin_sample_uv(
                    store,
                    chart.surface_id,
                    chart.surface,
                    acc,
                    fin,
                    sample,
                    work,
                )?,
            });
        } else {
            for &sample in line.samples.iter().rev().take(line.samples.len() - 1) {
                chain.push(RawUvSample {
                    vertex: sample.vertex,
                    uv: fin_sample_uv(
                        store,
                        chart.surface_id,
                        chart.surface,
                        acc,
                        fin,
                        sample,
                        work,
                    )?,
                });
            }
            let sample = line.samples[0];
            close = Some(RawUvSample {
                vertex: sample.vertex,
                uv: fin_sample_uv(
                    store,
                    chart.surface_id,
                    chart.surface,
                    acc,
                    fin,
                    sample,
                    work,
                )?,
            });
        }
    }
    let close = close.expect("non-empty fin list");
    if chain
        .first()
        .is_none_or(|first| first.vertex != close.vertex)
    {
        return Err(Error::InvalidGeometry {
            reason: "loop edge polyline does not close by shared vertex identity",
        }
        .into());
    }
    Ok(RawUvChain {
        samples: chain,
        close,
        declared_winding,
    })
}

/// Unwrap a raw per-fin pcurve chain with periodic continuity and measure
/// its winding. The explicit closing sample is essential for seam loops.
fn chain_uv(per: [Option<f64>; 2], raw: RawUvChain) -> Result<UvChain> {
    let mut ids = Vec::with_capacity(raw.samples.len());
    let mut uvs: Vec<Vec2> = Vec::with_capacity(raw.samples.len());
    for sample in raw.samples {
        let uv = match uvs.last() {
            Some(&prev) => unwrap_near(sample.uv, prev, per),
            None => sample.uv,
        };
        ids.push(sample.vertex);
        uvs.push(uv);
    }
    let close_uv = unwrap_near(raw.close.uv, *uvs.last().expect("non-empty chain"), per);
    let wind = |d: f64, p: Option<f64>| p.map_or(0, |p| (d / p).round() as i64);
    let winding = [
        wind(close_uv.x - uvs[0].x, per[0]),
        wind(close_uv.y - uvs[0].y, per[1]),
    ];
    if raw
        .declared_winding
        .is_some_and(|declared| declared != winding)
    {
        return Err(Error::InvalidGeometry {
            reason: "declared pcurve closure winding disagrees with tessellation chain",
        }
        .into());
    }
    Ok(UvChain {
        ids,
        uvs,
        close_uv,
        winding,
    })
}

/// Move an unwrapped loop onto the periodic branch selected by the face's
/// declared work box. Winding is unchanged; this only removes arbitrary
/// whole-period offsets introduced by inversion at seams.
fn anchor_chain_to_domain(chain: &mut UvChain, domain: FaceDomain, periods: [Option<f64>; 2]) {
    let centers = [
        domain.u.lo + domain.u.width() / 2.0,
        domain.v.lo + domain.v.width() / 2.0,
    ];
    for direction in 0..2 {
        let Some(period) = periods[direction] else {
            continue;
        };
        let coordinate = |uv: Vec2| if direction == 0 { uv.x } else { uv.y };
        let mean = chain.uvs.iter().copied().map(coordinate).sum::<f64>() / chain.uvs.len() as f64;
        let shift = period * ((centers[direction] - mean) / period).round();
        for uv in &mut chain.uvs {
            if direction == 0 {
                uv.x += shift;
            } else {
                uv.y += shift;
            }
        }
        if direction == 0 {
            chain.close_uv.x += shift;
        } else {
            chain.close_uv.y += shift;
        }
    }
}

/// Run kgeom's face tessellator over prepared UV loops and splice the
/// result into the body mesh: boundary vertices map to the pre-assigned
/// global ids, interior vertices become fresh ones, and triangles are
/// flipped when the face sense is reversed. Triangles that degenerate
/// under welding (sphere pole collapse) are dropped.
fn face_tessellation_limit_diagnostic(stage: StageId) -> Option<(DiagnosticCode, &'static str)> {
    match stage {
        FACE_TESSELLATION_BOUNDARY_DEPTH => Some((
            FACE_TESSELLATION_BOUNDARY_DEPTH_LIMIT,
            "face boundary refinement depth limit reached",
        )),
        FACE_TESSELLATION_REFINEMENT_PASSES => Some((
            FACE_TESSELLATION_REFINEMENT_PASS_LIMIT,
            "face interior refinement pass limit reached",
        )),
        FACE_TESSELLATION_BOUNDARY_SPLITS => Some((
            FACE_TESSELLATION_BOUNDARY_SPLIT_LIMIT,
            "face boundary split limit reached",
        )),
        FACE_TESSELLATION_MESH_VERTICES => Some((
            FACE_TESSELLATION_MESH_VERTEX_LIMIT,
            "face mesh vertex limit reached",
        )),
        FACE_TESSELLATION_MESH_TRIANGLES => Some((
            FACE_TESSELLATION_MESH_TRIANGLE_LIMIT,
            "face mesh triangle limit reached",
        )),
        kcore::operation::TOTAL_WORK_STAGE => Some((
            BODY_TESSELLATION_TOTAL_WORK_LIMIT_REACHED,
            "face tessellation total work limit reached",
        )),
        _ => None,
    }
}

fn run_kgeom(
    s: &dyn Surface,
    loops_pts: Vec<Vec<Vec2>>,
    loops_ids: &[Vec<u32>],
    flip: bool,
    acc: &mut MeshAcc,
    opts: &TessOptions,
    work: &mut BodyTessellationWork<'_, '_, '_>,
) -> Result<Vec<[u32; 3]>> {
    let mut trim_loops = Vec::with_capacity(loops_pts.len());
    for pts in loops_pts {
        trim_loops.push(TrimLoop::new(pts)?);
    }
    let face = TrimmedSurface::new(s, trim_loops)?;
    let mut patch = work
        .scope
        .ledger_mut()
        .sequential(FaceTessellationBudgetProfile::v1_defaults())
        .map_err(Error::from)?;
    let fm = tessellate_in_sequential_ledger(&face, opts, &mut patch);
    drop(patch);
    let fm = match fm {
        Ok(mesh) => mesh,
        Err(error) => {
            let snapshot = match &error {
                Error::OperationPolicy {
                    source: OperationPolicyError::LimitReached(snapshot),
                }
                | Error::ResourceLimit { snapshot } => Some(*snapshot),
                _ => None,
            };
            if let Some(snapshot) = snapshot {
                let Some((code, message)) = face_tessellation_limit_diagnostic(snapshot.stage)
                else {
                    return Err(error.into());
                };
                work.scope.diagnose(
                    snapshot.stage,
                    code,
                    DiagnosticKind::LimitReached(snapshot),
                    message,
                );
                return Err(Error::ResourceLimit { snapshot }.into());
            }
            return Err(error.into());
        }
    };

    if fm.boundary.len() != loops_ids.len() {
        return Err(Error::InvalidGeometry {
            reason: "internal: face boundary loop count mismatch",
        }
        .into());
    }
    let mut l2g: Vec<Option<u32>> = vec![None; fm.positions.len()];
    for (bl, ids) in fm.boundary.iter().zip(loops_ids) {
        if bl.len() != ids.len() {
            // kgeom inserted a boundary vertex despite the margin rule:
            // that would be a cross-face crack, so fail loudly.
            return Err(Error::InvalidGeometry {
                reason: "internal: boundary refinement mismatch (potential crack)",
            }
            .into());
        }
        for (&li, &gid) in bl.iter().zip(ids) {
            l2g[li as usize] = Some(gid);
        }
    }
    let l2g: Vec<u32> = l2g
        .into_iter()
        .enumerate()
        .map(|(li, g)| match g {
            Some(gid) => Ok(gid),
            None => acc.push(fm.positions[li], work),
        })
        .collect::<Result<_>>()?;

    let mut out = Vec::with_capacity(fm.triangles.len());
    for t in &fm.triangles {
        let mut m = t.map(|i| l2g[i as usize]);
        if flip {
            m.swap(1, 2);
        }
        if m[0] != m[1] && m[1] != m[2] && m[2] != m[0] {
            out.push(m);
        }
    }
    Ok(out)
}

/// Ordinary face: every loop closes in UV. Outer loop first (positive
/// area), holes anchored onto the outer loop's periodic branch.
fn face_case_a(
    s: &dyn Surface,
    chains: Vec<UvChain>,
    flip: bool,
    acc: &mut MeshAcc,
    opts: &TessOptions,
    work: &mut BodyTessellationWork<'_, '_, '_>,
) -> Result<Vec<[u32; 3]>> {
    let per = s.periodicity();
    let area = |pts: &[Vec2]| -> f64 {
        let n = pts.len();
        (0..n).map(|i| pts[i].cross(pts[(i + 1) % n])).sum::<f64>() / 2.0
    };
    let outer_idx = {
        let mut positive: Vec<usize> = (0..chains.len())
            .filter(|&i| area(&chains[i].uvs) > 0.0)
            .collect();
        if positive.len() != 1 {
            return Err(Error::InvalidGeometry {
                reason: "face must have exactly one counterclockwise (outer) loop",
            }
            .into());
        }
        positive.pop().expect("one outer loop")
    };
    let mean_u = |c: &UvChain| c.uvs.iter().map(|p| p.x).sum::<f64>() / c.uvs.len() as f64;
    let mean_v = |c: &UvChain| c.uvs.iter().map(|p| p.y).sum::<f64>() / c.uvs.len() as f64;
    let (ou, ov) = (mean_u(&chains[outer_idx]), mean_v(&chains[outer_idx]));

    let mut loops_pts: Vec<Vec<Vec2>> = Vec::with_capacity(chains.len());
    let mut loops_ids: Vec<Vec<u32>> = Vec::with_capacity(chains.len());
    let order = core::iter::once(outer_idx).chain((0..chains.len()).filter(|&i| i != outer_idx));
    for i in order {
        let c = &chains[i];
        // Anchor holes onto the outer loop's periodic branch.
        let mut shift = Vec2::new(0.0, 0.0);
        if i != outer_idx {
            if let Some(p) = per[0] {
                shift.x = p * ((ou - mean_u(c)) / p).round();
            }
            if let Some(p) = per[1] {
                shift.y = p * ((ov - mean_v(c)) / p).round();
            }
        }
        loops_pts.push(c.uvs.iter().map(|&uv| uv + shift).collect());
        loops_ids.push(c.ids.clone());
    }
    run_kgeom(s, loops_pts, &loops_ids, flip, acc, opts, work)
}

/// Periodic side face (cylinder/cone-like): exactly one loop winds `+1`
/// (bottom) and one winds `-1` (top) around the `u` period; the domain is
/// seam-cut into one period-wide region whose left/right seam columns
/// share global vertices.
fn face_case_b(
    sg: &SurfaceGeom,
    chains: Vec<UvChain>,
    flip: bool,
    acc: &mut MeshAcc,
    opts: &TessOptions,
    ctx: Ctx,
    work: &mut BodyTessellationWork<'_, '_, '_>,
) -> Result<Vec<[u32; 3]>> {
    let s = require_leaf_surface(sg)?;
    let pu = s.periodicity()[0].ok_or(Error::InvalidGeometry {
        reason: "winding loop on a non-periodic surface direction",
    })?;
    let (mut bottom, mut top) = (None, None);
    let mut holes: Vec<UvChain> = Vec::new();
    for c in chains {
        match c.winding {
            [1, 0] if bottom.is_none() => bottom = Some(c),
            [-1, 0] if top.is_none() => top = Some(c),
            [0, 0] => holes.push(c),
            _ => {
                return Err(Error::InvalidGeometry {
                    reason: "unsupported loop winding configuration on periodic face",
                }
                .into());
            }
        }
    }
    let (bottom, top) = match (bottom, top) {
        (Some(b), Some(t)) => (b, t),
        // A single winding loop bounds a polar cap: the missing second
        // boundary is the pole contained in the face.
        (Some(c), None) | (None, Some(c)) => {
            return face_case_cap(sg, c, holes, acc, FaceRun { flip, opts, ctx }, work);
        }
        (None, None) => {
            return Err(Error::InvalidGeometry {
                reason: "seam-cut face needs one +1 and one -1 winding loop",
            }
            .into());
        }
    };
    let mean_v = |c: &UvChain| c.uvs.iter().map(|p| p.y).sum::<f64>() / c.uvs.len() as f64;
    if mean_v(&top) <= mean_v(&bottom) {
        return Err(Error::InvalidGeometry {
            reason: "seam-cut face has its winding loops on the wrong sides",
        }
        .into());
    }

    // Anchor the top chain so its end (low-u side) sits on the bottom
    // chain's branch; the seams connect chain endpoints.
    let shift = pu * ((bottom.uvs[0].x - top.close_uv.x) / pu).round();
    let t_first = top.uvs[0] + Vec2::new(shift, 0.0);

    // Right seam: bottom end → top start; the left seam reuses the same
    // global vertices one period lower.
    let seam = iso_arc(
        s,
        (bottom.close_uv, bottom.ids[0]),
        (t_first, top.ids[0]),
        acc,
        ctx,
        work,
    )?;

    let mut pts: Vec<Vec2> = Vec::new();
    let mut ids: Vec<u32> = Vec::new();
    // Bottom chain, including its closing point (the right seam's base).
    pts.extend_from_slice(&bottom.uvs);
    pts.push(bottom.close_uv);
    ids.extend_from_slice(&bottom.ids);
    ids.push(bottom.ids[0]);
    // Right seam interior.
    for &(uv, gid) in &seam[1..seam.len() - 1] {
        pts.push(uv);
        ids.push(gid);
    }
    // Top chain (traverses -u), including its closing point (left seam top).
    for (uv, gid) in top.uvs.iter().zip(&top.ids) {
        pts.push(*uv + Vec2::new(shift, 0.0));
        ids.push(*gid);
    }
    pts.push(top.close_uv + Vec2::new(shift, 0.0));
    ids.push(top.ids[0]);
    // Left seam interior, descending, same global vertices as the right.
    for &(uv, gid) in seam[1..seam.len() - 1].iter().rev() {
        pts.push(uv - Vec2::new(pu, 0.0));
        ids.push(gid);
    }

    let mut loops_pts = vec![pts];
    let mut loops_ids = vec![ids];
    let bu = bottom.uvs.iter().map(|p| p.x).sum::<f64>() / bottom.uvs.len() as f64;
    for h in holes {
        let hu = h.uvs.iter().map(|p| p.x).sum::<f64>() / h.uvs.len() as f64;
        let hs = pu * ((bu - hu) / pu).round();
        loops_pts.push(h.uvs.iter().map(|&uv| uv + Vec2::new(hs, 0.0)).collect());
        loops_ids.push(h.ids);
    }
    run_kgeom(s, loops_pts, &loops_ids, flip, acc, opts, work)
}

/// The four boundary arcs of one rectangular-ish patch, each stored in
/// its own forward direction (`bottom`: A→B, `right`: B→C, `top`: D→C,
/// `left`: A→D).
struct PatchArcs {
    bottom: Arc,
    right: Arc,
    top: Arc,
    left: Arc,
}

#[derive(Clone, Copy)]
struct FaceRun<'a> {
    flip: bool,
    opts: &'a TessOptions,
    ctx: Ctx,
}

/// Spherical polar cap: a face bounded by a single loop that winds the
/// `u` period once, with exactly one pole in its interior (the shape XT
/// cut spheres produce). The loop's material side picks the pole — chains
/// are already sense-normalized, so a `+1` winding keeps `+v` on its left
/// (north pole) and a `-1` winding the south pole. The cap is split into
/// two patches at an existing chain sample near the half period —
/// mirroring the closed-face splitting rule, and legal precisely because
/// the split seam runs from a *frozen chain vertex* to the pole — so no
/// patch domain welds two boundary points to one vertex except the
/// intended pole rows. The boundary chain need not be an iso-line of the
/// sphere (oblique plane cuts are fine); it must advance monotonically in
/// `u` enough that a sample near the half period splits it, else a typed
/// error is returned.
fn face_case_cap(
    sg: &SurfaceGeom,
    chain: UvChain,
    holes: Vec<UvChain>,
    acc: &mut MeshAcc,
    run: FaceRun<'_>,
    work: &mut BodyTessellationWork<'_, '_, '_>,
) -> Result<Vec<[u32; 3]>> {
    let SurfaceGeom::Sphere(sp) = sg else {
        // Cylinders and tori have no point that can close a single
        // winding loop; cone apex caps are deferred until full cones land.
        return Err(Error::InvalidGeometry {
            reason: "single-winding loop is only supported as a spherical polar cap",
        }
        .into());
    };
    let s = require_leaf_surface(sg)?;
    let tau = core::f64::consts::TAU;
    let half = core::f64::consts::FRAC_PI_2;
    let w = chain.winding[0];

    // The chain including its closing point, one period around.
    let mut cuvs = chain.uvs.clone();
    cuvs.push(chain.close_uv);
    let mut cids = chain.ids.clone();
    cids.push(chain.ids[0]);
    let n = cuvs.len() - 1;
    if n < 2 {
        return Err(Error::InvalidGeometry {
            reason: "cap boundary loop has too few samples to seam-split",
        }
        .into());
    }
    let pole_v = if w > 0 { half } else { -half };
    if cuvs
        .iter()
        .any(|uv| if w > 0 { uv.y >= half } else { uv.y <= -half })
    {
        return Err(Error::InvalidGeometry {
            reason: "cap boundary loop touches the pole",
        }
        .into());
    }

    let g_pole = acc.push(s.eval([cuvs[0].x, pole_v]), work)?;
    let pole_at = |u: f64| (Vec2::new(u, pole_v), g_pole);

    // Split at the existing chain sample nearest half a period around.
    let target = cuvs[0].x + w as f64 * (tau / 2.0);
    let k = (1..n)
        .min_by(|&a, &b| {
            let (da, db) = ((cuvs[a].x - target).abs(), (cuvs[b].x - target).abs());
            da.partial_cmp(&db).expect("finite uv").then(a.cmp(&b))
        })
        .expect("chain has interior samples");
    let ordered = |a: f64, b: f64, c: f64| a < b && b < c;
    let split_ok = if w > 0 {
        ordered(cuvs[0].x, cuvs[k].x, cuvs[n].x)
    } else {
        ordered(cuvs[n].x, cuvs[k].x, cuvs[0].x)
    };
    if !split_ok {
        return Err(Error::InvalidGeometry {
            reason: "cap boundary loop cannot be seam-split at an existing sample",
        }
        .into());
    }

    // Pole rows: uniform samples between two seam longitudes, all welded
    // to the single pole vertex (density from the equator sagitta, like
    // the closed-sphere case).
    let r = sp.radius();
    let mut theta = (8.0 * run.ctx.tol / r).sqrt().min(half);
    if run.ctx.max_len.is_finite() {
        theta = theta.min(run.ctx.max_len / r);
    }
    let row = |ua: f64, ub: f64| -> Arc {
        let m = (((ub - ua).abs() / theta).ceil() as usize).max(2);
        (0..=m)
            .map(|i| {
                (
                    Vec2::new(ua + (ub - ua) * i as f64 / m as f64, pole_v),
                    g_pole,
                )
            })
            .collect()
    };
    let seg = |lo: usize, hi: usize| -> Arc { (lo..=hi).map(|i| (cuvs[i], cids[i])).collect() };
    let rev = |arc: Arc| -> Arc { arc.into_iter().rev().collect() };

    let patches: [PatchArcs; 2] = if w > 0 {
        // Chain below (travels +u), pole row above.
        let m0 = iso_arc(
            s,
            (cuvs[0], cids[0]),
            pole_at(cuvs[0].x),
            acc,
            run.ctx,
            work,
        )?;
        let mk = iso_arc(
            s,
            (cuvs[k], cids[k]),
            pole_at(cuvs[k].x),
            acc,
            run.ctx,
            work,
        )?;
        [
            PatchArcs {
                bottom: seg(0, k),
                right: mk.clone(),
                top: row(cuvs[0].x, cuvs[k].x),
                left: m0.clone(),
            },
            PatchArcs {
                bottom: seg(k, n),
                right: shift_arc(&m0, Vec2::new(tau, 0.0)),
                top: row(cuvs[k].x, cuvs[n].x),
                left: mk,
            },
        ]
    } else {
        // Pole row below, chain above (travels -u); top arcs are stored
        // ascending in u, i.e. the chain segments reversed.
        let m0 = iso_arc(
            s,
            pole_at(cuvs[0].x),
            (cuvs[0], cids[0]),
            acc,
            run.ctx,
            work,
        )?;
        let mk = iso_arc(
            s,
            pole_at(cuvs[k].x),
            (cuvs[k], cids[k]),
            acc,
            run.ctx,
            work,
        )?;
        [
            PatchArcs {
                bottom: row(cuvs[k].x, cuvs[0].x),
                right: m0.clone(),
                top: rev(seg(0, k)),
                left: mk.clone(),
            },
            PatchArcs {
                bottom: row(cuvs[n].x, cuvs[k].x),
                right: mk,
                top: rev(seg(k, n)),
                left: shift_arc(&m0, Vec2::new(-tau, 0.0)),
            },
        ]
    };

    // Holes: anchor onto the chain's branch, then hand each to the patch
    // whose u-interval contains its mean.
    let chain_mean_u = cuvs.iter().map(|p| p.x).sum::<f64>() / cuvs.len() as f64;
    let u_split = cuvs[k].x;
    let mut patch_holes: [Vec<UvChain>; 2] = [Vec::new(), Vec::new()];
    for mut h in holes {
        let hu = h.uvs.iter().map(|p| p.x).sum::<f64>() / h.uvs.len() as f64;
        let hs = tau * ((chain_mean_u - hu) / tau).round();
        for uv in &mut h.uvs {
            uv.x += hs;
        }
        let hu = hu + hs;
        let first = if w > 0 { hu <= u_split } else { hu >= u_split };
        patch_holes[if first { 0 } else { 1 }].push(h);
    }

    let mut tris = Vec::new();
    for (patch, holes) in patches.iter().zip(patch_holes) {
        let (pts, ids) = patch_polygon(&patch.bottom, &patch.right, &patch.top, &patch.left);
        let mut loops_pts = vec![pts];
        let mut loops_ids = vec![ids];
        for h in holes {
            loops_pts.push(h.uvs);
            loops_ids.push(h.ids);
        }
        tris.extend(run_kgeom(
            s, loops_pts, &loops_ids, run.flip, acc, run.opts, work,
        )?);
    }
    Ok(tris)
}

/// Spherical face whose one trim loop passes through both parameter poles.
///
/// Longitude is undefined at a pole, so the ordinary unwrapped loop has a
/// spurious winding and diagonal UV segments at its pole samples. The loop
/// nevertheless has two well-defined pole-to-pole sides. This routine uses
/// those frozen sides as the left/right boundaries of one patch and inserts
/// collapsed pole rows between them. Every row vertex maps to the existing
/// frozen edge pole vertex, so the only added UV boundary segments disappear
/// under identity welding and the original 3D edge remains exact.
#[allow(clippy::too_many_arguments)]
fn face_case_bipolar_sphere(
    sp: &kgeom::surface::Sphere,
    s: &dyn Surface,
    chain: UvChain,
    holes: Vec<UvChain>,
    flip: bool,
    acc: &mut MeshAcc,
    opts: &TessOptions,
    ctx: Ctx,
    work: &mut BodyTessellationWork<'_, '_, '_>,
) -> Result<Vec<[u32; 3]>> {
    let half = core::f64::consts::FRAC_PI_2;
    let tau = core::f64::consts::TAU;
    let pole_eps = 64.0 * f64::EPSILON;
    let at_north = |uv: Vec2| (uv.y - half).abs() <= pole_eps;
    let at_south = |uv: Vec2| (uv.y + half).abs() <= pole_eps;
    let north = chain.uvs.iter().position(|&uv| at_north(uv));
    let south = chain.uvs.iter().position(|&uv| at_south(uv));
    let (Some(north), Some(south)) = (north, south) else {
        return Err(Error::InvalidGeometry {
            reason: "bipolar sphere loop does not contain both poles",
        }
        .into());
    };

    // Follow the already sense-normalized loop cyclically. Samples after a
    // wrap receive the loop's measured period shift, preserving the branch
    // established by chain_uv.
    let cyclic_arc = |from: usize, to: usize| -> Arc {
        let n = chain.uvs.len();
        let period_shift = Vec2::new(chain.close_uv.x - chain.uvs[0].x, 0.0);
        let mut out = Vec::new();
        let mut i = from;
        let mut shift = Vec2::new(0.0, 0.0);
        loop {
            out.push((chain.uvs[i] + shift, chain.ids[i]));
            if i == to {
                break;
            }
            i += 1;
            if i == n {
                i = 0;
                shift = period_shift;
            }
        }
        out
    };
    let mut right = cyclic_arc(south, north); // south -> north
    let mut left_desc = cyclic_arc(north, south); // north -> south
    if right.len() < 3 || left_desc.len() < 3 {
        return Err(Error::InvalidGeometry {
            reason: "bipolar sphere boundary needs a non-pole sample on each side",
        }
        .into());
    }

    // Replace each singular endpoint longitude with the adjacent side's
    // limiting branch. This turns the two sides into faithful UV images of
    // the frozen edge polyline instead of diagonal shortcuts at the poles.
    right[0].0.x = right[1].0.x;
    let rlast = right.len() - 1;
    right[rlast].0.x = right[rlast - 1].0.x;
    left_desc[0].0.x = left_desc[1].0.x;
    let llast = left_desc.len() - 1;
    left_desc[llast].0.x = left_desc[llast - 1].0.x;
    right[0].0.y = -half;
    right[rlast].0.y = half;
    left_desc[0].0.y = half;
    left_desc[llast].0.y = -half;
    let mut left: Arc = left_desc.into_iter().rev().collect(); // south -> north

    // Put the right side on the first equivalent periodic branch strictly
    // to the right of the left side. The resulting width chooses the
    // material side encoded by the normalized loop traversal.
    let side_mean = |arc: &Arc| {
        arc[1..arc.len() - 1]
            .iter()
            .map(|(uv, _)| uv.x)
            .sum::<f64>()
            / (arc.len() - 2) as f64
    };
    let lu = side_mean(&left);
    let ru = side_mean(&right);
    let width = (ru - lu).rem_euclid(tau);
    if width <= 64.0 * f64::EPSILON || width >= tau - 64.0 * f64::EPSILON {
        return Err(Error::InvalidGeometry {
            reason: "bipolar sphere boundary sides do not enclose a finite patch",
        }
        .into());
    }
    let shift = lu + width - ru;
    for (uv, _) in &mut right {
        uv.x += shift;
    }

    // Pole-row density follows the closed-sphere/cap rule. All samples on
    // a row intentionally share the pole's existing global vertex id.
    let r = sp.radius();
    let mut theta = (8.0 * ctx.tol / r).sqrt().min(half);
    if ctx.max_len.is_finite() {
        theta = theta.min(ctx.max_len / r);
    }
    let row = |ua: f64, ub: f64, v: f64, gid: u32| -> Arc {
        let m = (((ub - ua).abs() / theta).ceil() as usize).max(2);
        (0..=m)
            .map(|i| (Vec2::new(ua + (ub - ua) * i as f64 / m as f64, v), gid))
            .collect()
    };
    let bottom = row(left[0].0.x, right[0].0.x, -half, left[0].1);
    let top = row(
        left.last().expect("non-empty left arc").0.x,
        right.last().expect("non-empty right arc").0.x,
        half,
        left.last().expect("non-empty left arc").1,
    );
    let patch = PatchArcs {
        bottom,
        right,
        top,
        left: core::mem::take(&mut left),
    };
    let (outer_pts, outer_ids) =
        patch_polygon(&patch.bottom, &patch.right, &patch.top, &patch.left);

    let center_u = outer_pts.iter().map(|uv| uv.x).sum::<f64>() / outer_pts.len() as f64;
    let mut loops_pts = vec![outer_pts];
    let mut loops_ids = vec![outer_ids];
    for h in holes {
        let hu = h.uvs.iter().map(|uv| uv.x).sum::<f64>() / h.uvs.len() as f64;
        let hs = tau * ((center_u - hu) / tau).round();
        loops_pts.push(
            h.uvs
                .into_iter()
                .map(|uv| uv + Vec2::new(hs, 0.0))
                .collect(),
        );
        loops_ids.push(h.ids);
    }
    run_kgeom(s, loops_pts, &loops_ids, flip, acc, opts, work)
}

/// Rectangular patch boundary assembled from four arcs (each including
/// both endpoints): bottom, right, top, left in counterclockwise order.
/// Each side contributes all points except its last, so corners appear
/// exactly once.
fn patch_polygon(bottom: &Arc, right: &Arc, top: &Arc, left: &Arc) -> (Vec<Vec2>, Vec<u32>) {
    let mut pts = Vec::new();
    let mut ids = Vec::new();
    let mut side = |arc: &[(Vec2, u32)]| {
        for &(uv, gid) in &arc[..arc.len() - 1] {
            pts.push(uv);
            ids.push(gid);
        }
    };
    side(bottom);
    side(right);
    let top_rev: Vec<_> = top.iter().rev().copied().collect();
    side(&top_rev);
    let left_rev: Vec<_> = left.iter().rev().copied().collect();
    side(&left_rev);
    (pts, ids)
}

fn shift_arc(arc: &Arc, d: Vec2) -> Arc {
    arc.iter().map(|&(uv, gid)| (uv + d, gid)).collect()
}

/// Zero-loop face covering a closed surface. Sphere: two half-period
/// patches with pole rows collapsed to single vertices; torus: four
/// quarter patches. Splitting at half-periods guarantees no patch domain
/// contains two boundary points welded to the same vertex (other than the
/// intended pole rows).
fn face_case_c(
    sg: &SurfaceGeom,
    flip: bool,
    acc: &mut MeshAcc,
    opts: &TessOptions,
    ctx: Ctx,
    work: &mut BodyTessellationWork<'_, '_, '_>,
) -> Result<Vec<[u32; 3]>> {
    let pi = core::f64::consts::PI;
    let tau = core::f64::consts::TAU;
    let s = require_leaf_surface(sg)?;
    let mut tris = Vec::new();
    match sg {
        SurfaceGeom::Sphere(sp) => {
            let half = core::f64::consts::FRAC_PI_2;
            let g_s = acc.push(s.eval([0.0, -half]), work)?;
            let g_n = acc.push(s.eval([0.0, half]), work)?;
            // Meridian arcs at u = 0 and u = π; u = 2π reuses the first.
            let meridian =
                |u: f64, acc: &mut MeshAcc, work: &mut BodyTessellationWork<'_, '_, '_>| {
                    iso_arc(
                        s,
                        (Vec2::new(u, -half), g_s),
                        (Vec2::new(u, half), g_n),
                        acc,
                        ctx,
                        work,
                    )
                };
            let m0 = meridian(0.0, acc, work)?;
            let m1 = meridian(pi, acc, work)?;
            let m2 = shift_arc(&m0, Vec2::new(tau, 0.0));
            // Pole-row sampling density from the equator sagitta.
            let r = sp.radius();
            let mut theta = (8.0 * ctx.tol / r).sqrt().min(half);
            if ctx.max_len.is_finite() {
                theta = theta.min(ctx.max_len / r);
            }
            let n = ((pi / theta).ceil() as usize).max(2);
            for (patch, (left, right)) in [(0, (&m0, &m1)), (1, (&m1, &m2))] {
                let u_lo = pi * f64::from(patch);
                let row = |v: f64, g: u32| -> Arc {
                    (0..=n)
                        .map(|i| (Vec2::new(u_lo + pi * i as f64 / n as f64, v), g))
                        .collect()
                };
                let (pts, ids) = patch_polygon(&row(-half, g_s), right, &row(half, g_n), left);
                tris.extend(run_kgeom(s, vec![pts], &[ids], flip, acc, opts, work)?);
            }
        }
        SurfaceGeom::Torus(_) => {
            // Corner vertices at half-period grid points.
            let corner = |i: usize, j: usize| [pi * i as f64, pi * j as f64];
            let mut g = [[0u32; 2]; 2];
            for (i, gi) in g.iter_mut().enumerate() {
                for (j, gij) in gi.iter_mut().enumerate() {
                    let [u, v] = corner(i, j);
                    *gij = acc.push(s.eval([u, v]), work)?;
                }
            }
            let at = |i: usize, j: usize| {
                let [u, v] = corner(i, j);
                (Vec2::new(u, v), g[i % 2][j % 2])
            };
            // u-arcs au[i][j]: (u_i → u_{i+1}) at v_j; v-arcs av[j][i].
            let mut au = Vec::new();
            let mut av = Vec::new();
            for i in 0..2 {
                let mut row = Vec::new();
                for j in 0..2 {
                    row.push(iso_arc(s, at(i, j), at(i + 1, j), acc, ctx, work)?);
                }
                au.push(row);
            }
            for j in 0..2 {
                let mut col = Vec::new();
                for i in 0..2 {
                    col.push(iso_arc(s, at(i, j), at(i, j + 1), acc, ctx, work)?);
                }
                av.push(col);
            }
            for i in 0..2 {
                for j in 0..2 {
                    let bottom = &au[i][j];
                    let right = if i == 1 {
                        shift_arc(&av[j][0], Vec2::new(tau, 0.0))
                    } else {
                        av[j][i + 1].clone()
                    };
                    let top = if j == 1 {
                        shift_arc(&au[i][0], Vec2::new(0.0, tau))
                    } else {
                        au[i][j + 1].clone()
                    };
                    let left = &av[j][i];
                    let (pts, ids) = patch_polygon(bottom, &right, &top, left);
                    tris.extend(run_kgeom(s, vec![pts], &[ids], flip, acc, opts, work)?);
                }
            }
        }
        _ => {
            return Err(Error::InvalidGeometry {
                reason: "zero-loop face on a surface that is not closed",
            }
            .into());
        }
    }
    Ok(tris)
}

/// Tessellate one face into globally indexed triangles.
fn tess_face(
    store: &Store,
    elines: &EdgeLines,
    acc: &mut MeshAcc,
    face_id: FaceId,
    opts: &TessOptions,
    ctx: Ctx,
    work: &mut BodyTessellationWork<'_, '_, '_>,
) -> Result<Vec<[u32; 3]>> {
    let face = store.get(face_id)?;
    let sg = store.get(face.surface)?;
    let flip = face.sense == Sense::Reversed;

    if face.loops.is_empty() {
        return face_case_c(sg, flip, acc, opts, ctx, work);
    }
    let mut chains = Vec::with_capacity(face.loops.len());
    let periods = surface_periodicity(store, face.surface, work)?;
    for &lp in &face.loops {
        let raw = loop_chain(
            store,
            elines,
            FaceChart {
                surface_id: face.surface,
                surface: sg,
            },
            acc,
            lp,
            flip,
            work,
        )?;
        chains.push(chain_uv(periods, raw)?);
    }
    if let Some(domain) = face.domain {
        for chain in &mut chains {
            anchor_chain_to_domain(chain, domain, periods);
        }
    }
    if matches!(sg, SurfaceGeom::Offset(_)) {
        if chains.iter().any(|chain| chain.winding != [0, 0]) {
            return Err(TessellationError::Unsupported {
                capability: OFFSET_PERIODIC_WINDING,
            });
        }
        return face_case_planar_offset(store, face.surface, chains, flip, acc, opts, work);
    }
    // A meridional boundary can pass through both sphere poles while
    // acquiring either ±1 *or zero* winding from their arbitrary singular
    // longitudes. Classify it geometrically before the winding cases.
    if let SurfaceGeom::Sphere(sp) = sg {
        let half = core::f64::consts::FRAC_PI_2;
        let eps = 64.0 * f64::EPSILON;
        let touches_both = |chain: &UvChain| {
            chain.uvs.iter().any(|uv| (uv.y - half).abs() <= eps)
                && chain.uvs.iter().any(|uv| (uv.y + half).abs() <= eps)
        };
        let bipolar: Vec<_> = chains
            .iter()
            .enumerate()
            .filter_map(|(i, chain)| touches_both(chain).then_some(i))
            .collect();
        if bipolar.len() > 1 {
            return Err(Error::InvalidGeometry {
                reason: "sphere face has multiple loops passing through both poles",
            }
            .into());
        }
        if let Some(&outer) = bipolar.first() {
            let chain = chains.remove(outer);
            return face_case_bipolar_sphere(
                sp,
                require_leaf_surface(sg)?,
                chain,
                chains,
                flip,
                acc,
                opts,
                ctx,
                work,
            );
        }
    }
    if chains.iter().all(|c| c.winding == [0, 0]) {
        face_case_a(require_leaf_surface(sg)?, chains, flip, acc, opts, work)
    } else {
        face_case_b(sg, chains, flip, acc, opts, ctx, work)
    }
}

/// Tessellate a body into one watertight mesh (see module docs).
pub fn tessellate_body(store: &Store, body: BodyId, opts: &TessOptions) -> Result<BodyMesh> {
    let session = SessionPolicy::new(
        SessionPrecision::parasolid(),
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        BodyTessellationBudgetProfile::v1_defaults(),
        PolicyVersion::V1,
    );
    let context = OperationContext::new(&session, Tolerances::default())
        .expect("validated default tolerances satisfy v1 session precision");
    tessellate_body_with_context(store, body, opts, &context)
        .expect("built-in v1 body-tessellation policy is valid")
        .into_result()
        .map_err(legacy_body_tessellation_error)
}

/// Tessellate a body with deterministic whole-operation accounting.
///
/// Body-family defaults fill omitted stages below matching session entries
/// and explicit request overrides. The complete shared profile is validated
/// before options, topology, or geometry are inspected.
pub fn tessellate_body_with_context(
    store: &Store,
    body: BodyId,
    opts: &TessOptions,
    context: &OperationContext<'_>,
) -> core::result::Result<OperationOutcome<BodyMesh, TessellationError>, OperationPolicyError> {
    let context = context
        .clone()
        .with_family_budget_defaults(BodyTessellationBudgetProfile::v1_defaults());
    let effective = context.effective_budget();
    validate_body_tessellation_budget(|stage, resource, mode| {
        effective.require_limit(stage, resource, mode)
    })?;
    let mut scope = OperationScope::new(&context);
    let result = tessellate_body_in_scope(store, body, opts, &mut scope);
    Ok(scope.finish_typed(result))
}

/// Tessellate a body using an existing operation scope.
///
/// All graph evaluation, projection, edge/iso refinement, retained mesh
/// vertices, and face-patch refinement compose through the caller's ledger.
pub fn tessellate_body_in_scope(
    store: &Store,
    body: BodyId,
    opts: &TessOptions,
    scope: &mut OperationScope<'_, '_>,
) -> Result<BodyMesh> {
    validate_body_tessellation_budget(|stage, resource, mode| {
        scope.ledger().require_limit(stage, resource, mode)
    })
    .map_err(Error::from)?;
    if !opts.chord_tol.is_finite() || opts.chord_tol <= 0.0 {
        return Err(Error::InvalidTolerance {
            value: opts.chord_tol,
        }
        .into());
    }
    if let Some(l) = opts.max_edge_len
        && (!l.is_finite() || l <= 0.0)
    {
        return Err(Error::InvalidTolerance { value: l }.into());
    }
    let ctx = Ctx {
        tol: opts.chord_tol * MARGIN,
        max_len: opts.max_edge_len.unwrap_or(f64::INFINITY) * MARGIN,
    };

    let faces = store.faces_of_body(body)?;
    if faces.is_empty() {
        return Err(Error::InvalidGeometry {
            reason: "body has no faces to tessellate",
        }
        .into());
    }

    let mut acc = MeshAcc {
        positions: Vec::new(),
    };
    let mut work = BodyTessellationWork { scope };
    // One global vertex per topological vertex.
    let mut vgids: Vec<(VertexId, u32)> = Vec::new();
    for v in store.vertices_of_body(body)? {
        let gid = acc.push(store.vertex_position(v)?, &mut work)?;
        vgids.push((v, gid));
    }
    // Every edge discretized exactly once.
    let mut elines: EdgeLines = Vec::new();
    for e in store.edges_of_body(body)? {
        let line = discretize_edge(store, e, &vgids, &mut acc, ctx, &mut work)?;
        elines.push(line);
    }
    // Faces, assembled by index mapping.
    let mut triangles: Vec<[u32; 3]> = Vec::new();
    let mut face_ranges = Vec::with_capacity(faces.len());
    for face in faces {
        let start = triangles.len();
        triangles.extend(tess_face(
            store, &elines, &mut acc, face, opts, ctx, &mut work,
        )?);
        face_ranges.push((face, start..triangles.len()));
    }
    Ok(BodyMesh {
        positions: acc.positions,
        triangles,
        face_ranges,
        edge_polylines: elines
            .into_iter()
            .map(|line| {
                (
                    line.edge,
                    line.samples
                        .into_iter()
                        .map(|sample| sample.vertex)
                        .collect(),
                )
            })
            .collect(),
    })
}

fn legacy_body_tessellation_error(error: TessellationError) -> TessellationError {
    match error {
        TessellationError::SurfacePoint(_) => Error::InvalidGeometry {
            reason: "closest-point projection onto NURBS surface failed",
        }
        .into(),
        TessellationError::Kernel(Error::OperationPolicy {
            source: OperationPolicyError::LimitReached(snapshot),
        }) => legacy_body_tessellation_error(Error::ResourceLimit { snapshot }.into()),
        TessellationError::Kernel(Error::ResourceLimit { snapshot })
            if snapshot.stage == kgraph::eval_stage::NODE_VISITS =>
        {
            TessellationError::Evaluation(kgraph::EvalError::NodeVisitLimitExceeded {
                consumed: usize::try_from(snapshot.consumed).unwrap_or(usize::MAX),
                limit: usize::try_from(snapshot.allowed).unwrap_or(usize::MAX),
            })
        }
        TessellationError::Kernel(Error::ResourceLimit { snapshot })
            if snapshot.stage == kgraph::eval_stage::DEPENDENCY_DEPTH =>
        {
            TessellationError::Evaluation(kgraph::EvalError::DependencyDepthExceeded {
                consumed: usize::try_from(snapshot.consumed).unwrap_or(usize::MAX),
                limit: usize::try_from(snapshot.allowed).unwrap_or(usize::MAX),
            })
        }
        TessellationError::Kernel(Error::ResourceLimit { snapshot })
            if snapshot.stage == FACE_TESSELLATION_BOUNDARY_DEPTH =>
        {
            Error::AlgorithmLimit {
                operation: "tessellation boundary refinement depth",
                limit: 16,
            }
            .into()
        }
        TessellationError::Kernel(Error::ResourceLimit { snapshot })
            if snapshot.stage == FACE_TESSELLATION_REFINEMENT_PASSES =>
        {
            Error::AlgorithmLimit {
                operation: "tessellation interior refinement passes",
                limit: 24,
            }
            .into()
        }
        TessellationError::Kernel(Error::ResourceLimit { snapshot })
            if snapshot.stage == FACE_TESSELLATION_MESH_TRIANGLES =>
        {
            Error::AlgorithmLimit {
                operation: "tessellation triangle count",
                limit: 200_000,
            }
            .into()
        }
        other => other,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::check::{
        CheckLevel, CheckOutcome, FaultKind, VerificationGapKind, check_body, check_body_report,
    };
    use crate::entity::{Face, Fin, Loop, PcurveChart, ShellId};
    use crate::geom::CurveGeom;
    use crate::make::{block, cylinder, planar_sheet, solid_body_scaffold};
    use core::num::NonZeroUsize;
    use kcore::math;
    use kcore::operation::DiagnosticLevel;
    use kgeom::aabb::Aabb3;
    use kgeom::curve::{Circle, CurveDerivs, Line};
    use kgeom::frame::Frame;
    use kgeom::nurbs::NurbsSurface;
    use kgeom::param::ParamRange;
    use kgeom::surface::{Cone, Cylinder, Plane, Sphere, SurfaceDerivs, Torus};
    use kgeom::vec::Vec3;
    use kgraph::{EvalError, GeometryRef, OffsetSurfaceDescriptor};
    use std::cell::Cell;

    fn assert_watertight(mesh: &BodyMesh) {
        let problems = check_watertight(mesh);
        assert!(
            problems.is_empty(),
            "mesh not watertight:\n{}",
            problems.join("\n")
        );
    }

    fn opts(chord_tol: f64) -> TessOptions {
        TessOptions {
            chord_tol,
            max_edge_len: None,
        }
    }

    macro_rules! with_body_work {
        ($work:ident, $body:block) => {{
            let session = SessionPolicy::new(
                SessionPrecision::parasolid(),
                NumericalPolicy::v1(),
                ExecutionPolicy::Serial,
                BodyTessellationBudgetProfile::v1_defaults(),
                PolicyVersion::V1,
            );
            let context = OperationContext::new(&session, Tolerances::default()).unwrap();
            let mut scope = OperationScope::new(&context);
            let mut $work = BodyTessellationWork { scope: &mut scope };
            $body
        }};
    }

    fn assert_depth_limit(error: TessellationError, stage: StageId, allowed: u64) {
        let snapshot = LimitSnapshot {
            stage,
            resource: ResourceKind::Depth,
            consumed: allowed + 1,
            allowed,
        };
        assert_eq!(
            error,
            TessellationError::Kernel(Error::ResourceLimit { snapshot })
        );
        assert_eq!(error.limit(), Some(snapshot));
    }

    struct CountingCurve<C> {
        inner: C,
        evaluations: Cell<usize>,
    }

    impl<C> CountingCurve<C> {
        fn new(inner: C) -> Self {
            Self {
                inner,
                evaluations: Cell::new(0),
            }
        }

        fn evaluations(&self) -> usize {
            self.evaluations.get()
        }

        fn reset(&self) {
            self.evaluations.set(0);
        }
    }

    impl<C: Curve> Curve for CountingCurve<C> {
        fn as_any(&self) -> &dyn core::any::Any {
            self
        }

        fn eval_derivs(&self, t: f64, order: usize) -> CurveDerivs {
            self.evaluations.set(self.evaluations.get() + 1);
            self.inner.eval_derivs(t, order)
        }

        fn param_range(&self) -> ParamRange {
            self.inner.param_range()
        }

        fn periodicity(&self) -> Option<f64> {
            self.inner.periodicity()
        }

        fn bounding_box(&self, range: ParamRange) -> Aabb3 {
            self.inner.bounding_box(range)
        }
    }

    struct CountingSurface<S> {
        inner: S,
        evaluations: Cell<usize>,
    }

    impl<S> CountingSurface<S> {
        fn new(inner: S) -> Self {
            Self {
                inner,
                evaluations: Cell::new(0),
            }
        }

        fn evaluations(&self) -> usize {
            self.evaluations.get()
        }

        fn reset(&self) {
            self.evaluations.set(0);
        }
    }

    impl<S: Surface> Surface for CountingSurface<S> {
        fn as_any(&self) -> &dyn core::any::Any {
            self
        }

        fn eval_derivs(&self, uv: [f64; 2], order: usize) -> SurfaceDerivs {
            self.evaluations.set(self.evaluations.get() + 1);
            self.inner.eval_derivs(uv, order)
        }

        fn param_range(&self) -> [ParamRange; 2] {
            self.inner.param_range()
        }

        fn periodicity(&self) -> [Option<f64>; 2] {
            self.inner.periodicity()
        }

        fn degeneracies(&self) -> Vec<kgeom::surface::Degeneracy> {
            self.inner.degeneracies()
        }

        fn bounding_box(&self, range: [ParamRange; 2]) -> Aabb3 {
            self.inner.bounding_box(range)
        }
    }

    struct TentCurve;

    impl Curve for TentCurve {
        fn as_any(&self) -> &dyn core::any::Any {
            self
        }

        fn eval_derivs(&self, t: f64, _order: usize) -> CurveDerivs {
            let height = if t <= 0.5 { 2.0 * t } else { 2.0 * (1.0 - t) };
            let mut result = CurveDerivs::default();
            result.d[0] = Point3::new(t, height, 0.0);
            result
        }

        fn param_range(&self) -> ParamRange {
            ParamRange::new(0.0, 1.0)
        }

        fn periodicity(&self) -> Option<f64> {
            None
        }

        fn bounding_box(&self, _range: ParamRange) -> Aabb3 {
            Aabb3::from_points(&[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.5, 1.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ])
        }
    }

    struct TentSurface;

    impl Surface for TentSurface {
        fn as_any(&self) -> &dyn core::any::Any {
            self
        }

        fn eval_derivs(&self, uv: [f64; 2], _order: usize) -> SurfaceDerivs {
            let height = if uv[0] <= 0.5 {
                2.0 * uv[0]
            } else {
                2.0 * (1.0 - uv[0])
            };
            SurfaceDerivs {
                p: Point3::new(uv[0], uv[1], height),
                ..SurfaceDerivs::default()
            }
        }

        fn param_range(&self) -> [ParamRange; 2] {
            [ParamRange::new(0.0, 1.0), ParamRange::new(0.0, 1.0)]
        }

        fn periodicity(&self) -> [Option<f64>; 2] {
            [None, None]
        }

        fn bounding_box(&self, _range: [ParamRange; 2]) -> Aabb3 {
            Aabb3::from_points(&[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.5, 0.0, 1.0),
                Point3::new(1.0, 1.0, 0.0),
            ])
        }
    }

    fn curve_refinement_outcome(
        curve: &dyn Curve,
        max_len: f64,
        plan: BudgetPlan,
    ) -> OperationOutcome<Vec<(f64, Point3)>, TessellationError> {
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            plan,
            PolicyVersion::V1,
        );
        let context = OperationContext::new(&session, Tolerances::default())
            .unwrap()
            .with_diagnostics(DiagnosticLevel::Summary, 8);
        let mut scope = OperationScope::new(&context);
        let mut interior = Vec::new();
        let result = {
            let refine = CurveRefine {
                curve: Some(curve),
                face_uses: Vec::new(),
                ctx: Ctx { tol: 0.0, max_len },
            };
            let mut work = BodyTessellationWork { scope: &mut scope };
            refine.refine(
                (0.0, curve.eval(0.0)),
                (1.0, curve.eval(1.0)),
                0,
                &mut interior,
                &mut work,
            )
        };
        scope.finish_typed(result.map(|()| interior))
    }

    fn iso_refinement_outcome(
        surface: &dyn Surface,
        max_len: f64,
        plan: BudgetPlan,
    ) -> OperationOutcome<Vec<(Vec2, Point3)>, TessellationError> {
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            plan,
            PolicyVersion::V1,
        );
        let context = OperationContext::new(&session, Tolerances::default())
            .unwrap()
            .with_diagnostics(DiagnosticLevel::Summary, 8);
        let mut scope = OperationScope::new(&context);
        let mut interior = Vec::new();
        let a_uv = Vec2::new(0.0, 0.0);
        let b_uv = Vec2::new(1.0, 0.0);
        let result = {
            let mut work = BodyTessellationWork { scope: &mut scope };
            refine_uv_seg(
                surface,
                (a_uv, surface.eval([a_uv.x, a_uv.y])),
                (b_uv, surface.eval([b_uv.x, b_uv.y])),
                Ctx { tol: 0.0, max_len },
                0,
                &mut interior,
                &mut work,
            )
        };
        scope.finish_typed(result.map(|()| interior))
    }

    #[test]
    fn contextual_body_tessellation_matches_legacy_for_every_execution_policy() {
        let mut store = Store::new();
        let body = block(&mut store, &Frame::world(), [1.0, 2.0, 3.0]).unwrap();
        let options = opts(0.25);
        let legacy = tessellate_body(&store, body, &options).unwrap();
        let executions = [
            ExecutionPolicy::Serial,
            ExecutionPolicy::AtMost(NonZeroUsize::new(1).unwrap()),
            ExecutionPolicy::AtMost(NonZeroUsize::new(2).unwrap()),
            ExecutionPolicy::Available,
        ];
        let mut reports = Vec::new();
        for execution in executions {
            let session = SessionPolicy::new(
                SessionPrecision::parasolid(),
                NumericalPolicy::v1(),
                execution,
                BodyTessellationBudgetProfile::v1_defaults(),
                PolicyVersion::V1,
            );
            let context = OperationContext::new(&session, Tolerances::default()).unwrap();
            let outcome = tessellate_body_with_context(&store, body, &options, &context).unwrap();
            assert_eq!(outcome.result(), Ok(&legacy));
            reports.push(outcome.report().clone());
        }
        assert!(reports.windows(2).all(|pair| pair[0] == pair[1]));
        let consumed: Vec<_> = reports[0]
            .usage()
            .iter()
            .map(|entry| (entry.stage, entry.resource, entry.consumed))
            .collect();
        assert_eq!(
            consumed,
            [
                (
                    kgeom::project::SURFACE_PROJECTION_HALVINGS,
                    ResourceKind::Depth,
                    0
                ),
                (
                    kgeom::project::SURFACE_PROJECTION_CANDIDATES,
                    ResourceKind::Items,
                    0
                ),
                (
                    kgeom::project::SURFACE_PROJECTION_NEWTON_ITERATIONS,
                    ResourceKind::Depth,
                    0
                ),
                (
                    kgeom::project::SURFACE_PROJECTION_QUERIES,
                    ResourceKind::Work,
                    0
                ),
                (
                    kgeom::project::SURFACE_PROJECTION_SAMPLES,
                    ResourceKind::Items,
                    0
                ),
                (FACE_TESSELLATION_BOUNDARY_DEPTH, ResourceKind::Depth, 0),
                (FACE_TESSELLATION_BOUNDARY_SPLITS, ResourceKind::Work, 0),
                (FACE_TESSELLATION_REFINEMENT_PASSES, ResourceKind::Work, 0),
                (FACE_TESSELLATION_MESH_TRIANGLES, ResourceKind::Items, 2),
                (FACE_TESSELLATION_MESH_VERTICES, ResourceKind::Items, 24),
                (kgraph::eval_stage::DEPENDENCY_DEPTH, ResourceKind::Depth, 1),
                (kgraph::eval_stage::NODE_VISITS, ResourceKind::Work, 150),
                (BODY_TESSELLATION_EDGE_DEPTH, ResourceKind::Depth, 0),
                (BODY_TESSELLATION_EDGE_SPLITS, ResourceKind::Work, 0),
                (BODY_TESSELLATION_ISO_ARC_DEPTH, ResourceKind::Depth, 0),
                (BODY_TESSELLATION_ISO_ARC_SPLITS, ResourceKind::Work, 0),
                (BODY_TESSELLATION_MESH_VERTICES, ResourceKind::Items, 8),
            ]
        );
        assert!(reports[0].limit_events().is_empty());
        assert!(reports[0].diagnostics().is_empty());
    }

    #[test]
    fn shared_scope_budget_validation_precedes_options_and_topology() {
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            BudgetPlan::empty(),
            PolicyVersion::V1,
        );
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let mut scope = OperationScope::new(&context);
        let mut source = Store::new();
        let stale = block(&mut source, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        let store = Store::new();
        assert_eq!(
            tessellate_body_in_scope(
                &store,
                stale,
                &TessOptions {
                    chord_tol: f64::NAN,
                    max_edge_len: None,
                },
                &mut scope,
            ),
            Err(TessellationError::Kernel(Error::OperationPolicy {
                source: OperationPolicyError::UnknownLimit {
                    stage: kgeom::project::SURFACE_PROJECTION_HALVINGS,
                    resource: ResourceKind::Depth,
                },
            }))
        );
    }

    #[test]
    fn body_local_depth_cap_survives_a_looser_parent_override() {
        let plan = BodyTessellationBudgetProfile::v1_defaults().overlaid(
            &BudgetPlan::new([LimitSpec::new(
                BODY_TESSELLATION_EDGE_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                17,
            )])
            .unwrap(),
        );
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            plan,
            PolicyVersion::V1,
        );
        let context = OperationContext::new(&session, Tolerances::default())
            .unwrap()
            .with_diagnostics(DiagnosticLevel::Summary, 4);
        let mut scope = OperationScope::new(&context);
        let error = {
            let mut work = BodyTessellationWork { scope: &mut scope };
            next_refinement_depth(
                MAX_DEPTH,
                BODY_TESSELLATION_EDGE_DEPTH,
                BODY_TESSELLATION_EDGE_DEPTH_LIMIT_REACHED,
                "whole-body exact-edge refinement depth limit reached",
                &mut work,
            )
            .unwrap_err()
        };
        assert_depth_limit(
            error,
            BODY_TESSELLATION_EDGE_DEPTH,
            BODY_TESSELLATION_EDGE_DEPTH_LIMIT,
        );
        let report = scope.finish_typed::<(), TessellationError>(Ok(()));
        assert_eq!(
            report.report().limit_events(),
            &[LimitSnapshot {
                stage: BODY_TESSELLATION_EDGE_DEPTH,
                resource: ResourceKind::Depth,
                consumed: 17,
                allowed: 16,
            }]
        );
        assert_eq!(report.report().diagnostics().len(), 1);
        assert_eq!(
            report.report().diagnostics()[0].code,
            BODY_TESSELLATION_EDGE_DEPTH_LIMIT_REACHED
        );
    }

    fn split_limit_plan(stage: StageId, allowed: u64) -> BudgetPlan {
        BodyTessellationBudgetProfile::v1_defaults().overlaid(
            &BudgetPlan::new([LimitSpec::new(
                stage,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            )])
            .unwrap(),
        )
    }

    fn consumed(outcome: &OperationOutcome<impl Sized, TessellationError>, stage: StageId) -> u64 {
        outcome
            .report()
            .usage()
            .iter()
            .find(|entry| entry.stage == stage)
            .expect("stage exists in body report")
            .consumed
    }

    #[test]
    fn edge_and_iso_split_work_accept_n_and_reject_n_plus_one_atomically() {
        let line = Line::new(Point3::default(), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let accepted = curve_refinement_outcome(
            &line,
            0.5,
            split_limit_plan(BODY_TESSELLATION_EDGE_SPLITS, 1),
        );
        assert_eq!(accepted.result().as_ref().unwrap().len(), 1);
        assert_eq!(consumed(&accepted, BODY_TESSELLATION_EDGE_SPLITS), 1);
        assert!(accepted.report().limit_events().is_empty());

        let rejected = curve_refinement_outcome(
            &line,
            0.49,
            split_limit_plan(BODY_TESSELLATION_EDGE_SPLITS, 1),
        );
        let edge_snapshot = LimitSnapshot {
            stage: BODY_TESSELLATION_EDGE_SPLITS,
            resource: ResourceKind::Work,
            consumed: 2,
            allowed: 1,
        };
        assert_eq!(
            rejected.result(),
            Err(&TessellationError::Kernel(Error::ResourceLimit {
                snapshot: edge_snapshot,
            }))
        );
        assert_eq!(consumed(&rejected, BODY_TESSELLATION_EDGE_SPLITS), 1);
        assert_eq!(rejected.report().limit_events(), &[edge_snapshot]);
        assert_eq!(
            rejected.report().diagnostics()[0].code,
            BODY_TESSELLATION_EDGE_SPLIT_LIMIT_REACHED
        );

        let plane = Plane::new(Frame::world());
        let accepted = iso_refinement_outcome(
            &plane,
            0.5,
            split_limit_plan(BODY_TESSELLATION_ISO_ARC_SPLITS, 1),
        );
        assert_eq!(accepted.result().as_ref().unwrap().len(), 1);
        assert_eq!(consumed(&accepted, BODY_TESSELLATION_ISO_ARC_SPLITS), 1);
        assert!(accepted.report().limit_events().is_empty());

        let rejected = iso_refinement_outcome(
            &plane,
            0.49,
            split_limit_plan(BODY_TESSELLATION_ISO_ARC_SPLITS, 1),
        );
        let iso_snapshot = LimitSnapshot {
            stage: BODY_TESSELLATION_ISO_ARC_SPLITS,
            resource: ResourceKind::Work,
            consumed: 2,
            allowed: 1,
        };
        assert_eq!(
            rejected.result(),
            Err(&TessellationError::Kernel(Error::ResourceLimit {
                snapshot: iso_snapshot,
            }))
        );
        assert_eq!(consumed(&rejected, BODY_TESSELLATION_ISO_ARC_SPLITS), 1);
        assert_eq!(rejected.report().limit_events(), &[iso_snapshot]);
        assert_eq!(
            rejected.report().diagnostics()[0].code,
            BODY_TESSELLATION_ISO_ARC_SPLIT_LIMIT_REACHED
        );
    }

    #[test]
    fn depth_precedes_split_work_and_length_denial_skips_midpoint_evaluation() {
        let edge =
            CountingCurve::new(Line::new(Point3::default(), Vec3::new(1.0, 0.0, 0.0)).unwrap());
        let edge_plan = |depth, splits| {
            BodyTessellationBudgetProfile::v1_defaults().overlaid(
                &BudgetPlan::new([
                    LimitSpec::new(
                        BODY_TESSELLATION_EDGE_DEPTH,
                        ResourceKind::Depth,
                        AccountingMode::HighWater,
                        depth,
                    ),
                    LimitSpec::new(
                        BODY_TESSELLATION_EDGE_SPLITS,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        splits,
                    ),
                ])
                .unwrap(),
            )
        };
        let depth_denied = curve_refinement_outcome(&edge, 0.5, edge_plan(0, 0));
        assert!(matches!(
            depth_denied.result(),
            Err(TessellationError::Kernel(Error::ResourceLimit {
                snapshot: LimitSnapshot {
                    stage: BODY_TESSELLATION_EDGE_DEPTH,
                    consumed: 1,
                    allowed: 0,
                    ..
                }
            }))
        ));
        assert_eq!(edge.evaluations(), 2, "only the two caller endpoints ran");
        assert_eq!(consumed(&depth_denied, BODY_TESSELLATION_EDGE_SPLITS), 0);

        edge.reset();
        let split_denied = curve_refinement_outcome(&edge, 0.5, edge_plan(1, 0));
        assert!(matches!(
            split_denied.result(),
            Err(TessellationError::Kernel(Error::ResourceLimit {
                snapshot: LimitSnapshot {
                    stage: BODY_TESSELLATION_EDGE_SPLITS,
                    consumed: 1,
                    allowed: 0,
                    ..
                }
            }))
        ));
        assert_eq!(
            edge.evaluations(),
            2,
            "denied split did not evaluate midpoint"
        );
        assert_eq!(consumed(&split_denied, BODY_TESSELLATION_EDGE_DEPTH), 1);

        let iso = CountingSurface::new(Plane::new(Frame::world()));
        let iso_plan = |depth, splits| {
            BodyTessellationBudgetProfile::v1_defaults().overlaid(
                &BudgetPlan::new([
                    LimitSpec::new(
                        BODY_TESSELLATION_ISO_ARC_DEPTH,
                        ResourceKind::Depth,
                        AccountingMode::HighWater,
                        depth,
                    ),
                    LimitSpec::new(
                        BODY_TESSELLATION_ISO_ARC_SPLITS,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        splits,
                    ),
                ])
                .unwrap(),
            )
        };
        let depth_denied = iso_refinement_outcome(&iso, 0.5, iso_plan(0, 0));
        assert!(matches!(
            depth_denied.result(),
            Err(TessellationError::Kernel(Error::ResourceLimit {
                snapshot: LimitSnapshot {
                    stage: BODY_TESSELLATION_ISO_ARC_DEPTH,
                    consumed: 1,
                    allowed: 0,
                    ..
                }
            }))
        ));
        assert_eq!(iso.evaluations(), 2, "only the two caller endpoints ran");
        assert_eq!(consumed(&depth_denied, BODY_TESSELLATION_ISO_ARC_SPLITS), 0);

        iso.reset();
        let split_denied = iso_refinement_outcome(&iso, 0.5, iso_plan(1, 0));
        assert!(matches!(
            split_denied.result(),
            Err(TessellationError::Kernel(Error::ResourceLimit {
                snapshot: LimitSnapshot {
                    stage: BODY_TESSELLATION_ISO_ARC_SPLITS,
                    consumed: 1,
                    allowed: 0,
                    ..
                }
            }))
        ));
        assert_eq!(
            iso.evaluations(),
            2,
            "denied split did not evaluate midpoint"
        );
        assert_eq!(consumed(&split_denied, BODY_TESSELLATION_ISO_ARC_DEPTH), 1);
    }

    #[test]
    fn curvature_decision_midpoints_are_cached_across_split_admission() {
        let edge = CountingCurve::new(TentCurve);
        let accepted = curve_refinement_outcome(
            &edge,
            f64::INFINITY,
            split_limit_plan(BODY_TESSELLATION_EDGE_SPLITS, 1),
        );
        assert_eq!(accepted.result().as_ref().unwrap().len(), 1);
        assert_eq!(edge.evaluations(), 5);
        assert_eq!(consumed(&accepted, BODY_TESSELLATION_EDGE_SPLITS), 1);

        edge.reset();
        let denied = curve_refinement_outcome(
            &edge,
            f64::INFINITY,
            split_limit_plan(BODY_TESSELLATION_EDGE_SPLITS, 0),
        );
        assert!(denied.result().is_err());
        assert_eq!(edge.evaluations(), 3);
        assert_eq!(consumed(&denied, BODY_TESSELLATION_EDGE_SPLITS), 0);

        let iso = CountingSurface::new(TentSurface);
        let accepted = iso_refinement_outcome(
            &iso,
            f64::INFINITY,
            split_limit_plan(BODY_TESSELLATION_ISO_ARC_SPLITS, 1),
        );
        assert_eq!(accepted.result().as_ref().unwrap().len(), 1);
        assert_eq!(iso.evaluations(), 5);
        assert_eq!(consumed(&accepted, BODY_TESSELLATION_ISO_ARC_SPLITS), 1);

        iso.reset();
        let denied = iso_refinement_outcome(
            &iso,
            f64::INFINITY,
            split_limit_plan(BODY_TESSELLATION_ISO_ARC_SPLITS, 0),
        );
        assert!(denied.result().is_err());
        assert_eq!(iso.evaluations(), 3);
        assert_eq!(consumed(&denied, BODY_TESSELLATION_ISO_ARC_SPLITS), 0);
    }

    #[test]
    fn root_work_aggregates_body_and_face_splits_and_rolls_back_the_crossing() {
        let plan = BodyTessellationBudgetProfile::v1_defaults().with_total_work_limit(4);
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            plan,
            PolicyVersion::V1,
        );
        let context = OperationContext::new(&session, Tolerances::default())
            .unwrap()
            .with_diagnostics(DiagnosticLevel::Summary, 8);
        let mut scope = OperationScope::new(&context);
        let error = {
            let mut work = BodyTessellationWork { scope: &mut scope };
            work.charge_split(
                BODY_TESSELLATION_EDGE_SPLITS,
                BODY_TESSELLATION_EDGE_SPLIT_LIMIT_REACHED,
                "edge split",
            )
            .unwrap();
            work.charge_split(
                BODY_TESSELLATION_ISO_ARC_SPLITS,
                BODY_TESSELLATION_ISO_ARC_SPLIT_LIMIT_REACHED,
                "iso split",
            )
            .unwrap();
            work.scope
                .ledger_mut()
                .charge_resource(FACE_TESSELLATION_BOUNDARY_SPLITS, ResourceKind::Work, 1)
                .unwrap();
            work.scope
                .ledger_mut()
                .charge_resource(FACE_TESSELLATION_REFINEMENT_PASSES, ResourceKind::Work, 1)
                .unwrap();
            work.charge_split(
                BODY_TESSELLATION_EDGE_SPLITS,
                BODY_TESSELLATION_EDGE_SPLIT_LIMIT_REACHED,
                "edge split",
            )
            .unwrap_err()
        };
        let snapshot = LimitSnapshot {
            stage: kcore::operation::TOTAL_WORK_STAGE,
            resource: ResourceKind::Work,
            consumed: 5,
            allowed: 4,
        };
        assert_eq!(
            error,
            TessellationError::Kernel(Error::ResourceLimit { snapshot })
        );
        let outcome = scope.finish_typed::<(), TessellationError>(Ok(()));
        for stage in [
            BODY_TESSELLATION_EDGE_SPLITS,
            BODY_TESSELLATION_ISO_ARC_SPLITS,
            FACE_TESSELLATION_BOUNDARY_SPLITS,
            FACE_TESSELLATION_REFINEMENT_PASSES,
        ] {
            assert_eq!(consumed(&outcome, stage), 1);
        }
        assert_eq!(outcome.report().limit_events(), &[snapshot]);
        assert_eq!(outcome.report().diagnostics().len(), 1);
        assert_eq!(
            outcome.report().diagnostics()[0].code,
            BODY_TESSELLATION_TOTAL_WORK_LIMIT_REACHED
        );
    }

    #[test]
    fn positive_split_work_is_execution_policy_equivalent() {
        let mut store = Store::new();
        let block_body = block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        let sphere_body = closed_body(&mut store, Sphere::new(Frame::world(), 1.0).unwrap().into());
        let cases = [
            (
                block_body,
                TessOptions {
                    chord_tol: 0.1,
                    max_edge_len: Some(0.4),
                },
                BODY_TESSELLATION_EDGE_SPLITS,
            ),
            (sphere_body, opts(0.25), BODY_TESSELLATION_ISO_ARC_SPLITS),
        ];
        for (body, options, expected_stage) in cases {
            let legacy = tessellate_body(&store, body, &options).unwrap();
            let mut reports = Vec::new();
            for execution in [
                ExecutionPolicy::Serial,
                ExecutionPolicy::AtMost(NonZeroUsize::new(1).unwrap()),
                ExecutionPolicy::AtMost(NonZeroUsize::new(2).unwrap()),
                ExecutionPolicy::Available,
            ] {
                let session = SessionPolicy::new(
                    SessionPrecision::parasolid(),
                    NumericalPolicy::v1(),
                    execution,
                    BodyTessellationBudgetProfile::v1_defaults(),
                    PolicyVersion::V1,
                );
                let context = OperationContext::new(&session, Tolerances::default()).unwrap();
                let outcome =
                    tessellate_body_with_context(&store, body, &options, &context).unwrap();
                assert_eq!(outcome.result(), Ok(&legacy));
                assert!(consumed(&outcome, expected_stage) > 0);
                reports.push(outcome.report().clone());
            }
            assert!(reports.windows(2).all(|pair| pair[0] == pair[1]));
        }
    }

    #[test]
    fn configured_mesh_cap_rejects_atomically_with_report_evidence() {
        let mut store = Store::new();
        let body = block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        let plan = BodyTessellationBudgetProfile::v1_defaults().overlaid(
            &BudgetPlan::new([LimitSpec::new(
                BODY_TESSELLATION_MESH_VERTICES,
                ResourceKind::Items,
                AccountingMode::Cumulative,
                7,
            )])
            .unwrap(),
        );
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            plan,
            PolicyVersion::V1,
        );
        let context = OperationContext::new(&session, Tolerances::default())
            .unwrap()
            .with_diagnostics(DiagnosticLevel::Summary, 4);
        let outcome = tessellate_body_with_context(&store, body, &opts(0.25), &context).unwrap();
        let snapshot = LimitSnapshot {
            stage: BODY_TESSELLATION_MESH_VERTICES,
            resource: ResourceKind::Items,
            consumed: 8,
            allowed: 7,
        };
        assert_eq!(
            outcome.result(),
            Err(&TessellationError::Kernel(Error::ResourceLimit {
                snapshot
            }))
        );
        let usage = outcome
            .report()
            .usage()
            .iter()
            .find(|entry| entry.stage == BODY_TESSELLATION_MESH_VERTICES)
            .unwrap();
        assert_eq!(usage.consumed, 7);
        assert_eq!(outcome.report().limit_events(), &[snapshot]);
        assert_eq!(outcome.report().diagnostics().len(), 1);
        assert_eq!(
            outcome.report().diagnostics()[0].code,
            BODY_TESSELLATION_MESH_VERTEX_LIMIT_REACHED
        );
    }

    #[cfg(target_pointer_width = "64")]
    #[test]
    fn raised_mesh_policy_cannot_expand_the_physical_u32_index_space() {
        let plan = BodyTessellationBudgetProfile::v1_defaults().overlaid(
            &BudgetPlan::new([LimitSpec::new(
                BODY_TESSELLATION_MESH_VERTICES,
                ResourceKind::Items,
                AccountingMode::Cumulative,
                BODY_TESSELLATION_MESH_VERTEX_LIMIT + 10,
            )])
            .unwrap(),
        );
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            plan,
            PolicyVersion::V1,
        );
        let context = OperationContext::new(&session, Tolerances::default())
            .unwrap()
            .with_diagnostics(DiagnosticLevel::Summary, 4);
        let mut scope = OperationScope::new(&context);
        scope
            .ledger_mut()
            .charge_resource(
                BODY_TESSELLATION_MESH_VERTICES,
                ResourceKind::Items,
                BODY_TESSELLATION_MESH_VERTEX_LIMIT,
            )
            .unwrap();
        let error = {
            let mut work = BodyTessellationWork { scope: &mut scope };
            work.reject_physical_mesh_vertex(BODY_TESSELLATION_MESH_VERTEX_LIMIT as usize)
                .unwrap_err()
        };
        let physical = LimitSnapshot {
            stage: BODY_TESSELLATION_MESH_VERTICES,
            resource: ResourceKind::Items,
            consumed: BODY_TESSELLATION_MESH_VERTEX_LIMIT + 1,
            allowed: BODY_TESSELLATION_MESH_VERTEX_LIMIT,
        };
        assert_eq!(
            error,
            TessellationError::Kernel(Error::ResourceLimit { snapshot: physical })
        );
        let outcome = scope.finish_typed::<(), TessellationError>(Ok(()));
        let usage = outcome
            .report()
            .usage()
            .iter()
            .find(|entry| entry.stage == BODY_TESSELLATION_MESH_VERTICES)
            .unwrap();
        assert_eq!(usage.consumed, BODY_TESSELLATION_MESH_VERTEX_LIMIT);
        assert!(outcome.report().limit_events().is_empty());
        assert_eq!(outcome.report().diagnostics().len(), 1);
        assert_eq!(
            outcome.report().diagnostics()[0].kind,
            DiagnosticKind::LimitReached(physical)
        );
    }

    #[test]
    fn legacy_error_mapping_restores_graph_and_face_failure_shapes() {
        let graph = LimitSnapshot {
            stage: kgraph::eval_stage::NODE_VISITS,
            resource: ResourceKind::Work,
            consumed: 4097,
            allowed: 4096,
        };
        assert_eq!(
            legacy_body_tessellation_error(Error::ResourceLimit { snapshot: graph }.into()),
            TessellationError::Evaluation(EvalError::NodeVisitLimitExceeded {
                consumed: 4097,
                limit: 4096,
            })
        );
        for (stage, resource, operation, limit) in [
            (
                FACE_TESSELLATION_BOUNDARY_DEPTH,
                ResourceKind::Depth,
                "tessellation boundary refinement depth",
                16,
            ),
            (
                FACE_TESSELLATION_REFINEMENT_PASSES,
                ResourceKind::Work,
                "tessellation interior refinement passes",
                24,
            ),
            (
                FACE_TESSELLATION_MESH_TRIANGLES,
                ResourceKind::Items,
                "tessellation triangle count",
                200_000,
            ),
        ] {
            let error = legacy_body_tessellation_error(
                Error::ResourceLimit {
                    snapshot: LimitSnapshot {
                        stage,
                        resource,
                        consumed: limit as u64 + 1,
                        allowed: limit as u64,
                    },
                }
                .into(),
            );
            assert_eq!(
                error,
                TessellationError::Kernel(Error::AlgorithmLimit { operation, limit })
            );
        }
    }

    #[test]
    fn face_limit_diagnostic_mapping_covers_all_composed_stages_and_generic_root() {
        for (stage, code) in [
            (
                FACE_TESSELLATION_BOUNDARY_DEPTH,
                FACE_TESSELLATION_BOUNDARY_DEPTH_LIMIT,
            ),
            (
                FACE_TESSELLATION_BOUNDARY_SPLITS,
                FACE_TESSELLATION_BOUNDARY_SPLIT_LIMIT,
            ),
            (
                FACE_TESSELLATION_REFINEMENT_PASSES,
                FACE_TESSELLATION_REFINEMENT_PASS_LIMIT,
            ),
            (
                FACE_TESSELLATION_MESH_TRIANGLES,
                FACE_TESSELLATION_MESH_TRIANGLE_LIMIT,
            ),
            (
                FACE_TESSELLATION_MESH_VERTICES,
                FACE_TESSELLATION_MESH_VERTEX_LIMIT,
            ),
            (
                kcore::operation::TOTAL_WORK_STAGE,
                BODY_TESSELLATION_TOTAL_WORK_LIMIT_REACHED,
            ),
        ] {
            assert_eq!(
                face_tessellation_limit_diagnostic(stage).map(|mapped| mapped.0),
                Some(code)
            );
        }
        assert_eq!(
            face_tessellation_limit_diagnostic(kgraph::eval_stage::NODE_VISITS),
            None
        );
    }

    #[test]
    fn nurbs_inversion_uses_the_shared_projection_scope() {
        let surface = SurfaceGeom::Nurbs(
            NurbsSurface::new(
                1,
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![0.0, 0.0, 1.0, 1.0],
                vec![
                    Vec3::new(0.0, 0.0, 0.0),
                    Vec3::new(0.0, 1.0, 0.0),
                    Vec3::new(1.0, 0.0, 0.0),
                    Vec3::new(1.0, 1.0, 0.0),
                ],
                None,
            )
            .unwrap(),
        );
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            BodyTessellationBudgetProfile::v1_defaults(),
            PolicyVersion::V1,
        );
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let mut scope = OperationScope::new(&context);
        let mapped = {
            let mut work = BodyTessellationWork { scope: &mut scope };
            invert_uv(&surface, Point3::new(0.25, 0.75, 0.0), &mut work).unwrap()
        };
        assert!((mapped.x - 0.25).abs() < 1.0e-10);
        assert!((mapped.y - 0.75).abs() < 1.0e-10);
        let outcome = scope.finish_typed::<(), TessellationError>(Ok(()));
        let usage = outcome.report().usage();
        let consumed = |stage| {
            usage
                .iter()
                .find(|entry| entry.stage == stage)
                .unwrap()
                .consumed
        };
        assert_eq!(consumed(kgeom::project::SURFACE_PROJECTION_QUERIES), 1);
        assert_eq!(consumed(kgeom::project::SURFACE_PROJECTION_SAMPLES), 625);
        assert!(outcome.report().limit_events().is_empty());

        let limited = BodyTessellationBudgetProfile::v1_defaults().overlaid(
            &BudgetPlan::new([LimitSpec::new(
                kgeom::project::SURFACE_PROJECTION_QUERIES,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                0,
            )])
            .unwrap(),
        );
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            limited,
            PolicyVersion::V1,
        );
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let mut scope = OperationScope::new(&context);
        let error = {
            let mut work = BodyTessellationWork { scope: &mut scope };
            invert_uv(&surface, Point3::new(0.25, 0.75, 0.0), &mut work).unwrap_err()
        };
        assert!(matches!(
            error,
            TessellationError::SurfacePoint(
                kgeom::surface_point::SurfacePointContextError::Projection(
                    kgeom::project::ProjectionError::Policy(OperationPolicyError::LimitReached(
                        LimitSnapshot {
                            stage: kgeom::project::SURFACE_PROJECTION_QUERIES,
                            resource: ResourceKind::Work,
                            consumed: 1,
                            allowed: 0,
                        }
                    ))
                )
            )
        ));
        let report = scope.finish_typed::<(), TessellationError>(Ok(()));
        assert_eq!(
            report.report().limit_events(),
            &[LimitSnapshot {
                stage: kgeom::project::SURFACE_PROJECTION_QUERIES,
                resource: ResourceKind::Work,
                consumed: 1,
                allowed: 0,
            }]
        );
    }

    #[test]
    fn face_refinement_aggregate_and_root_limits_retain_body_diagnostics() {
        let mut store = Store::new();
        let body = closed_body(&mut store, Sphere::new(Frame::world(), 1.0).unwrap().into());
        let options = opts(0.1);
        let run = |plan: BudgetPlan| {
            let session = SessionPolicy::new(
                SessionPrecision::parasolid(),
                NumericalPolicy::v1(),
                ExecutionPolicy::Serial,
                plan,
                PolicyVersion::V1,
            );
            let context = OperationContext::new(&session, Tolerances::default())
                .unwrap()
                .with_diagnostics(DiagnosticLevel::Summary, 4);
            tessellate_body_with_context(&store, body, &options, &context).unwrap()
        };
        let baseline = run(BodyTessellationBudgetProfile::v1_defaults());
        assert!(baseline.result().is_ok());
        let passes = baseline
            .report()
            .usage()
            .iter()
            .find(|entry| entry.stage == FACE_TESSELLATION_REFINEMENT_PASSES)
            .unwrap()
            .consumed;
        assert!(passes > 0);

        for (plan, stage) in [
            (
                BodyTessellationBudgetProfile::v1_defaults().overlaid(
                    &BudgetPlan::new([LimitSpec::new(
                        FACE_TESSELLATION_REFINEMENT_PASSES,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        0,
                    )])
                    .unwrap(),
                ),
                FACE_TESSELLATION_REFINEMENT_PASSES,
            ),
            (
                BodyTessellationBudgetProfile::v1_defaults().with_total_work_limit(0),
                kcore::operation::TOTAL_WORK_STAGE,
            ),
        ] {
            let outcome = run(plan);
            let snapshot = LimitSnapshot {
                stage,
                resource: ResourceKind::Work,
                consumed: 1,
                allowed: 0,
            };
            assert_eq!(
                outcome.result(),
                Err(&TessellationError::Kernel(Error::ResourceLimit {
                    snapshot
                }))
            );
            assert_eq!(outcome.report().limit_events(), &[snapshot]);
            assert_eq!(outcome.report().diagnostics().len(), 1);
            assert_eq!(
                outcome.report().diagnostics()[0].code,
                if stage == kcore::operation::TOTAL_WORK_STAGE {
                    BODY_TESSELLATION_TOTAL_WORK_LIMIT_REACHED
                } else {
                    FACE_TESSELLATION_REFINEMENT_PASS_LIMIT
                }
            );
        }
    }

    #[test]
    fn curve_refinement_enforces_quality_at_n_minus_one_n_and_n_plus_one() {
        let line = Line::new(Point3::default(), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        with_body_work!(work, {
            for required_depth in [MAX_DEPTH - 1, MAX_DEPTH] {
                let refine = CurveRefine {
                    curve: Some(&line),
                    face_uses: Vec::new(),
                    ctx: Ctx {
                        tol: 0.0,
                        max_len: 2.0_f64.powi(-(required_depth as i32)),
                    },
                };
                let mut interior = Vec::new();
                refine
                    .refine(
                        (0.0, line.eval(0.0)),
                        (1.0, line.eval(1.0)),
                        0,
                        &mut interior,
                        &mut work,
                    )
                    .unwrap();

                let segment_count = 1_usize << required_depth;
                assert_eq!(interior.len(), segment_count - 1);
                for (i, &(parameter, point)) in interior.iter().enumerate() {
                    let expected = (i + 1) as f64 / segment_count as f64;
                    assert_eq!(parameter.to_bits(), expected.to_bits());
                    assert_eq!(point.x.to_bits(), expected.to_bits());
                }
            }

            let refine = CurveRefine {
                curve: Some(&line),
                face_uses: Vec::new(),
                ctx: Ctx {
                    tol: 0.0,
                    max_len: 2.0_f64.powi(-((MAX_DEPTH + 1) as i32)),
                },
            };
            let error = refine
                .refine(
                    (0.0, line.eval(0.0)),
                    (1.0, line.eval(1.0)),
                    0,
                    &mut Vec::new(),
                    &mut work,
                )
                .unwrap_err();
            assert_depth_limit(
                error,
                BODY_TESSELLATION_EDGE_DEPTH,
                BODY_TESSELLATION_EDGE_DEPTH_LIMIT,
            );
        });
    }

    #[test]
    fn uv_refinement_enforces_quality_at_n_minus_one_n_and_n_plus_one() {
        let plane = Plane::new(Frame::world());
        let a = (Vec2::new(0.0, 0.0), plane.eval([0.0, 0.0]));
        let b = (Vec2::new(1.0, 0.0), plane.eval([1.0, 0.0]));
        with_body_work!(work, {
            for required_depth in [MAX_DEPTH - 1, MAX_DEPTH] {
                let mut interior = Vec::new();
                refine_uv_seg(
                    &plane,
                    a,
                    b,
                    Ctx {
                        tol: 0.0,
                        max_len: 2.0_f64.powi(-(required_depth as i32)),
                    },
                    0,
                    &mut interior,
                    &mut work,
                )
                .unwrap();

                let segment_count = 1_usize << required_depth;
                assert_eq!(interior.len(), segment_count - 1);
                for (i, &(uv, point)) in interior.iter().enumerate() {
                    let expected = (i + 1) as f64 / segment_count as f64;
                    assert_eq!(uv.x.to_bits(), expected.to_bits());
                    assert_eq!(point.x.to_bits(), expected.to_bits());
                }
            }

            let error = refine_uv_seg(
                &plane,
                a,
                b,
                Ctx {
                    tol: 0.0,
                    max_len: 2.0_f64.powi(-((MAX_DEPTH + 1) as i32)),
                },
                0,
                &mut Vec::new(),
                &mut work,
            )
            .unwrap_err();
            assert_depth_limit(
                error,
                BODY_TESSELLATION_ISO_ARC_DEPTH,
                BODY_TESSELLATION_ISO_ARC_DEPTH_LIMIT,
            );
        });
    }

    #[cfg(target_pointer_width = "64")]
    #[test]
    fn mesh_vertex_index_reports_n_minus_one_n_and_n_plus_one_items() {
        let capacity = BODY_TESSELLATION_MESH_VERTEX_LIMIT as usize;
        assert_eq!(mesh_vertex_index(capacity - 2).unwrap(), u32::MAX - 1);
        assert_eq!(mesh_vertex_index(capacity - 1).unwrap(), u32::MAX);

        let error = mesh_vertex_index(capacity).unwrap_err();
        let snapshot = LimitSnapshot {
            stage: BODY_TESSELLATION_MESH_VERTICES,
            resource: ResourceKind::Items,
            consumed: BODY_TESSELLATION_MESH_VERTEX_LIMIT + 1,
            allowed: BODY_TESSELLATION_MESH_VERTEX_LIMIT,
        };
        assert_eq!(
            error,
            TessellationError::Kernel(Error::ResourceLimit { snapshot })
        );
        assert_eq!(error.limit(), Some(snapshot));
    }

    #[test]
    fn checked_planar_offset_face_tessellates_through_pcurves_without_basis_copy() {
        let mut store = Store::new();
        let world = Frame::world();
        let translated = Frame::new(
            world.origin() + Vec3::new(0.0, 0.0, 1.0),
            world.z(),
            world.x(),
        )
        .unwrap();
        let body = planar_sheet(
            &mut store,
            &translated,
            &[
                Vec2::new(-1.0, -1.0),
                Vec2::new(1.0, -1.0),
                Vec2::new(1.0, 1.0),
                Vec2::new(-1.0, 1.0),
            ],
        )
        .unwrap();
        let face = store.faces_of_body(body).unwrap()[0];
        let old_surface = store.get(face).unwrap().surface;

        let mut transaction = store.transaction().unwrap();
        let (basis, offset) = {
            let mut assembly = transaction.assembly();
            let basis = assembly
                .insert_surface(Plane::new(Frame::world()).into())
                .unwrap();
            let offset = assembly
                .insert_surface(OffsetSurfaceDescriptor::new(basis, 1.0).into())
                .unwrap();
            assembly.get_mut(face).unwrap().surface = offset;
            assembly.remove_surface(old_surface).unwrap();
            (basis, offset)
        };
        transaction.commit_checked_body(body).unwrap();

        assert_eq!(store.geometry().surface_count(), 2);
        assert_eq!(
            store
                .geometry()
                .dependency_closure(GeometryRef::Surface(offset))
                .unwrap(),
            vec![GeometryRef::Surface(basis), GeometryRef::Surface(offset)]
        );
        assert!(check_body(&store, body).unwrap().is_empty());
        let report = check_body_report(&store, body, CheckLevel::Full).unwrap();
        assert_eq!(report.outcome(), CheckOutcome::Indeterminate);
        assert!(report.gaps.iter().any(|gap| {
            gap.entity == crate::entity::EntityRef::Face(face)
                && gap.kind == VerificationGapKind::SurfaceRegularity
        }));

        let mesh = tessellate_body(&store, body, &opts(1.0e-4)).unwrap();
        assert!(!mesh.triangles.is_empty());
        assert!(
            mesh.positions
                .iter()
                .all(|point| (point.z - 1.0).abs() <= 1.0e-12)
        );
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            BodyTessellationBudgetProfile::v1_defaults(),
            PolicyVersion::V1,
        );
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let contextual =
            tessellate_body_with_context(&store, body, &opts(1.0e-4), &context).unwrap();
        assert_eq!(contextual.result(), Ok(&mesh));
        let usage = contextual.report().usage();
        let consumed = |stage| {
            usage
                .iter()
                .find(|entry| entry.stage == stage)
                .unwrap()
                .consumed
        };
        // Leaf proof, periodicity, exact sampling, and derivative-frame
        // reconstruction all traverse the procedural graph explicitly.
        assert!(consumed(kgraph::eval_stage::NODE_VISITS) >= 8);
        assert_eq!(consumed(kgraph::eval_stage::DEPENDENCY_DEPTH), 2);
        assert_eq!(consumed(kgeom::project::SURFACE_PROJECTION_QUERIES), 0);

        let max_edge_len = 0.2;
        let constrained = tessellate_body(
            &store,
            body,
            &TessOptions {
                chord_tol: 1.0e-4,
                max_edge_len: Some(max_edge_len),
            },
        )
        .unwrap();
        for triangle in constrained.triangles {
            let [a, b, c] = triangle.map(|index| constrained.positions[index as usize]);
            for length in [a.dist(b), b.dist(c), c.dist(a)] {
                assert!(length <= max_edge_len + 1.0e-12, "mesh edge {length}");
            }
        }
    }

    #[test]
    fn failed_checked_offset_retarget_rolls_back_graph_and_topology() {
        let mut store = Store::new();
        let world = Frame::world();
        let translated = Frame::new(
            world.origin() + Vec3::new(0.0, 0.0, 1.0),
            world.z(),
            world.x(),
        )
        .unwrap();
        let body = planar_sheet(
            &mut store,
            &translated,
            &[
                Vec2::new(0.0, 0.0),
                Vec2::new(1.0, 0.0),
                Vec2::new(1.0, 1.0),
                Vec2::new(0.0, 1.0),
            ],
        )
        .unwrap();
        let face = store.faces_of_body(body).unwrap()[0];
        let original_surface = store.get(face).unwrap().surface;
        let original_count = store.geometry().surface_count();

        let mut transaction = store.transaction().unwrap();
        {
            let mut assembly = transaction.assembly();
            let basis = assembly
                .insert_surface(Plane::new(Frame::world()).into())
                .unwrap();
            let bad_offset = assembly
                .insert_surface(OffsetSurfaceDescriptor::new(basis, 2.0).into())
                .unwrap();
            assembly.get_mut(face).unwrap().surface = bad_offset;
        }
        let faults = check_body(transaction.store(), body).unwrap();
        assert!(
            faults
                .iter()
                .any(|fault| fault.kind == FaultKind::PcurveOffSurface)
        );
        assert!(transaction.commit_checked_body(body).is_err());

        assert_eq!(store.geometry().surface_count(), original_count);
        assert_eq!(store.get(face).unwrap().surface, original_surface);
        store.geometry().validate().unwrap();
    }

    #[test]
    fn procedural_face_requires_pcurves_and_rejects_unclassifiable_samples() {
        let polygon = [
            Vec2::new(0.0, 0.0),
            Vec2::new(1.0, 0.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(0.0, 1.0),
        ];

        let mut store = Store::new();
        let body = planar_sheet(&mut store, &Frame::world(), &polygon).unwrap();
        let face = store.faces_of_body(body).unwrap()[0];
        let loop_id = store.get(face).unwrap().loops[0];
        let fin = store.get(loop_id).unwrap().fins[0];
        let mut transaction = store.transaction().unwrap();
        {
            let mut assembly = transaction.assembly();
            let basis = assembly
                .insert_surface(Plane::new(Frame::world()).into())
                .unwrap();
            let offset = assembly
                .insert_surface(OffsetSurfaceDescriptor::new(basis, 0.0).into())
                .unwrap();
            assembly.get_mut(face).unwrap().surface = offset;
            assembly.get_mut(fin).unwrap().pcurve = None;
        }
        let faults = check_body(transaction.store(), body).unwrap();
        assert!(faults.iter().any(|fault| {
            fault.entity == crate::entity::EntityRef::Fin(fin)
                && fault.kind == FaultKind::MissingPcurve
        }));
        assert!(transaction.commit_checked_body(body).is_err());
        assert_eq!(
            store.get(fin).unwrap().pcurve.unwrap().closure_winding(),
            None
        );

        let mut transaction = store.transaction().unwrap();
        {
            let mut assembly = transaction.assembly();
            let basis = assembly
                .insert_surface(Plane::new(Frame::world()).into())
                .unwrap();
            let offset = assembly
                .insert_surface(OffsetSurfaceDescriptor::new(basis, 0.0).into())
                .unwrap();
            assembly.get_mut(face).unwrap().surface = offset;
            let use_ = assembly.get(fin).unwrap().pcurve.unwrap();
            assembly.get_mut(fin).unwrap().pcurve =
                Some(use_.with_chart(PcurveChart::shifted([1, 0])));
        }
        let faults = check_body(transaction.store(), body).unwrap();
        assert!(faults.iter().any(|fault| {
            fault.entity == crate::entity::EntityRef::Fin(fin)
                && fault.kind == FaultKind::BadPcurveChart
        }));
        drop(transaction);

        let mut store = Store::new();
        let body = planar_sheet(&mut store, &Frame::world(), &polygon).unwrap();
        let face = store.faces_of_body(body).unwrap()[0];
        let mut transaction = store.transaction().unwrap();
        {
            let mut assembly = transaction.assembly();
            let basis = assembly
                .insert_surface(Cylinder::new(Frame::world(), 2.0).unwrap().into())
                .unwrap();
            let singular = assembly
                .insert_surface(OffsetSurfaceDescriptor::new(basis, -2.0).into())
                .unwrap();
            assembly.get_mut(face).unwrap().surface = singular;
        }
        let faults = check_body(transaction.store(), body).unwrap();
        assert!(
            faults
                .iter()
                .any(|fault| fault.kind == FaultKind::SurfaceSingular)
        );
        drop(transaction);

        let epsilon = 1.0e-12;
        let ill_conditioned = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, epsilon, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(2.0, epsilon, 0.0),
            ],
            None,
        )
        .unwrap();
        let face = store.faces_of_body(body).unwrap()[0];
        let loop_id = store.get(face).unwrap().loops[0];
        let fin = store.get(loop_id).unwrap().fins[0];
        let mut transaction = store.transaction().unwrap();
        {
            let mut assembly = transaction.assembly();
            let basis = assembly
                .insert_surface(ill_conditioned.clone().into())
                .unwrap();
            let offset = assembly
                .insert_surface(OffsetSurfaceDescriptor::new(basis, 1.0).into())
                .unwrap();
            assembly.get_mut(face).unwrap().surface = offset;
            let use_ = assembly.get(fin).unwrap().pcurve.unwrap();
            assembly.get_mut(fin).unwrap().pcurve = Some(use_.with_closure_winding([1, 0]));
        }
        let faults = check_body(transaction.store(), body).unwrap();
        assert!(faults.iter().any(|fault| {
            fault.entity == crate::entity::EntityRef::Fin(fin)
                && fault.kind == FaultKind::BadPcurveClosure
        }));
        assert!(transaction.commit_checked_body(body).is_err());
        assert_eq!(
            store.get(fin).unwrap().pcurve.unwrap().closure_winding(),
            None
        );

        let mut transaction = store.transaction().unwrap();
        {
            let mut assembly = transaction.assembly();
            let basis = assembly.insert_surface(ill_conditioned.into()).unwrap();
            let offset = assembly
                .insert_surface(OffsetSurfaceDescriptor::new(basis, 1.0).into())
                .unwrap();
            assembly.get_mut(face).unwrap().surface = offset;
        }
        let faults = check_body(transaction.store(), body).unwrap();
        assert!(faults.is_empty());
        let report = check_body_report(transaction.store(), body, CheckLevel::Full).unwrap();
        assert_eq!(report.outcome(), CheckOutcome::Indeterminate);
        assert!(report.gaps.iter().any(|gap| {
            gap.entity == crate::entity::EntityRef::Face(face)
                && gap.kind == VerificationGapKind::SurfaceRegularity
        }));
        transaction.commit_checked_body(body).unwrap();
        assert!(matches!(
            tessellate_body(&store, body, &opts(1.0e-3)),
            Err(TessellationError::Indeterminate {
                surface: _,
                source: Some(EvalError::IllConditionedSurface { .. })
            })
        ));
    }

    #[test]
    fn explicit_periodic_pcurve_branch_drives_uv_and_tessellation() {
        let mut store = Store::new();
        let body = cylinder(&mut store, &Frame::world(), 1.0, 2.0).unwrap();
        let side = store.faces_of_body(body).unwrap()[0];
        let surface_id = store.get(side).unwrap().surface;
        let lp = store.get(side).unwrap().loops[0];
        let fin_id = store.get(lp).unwrap().fins[0];
        let tau = core::f64::consts::TAU;
        let loops = store.get(side).unwrap().loops.clone();
        for loop_id in loops {
            let fins = store.get(loop_id).unwrap().fins.clone();
            for fin in fins {
                let use_ = store.get(fin).unwrap().pcurve.unwrap();
                store.get_mut(fin).unwrap().pcurve =
                    Some(use_.with_chart(PcurveChart::shifted([1, 0])));
            }
        }
        let domain = crate::domain::derive_face_domain(&store, side).unwrap();
        store.get_mut(side).unwrap().domain = domain;

        // A whole-period chart shift lifts to the same cylinder and stays
        // checker-clean without duplicating pcurve geometry, but it is
        // observably distinct from 3D inversion.
        assert!(check_body(&store, body).unwrap().is_empty());
        let fin = store.get(fin_id).unwrap();
        let edge = store.get(fin.edge).unwrap();
        let curve = store.get(edge.curve.unwrap()).unwrap().as_curve();
        let t = curve.param_range().lo;
        let point = curve.eval(t);
        let acc = MeshAcc {
            positions: vec![point],
        };
        let sg = store.get(surface_id).unwrap();
        with_body_work!(work, {
            let uv = fin_sample_uv(
                &store,
                surface_id,
                sg,
                &acc,
                fin,
                EdgeSample {
                    parameter: t,
                    vertex: 0,
                },
                &mut work,
            )
            .unwrap();
            let inverted = invert_uv(sg, point, &mut work).unwrap();
            assert!((uv.x - inverted.x - tau).abs() < 1e-12);
        });

        let mesh = tessellate_body(&store, body, &opts(1e-3)).unwrap();
        assert_watertight(&mesh);
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            BodyTessellationBudgetProfile::v1_defaults(),
            PolicyVersion::V1,
        );
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let contextual = tessellate_body_with_context(&store, body, &opts(1e-3), &context).unwrap();
        assert_eq!(contextual.result(), Ok(&mesh));
        assert_eq!(
            contextual
                .report()
                .usage()
                .iter()
                .find(|entry| entry.stage == kgeom::project::SURFACE_PROJECTION_QUERIES)
                .unwrap()
                .consumed,
            0,
            "explicit pcurves must not fall back to surface projection"
        );
    }

    fn tilted() -> Frame {
        Frame::new(
            Point3::new(0.3, -1.2, 2.1),
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap()
    }

    /// A vertex-less ring edge over one full period of `circle`.
    fn ring_edge(store: &mut Store, circle: Circle) -> EdgeId {
        let curve = store.insert_curve(CurveGeom::Circle(circle)).unwrap();
        store.add(Edge {
            curve: Some(curve),
            vertices: [None, None],
            bounds: None,
            fins: Vec::new(),
            tolerance: None,
        })
    }

    fn add_face(store: &mut Store, shell: ShellId, surface: SurfaceGeom) -> FaceId {
        let surface = store.insert_surface(surface).unwrap();
        let face = store.add(Face {
            shell,
            loops: Vec::new(),
            surface,
            sense: Sense::Forward,
            domain: None,
            tolerance: None,
        });
        store.get_mut(shell).unwrap().faces.push(face);
        face
    }

    fn add_ring_loop(store: &mut Store, face: FaceId, edge: EdgeId, sense: Sense) {
        let lp = store.add(Loop {
            face,
            fins: Vec::new(),
        });
        store.get_mut(face).unwrap().loops.push(lp);
        let fin = store.add(Fin {
            parent: lp,
            edge,
            sense,
            pcurve: None,
        });
        store.get_mut(lp).unwrap().fins.push(fin);
        store.get_mut(edge).unwrap().fins.push(fin);
    }

    /// Hand-built solid cylinder: side face with two ring-edge loops plus
    /// two planar caps (stand-in until `make::cylinder` is integrated).
    fn cylinder_body(store: &mut Store, f: &Frame, r: f64, h: f64) -> BodyId {
        let (body, shell) = solid_body_scaffold(store);
        let top_center = f.origin() + f.z() * h;
        let e_bot = ring_edge(
            store,
            Circle::new(Frame::new(f.origin(), f.z(), f.x()).unwrap(), r).unwrap(),
        );
        let e_top = ring_edge(
            store,
            Circle::new(Frame::new(top_center, f.z(), f.x()).unwrap(), r).unwrap(),
        );
        // Side face: interior lies above the bottom ring (+v) and below
        // the top ring, so the bottom fin runs forward and the top fin
        // reversed (interior-on-the-left with the outward normal up).
        let side = add_face(store, shell, Cylinder::new(*f, r).unwrap().into());
        add_ring_loop(store, side, e_bot, Sense::Forward);
        add_ring_loop(store, side, e_top, Sense::Reversed);
        // Caps: plane normals point outward (down / up); the disc interior
        // is on the left of the reversed / forward circle traversal.
        let bot_cap = add_face(
            store,
            shell,
            Plane::new(Frame::new(f.origin(), -f.z(), f.x()).unwrap()).into(),
        );
        add_ring_loop(store, bot_cap, e_bot, Sense::Reversed);
        let top_cap = add_face(
            store,
            shell,
            Plane::new(Frame::new(top_center, f.z(), f.x()).unwrap()).into(),
        );
        add_ring_loop(store, top_cap, e_top, Sense::Forward);
        body
    }

    /// Hand-built solid cone frustum between `v = 0` and `v = h / cos α`.
    fn cone_body(store: &mut Store, f: &Frame, r0: f64, alpha: f64, h: f64) -> BodyId {
        let (body, shell) = solid_body_scaffold(store);
        let (sin_a, cos_a) = math::sincos(alpha);
        let v1 = h / cos_a;
        let r1 = r0 + v1 * sin_a;
        let top_center = f.origin() + f.z() * h;
        let e_bot = ring_edge(
            store,
            Circle::new(Frame::new(f.origin(), f.z(), f.x()).unwrap(), r0).unwrap(),
        );
        let e_top = ring_edge(
            store,
            Circle::new(Frame::new(top_center, f.z(), f.x()).unwrap(), r1).unwrap(),
        );
        let side = add_face(store, shell, Cone::new(*f, r0, alpha).unwrap().into());
        add_ring_loop(store, side, e_bot, Sense::Forward);
        add_ring_loop(store, side, e_top, Sense::Reversed);
        let bot_cap = add_face(
            store,
            shell,
            Plane::new(Frame::new(f.origin(), -f.z(), f.x()).unwrap()).into(),
        );
        add_ring_loop(store, bot_cap, e_bot, Sense::Reversed);
        let top_cap = add_face(
            store,
            shell,
            Plane::new(Frame::new(top_center, f.z(), f.x()).unwrap()).into(),
        );
        add_ring_loop(store, top_cap, e_top, Sense::Forward);
        body
    }

    /// Hand-built solid body covered by one zero-loop face.
    fn closed_body(store: &mut Store, surface: SurfaceGeom) -> BodyId {
        let (body, shell) = solid_body_scaffold(store);
        add_face(store, shell, surface);
        body
    }

    /// Hand-built cut sphere: the sphere portion on one side of a cutting
    /// plane (a spherical polar-cap face bounded by a single ring edge)
    /// plus the planar disk. `circle_frame.z` is the cut-plane normal;
    /// `keep_normal_side` selects which half of the sphere is material.
    fn cut_sphere_body(
        store: &mut Store,
        sphere: Sphere,
        circle_frame: Frame,
        circle_radius: f64,
        keep_normal_side: bool,
    ) -> BodyId {
        let (body, shell) = solid_body_scaffold(store);
        let e = ring_edge(store, Circle::new(circle_frame, circle_radius).unwrap());
        let sphere_face = add_face(store, shell, sphere.into());
        // Interior-on-the-left: keeping the circle-normal side means the
        // circle's own +t direction (counterclockwise around the plane
        // normal) bounds the material; otherwise it is reversed.
        let sphere_sense = if keep_normal_side {
            Sense::Forward
        } else {
            Sense::Reversed
        };
        add_ring_loop(store, sphere_face, e, sphere_sense);
        // The disk's outward normal points away from the material.
        let cap_normal = if keep_normal_side {
            -circle_frame.z()
        } else {
            circle_frame.z()
        };
        let cap = add_face(
            store,
            shell,
            Plane::new(Frame::new(circle_frame.origin(), cap_normal, circle_frame.x()).unwrap())
                .into(),
        );
        add_ring_loop(store, cap, e, sphere_sense.flipped());
        body
    }

    /// Volume of the spherical cap of height `h` on a sphere of radius `r`.
    fn cap_volume(r: f64, h: f64) -> f64 {
        core::f64::consts::PI * h * h * (r - h / 3.0)
    }

    #[test]
    fn cut_sphere_north_cap_is_watertight_with_correct_volume() {
        // Material above the cut: a polar cap containing the north pole
        // (single loop winding +1).
        let mut store = Store::new();
        let f = tilted();
        let (r, v_c) = (1.1, 0.35);
        let (sin_v, cos_v) = math::sincos(v_c);
        let circle_frame = Frame::new(f.origin() + f.z() * (r * sin_v), f.z(), f.x()).unwrap();
        let body = cut_sphere_body(
            &mut store,
            Sphere::new(f, r).unwrap(),
            circle_frame,
            r * cos_v,
            true,
        );
        let mesh = tessellate_body(&store, body, &opts(1e-3)).unwrap();
        assert_watertight(&mesh);
        let exact = cap_volume(r, r - r * sin_v);
        let vol = signed_volume(&mesh);
        assert!(vol > 0.0, "orientation must be outward");
        assert!(
            (vol - exact).abs() / exact < 0.015,
            "volume {vol} vs exact {exact}"
        );
    }

    #[test]
    fn cut_sphere_south_body_is_watertight_with_correct_volume() {
        // Material below the cut: the large portion containing the south
        // pole (single loop winding -1).
        let mut store = Store::new();
        let f = tilted();
        let (r, v_c) = (1.1, 0.35);
        let (sin_v, cos_v) = math::sincos(v_c);
        let circle_frame = Frame::new(f.origin() + f.z() * (r * sin_v), f.z(), f.x()).unwrap();
        let body = cut_sphere_body(
            &mut store,
            Sphere::new(f, r).unwrap(),
            circle_frame,
            r * cos_v,
            false,
        );
        let mesh = tessellate_body(&store, body, &opts(1e-3)).unwrap();
        assert_watertight(&mesh);
        let full = 4.0 / 3.0 * core::f64::consts::PI * r * r * r;
        let exact = full - cap_volume(r, r - r * sin_v);
        let vol = signed_volume(&mesh);
        assert!(
            (vol - exact).abs() / exact < 0.015,
            "volume {vol} vs exact {exact}"
        );
    }

    #[test]
    fn oblique_cut_sphere_cap_is_watertight() {
        // The cutting plane is oblique to the sphere's parameter frame, so
        // the cap's boundary loop is NOT a v = const iso-line; the sphere-
        // frame south pole lies inside the kept portion.
        let mut store = Store::new();
        let sphere_frame = Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let (r, z0) = (1.0, 0.3);
        let circle_frame = Frame::new(
            Point3::new(0.0, 0.0, z0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let body = cut_sphere_body(
            &mut store,
            Sphere::new(sphere_frame, r).unwrap(),
            circle_frame,
            (r * r - z0 * z0).sqrt(),
            false,
        );
        for tol in [1e-3, 1e-2] {
            let mesh = tessellate_body(&store, body, &opts(tol)).unwrap();
            assert_watertight(&mesh);
            let full = 4.0 / 3.0 * core::f64::consts::PI * r * r * r;
            let exact = full - cap_volume(r, r - z0);
            let vol = signed_volume(&mesh);
            assert!(
                (vol - exact).abs() / exact < 0.02,
                "tol {tol}: volume {vol} vs exact {exact}"
            );
        }
    }

    #[test]
    fn meridional_cut_sphere_is_watertight_on_both_sides() {
        // A great-circle cut through the parameter axis touches both poles.
        // Its trim loop acquires an artificial winding from singular pole
        // longitudes, but geometrically bounds a hemisphere on either side.
        let r = 1.1;
        let sphere = Sphere::new(tilted(), r).unwrap();
        let plane = Frame::new(
            sphere.frame().origin(),
            sphere.frame().x(),
            sphere.frame().y(),
        )
        .unwrap();
        let exact = 2.0 / 3.0 * core::f64::consts::PI * r * r * r;
        for keep_normal_side in [false, true] {
            let mut store = Store::new();
            let body = cut_sphere_body(&mut store, sphere, plane, r, keep_normal_side);
            let mesh = tessellate_body(&store, body, &opts(1e-3)).unwrap();
            assert_watertight(&mesh);
            let vol = signed_volume(&mesh);
            assert!(vol > 0.0, "orientation must be outward");
            assert!(
                (vol - exact).abs() / exact < 0.015,
                "side {keep_normal_side}: volume {vol} vs exact {exact}"
            );
        }
    }

    #[test]
    fn single_winding_loop_is_rejected_off_spheres() {
        // A cylinder side face missing its top loop cannot be capped.
        let mut store = Store::new();
        let f = Frame::world();
        let (body, shell) = solid_body_scaffold(&mut store);
        let e = ring_edge(
            &mut store,
            Circle::new(Frame::new(f.origin(), f.z(), f.x()).unwrap(), 1.0).unwrap(),
        );
        let side = add_face(&mut store, shell, Cylinder::new(f, 1.0).unwrap().into());
        add_ring_loop(&mut store, side, e, Sense::Forward);
        let err = tessellate_body(&store, body, &opts(1e-3)).unwrap_err();
        assert_eq!(
            err,
            TessellationError::Kernel(Error::InvalidGeometry {
                reason: "single-winding loop is only supported as a spherical polar cap",
            })
        );
    }

    #[test]
    fn block_coarse_mesh_is_watertight_and_exact() {
        let mut store = Store::new();
        let body = block(&mut store, &Frame::world(), [2.0, 3.0, 4.0]).unwrap();
        let mesh = tessellate_body(&store, body, &opts(1e-4)).unwrap();
        // Planar faces bounded by straight edges never refine.
        assert_eq!(mesh.positions.len(), 8);
        assert_eq!(mesh.triangles.len(), 12);
        assert_eq!(mesh.face_ranges.len(), 6);
        assert_eq!(mesh.edge_polylines.len(), 12);
        for (_, line) in &mesh.edge_polylines {
            assert_eq!(line.len(), 2);
        }
        assert_watertight(&mesh);
        assert!((signed_volume(&mesh) - 24.0).abs() < 1e-12);
    }

    #[test]
    fn tilted_block_is_watertight_with_outward_orientation() {
        let mut store = Store::new();
        let body = block(&mut store, &tilted(), [1.0, 2.0, 0.5]).unwrap();
        let mesh = tessellate_body(&store, body, &opts(1e-4)).unwrap();
        assert_watertight(&mesh);
        assert!((signed_volume(&mesh) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn cylinder_is_watertight_with_correct_volume() {
        let mut store = Store::new();
        let (r, h) = (0.7, 1.6);
        let body = cylinder_body(&mut store, &tilted(), r, h);
        let mesh = tessellate_body(&store, body, &opts(1e-3)).unwrap();
        assert_watertight(&mesh);
        let exact = core::f64::consts::PI * r * r * h;
        let vol = signed_volume(&mesh);
        assert!(vol > 0.0, "orientation must be outward");
        assert!(
            (vol - exact).abs() / exact < 0.01,
            "volume {vol} vs exact {exact}"
        );
        // Two ring edges, closed polylines (first index repeated last).
        assert_eq!(mesh.edge_polylines.len(), 2);
        for (_, line) in &mesh.edge_polylines {
            assert!(line.len() > 4);
            assert_eq!(line.first(), line.last());
        }
    }

    #[test]
    fn cone_frustum_is_watertight_with_correct_volume() {
        let mut store = Store::new();
        let (r0, alpha, h) = (0.8, 0.35, 1.2);
        let body = cone_body(&mut store, &tilted(), r0, alpha, h);
        let mesh = tessellate_body(&store, body, &opts(1e-3)).unwrap();
        assert_watertight(&mesh);
        let (sin_a, cos_a) = math::sincos(alpha);
        let r1 = r0 + (h / cos_a) * sin_a;
        let exact = core::f64::consts::PI * h / 3.0 * (r0 * r0 + r0 * r1 + r1 * r1);
        let vol = signed_volume(&mesh);
        assert!(
            (vol - exact).abs() / exact < 0.015,
            "volume {vol} vs exact {exact}"
        );
    }

    #[test]
    fn sphere_is_watertight_with_correct_volume() {
        let mut store = Store::new();
        let r = 0.9;
        let body = closed_body(&mut store, Sphere::new(tilted(), r).unwrap().into());
        let mesh = tessellate_body(&store, body, &opts(1e-3)).unwrap();
        assert_watertight(&mesh);
        let exact = 4.0 / 3.0 * core::f64::consts::PI * r * r * r;
        let vol = signed_volume(&mesh);
        assert!(
            (vol - exact).abs() / exact < 0.015,
            "volume {vol} vs exact {exact}"
        );
        assert_eq!(mesh.face_ranges.len(), 1);
        assert!(mesh.edge_polylines.is_empty());
    }

    #[test]
    fn torus_is_watertight_with_correct_volume() {
        let mut store = Store::new();
        let (rr, r) = (1.0, 0.35);
        let body = closed_body(&mut store, Torus::new(tilted(), rr, r).unwrap().into());
        let mesh = tessellate_body(&store, body, &opts(1e-3)).unwrap();
        assert_watertight(&mesh);
        let exact = 2.0 * core::f64::consts::PI * core::f64::consts::PI * rr * r * r;
        let vol = signed_volume(&mesh);
        assert!(
            (vol - exact).abs() / exact < 0.02,
            "volume {vol} vs exact {exact}"
        );
    }

    #[test]
    fn coarse_tolerances_still_produce_watertight_meshes() {
        // Very loose tolerances exercise the minimal seam/pole meshes.
        for tol in [3e-2, 1e-2] {
            let mut store = Store::new();
            let body = cylinder_body(&mut store, &Frame::world(), 1.0, 2.0);
            assert_watertight(&tessellate_body(&store, body, &opts(tol)).unwrap());

            let mut store = Store::new();
            let body = closed_body(&mut store, Sphere::new(Frame::world(), 1.0).unwrap().into());
            assert_watertight(&tessellate_body(&store, body, &opts(tol)).unwrap());

            let mut store = Store::new();
            let body = closed_body(
                &mut store,
                Torus::new(Frame::world(), 1.0, 0.4).unwrap().into(),
            );
            assert_watertight(&tessellate_body(&store, body, &opts(tol)).unwrap());
        }
    }

    #[test]
    fn tessellation_is_bitwise_deterministic() {
        let build = || {
            let mut store = Store::new();
            let body = cylinder_body(&mut store, &tilted(), 0.7, 1.6);
            tessellate_body(&store, body, &opts(1e-3)).unwrap()
        };
        let (m1, m2) = (build(), build());
        assert_eq!(m1.triangles, m2.triangles);
        let bits = |m: &BodyMesh| -> Vec<[u64; 3]> {
            m.positions
                .iter()
                .map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
                .collect()
        };
        assert_eq!(bits(&m1), bits(&m2), "positions must be bit-identical");
    }

    #[test]
    fn max_edge_len_is_respected_on_edges() {
        let mut store = Store::new();
        let body = block(&mut store, &Frame::world(), [4.0, 4.0, 4.0]).unwrap();
        let mesh = tessellate_body(
            &store,
            body,
            &TessOptions {
                chord_tol: 1e-4,
                max_edge_len: Some(1.0),
            },
        )
        .unwrap();
        assert_watertight(&mesh);
        for t in &mesh.triangles {
            for (i, j) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                let len = mesh.positions[i as usize].dist(mesh.positions[j as usize]);
                assert!(len <= 1.0 + 1e-9, "edge length {len} exceeds cap");
            }
        }
        assert!((signed_volume(&mesh) - 64.0).abs() < 1e-9);
    }

    #[test]
    fn obj_export_lists_vertices_and_faces() {
        let mut store = Store::new();
        let body = block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        let mesh = tessellate_body(&store, body, &opts(1e-4)).unwrap();
        let obj = mesh.to_obj();
        assert_eq!(obj.lines().filter(|l| l.starts_with("v ")).count(), 8);
        assert_eq!(obj.lines().filter(|l| l.starts_with("f ")).count(), 12);
    }

    #[test]
    fn check_watertight_reports_holes_and_degenerates() {
        let mut store = Store::new();
        let body = block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        let mesh = tessellate_body(&store, body, &opts(1e-4)).unwrap();
        let mut holed = mesh.clone();
        holed.triangles.pop();
        assert!(!check_watertight(&holed).is_empty());
        let mut degenerate = mesh.clone();
        degenerate.triangles[0] = [0, 0, 1];
        assert!(!check_watertight(&degenerate).is_empty());
    }

    #[test]
    fn invalid_inputs_are_rejected() {
        let mut store = Store::new();
        let body = block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        assert!(tessellate_body(&store, body, &opts(0.0)).is_err());
        assert!(tessellate_body(&store, body, &opts(f64::NAN)).is_err());
        assert!(
            tessellate_body(
                &store,
                body,
                &TessOptions {
                    chord_tol: 1e-4,
                    max_edge_len: Some(-1.0),
                },
            )
            .is_err()
        );
        // A faceless body cannot be tessellated.
        let (empty, _) = solid_body_scaffold(&mut store);
        assert!(tessellate_body(&store, empty, &opts(1e-4)).is_err());
    }

    #[test]
    fn ring_edge_polylines_are_shared_between_faces() {
        // The bottom ring's vertex indices must appear in both the side
        // face's and the bottom cap's triangles — shared by index, not by
        // position.
        let mut store = Store::new();
        let body = cylinder_body(&mut store, &Frame::world(), 1.0, 2.0);
        let mesh = tessellate_body(&store, body, &opts(1e-3)).unwrap();
        let (_, ring) = &mesh.edge_polylines[0];
        let ring_set: std::collections::BTreeSet<u32> = ring.iter().copied().collect();
        let mut faces_using = 0;
        for (_, range) in &mesh.face_ranges {
            let uses = mesh.triangles[range.clone()]
                .iter()
                .any(|t| t.iter().any(|i| ring_set.contains(i)));
            if uses {
                faces_using += 1;
            }
        }
        assert_eq!(faces_using, 2, "ring edge must stitch exactly two faces");
    }
}
