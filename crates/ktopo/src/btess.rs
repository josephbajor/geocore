//! Whole-body tessellation: one watertight mesh per body.
//!
//! The crack-elimination contract (spec §L2, M2 exit criterion):
//!
//! 1. Every topological **edge is discretized exactly once** into a 3D
//!    polyline (chordal refinement of its curve), producing shared mesh
//!    vertices.
//! 2. Each face builds its UV trim loops from those *frozen* edge
//!    polylines (surface-inverting each polyline point; periodic surfaces
//!    are seam-cut, zero-loop closed faces get full-period rectangles with
//!    seam and pole rows welded by index).
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
//! chain sample near the half period.

use crate::entity::{BodyId, Edge, EdgeId, FaceId, Sense, SurfaceId, VertexId};
use crate::geom::SurfaceGeom;
use crate::store::Store;
use kcore::error::{Error, Result};
use kcore::math;
use kgeom::curve::Curve;
use kgeom::param::wrap_periodic;
use kgeom::surface::Surface;
pub use kgeom::tess::TessOptions;
use kgeom::tess::{TrimLoop, TrimmedSurface, tessellate};
use kgeom::vec::{Point3, Vec2};

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
    fn push(&mut self, p: Point3) -> u32 {
        let i = u32::try_from(self.positions.len()).expect("mesh exceeded u32 vertex capacity");
        self.positions.push(p);
        i
    }
    fn pos(&self, gid: u32) -> Point3 {
        self.positions[gid as usize]
    }
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

/// Invert a point known to lie on the surface to UV coordinates, with
/// periodic parameters wrapped into the surface's base range.
fn invert_uv(sg: &SurfaceGeom, p: Point3) -> Result<Vec2> {
    let tau = core::f64::consts::TAU;
    Ok(match sg {
        SurfaceGeom::Plane(s) => {
            let l = s.frame().to_local(p);
            Vec2::new(l.x, l.y)
        }
        SurfaceGeom::Cylinder(s) => {
            let l = s.frame().to_local(p);
            Vec2::new(wrap_periodic(math::atan2(l.y, l.x), 0.0, tau), l.z)
        }
        SurfaceGeom::Cone(s) => {
            let l = s.frame().to_local(p);
            let u = wrap_periodic(math::atan2(l.y, l.x), 0.0, tau);
            // P_z = v cos α is exact and linear in v.
            Vec2::new(u, l.z / math::cos(s.half_angle()))
        }
        SurfaceGeom::Sphere(s) => {
            let l = s.frame().to_local(p);
            let u = wrap_periodic(math::atan2(l.y, l.x), 0.0, tau);
            Vec2::new(u, math::atan2(l.z, (l.x * l.x + l.y * l.y).sqrt()))
        }
        SurfaceGeom::Torus(s) => {
            let l = s.frame().to_local(p);
            let u = wrap_periodic(math::atan2(l.y, l.x), 0.0, tau);
            let rho = (l.x * l.x + l.y * l.y).sqrt();
            let v = wrap_periodic(math::atan2(l.z, rho - s.major_radius()), 0.0, tau);
            Vec2::new(u, v)
        }
        SurfaceGeom::Nurbs(s) => {
            let [ur, vr] = s.param_range();
            let proj = kgeom::project::project_to_surface(s, p, [ur, vr]).ok_or(
                Error::InvalidGeometry {
                    reason: "closest-point projection onto NURBS surface failed",
                },
            )?;
            Vec2::new(proj.uv[0], proj.uv[1])
        }
    })
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

/// Edge-polyline refinement: split until the curve chord criterion *and*
/// kgeom's boundary criterion against every adjacent face surface hold
/// with margin.
struct CurveRefine<'a> {
    curve: &'a dyn Curve,
    surfs: Vec<&'a SurfaceGeom>,
    ctx: Ctx,
}

impl CurveRefine<'_> {
    fn needs_split(&self, a: (f64, Point3), b: (f64, Point3)) -> Result<bool> {
        if a.1.dist(b.1) > self.ctx.max_len {
            return Ok(true);
        }
        let mid = self.curve.eval((a.0 + b.0) / 2.0);
        if point_seg_dist(mid, a.1, b.1) > self.ctx.tol {
            return Ok(true);
        }
        for sg in &self.surfs {
            let ua = invert_uv(sg, a.1)?;
            let ub = unwrap_near(invert_uv(sg, b.1)?, ua, sg.as_surface().periodicity());
            let um = (ua + ub) / 2.0;
            let q = sg.as_surface().eval([um.x, um.y]);
            if point_seg_dist(q, a.1, b.1) > self.ctx.tol {
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// Append the interior refinement points of `(a, b)` (exclusive).
    fn refine(
        &self,
        a: (f64, Point3),
        b: (f64, Point3),
        depth: usize,
        out: &mut Vec<(f64, Point3)>,
    ) -> Result<()> {
        if depth >= MAX_DEPTH || !self.needs_split(a, b)? {
            return Ok(());
        }
        let tm = (a.0 + b.0) / 2.0;
        let m = (tm, self.curve.eval(tm));
        self.refine(a, m, depth + 1, out)?;
        out.push(m);
        self.refine(m, b, depth + 1, out)
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
) {
    if depth >= MAX_DEPTH {
        return;
    }
    let mid_uv = (a.0 + b.0) / 2.0;
    let mid_p = s.eval([mid_uv.x, mid_uv.y]);
    if point_seg_dist(mid_p, a.1, b.1) <= ctx.tol && a.1.dist(b.1) <= ctx.max_len {
        return;
    }
    let m = (mid_uv, mid_p);
    refine_uv_seg(s, a, m, ctx, depth + 1, out);
    out.push(m);
    refine_uv_seg(s, m, b, ctx, depth + 1, out);
}

/// An iso/seam arc: UV points with their global vertex ids, endpoints
/// included.
type Arc = Vec<(Vec2, u32)>;

/// Build an arc between two existing global vertices by refining the
/// straight UV segment; interior points become fresh global vertices.
fn iso_arc(s: &dyn Surface, a: (Vec2, u32), b: (Vec2, u32), acc: &mut MeshAcc, ctx: Ctx) -> Arc {
    let mut interior = Vec::new();
    refine_uv_seg(
        s,
        (a.0, acc.pos(a.1)),
        (b.0, acc.pos(b.1)),
        ctx,
        0,
        &mut interior,
    );
    let mut arc = Vec::with_capacity(interior.len() + 2);
    arc.push(a);
    for (uv, p) in interior {
        let gid = acc.push(p);
        arc.push((uv, gid));
    }
    arc.push(b);
    arc
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

/// The discretized edges of a body, parallel to [`Store::edges_of_body`].
type EdgeLines = Vec<(EdgeId, Vec<u32>)>;

fn find_eline(elines: &EdgeLines, edge: EdgeId) -> Result<&Vec<u32>> {
    elines
        .iter()
        .find(|(e, _)| *e == edge)
        .map(|(_, l)| l)
        .ok_or(Error::StaleHandle)
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
) -> Result<Vec<u32>> {
    let e: &Edge = store.get(edge)?;
    let curve_id = e.curve.ok_or(Error::InvalidGeometry {
        reason: "edge has no curve geometry attached",
    })?;
    let cg = store.get(curve_id)?;
    let c = cg.as_curve();

    // Parameter interval: explicit bounds, or one full period for a ring.
    let (t0, t1) = match e.bounds {
        Some((a, b)) => {
            if !(a.is_finite() && b.is_finite() && a < b) {
                return Err(Error::InvalidGeometry {
                    reason: "edge bounds are not a finite increasing interval",
                });
            }
            (a, b)
        }
        None => {
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
            .ok_or(Error::StaleHandle)
    };
    let (g_start, g_end, closed) = match e.vertices {
        [Some(v0), Some(v1)] => (vgid(v0)?, vgid(v1)?, v0 == v1),
        [None, None] => {
            let g = acc.push(c.eval(t0));
            (g, g, true)
        }
        _ => {
            return Err(Error::InvalidGeometry {
                reason: "edge has exactly one vertex",
            });
        }
    };

    // Adjacent face surfaces (deduplicated, deterministic order): the
    // crack-prevention rule refines against each of them.
    let mut surf_ids: Vec<SurfaceId> = Vec::new();
    for &fin in &e.fins {
        let lp = store.get(fin)?.parent;
        let face = store.get(store.get(lp)?.face)?;
        if !surf_ids.contains(&face.surface) {
            surf_ids.push(face.surface);
        }
    }
    let mut surfs = Vec::with_capacity(surf_ids.len());
    for sid in surf_ids {
        surfs.push(store.get(sid)?);
    }
    let refine = CurveRefine {
        curve: c,
        surfs,
        ctx,
    };

    // Seed: closed polylines start from quarter points (their full-span
    // chord is degenerate); open ones from the single endpoint chord.
    let mut seed: Vec<(f64, Point3)> = Vec::new();
    seed.push((t0, acc.pos(g_start)));
    if closed {
        for k in 1..4 {
            let t = t0 + (t1 - t0) * f64::from(k) / 4.0;
            seed.push((t, c.eval(t)));
        }
    }
    seed.push((t1, acc.pos(g_end)));

    let mut gids: Vec<u32> = vec![g_start];
    for w in seed.windows(2) {
        let mut interior = Vec::new();
        refine.refine(w[0], w[1], 0, &mut interior)?;
        for (_, p) in interior {
            gids.push(acc.push(p));
        }
        // Segment end: a seed interior point gets a fresh vertex; the
        // final endpoint reuses its anchor id.
        if w[1].0 < t1 {
            gids.push(acc.push(w[1].1));
        }
    }
    gids.push(g_end);
    Ok(gids)
}

/// Assemble the oriented vertex-id chain of one loop by concatenating its
/// fins' edge polylines (each contributes all but its final point). When
/// `reverse` is set the whole traversal is flipped (used to normalize
/// reversed-sense faces so their loops read counterclockwise in UV).
fn loop_chain(
    store: &Store,
    elines: &EdgeLines,
    lp: crate::entity::LoopId,
    reverse: bool,
) -> Result<Vec<u32>> {
    let fins = &store.get(lp)?.fins;
    if fins.is_empty() {
        return Err(Error::InvalidGeometry {
            reason: "loop has no fins",
        });
    }
    let mut chain: Vec<u32> = Vec::new();
    let ordered: Vec<_> = if reverse {
        fins.iter().rev().copied().collect()
    } else {
        fins.to_vec()
    };
    for fin_id in ordered {
        let fin = store.get(fin_id)?;
        let line = find_eline(elines, fin.edge)?;
        let forward = fin.sense.is_forward() != reverse;
        if forward {
            chain.extend_from_slice(&line[..line.len() - 1]);
        } else {
            chain.extend(line.iter().rev().take(line.len() - 1));
        }
    }
    Ok(chain)
}

/// Map an id chain into the face surface's UV space with periodic
/// continuity, and measure its winding.
fn chain_uv(sg: &SurfaceGeom, acc: &MeshAcc, ids: &[u32]) -> Result<UvChain> {
    let s = sg.as_surface();
    let per = s.periodicity();
    let mut uvs: Vec<Vec2> = Vec::with_capacity(ids.len());
    for &gid in ids {
        let raw = invert_uv(sg, acc.pos(gid))?;
        let uv = match uvs.last() {
            Some(&prev) => unwrap_near(raw, prev, per),
            None => raw,
        };
        uvs.push(uv);
    }
    let close_raw = invert_uv(sg, acc.pos(ids[0]))?;
    let close_uv = unwrap_near(close_raw, *uvs.last().expect("non-empty chain"), per);
    let wind = |d: f64, p: Option<f64>| p.map_or(0, |p| (d / p).round() as i64);
    let winding = [
        wind(close_uv.x - uvs[0].x, per[0]),
        wind(close_uv.y - uvs[0].y, per[1]),
    ];
    Ok(UvChain {
        ids: ids.to_vec(),
        uvs,
        close_uv,
        winding,
    })
}

/// Run kgeom's face tessellator over prepared UV loops and splice the
/// result into the body mesh: boundary vertices map to the pre-assigned
/// global ids, interior vertices become fresh ones, and triangles are
/// flipped when the face sense is reversed. Triangles that degenerate
/// under welding (sphere pole collapse) are dropped.
fn run_kgeom(
    s: &dyn Surface,
    loops_pts: Vec<Vec<Vec2>>,
    loops_ids: &[Vec<u32>],
    flip: bool,
    acc: &mut MeshAcc,
    opts: &TessOptions,
) -> Result<Vec<[u32; 3]>> {
    let mut trim_loops = Vec::with_capacity(loops_pts.len());
    for pts in loops_pts {
        trim_loops.push(TrimLoop::new(pts)?);
    }
    let face = TrimmedSurface::new(s, trim_loops)?;
    let fm = tessellate(&face, opts)?;

    if fm.boundary.len() != loops_ids.len() {
        return Err(Error::InvalidGeometry {
            reason: "internal: face boundary loop count mismatch",
        });
    }
    let mut l2g: Vec<Option<u32>> = vec![None; fm.positions.len()];
    for (bl, ids) in fm.boundary.iter().zip(loops_ids) {
        if bl.len() != ids.len() {
            // kgeom inserted a boundary vertex despite the margin rule:
            // that would be a cross-face crack, so fail loudly.
            return Err(Error::InvalidGeometry {
                reason: "internal: boundary refinement mismatch (potential crack)",
            });
        }
        for (&li, &gid) in bl.iter().zip(ids) {
            l2g[li as usize] = Some(gid);
        }
    }
    let l2g: Vec<u32> = l2g
        .into_iter()
        .enumerate()
        .map(|(li, g)| g.unwrap_or_else(|| acc.push(fm.positions[li])))
        .collect();

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
            });
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
    run_kgeom(s, loops_pts, &loops_ids, flip, acc, opts)
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
) -> Result<Vec<[u32; 3]>> {
    let s = sg.as_surface();
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
                });
            }
        }
    }
    let (bottom, top) = match (bottom, top) {
        (Some(b), Some(t)) => (b, t),
        // A single winding loop bounds a polar cap: the missing second
        // boundary is the pole contained in the face.
        (Some(c), None) | (None, Some(c)) => {
            return face_case_cap(sg, c, holes, flip, acc, opts, ctx);
        }
        (None, None) => {
            return Err(Error::InvalidGeometry {
                reason: "seam-cut face needs one +1 and one -1 winding loop",
            });
        }
    };
    let mean_v = |c: &UvChain| c.uvs.iter().map(|p| p.y).sum::<f64>() / c.uvs.len() as f64;
    if mean_v(&top) <= mean_v(&bottom) {
        return Err(Error::InvalidGeometry {
            reason: "seam-cut face has its winding loops on the wrong sides",
        });
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
    );

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
    run_kgeom(s, loops_pts, &loops_ids, flip, acc, opts)
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
    flip: bool,
    acc: &mut MeshAcc,
    opts: &TessOptions,
    ctx: Ctx,
) -> Result<Vec<[u32; 3]>> {
    let SurfaceGeom::Sphere(sp) = sg else {
        // Cylinders and tori have no point that can close a single
        // winding loop; cone apex caps are deferred until full cones land.
        return Err(Error::InvalidGeometry {
            reason: "single-winding loop is only supported as a spherical polar cap",
        });
    };
    let s = sg.as_surface();
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
        });
    }
    let pole_v = if w > 0 { half } else { -half };
    if cuvs
        .iter()
        .any(|uv| if w > 0 { uv.y >= half } else { uv.y <= -half })
    {
        return Err(Error::InvalidGeometry {
            reason: "cap boundary loop touches the pole",
        });
    }

    let g_pole = acc.push(s.eval([cuvs[0].x, pole_v]));
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
        });
    }

    // Pole rows: uniform samples between two seam longitudes, all welded
    // to the single pole vertex (density from the equator sagitta, like
    // the closed-sphere case).
    let r = sp.radius();
    let mut theta = (8.0 * ctx.tol / r).sqrt().min(half);
    if ctx.max_len.is_finite() {
        theta = theta.min(ctx.max_len / r);
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
        let m0 = iso_arc(s, (cuvs[0], cids[0]), pole_at(cuvs[0].x), acc, ctx);
        let mk = iso_arc(s, (cuvs[k], cids[k]), pole_at(cuvs[k].x), acc, ctx);
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
        let m0 = iso_arc(s, pole_at(cuvs[0].x), (cuvs[0], cids[0]), acc, ctx);
        let mk = iso_arc(s, pole_at(cuvs[k].x), (cuvs[k], cids[k]), acc, ctx);
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
        tris.extend(run_kgeom(s, loops_pts, &loops_ids, flip, acc, opts)?);
    }
    Ok(tris)
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
) -> Result<Vec<[u32; 3]>> {
    let pi = core::f64::consts::PI;
    let tau = core::f64::consts::TAU;
    let s = sg.as_surface();
    let mut tris = Vec::new();
    match sg {
        SurfaceGeom::Sphere(sp) => {
            let half = core::f64::consts::FRAC_PI_2;
            let g_s = acc.push(s.eval([0.0, -half]));
            let g_n = acc.push(s.eval([0.0, half]));
            // Meridian arcs at u = 0 and u = π; u = 2π reuses the first.
            let meridian = |u: f64, acc: &mut MeshAcc| {
                iso_arc(
                    s,
                    (Vec2::new(u, -half), g_s),
                    (Vec2::new(u, half), g_n),
                    acc,
                    ctx,
                )
            };
            let m0 = meridian(0.0, acc);
            let m1 = meridian(pi, acc);
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
                tris.extend(run_kgeom(s, vec![pts], &[ids], flip, acc, opts)?);
            }
        }
        SurfaceGeom::Torus(_) => {
            // Corner vertices at half-period grid points.
            let corner = |i: usize, j: usize| [pi * i as f64, pi * j as f64];
            let mut g = [[0u32; 2]; 2];
            for (i, gi) in g.iter_mut().enumerate() {
                for (j, gij) in gi.iter_mut().enumerate() {
                    let [u, v] = corner(i, j);
                    *gij = acc.push(s.eval([u, v]));
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
                    row.push(iso_arc(s, at(i, j), at(i + 1, j), acc, ctx));
                }
                au.push(row);
            }
            for j in 0..2 {
                let mut col = Vec::new();
                for i in 0..2 {
                    col.push(iso_arc(s, at(i, j), at(i, j + 1), acc, ctx));
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
                    tris.extend(run_kgeom(s, vec![pts], &[ids], flip, acc, opts)?);
                }
            }
        }
        _ => {
            return Err(Error::InvalidGeometry {
                reason: "zero-loop face on a surface that is not closed",
            });
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
) -> Result<Vec<[u32; 3]>> {
    let face = store.get(face_id)?;
    let sg = store.get(face.surface)?;
    let flip = face.sense == Sense::Reversed;

    if face.loops.is_empty() {
        return face_case_c(sg, flip, acc, opts, ctx);
    }
    let mut chains = Vec::with_capacity(face.loops.len());
    for &lp in &face.loops {
        let ids = loop_chain(store, elines, lp, flip)?;
        chains.push(chain_uv(sg, acc, &ids)?);
    }
    if chains.iter().all(|c| c.winding == [0, 0]) {
        face_case_a(sg.as_surface(), chains, flip, acc, opts)
    } else {
        face_case_b(sg, chains, flip, acc, opts, ctx)
    }
}

/// Tessellate a body into one watertight mesh (see module docs).
pub fn tessellate_body(store: &Store, body: BodyId, opts: &TessOptions) -> Result<BodyMesh> {
    if !opts.chord_tol.is_finite() || opts.chord_tol <= 0.0 {
        return Err(Error::InvalidTolerance {
            value: opts.chord_tol,
        });
    }
    if let Some(l) = opts.max_edge_len
        && (!l.is_finite() || l <= 0.0)
    {
        return Err(Error::InvalidTolerance { value: l });
    }
    let ctx = Ctx {
        tol: opts.chord_tol * MARGIN,
        max_len: opts.max_edge_len.unwrap_or(f64::INFINITY) * MARGIN,
    };

    let faces = store.faces_of_body(body)?;
    if faces.is_empty() {
        return Err(Error::InvalidGeometry {
            reason: "body has no faces to tessellate",
        });
    }

    let mut acc = MeshAcc {
        positions: Vec::new(),
    };
    // One global vertex per topological vertex.
    let mut vgids: Vec<(VertexId, u32)> = Vec::new();
    for v in store.vertices_of_body(body)? {
        let gid = acc.push(store.vertex_position(v)?);
        vgids.push((v, gid));
    }
    // Every edge discretized exactly once.
    let mut elines: EdgeLines = Vec::new();
    for e in store.edges_of_body(body)? {
        let line = discretize_edge(store, e, &vgids, &mut acc, ctx)?;
        elines.push((e, line));
    }
    // Faces, assembled by index mapping.
    let mut triangles: Vec<[u32; 3]> = Vec::new();
    let mut face_ranges = Vec::with_capacity(faces.len());
    for face in faces {
        let start = triangles.len();
        triangles.extend(tess_face(store, &elines, &mut acc, face, opts, ctx)?);
        face_ranges.push((face, start..triangles.len()));
    }
    Ok(BodyMesh {
        positions: acc.positions,
        triangles,
        face_ranges,
        edge_polylines: elines,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::entity::{Face, Fin, Loop, ShellId};
    use crate::geom::CurveGeom;
    use crate::make::{block, solid_body_scaffold};
    use kgeom::curve::Circle;
    use kgeom::frame::Frame;
    use kgeom::surface::{Cone, Cylinder, Plane, Sphere, Torus};
    use kgeom::vec::Vec3;

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
        let curve = store.add(CurveGeom::Circle(circle));
        store.add(Edge {
            curve: Some(curve),
            vertices: [None, None],
            bounds: None,
            fins: Vec::new(),
            tolerance: None,
        })
    }

    fn add_face(store: &mut Store, shell: ShellId, surface: SurfaceGeom) -> FaceId {
        let surface = store.add(surface);
        let face = store.add(Face {
            shell,
            loops: Vec::new(),
            surface,
            sense: Sense::Forward,
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
            Error::InvalidGeometry {
                reason: "single-winding loop is only supported as a spherical polar cap",
            }
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
