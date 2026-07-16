//! Tolerance-driven tessellation of (trimmed) faces.
//!
//! M1 scope: a single face — a surface plus polygonal parameter-space trim
//! loops — is triangulated so that the mesh is **watertight with its own
//! boundary**: the boundary edges of the output triangulation are exactly
//! the tolerance-refined trim-loop segments, in loop order. Cross-face crack
//! elimination (shared-edge vertex agreement between faces) is layered on
//! top in M2 using [`FaceMesh::boundary`].
//!
//! Pipeline:
//! 1. **Boundary refinement** — every loop edge is recursively midpoint-split
//!    (in parameter space) until the 3D chordal deviation of the mapped
//!    midpoint from the 3D chord is within tolerance. The resulting boundary
//!    vertices are frozen; no later stage inserts or removes boundary points.
//! 2. **Triangulation** — holes are joined to the outer loop with visibility
//!    bridges, then the merged polygon is ear-clipped. All orientation
//!    decisions route through [`kcore::predicates::orient2d`].
//! 3. **Interior refinement** — interior edges whose mapped midpoint deviates
//!    from their 3D chord by more than tolerance are split at their parameter
//!    midpoint; splits are applied per edge (shared by both adjacent
//!    triangles), so the mesh stays conforming — no T-junctions. Boundary
//!    edges are never split (they already satisfy the tolerance).
//!
//! Triangle *quality* (aspect ratio) is explicitly not optimized in M1; the
//! guarantees are chordal accuracy, watertightness, and determinism.

use crate::param::ParamRange;
use crate::surface::Surface;
use crate::vec::{Point3, Vec2};
use kcore::error::{Error, Result};
use kcore::operation::{
    AccountingMode, ChildWorkLedger, DiagnosticKind, ExecutionPolicy, LimitSnapshot,
    NumericalPolicy, OperationContext, OperationOutcome, OperationPolicyError, OperationScope,
    PolicyVersion, ResourceKind, SequentialWorkLedger, SessionPolicy, SessionPrecision,
};
use kcore::predicates::{Orientation, orient2d, polygon_orientation2d_iter};
use kcore::tolerance::Tolerances;
use std::collections::{BTreeMap, BTreeSet};

mod policy;

pub use policy::{
    FACE_TESSELLATION_BOUNDARY_DEPTH, FACE_TESSELLATION_BOUNDARY_DEPTH_LIMIT,
    FACE_TESSELLATION_BOUNDARY_SPLIT_LIMIT, FACE_TESSELLATION_BOUNDARY_SPLITS,
    FACE_TESSELLATION_MESH_TRIANGLE_LIMIT, FACE_TESSELLATION_MESH_TRIANGLES,
    FACE_TESSELLATION_MESH_VERTEX_LIMIT, FACE_TESSELLATION_MESH_VERTICES,
    FACE_TESSELLATION_REFINEMENT_PASS_LIMIT, FACE_TESSELLATION_REFINEMENT_PASSES,
    FACE_TESSELLATION_U32_ITEM_LIMIT, FaceTessellationBudgetProfile,
};

/// A closed polygonal trim loop in a surface's parameter space.
///
/// The polygon is implicitly closed (last vertex connects to first). The
/// outer loop of a face winds counterclockwise; hole loops wind clockwise,
/// so that the face interior is always to the left of the traversal.
#[derive(Debug, Clone, PartialEq)]
pub struct TrimLoop {
    /// Loop vertices in traversal order; consecutive duplicates are removed
    /// on construction.
    pub points: Vec<Vec2>,
}

impl TrimLoop {
    /// Return the number of points retained by [`TrimLoop::new`] without
    /// allocating its cleaned copy.
    ///
    /// Higher-level accounted builders use this inspection seam to admit the
    /// exact cleaned-copy length before construction. Validation and exact
    /// orientation decisions deliberately match [`TrimLoop::new`].
    pub fn cleaned_point_count(points: &[Vec2]) -> Result<usize> {
        let mut first = None;
        let mut previous = None;
        let mut count = 0_usize;
        for &point in points {
            if !point.x.is_finite() || !point.y.is_finite() {
                return Err(Error::InvalidGeometry {
                    reason: "trim loop vertex is not finite",
                });
            }
            if previous == Some(point) {
                continue;
            }
            if previous.is_none() {
                first = Some(point);
            }
            previous = Some(point);
            count = count.checked_add(1).ok_or(Error::InvalidGeometry {
                reason: "trim loop point count exceeds platform capacity",
            })?;
        }
        if count > 1 && first == previous {
            count -= 1;
        }
        if count < 3 {
            return Err(Error::InvalidGeometry {
                reason: "trim loop needs at least 3 distinct vertices",
            });
        }
        if polygon_orientation2d_iter(points.iter().map(|point| [point.x, point.y]))
            == Orientation::Zero
        {
            return Err(Error::InvalidGeometry {
                reason: "trim loop has zero area",
            });
        }
        Ok(count)
    }

    /// Build a loop from vertices, removing consecutive (and closing)
    /// duplicate points. Fails if fewer than 3 distinct vertices remain or
    /// the signed area is degenerate (zero).
    pub fn new(points: Vec<Vec2>) -> Result<TrimLoop> {
        let cleaned_count = Self::cleaned_point_count(&points)?;
        let mut cleaned: Vec<Vec2> = Vec::with_capacity(cleaned_count);
        for p in points {
            if cleaned.len() < cleaned_count && cleaned.last() != Some(&p) {
                cleaned.push(p);
            }
        }
        debug_assert_eq!(cleaned.len(), cleaned_count);
        let l = TrimLoop { points: cleaned };
        debug_assert_ne!(trim_loop_orientation(&l), Orientation::Zero);
        Ok(l)
    }

    /// Rounded shoelace signed area for reporting and magnitude estimates.
    ///
    /// Its rounded sign must not decide winding or degeneracy; those decisions
    /// use [`polygon_orientation2d_iter`].
    pub fn signed_area(&self) -> f64 {
        let n = self.points.len();
        let mut a = 0.0;
        for i in 0..n {
            let p = self.points[i];
            let q = self.points[(i + 1) % n];
            a += p.cross(q);
        }
        a / 2.0
    }
}

/// A surface with polygonal parameter-space trim loops: the M1 stand-in for
/// a topological face.
pub struct TrimmedSurface<'a> {
    surface: &'a dyn Surface,
    loops: Vec<TrimLoop>,
}

impl<'a> TrimmedSurface<'a> {
    /// Build a trimmed surface. The first loop is the outer boundary and
    /// must wind counterclockwise; any further loops are holes, must wind
    /// clockwise, and must lie inside the outer loop's bounding box.
    pub fn new(surface: &'a dyn Surface, loops: Vec<TrimLoop>) -> Result<TrimmedSurface<'a>> {
        let Some(outer) = loops.first() else {
            return Err(Error::InvalidGeometry {
                reason: "trimmed surface needs at least one loop",
            });
        };
        if trim_loop_orientation(outer) != Orientation::Positive {
            return Err(Error::InvalidGeometry {
                reason: "outer trim loop must wind counterclockwise",
            });
        }
        let outer_bb = crate::aabb::Aabb2::from_points(&outer.points);
        for hole in &loops[1..] {
            if trim_loop_orientation(hole) != Orientation::Negative {
                return Err(Error::InvalidGeometry {
                    reason: "hole trim loops must wind clockwise",
                });
            }
            if !hole.points.iter().all(|&p| outer_bb.contains(p)) {
                return Err(Error::InvalidGeometry {
                    reason: "hole trim loop lies outside the outer loop",
                });
            }
        }
        Ok(TrimmedSurface { surface, loops })
    }

    /// The natural rectangular trim of an untrimmed patch over finite
    /// parameter ranges.
    pub fn rectangle(
        surface: &'a dyn Surface,
        range: [ParamRange; 2],
    ) -> Result<TrimmedSurface<'a>> {
        if !range[0].is_finite() || !range[1].is_finite() {
            return Err(Error::InvalidGeometry {
                reason: "rectangular trim requires finite parameter ranges",
            });
        }
        let l = TrimLoop::new(vec![
            Vec2::new(range[0].lo, range[1].lo),
            Vec2::new(range[0].hi, range[1].lo),
            Vec2::new(range[0].hi, range[1].hi),
            Vec2::new(range[0].lo, range[1].hi),
        ])?;
        TrimmedSurface::new(surface, vec![l])
    }

    /// The underlying surface.
    pub fn surface(&self) -> &dyn Surface {
        self.surface
    }

    /// The trim loops (outer first, then holes).
    pub fn loops(&self) -> &[TrimLoop] {
        &self.loops
    }
}

fn trim_loop_orientation(loop_: &TrimLoop) -> Orientation {
    polygon_orientation2d_iter(loop_.points.iter().map(|point| [point.x, point.y]))
}

/// Tessellation controls.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TessOptions {
    /// Maximum allowed 3D distance between the mesh and the surface,
    /// measured at edge midpoints and triangle centroids. Meters.
    pub chord_tol: f64,
    /// Optional maximum 3D edge length. Applied during boundary refinement
    /// and interior refinement alike.
    pub max_edge_len: Option<f64>,
}

impl Default for TessOptions {
    fn default() -> Self {
        TessOptions {
            chord_tol: 1e-4,
            max_edge_len: None,
        }
    }
}

/// A face tessellation: vertices in both parameter and model space, CCW
/// (in parameter space) triangles, and the boundary vertex indices.
#[derive(Debug, Clone, PartialEq)]
pub struct FaceMesh {
    /// Vertex positions in model space (`positions[i]` maps `uvs[i]`).
    pub positions: Vec<Point3>,
    /// Vertex parameters.
    pub uvs: Vec<Vec2>,
    /// Triangles as vertex-index triples, counterclockwise in parameter
    /// space.
    pub triangles: Vec<[u32; 3]>,
    /// Per trim loop, the refined boundary vertex indices in traversal
    /// order. Consecutive pairs (wrapping) are exactly the boundary edges
    /// of the triangulation; M2 face stitching consumes this.
    pub boundary: Vec<Vec<u32>>,
}

/// Refinement pass cap; each pass halves offending edges so this bounds
/// boundary-to-interior resolution ratios of about 2^24.
const MAX_REFINE_PASSES: usize = 24;
/// Hard triangle-count backstop; hitting it returns an error.
const MAX_TRIANGLES: usize = 200_000;
/// Recursion cap for boundary edge refinement (2^16 segments per edge).
const MAX_BOUNDARY_DEPTH: usize = 16;

/// Tessellate a trimmed face to the requested tolerance.
///
/// The returned mesh is deterministic (bit-identical across runs and
/// platforms) and watertight with its refined boundary: an edge is used by
/// exactly one triangle iff it is a consecutive pair in
/// [`FaceMesh::boundary`].
pub fn tessellate(face: &TrimmedSurface<'_>, opts: &TessOptions) -> Result<FaceMesh> {
    let session = SessionPolicy::new(
        SessionPrecision::parasolid(),
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        FaceTessellationBudgetProfile::v1_defaults(),
        PolicyVersion::V1,
    );
    let context = OperationContext::new(&session, Tolerances::default())
        .expect("validated default tolerances satisfy v1 session precision");
    tessellate_with_context(face, opts, &context)
        .expect("built-in v1 face-tessellation policy is valid")
        .into_result()
        .map_err(legacy_tessellation_error)
}

/// Tessellate a trimmed face with deterministic resource accounting.
///
/// Family defaults fill tessellation stages omitted by the caller. Matching
/// session entries override those defaults, and explicit request overrides
/// have final precedence. Configuration errors are returned separately from
/// geometry and limit failures. Budget validation precedes option validation
/// and geometry evaluation.
pub fn tessellate_with_context(
    face: &TrimmedSurface<'_>,
    opts: &TessOptions,
    context: &OperationContext<'_>,
) -> core::result::Result<OperationOutcome<FaceMesh>, OperationPolicyError> {
    let context = context
        .clone()
        .with_family_budget_defaults(FaceTessellationBudgetProfile::v1_defaults());
    let effective_budget = context.effective_budget();
    validate_tessellation_budget(|stage, resource, mode| {
        effective_budget.require_limit(stage, resource, mode)
    })?;
    let mut scope = OperationScope::new(&context);
    let result = tessellate_in_scope(face, opts, &mut scope);
    Ok(scope.finish(result))
}

/// Tessellate a trimmed face using an existing operation scope.
///
/// Higher-level tessellation composes through this seam so nested work reuses
/// the caller's ledger instead of resetting the face-level allowances. The
/// active ledger is validated before options or geometry are inspected; an
/// incompatible shared scope is returned as [`Error::OperationPolicy`], while
/// [`tessellate_with_context`] retains configuration errors in its outer
/// [`OperationPolicyError`] result.
pub fn tessellate_in_scope(
    face: &TrimmedSurface<'_>,
    opts: &TessOptions,
    scope: &mut OperationScope<'_, '_>,
) -> Result<FaceMesh> {
    tessellate_accounted(face, opts, scope)
}

/// Tessellate one face using a deterministically reserved child ledger.
///
/// Higher-level tessellators use this seam to give each face the exact
/// [`FaceTessellationBudgetProfile::v1_defaults`] allowance without starting
/// a nested operation scope. The caller remains responsible for reserving and
/// merging the child. Limit failures retain their structured snapshot and the
/// child ledger retains the corresponding limit event; semantic diagnostics
/// remain the responsibility of the owning operation scope.
pub fn tessellate_in_child_ledger(
    face: &TrimmedSurface<'_>,
    opts: &TessOptions,
    child: &mut ChildWorkLedger,
) -> Result<FaceMesh> {
    tessellate_accounted(face, opts, child)
}

/// Tessellate one face with invocation-local caps and real-time parent usage.
///
/// This seam is for strictly sequential higher-level tessellators. The
/// sequential ledger keeps the face profile local to this invocation while
/// forwarding every accepted unit and evidence event to its borrowed parent.
/// Unlike [`tessellate_in_child_ledger`], no capacity is reserved and no join
/// step is required.
pub fn tessellate_in_sequential_ledger(
    face: &TrimmedSurface<'_>,
    opts: &TessOptions,
    ledger: &mut SequentialWorkLedger<'_>,
) -> Result<FaceMesh> {
    tessellate_accounted(face, opts, ledger)
}

fn tessellate_accounted(
    face: &TrimmedSurface<'_>,
    opts: &TessOptions,
    accounting: &mut impl TessellationAccounting,
) -> Result<FaceMesh> {
    validate_tessellation_budget(|stage, resource, mode| {
        accounting.require_limit(stage, resource, mode)
    })
    .map_err(Error::from)?;
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
    let ctx = RefineCtx {
        surface: face.surface,
        tol: opts.chord_tol,
        max_len: opts.max_edge_len.unwrap_or(f64::INFINITY),
    };

    let initial_trim_items = face.loops.iter().try_fold(0_u64, |total, loop_| {
        let count = u64::try_from(loop_.points.len()).map_err(|_| {
            Error::from(OperationPolicyError::AccountingOverflow {
                stage: FACE_TESSELLATION_MESH_VERTICES,
                resource: ResourceKind::Items,
            })
        })?;
        total.checked_add(count).ok_or_else(|| {
            Error::from(OperationPolicyError::AccountingOverflow {
                stage: FACE_TESSELLATION_MESH_VERTICES,
                resource: ResourceKind::Items,
            })
        })
    })?;
    admit_mesh_vertices(accounting, initial_trim_items)?;

    // Stage 1: boundary refinement. Vertices 0..N are the refined loop
    // vertices, loop by loop, in traversal order.
    let mut uvs: Vec<Vec2> = Vec::new();
    let mut positions: Vec<Point3> = Vec::new();
    let mut boundary: Vec<Vec<u32>> = Vec::new();
    for l in &face.loops {
        let refined = refine_loop(&ctx, &l.points, accounting)?;
        let start = uvs.len();
        let mut indices = Vec::with_capacity(refined.len());
        for (uv, p) in refined {
            let logical_index = start.checked_add(indices.len()).ok_or_else(|| {
                Error::from(OperationPolicyError::AccountingOverflow {
                    stage: FACE_TESSELLATION_MESH_VERTICES,
                    resource: ResourceKind::Items,
                })
            })?;
            indices
                .push(u32::try_from(logical_index).map_err(|_| mesh_vertex_address_limit_error())?);
            uvs.push(uv);
            positions.push(p);
        }
        boundary.push(indices);
    }
    let boundary_edges: BTreeSet<(u32, u32)> = boundary
        .iter()
        .flat_map(|l| loop_edges(l).map(sorted_pair))
        .collect();

    // Stage 2: bridge holes and ear-clip.
    let merged = bridge_holes(&uvs, &boundary)?;
    let mut triangles = earclip(&uvs, &merged, accounting)?;

    // Stage 3: conforming interior refinement.
    loop {
        let mut marked: BTreeSet<(u32, u32)> = BTreeSet::new();
        let mut centroid_tris: Vec<usize> = Vec::new();
        for tri in &triangles {
            for (a, b) in tri_edges(tri) {
                let key = sorted_pair((a, b));
                if boundary_edges.contains(&key) || marked.contains(&key) {
                    continue;
                }
                if edge_needs_split(&ctx, &uvs, &positions, key) {
                    marked.insert(key);
                }
            }
        }
        for (ti, tri) in triangles.iter().enumerate() {
            if tri_edges(tri).any(|e| marked.contains(&sorted_pair(e))) {
                continue;
            }
            if !centroid_needs_split(&ctx, &uvs, &positions, tri) {
                continue;
            }
            // Split the longest non-boundary edge; a triangle whose three
            // edges are all boundary gets a centroid insertion instead.
            let longest = tri_edges(tri)
                .map(sorted_pair)
                .filter(|k| !boundary_edges.contains(k))
                .max_by(|&a, &b| {
                    let la = uvs[a.0 as usize].dist(uvs[a.1 as usize]);
                    let lb = uvs[b.0 as usize].dist(uvs[b.1 as usize]);
                    la.partial_cmp(&lb).expect("finite uv lengths").then(
                        // Deterministic tie-break on the index pair.
                        b.cmp(&a),
                    )
                });
            match longest {
                Some(key) => {
                    marked.insert(key);
                }
                None => centroid_tris.push(ti),
            }
        }
        if marked.is_empty() && centroid_tris.is_empty() {
            break;
        }
        let new_vertex_count = u64::try_from(marked.len())
            .ok()
            .and_then(|count| {
                u64::try_from(centroid_tris.len())
                    .ok()
                    .and_then(|centroids| count.checked_add(centroids))
            })
            .ok_or_else(|| {
                Error::from(OperationPolicyError::AccountingOverflow {
                    stage: FACE_TESSELLATION_MESH_VERTICES,
                    resource: ResourceKind::Items,
                })
            })?;
        let next_triangle_count =
            prospective_refinement_triangle_count(&triangles, &marked, &centroid_tris)?;
        preflight_refinement_generation(accounting, new_vertex_count, next_triangle_count)?;

        // Allocate midpoint vertices (sorted edge order → deterministic ids).
        let mut midpoint: BTreeMap<(u32, u32), u32> = BTreeMap::new();
        for &key in &marked {
            let uv = (uvs[key.0 as usize] + uvs[key.1 as usize]) / 2.0;
            midpoint.insert(
                key,
                push_vertex(&mut uvs, &mut positions, &ctx, uv, accounting)?,
            );
        }
        let centroid_set: BTreeSet<usize> = centroid_tris.into_iter().collect();

        let next_capacity = usize::try_from(next_triangle_count).map_err(|_| {
            Error::from(OperationPolicyError::AccountingOverflow {
                stage: FACE_TESSELLATION_MESH_TRIANGLES,
                resource: ResourceKind::Items,
            })
        })?;
        let mut next: Vec<[u32; 3]> = Vec::with_capacity(next_capacity);
        for (ti, tri) in triangles.iter().enumerate() {
            if centroid_set.contains(&ti) {
                let [a, b, c] = *tri;
                let g = (uvs[a as usize] + uvs[b as usize] + uvs[c as usize]) / 3.0;
                let gi = push_vertex(&mut uvs, &mut positions, &ctx, g, accounting)?;
                next.extend([[a, b, gi], [b, c, gi], [c, a, gi]]);
            } else {
                subdivide_triangle(*tri, &midpoint, &uvs, &mut next);
            }
        }
        debug_assert_eq!(next.len(), next_capacity);
        charge_refinement_pass(accounting)?;
        triangles = next;
    }

    Ok(FaceMesh {
        positions,
        uvs,
        triangles,
        boundary,
    })
}

trait TessellationAccounting {
    fn require_limit(
        &self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
        mode: AccountingMode,
    ) -> core::result::Result<(), OperationPolicyError>;
    fn snapshot(
        &self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
    ) -> core::result::Result<LimitSnapshot, OperationPolicyError>;
    fn check_charge(
        &self,
        stage: kcore::operation::StageId,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError>;
    fn check_charge_resource(
        &self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError>;
    fn charge(
        &mut self,
        stage: kcore::operation::StageId,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError>;
    fn charge_resource(
        &mut self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError>;
    fn observe(
        &mut self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
        value: u64,
    ) -> core::result::Result<(), OperationPolicyError>;
    fn diagnose_limit(
        &mut self,
        snapshot: LimitSnapshot,
        code: kcore::operation::DiagnosticCode,
        message: &'static str,
    );
}

impl TessellationAccounting for OperationScope<'_, '_> {
    fn require_limit(
        &self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
        mode: AccountingMode,
    ) -> core::result::Result<(), OperationPolicyError> {
        OperationScope::ledger(self).require_limit(stage, resource, mode)
    }

    fn snapshot(
        &self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
    ) -> core::result::Result<LimitSnapshot, OperationPolicyError> {
        snapshot_from_ledger(OperationScope::ledger(self), stage, resource)
    }

    fn check_charge(
        &self,
        stage: kcore::operation::StageId,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        OperationScope::ledger(self).check_charge(stage, amount)
    }

    fn check_charge_resource(
        &self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        OperationScope::ledger(self).check_charge_resource(stage, resource, amount)
    }

    fn charge(
        &mut self,
        stage: kcore::operation::StageId,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        OperationScope::ledger_mut(self).charge(stage, amount)
    }

    fn charge_resource(
        &mut self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        OperationScope::ledger_mut(self).charge_resource(stage, resource, amount)
    }

    fn observe(
        &mut self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
        value: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        OperationScope::ledger_mut(self).observe(stage, resource, value)
    }

    fn diagnose_limit(
        &mut self,
        snapshot: LimitSnapshot,
        code: kcore::operation::DiagnosticCode,
        message: &'static str,
    ) {
        self.diagnose(
            snapshot.stage,
            code,
            DiagnosticKind::LimitReached(snapshot),
            message,
        );
    }
}

impl TessellationAccounting for ChildWorkLedger {
    fn require_limit(
        &self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
        mode: AccountingMode,
    ) -> core::result::Result<(), OperationPolicyError> {
        ChildWorkLedger::ledger(self).require_limit(stage, resource, mode)
    }

    fn snapshot(
        &self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
    ) -> core::result::Result<LimitSnapshot, OperationPolicyError> {
        snapshot_from_ledger(ChildWorkLedger::ledger(self), stage, resource)
    }

    fn check_charge(
        &self,
        stage: kcore::operation::StageId,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        ChildWorkLedger::ledger(self).check_charge(stage, amount)
    }

    fn check_charge_resource(
        &self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        ChildWorkLedger::ledger(self).check_charge_resource(stage, resource, amount)
    }

    fn charge(
        &mut self,
        stage: kcore::operation::StageId,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        ChildWorkLedger::ledger_mut(self).charge(stage, amount)
    }

    fn charge_resource(
        &mut self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        ChildWorkLedger::ledger_mut(self).charge_resource(stage, resource, amount)
    }

    fn observe(
        &mut self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
        value: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        ChildWorkLedger::ledger_mut(self).observe(stage, resource, value)
    }

    fn diagnose_limit(
        &mut self,
        _snapshot: LimitSnapshot,
        _code: kcore::operation::DiagnosticCode,
        _message: &'static str,
    ) {
    }
}

impl TessellationAccounting for SequentialWorkLedger<'_> {
    fn require_limit(
        &self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
        mode: AccountingMode,
    ) -> core::result::Result<(), OperationPolicyError> {
        SequentialWorkLedger::require_limit(self, stage, resource, mode)
    }

    fn snapshot(
        &self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
    ) -> core::result::Result<LimitSnapshot, OperationPolicyError> {
        SequentialWorkLedger::snapshots(self)
            .into_iter()
            .find(|entry| entry.stage == stage && entry.resource == resource)
            .ok_or(OperationPolicyError::UnknownLimit { stage, resource })
    }

    fn check_charge(
        &self,
        stage: kcore::operation::StageId,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        SequentialWorkLedger::check_charge(self, stage, amount)
    }

    fn check_charge_resource(
        &self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        SequentialWorkLedger::check_charge_resource(self, stage, resource, amount)
    }

    fn charge(
        &mut self,
        stage: kcore::operation::StageId,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        SequentialWorkLedger::charge(self, stage, amount)
    }

    fn charge_resource(
        &mut self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        SequentialWorkLedger::charge_resource(self, stage, resource, amount)
    }

    fn observe(
        &mut self,
        stage: kcore::operation::StageId,
        resource: ResourceKind,
        value: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        SequentialWorkLedger::observe(self, stage, resource, value)
    }

    fn diagnose_limit(
        &mut self,
        _snapshot: LimitSnapshot,
        _code: kcore::operation::DiagnosticCode,
        _message: &'static str,
    ) {
    }
}

fn snapshot_from_ledger(
    ledger: &kcore::operation::WorkLedger,
    stage: kcore::operation::StageId,
    resource: ResourceKind,
) -> core::result::Result<LimitSnapshot, OperationPolicyError> {
    ledger
        .snapshots()
        .into_iter()
        .find(|entry| entry.stage == stage && entry.resource == resource)
        .ok_or(OperationPolicyError::UnknownLimit { stage, resource })
}

fn validate_tessellation_budget(
    mut require_limit: impl FnMut(
        kcore::operation::StageId,
        ResourceKind,
        AccountingMode,
    ) -> core::result::Result<(), OperationPolicyError>,
) -> core::result::Result<(), OperationPolicyError> {
    for required in FaceTessellationBudgetProfile::v1_defaults().limits() {
        require_limit(required.stage, required.resource, required.mode)?;
    }
    Ok(())
}

fn legacy_tessellation_error(error: Error) -> Error {
    match error {
        Error::ResourceLimit { snapshot } if snapshot.stage == FACE_TESSELLATION_BOUNDARY_DEPTH => {
            Error::AlgorithmLimit {
                operation: "tessellation boundary refinement depth",
                limit: MAX_BOUNDARY_DEPTH,
            }
        }
        Error::ResourceLimit { snapshot }
            if snapshot.stage == FACE_TESSELLATION_REFINEMENT_PASSES =>
        {
            Error::AlgorithmLimit {
                operation: "tessellation interior refinement passes",
                limit: MAX_REFINE_PASSES,
            }
        }
        Error::ResourceLimit { snapshot } if snapshot.stage == FACE_TESSELLATION_MESH_TRIANGLES => {
            Error::AlgorithmLimit {
                operation: "tessellation triangle count",
                limit: MAX_TRIANGLES,
            }
        }
        other => other,
    }
}

fn charge_mesh_vertices(accounting: &mut impl TessellationAccounting, amount: u64) -> Result<()> {
    match accounting.charge_resource(FACE_TESSELLATION_MESH_VERTICES, ResourceKind::Items, amount) {
        Ok(()) => Ok(()),
        Err(OperationPolicyError::LimitReached(snapshot)) => {
            accounting.diagnose_limit(
                snapshot,
                FACE_TESSELLATION_MESH_VERTEX_LIMIT,
                "face tessellation mesh-vertex limit reached",
            );
            Err(Error::ResourceLimit { snapshot })
        }
        Err(error) => Err(error.into()),
    }
}

fn admit_mesh_vertices(accounting: &mut impl TessellationAccounting, amount: u64) -> Result<()> {
    preflight_mesh_vertices(accounting, amount)?;
    charge_mesh_vertices(accounting, amount)
}

fn preflight_mesh_vertices(
    accounting: &mut impl TessellationAccounting,
    amount: u64,
) -> Result<()> {
    let configured = accounting
        .snapshot(FACE_TESSELLATION_MESH_VERTICES, ResourceKind::Items)
        .map_err(Error::from)?;
    let prospective = configured.consumed.checked_add(amount).ok_or_else(|| {
        Error::from(OperationPolicyError::AccountingOverflow {
            stage: FACE_TESSELLATION_MESH_VERTICES,
            resource: ResourceKind::Items,
        })
    })?;
    if prospective > FACE_TESSELLATION_U32_ITEM_LIMIT {
        let physical = LimitSnapshot {
            stage: FACE_TESSELLATION_MESH_VERTICES,
            resource: ResourceKind::Items,
            consumed: prospective,
            allowed: FACE_TESSELLATION_U32_ITEM_LIMIT,
        };
        if configured.allowed <= FACE_TESSELLATION_U32_ITEM_LIMIT {
            return charge_mesh_vertices(accounting, amount);
        }
        accounting.diagnose_limit(
            physical,
            FACE_TESSELLATION_MESH_VERTEX_LIMIT,
            "face tessellation mesh-vertex format limit reached",
        );
        return Err(Error::ResourceLimit { snapshot: physical });
    }
    match accounting.check_charge_resource(
        FACE_TESSELLATION_MESH_VERTICES,
        ResourceKind::Items,
        amount,
    ) {
        Ok(()) => Ok(()),
        Err(OperationPolicyError::LimitReached(_)) => charge_mesh_vertices(accounting, amount),
        Err(error) => Err(error.into()),
    }
}

fn charge_boundary_split(accounting: &mut impl TessellationAccounting) -> Result<()> {
    match accounting.charge(FACE_TESSELLATION_BOUNDARY_SPLITS, 1) {
        Ok(()) => Ok(()),
        Err(OperationPolicyError::LimitReached(snapshot)) => {
            accounting.diagnose_limit(
                snapshot,
                FACE_TESSELLATION_BOUNDARY_SPLIT_LIMIT,
                "face tessellation boundary-split limit reached",
            );
            Err(Error::ResourceLimit { snapshot })
        }
        Err(error) => Err(error.into()),
    }
}

/// Admit a boundary split and its new mesh vertex as one logical unit.
/// Both ledgers are preflighted before either accepted counter changes.
fn charge_boundary_split_and_vertex(accounting: &mut impl TessellationAccounting) -> Result<()> {
    match accounting.check_charge(FACE_TESSELLATION_BOUNDARY_SPLITS, 1) {
        Ok(()) => {}
        Err(OperationPolicyError::LimitReached(_)) => return charge_boundary_split(accounting),
        Err(error) => return Err(error.into()),
    }
    preflight_mesh_vertices(accounting, 1)?;

    charge_boundary_split(accounting)?;
    charge_mesh_vertices(accounting, 1)
}

fn observe_mesh_triangles(
    accounting: &mut impl TessellationAccounting,
    triangle_count: u64,
) -> Result<()> {
    match accounting.observe(
        FACE_TESSELLATION_MESH_TRIANGLES,
        ResourceKind::Items,
        triangle_count,
    ) {
        Ok(()) => Ok(()),
        Err(OperationPolicyError::LimitReached(snapshot)) => {
            accounting.diagnose_limit(
                snapshot,
                FACE_TESSELLATION_MESH_TRIANGLE_LIMIT,
                "face tessellation mesh-triangle limit reached",
            );
            Err(Error::ResourceLimit { snapshot })
        }
        Err(error) => Err(error.into()),
    }
}

fn charge_refinement_pass(accounting: &mut impl TessellationAccounting) -> Result<()> {
    match accounting.charge(FACE_TESSELLATION_REFINEMENT_PASSES, 1) {
        Ok(()) => Ok(()),
        Err(OperationPolicyError::LimitReached(snapshot)) => {
            accounting.diagnose_limit(
                snapshot,
                FACE_TESSELLATION_REFINEMENT_PASS_LIMIT,
                "face tessellation interior-refinement pass limit reached",
            );
            Err(Error::ResourceLimit { snapshot })
        }
        Err(error) => Err(error.into()),
    }
}

fn preflight_refinement_pass(accounting: &mut impl TessellationAccounting) -> Result<()> {
    match accounting.check_charge(FACE_TESSELLATION_REFINEMENT_PASSES, 1) {
        Ok(()) => {}
        Err(OperationPolicyError::LimitReached(_)) => {
            return charge_refinement_pass(accounting);
        }
        Err(error) => return Err(error.into()),
    }
    Ok(())
}

fn preflight_refinement_generation(
    accounting: &mut impl TessellationAccounting,
    new_vertex_count: u64,
    next_triangle_count: u64,
) -> Result<()> {
    preflight_refinement_pass(accounting)?;
    preflight_mesh_vertices(accounting, new_vertex_count)?;
    observe_mesh_triangles(accounting, next_triangle_count)
}

fn prospective_refinement_triangle_count(
    triangles: &[[u32; 3]],
    marked: &BTreeSet<(u32, u32)>,
    centroid_tris: &[usize],
) -> Result<u64> {
    triangles
        .iter()
        .enumerate()
        .try_fold(0_u64, |total, (index, triangle)| {
            let generated = if centroid_tris.binary_search(&index).is_ok() {
                3
            } else {
                let marked_edges = tri_edges(triangle)
                    .filter(|edge| marked.contains(&sorted_pair(*edge)))
                    .count();
                u64::try_from(marked_edges)
                    .ok()
                    .and_then(|count| count.checked_add(1))
                    .ok_or_else(|| {
                        Error::from(OperationPolicyError::AccountingOverflow {
                            stage: FACE_TESSELLATION_MESH_TRIANGLES,
                            resource: ResourceKind::Items,
                        })
                    })?
            };
            total.checked_add(generated).ok_or_else(|| {
                Error::from(OperationPolicyError::AccountingOverflow {
                    stage: FACE_TESSELLATION_MESH_TRIANGLES,
                    resource: ResourceKind::Items,
                })
            })
        })
}

/// Shared refinement inputs.
struct RefineCtx<'a> {
    surface: &'a dyn Surface,
    tol: f64,
    max_len: f64,
}

fn push_vertex(
    uvs: &mut Vec<Vec2>,
    positions: &mut Vec<Point3>,
    ctx: &RefineCtx<'_>,
    uv: Vec2,
    accounting: &mut impl TessellationAccounting,
) -> Result<u32> {
    charge_mesh_vertices(accounting, 1)?;
    let idx = u32::try_from(uvs.len()).map_err(|_| mesh_vertex_address_limit_error())?;
    uvs.push(uv);
    positions.push(ctx.surface.eval([uv.x, uv.y]));
    Ok(idx)
}

fn mesh_vertex_address_limit_error() -> Error {
    Error::ResourceLimit {
        snapshot: LimitSnapshot {
            stage: FACE_TESSELLATION_MESH_VERTICES,
            resource: ResourceKind::Items,
            consumed: FACE_TESSELLATION_U32_ITEM_LIMIT + 1,
            allowed: FACE_TESSELLATION_U32_ITEM_LIMIT,
        },
    }
}

/// Distance from `p` to the segment `[a, b]` in 3D.
fn point_segment_dist(p: Point3, a: Point3, b: Point3) -> f64 {
    let ab = b - a;
    let len_sq = ab.norm_sq();
    if len_sq == 0.0 {
        return p.dist(a);
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    p.dist(a + ab * t)
}

/// Refine one loop: every edge is midpoint-split until the surface point at
/// the parameter midpoint is within tolerance of the 3D chord (and the chord
/// respects `max_len`).
fn refine_loop(
    ctx: &RefineCtx<'_>,
    points: &[Vec2],
    accounting: &mut impl TessellationAccounting,
) -> Result<Vec<(Vec2, Point3)>> {
    let n = points.len();
    let mut out: Vec<(Vec2, Point3)> = Vec::with_capacity(n);
    for i in 0..n {
        let a = points[i];
        let b = points[(i + 1) % n];
        let pa = ctx.surface.eval([a.x, a.y]);
        let pb = ctx.surface.eval([b.x, b.y]);
        out.push((a, pa));
        refine_edge(ctx, (a, pa), (b, pb), 0, &mut out, accounting)?;
    }
    Ok(out)
}

/// Append the interior refinement points of edge `(a, b)` (exclusive) in
/// order.
fn refine_edge(
    ctx: &RefineCtx<'_>,
    a: (Vec2, Point3),
    b: (Vec2, Point3),
    depth: u64,
    out: &mut Vec<(Vec2, Point3)>,
    accounting: &mut impl TessellationAccounting,
) -> Result<()> {
    #[derive(Clone, Copy)]
    enum Task {
        Segment {
            a: (Vec2, Point3),
            b: (Vec2, Point3),
            depth: u64,
        },
        Emit((Vec2, Point3)),
    }

    let mut tasks = vec![Task::Segment { a, b, depth }];
    while let Some(task) = tasks.pop() {
        match task {
            Task::Emit(midpoint) => out.push(midpoint),
            Task::Segment { a, b, depth } => {
                let mid_uv = (a.0 + b.0) / 2.0;
                let mid_p = ctx.surface.eval([mid_uv.x, mid_uv.y]);
                let deviation = point_segment_dist(mid_p, a.1, b.1);
                if deviation <= ctx.tol && a.1.dist(b.1) <= ctx.max_len {
                    continue;
                }
                let next_depth = depth.checked_add(1).ok_or_else(|| {
                    Error::from(OperationPolicyError::AccountingOverflow {
                        stage: FACE_TESSELLATION_BOUNDARY_DEPTH,
                        resource: ResourceKind::Depth,
                    })
                })?;
                observe_boundary_depth(accounting, next_depth)?;
                charge_boundary_split_and_vertex(accounting)?;
                let midpoint = (mid_uv, mid_p);
                tasks.push(Task::Segment {
                    a: midpoint,
                    b,
                    depth: next_depth,
                });
                tasks.push(Task::Emit(midpoint));
                tasks.push(Task::Segment {
                    a,
                    b: midpoint,
                    depth: next_depth,
                });
            }
        }
    }
    Ok(())
}

fn observe_boundary_depth(accounting: &mut impl TessellationAccounting, depth: u64) -> Result<()> {
    match accounting.observe(FACE_TESSELLATION_BOUNDARY_DEPTH, ResourceKind::Depth, depth) {
        Ok(()) => Ok(()),
        Err(OperationPolicyError::LimitReached(snapshot)) => {
            accounting.diagnose_limit(
                snapshot,
                FACE_TESSELLATION_BOUNDARY_DEPTH_LIMIT,
                "face tessellation boundary-refinement depth limit reached",
            );
            Err(Error::ResourceLimit { snapshot })
        }
        Err(error) => Err(error.into()),
    }
}

fn edge_needs_split(
    ctx: &RefineCtx<'_>,
    uvs: &[Vec2],
    positions: &[Point3],
    key: (u32, u32),
) -> bool {
    let (a, b) = (key.0 as usize, key.1 as usize);
    if positions[a].dist(positions[b]) > ctx.max_len {
        return true;
    }
    let mid_uv = (uvs[a] + uvs[b]) / 2.0;
    let mid_p = ctx.surface.eval([mid_uv.x, mid_uv.y]);
    point_segment_dist(mid_p, positions[a], positions[b]) > ctx.tol
}

fn centroid_needs_split(
    ctx: &RefineCtx<'_>,
    uvs: &[Vec2],
    positions: &[Point3],
    tri: &[u32; 3],
) -> bool {
    let [a, b, c] = tri.map(|i| i as usize);
    let g_uv = (uvs[a] + uvs[b] + uvs[c]) / 3.0;
    let g_lin = (positions[a] + positions[b] + positions[c]) / 3.0;
    ctx.surface.eval([g_uv.x, g_uv.y]).dist(g_lin) > ctx.tol
}

/// The three directed edges of a triangle.
fn tri_edges(tri: &[u32; 3]) -> impl Iterator<Item = (u32, u32)> {
    let [a, b, c] = *tri;
    [(a, b), (b, c), (c, a)].into_iter()
}

/// Directed consecutive pairs of a loop (wrapping).
fn loop_edges(l: &[u32]) -> impl Iterator<Item = (u32, u32)> + '_ {
    (0..l.len()).map(|i| (l[i], l[(i + 1) % l.len()]))
}

fn sorted_pair(e: (u32, u32)) -> (u32, u32) {
    if e.0 <= e.1 { e } else { (e.1, e.0) }
}

/// Split a triangle according to which of its edges have midpoints,
/// preserving counterclockwise orientation.
fn subdivide_triangle(
    tri: [u32; 3],
    midpoint: &BTreeMap<(u32, u32), u32>,
    uvs: &[Vec2],
    out: &mut Vec<[u32; 3]>,
) {
    let mid = |a: u32, b: u32| midpoint.get(&sorted_pair((a, b))).copied();
    // Rotate so the marked-edge pattern is canonical.
    let rotations = [
        [tri[0], tri[1], tri[2]],
        [tri[1], tri[2], tri[0]],
        [tri[2], tri[0], tri[1]],
    ];
    let count = tri_edges(&tri)
        .filter(|&(a, b)| mid(a, b).is_some())
        .count();
    match count {
        0 => out.push(tri),
        1 => {
            // Rotate so the split edge is (a, b).
            let r = rotations
                .into_iter()
                .find(|r| mid(r[0], r[1]).is_some())
                .expect("one edge is marked");
            let m = mid(r[0], r[1]).expect("marked");
            out.extend([[r[0], m, r[2]], [m, r[1], r[2]]]);
        }
        2 => {
            // Rotate so the unsplit edge is (c, a): splits on (a,b), (b,c).
            let r = rotations
                .into_iter()
                .find(|r| mid(r[2], r[0]).is_none())
                .expect("one edge is unmarked");
            let m1 = mid(r[0], r[1]).expect("marked");
            let m2 = mid(r[1], r[2]).expect("marked");
            out.push([m1, r[1], m2]);
            // Quad (a, m1, m2, c): split along the shorter diagonal,
            // deterministic tie toward (a, m2).
            let d_a_m2 = uvs[r[0] as usize].dist(uvs[m2 as usize]);
            let d_m1_c = uvs[m1 as usize].dist(uvs[r[2] as usize]);
            if d_a_m2 <= d_m1_c {
                out.extend([[r[0], m1, m2], [r[0], m2, r[2]]]);
            } else {
                out.extend([[r[0], m1, r[2]], [m1, m2, r[2]]]);
            }
        }
        3 => {
            let [a, b, c] = tri;
            let (mab, mbc, mca) = (
                mid(a, b).expect("marked"),
                mid(b, c).expect("marked"),
                mid(c, a).expect("marked"),
            );
            out.extend([[a, mab, mca], [mab, b, mbc], [mca, mbc, c], [mab, mbc, mca]]);
        }
        _ => unreachable!(),
    }
}

fn orient(uvs: &[Vec2], a: u32, b: u32, c: u32) -> Orientation {
    let p = |i: u32| {
        let v = uvs[i as usize];
        [v.x, v.y]
    };
    orient2d(p(a), p(b), p(c))
}

/// Inclusive point-in-triangle for a CCW triangle (`true` on edges).
fn in_triangle_inclusive(uvs: &[Vec2], tri: [u32; 3], p: u32) -> bool {
    orient(uvs, tri[0], tri[1], p) != Orientation::Negative
        && orient(uvs, tri[1], tri[2], p) != Orientation::Negative
        && orient(uvs, tri[2], tri[0], p) != Orientation::Negative
}

/// Proper segment-segment intersection used for bridge visibility. Any
/// collinear/touching configuration counts as blocked (conservative), except
/// segments sharing an endpoint bitwise, which the caller filters.
fn segments_cross(uvs: &[Vec2], a: u32, b: u32, c: u32, d: u32) -> bool {
    let o1 = orient(uvs, a, b, c);
    let o2 = orient(uvs, a, b, d);
    let o3 = orient(uvs, c, d, a);
    let o4 = orient(uvs, c, d, b);
    if o1 == Orientation::Zero
        || o2 == Orientation::Zero
        || o3 == Orientation::Zero
        || o4 == Orientation::Zero
    {
        // Collinear contact: block unless clearly separated. Cheap and
        // conservative — the caller just tries the next candidate pair.
        let bb =
            |i: u32, j: u32| crate::aabb::Aabb2::from_points(&[uvs[i as usize], uvs[j as usize]]);
        let (b1, b2) = (bb(a, b), bb(c, d));
        return b1.min.x <= b2.max.x
            && b2.min.x <= b1.max.x
            && b1.min.y <= b2.max.y
            && b2.min.y <= b1.max.y;
    }
    o1 != o2 && o3 != o4
}

/// Even-odd point containment against the whole trimmed region (outer loop
/// minus holes): inside iff the crossing parity over all loop edges is odd.
fn point_in_region(uvs: &[Vec2], loops: &[Vec<u32>], p: Vec2) -> bool {
    let mut inside = false;
    for l in loops {
        for (i, j) in loop_edges(l) {
            let a = uvs[i as usize];
            let b = uvs[j as usize];
            if (a.y > p.y) != (b.y > p.y) {
                let x = a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x);
                if x > p.x {
                    inside = !inside;
                }
            }
        }
    }
    inside
}

/// Join every hole to the outer loop with a visibility bridge, producing one
/// merged polygon (as a vertex-index sequence with duplicated bridge
/// vertices).
fn bridge_holes(uvs: &[Vec2], boundary: &[Vec<u32>]) -> Result<Vec<u32>> {
    let mut polygon: Vec<u32> = boundary[0].clone();
    // Deterministic hole order: by leftmost (then lowest) point.
    let mut order: Vec<usize> = (1..boundary.len()).collect();
    order.sort_by(|&i, &j| {
        let key = |l: &[u32]| {
            l.iter()
                .map(|&v| (uvs[v as usize].x, uvs[v as usize].y))
                .fold((f64::INFINITY, f64::INFINITY), |m, p| {
                    if (p.0, p.1) < m { p } else { m }
                })
        };
        key(&boundary[i])
            .partial_cmp(&key(&boundary[j]))
            .expect("finite uvs")
            .then(i.cmp(&j))
    });

    for (k, &hi) in order.iter().enumerate() {
        let hole = &boundary[hi];
        // Select the first visible bridge under the historical total order
        // (parameter-space length, hole index, polygon index) without
        // materializing the O(hole * polygon) candidate scratch vector.
        let mut chosen: Option<(f64, usize, usize)> = None;
        for (a, &hv) in hole.iter().enumerate() {
            for (b, &pv) in polygon.iter().enumerate() {
                let distance = uvs[hv as usize].dist(uvs[pv as usize]);
                if chosen.is_some_and(|best| {
                    distance
                        .partial_cmp(&best.0)
                        .expect("finite uvs")
                        .then((a, b).cmp(&(best.1, best.2)))
                        .is_ge()
                }) {
                    continue;
                }
                if uvs[hv as usize] == uvs[pv as usize] {
                    continue;
                }
                // The bridge may not cross any current-polygon or
                // pending-hole edge (shared endpoints are exempt).
                let blocked = |i: u32, j: u32| {
                    let share = |v: u32| {
                        uvs[v as usize] == uvs[hv as usize] || uvs[v as usize] == uvs[pv as usize]
                    };
                    if share(i) || share(j) {
                        return false;
                    }
                    segments_cross(uvs, hv, pv, i, j)
                };
                if loop_edges(&polygon).any(|(i, j)| blocked(i, j)) {
                    continue;
                }
                let mut pending_blocked = false;
                for &pending_index in &order[k..] {
                    if loop_edges(&boundary[pending_index]).any(|(i, j)| blocked(i, j)) {
                        pending_blocked = true;
                        break;
                    }
                }
                if pending_blocked {
                    continue;
                }
                // The bridge midpoint must lie inside the face region.
                let mid = (uvs[hv as usize] + uvs[pv as usize]) / 2.0;
                if !point_in_region(uvs, boundary, mid) {
                    continue;
                }
                chosen = Some((distance, a, b));
            }
        }
        let Some((_, a, b)) = chosen else {
            return Err(Error::InvalidGeometry {
                reason: "no visible bridge from hole to outer boundary",
            });
        };
        // Splice: ..., polygon[b], hole[a], hole[a+1], ..., hole[a],
        // polygon[b], ...
        let splice_capacity = polygon
            .len()
            .checked_add(hole.len())
            .and_then(|count| count.checked_add(2))
            .ok_or_else(|| {
                Error::from(OperationPolicyError::AccountingOverflow {
                    stage: FACE_TESSELLATION_MESH_VERTICES,
                    resource: ResourceKind::Items,
                })
            })?;
        let mut spliced: Vec<u32> = Vec::with_capacity(splice_capacity);
        spliced.extend_from_slice(&polygon[..=b]);
        spliced.extend_from_slice(&hole[a..]);
        spliced.extend_from_slice(&hole[..=a]);
        spliced.push(polygon[b]);
        spliced.extend_from_slice(&polygon[b + 1..]);
        polygon = spliced;
    }
    Ok(polygon)
}

/// Ear-clip a simple (bridged) polygon given as vertex indices, CCW.
fn earclip(
    uvs: &[Vec2],
    polygon: &[u32],
    accounting: &mut impl TessellationAccounting,
) -> Result<Vec<[u32; 3]>> {
    let n = polygon.len();
    if n < 3 {
        return Err(Error::InvalidGeometry {
            reason: "polygon has fewer than 3 vertices",
        });
    }
    let mut next: Vec<usize> = (0..n).map(|i| if i + 1 == n { 0 } else { i + 1 }).collect();
    let mut prev: Vec<usize> = (0..n).map(|i| if i == 0 { n - 1 } else { i - 1 }).collect();
    let mut alive = n;
    // Keep this unallocated until the first exact retained-triangle
    // observation has been admitted.
    let mut triangles: Vec<[u32; 3]> = Vec::new();

    let mut cursor = 0usize;
    let mut since_last_clip = 0usize;
    while alive > 3 {
        let (p, c, nx) = (prev[cursor], cursor, next[cursor]);
        let (vp, vc, vn) = (polygon[p], polygon[c], polygon[nx]);
        let o = orient(uvs, vp, vc, vn);
        let clip = match o {
            Orientation::Positive => {
                // Convex corner: an ear iff no other live vertex lies inside.
                let mut ear = true;
                let mut w = next[nx];
                while w != p {
                    let vw = polygon[w];
                    let coincident = uvs[vw as usize] == uvs[vp as usize]
                        || uvs[vw as usize] == uvs[vc as usize]
                        || uvs[vw as usize] == uvs[vn as usize];
                    if !coincident && in_triangle_inclusive(uvs, [vp, vc, vn], vw) {
                        ear = false;
                        break;
                    }
                    w = next[w];
                }
                if ear {
                    let prospective = u64::try_from(triangles.len())
                        .ok()
                        .and_then(|count| count.checked_add(1))
                        .ok_or_else(|| {
                            Error::from(OperationPolicyError::AccountingOverflow {
                                stage: FACE_TESSELLATION_MESH_TRIANGLES,
                                resource: ResourceKind::Items,
                            })
                        })?;
                    observe_mesh_triangles(accounting, prospective)?;
                    triangles.push([vp, vc, vn]);
                }
                ear
            }
            Orientation::Zero => {
                // Degenerate corner: prune silently only when it comes from
                // duplicated bridge vertices (coincident corners).
                uvs[vp as usize] == uvs[vc as usize] || uvs[vc as usize] == uvs[vn as usize]
            }
            Orientation::Negative => false,
        };
        if clip {
            next[p] = nx;
            prev[nx] = p;
            alive -= 1;
            since_last_clip = 0;
            cursor = p;
        } else {
            cursor = nx;
            since_last_clip += 1;
            if since_last_clip > alive {
                return Err(Error::InvalidGeometry {
                    reason: "ear clipping failed; trim loops may self-intersect",
                });
            }
        }
    }
    let (p, c, n2) = (cursor, next[cursor], next[next[cursor]]);
    if orient(uvs, polygon[p], polygon[c], polygon[n2]) == Orientation::Positive {
        let prospective = u64::try_from(triangles.len())
            .ok()
            .and_then(|count| count.checked_add(1))
            .ok_or_else(|| {
                Error::from(OperationPolicyError::AccountingOverflow {
                    stage: FACE_TESSELLATION_MESH_TRIANGLES,
                    resource: ResourceKind::Items,
                })
            })?;
        observe_mesh_triangles(accounting, prospective)?;
        triangles.push([polygon[p], polygon[c], polygon[n2]]);
    }
    Ok(triangles)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::Frame;
    use crate::surface::{Cylinder, Plane};
    use kcore::operation::{BudgetPlan, LimitSpec};
    use std::cell::Cell;
    use std::collections::BTreeMap;
    use std::num::NonZeroUsize;

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
    }

    impl<S: Surface> Surface for CountingSurface<S> {
        fn as_any(&self) -> &dyn std::any::Any {
            self
        }

        fn eval_derivs(&self, uv: [f64; 2], order: usize) -> crate::surface::SurfaceDerivs {
            self.evaluations.set(self.evaluations.get() + 1);
            self.inner.eval_derivs(uv, order)
        }

        fn param_range(&self) -> [ParamRange; 2] {
            self.inner.param_range()
        }

        fn periodicity(&self) -> [Option<f64>; 2] {
            self.inner.periodicity()
        }

        fn degeneracies(&self) -> Vec<crate::surface::Degeneracy> {
            self.inner.degeneracies()
        }

        fn bounding_box(&self, range: [ParamRange; 2]) -> crate::aabb::Aabb3 {
            self.inner.bounding_box(range)
        }
    }

    fn tessellation_session(execution: ExecutionPolicy) -> SessionPolicy {
        SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            execution,
            FaceTessellationBudgetProfile::v1_defaults(),
            PolicyVersion::V1,
        )
    }

    fn override_limit(
        stage: kcore::operation::StageId,
        resource: ResourceKind,
        mode: AccountingMode,
        allowed: u64,
    ) -> kcore::operation::BudgetPlan {
        kcore::operation::BudgetPlan::new([kcore::operation::LimitSpec::new(
            stage, resource, mode, allowed,
        )])
        .unwrap()
    }

    fn usage_for(
        report: &kcore::operation::OperationReport,
        stage: kcore::operation::StageId,
    ) -> kcore::operation::LimitSnapshot {
        snapshot_for(report.usage(), stage)
    }

    fn snapshot_for(
        snapshots: &[kcore::operation::LimitSnapshot],
        stage: kcore::operation::StageId,
    ) -> kcore::operation::LimitSnapshot {
        *snapshots
            .iter()
            .find(|snapshot| snapshot.stage == stage)
            .unwrap()
    }

    /// Watertightness: every directed edge appears at most once; undirected
    /// edges are shared by exactly two triangles except boundary edges,
    /// which appear exactly once and are exactly the refined loop segments
    /// traversed in loop direction.
    fn assert_watertight(mesh: &FaceMesh) {
        let mut directed: BTreeSet<(u32, u32)> = BTreeSet::new();
        let mut undirected: BTreeMap<(u32, u32), usize> = BTreeMap::new();
        for tri in &mesh.triangles {
            for e in tri_edges(tri) {
                assert!(directed.insert(e), "directed edge {e:?} repeated");
                *undirected.entry(sorted_pair(e)).or_insert(0) += 1;
            }
        }
        let mut loop_pairs: BTreeSet<(u32, u32)> = BTreeSet::new();
        for l in &mesh.boundary {
            for (i, j) in loop_edges(l) {
                assert!(
                    directed.contains(&(i, j)),
                    "boundary edge ({i}, {j}) missing or reversed in mesh"
                );
                loop_pairs.insert(sorted_pair((i, j)));
            }
        }
        for (edge, count) in &undirected {
            match count {
                1 => assert!(
                    loop_pairs.contains(edge),
                    "non-boundary edge {edge:?} used once"
                ),
                2 => assert!(
                    !loop_pairs.contains(edge),
                    "boundary edge {edge:?} used twice"
                ),
                _ => panic!("edge {edge:?} used {count} times"),
            }
        }
        for e in &loop_pairs {
            assert_eq!(undirected.get(e), Some(&1), "loop edge {e:?} not single");
        }
    }

    fn mesh_area(mesh: &FaceMesh) -> f64 {
        mesh.triangles
            .iter()
            .map(|t| {
                let [a, b, c] = t.map(|i| mesh.positions[i as usize]);
                (b - a).cross(c - a).norm() / 2.0
            })
            .sum()
    }

    #[test]
    fn planar_rectangle_is_two_exact_triangles() {
        let plane = Plane::new(Frame::world());
        let face = TrimmedSurface::rectangle(
            &plane,
            [ParamRange::new(0.0, 3.0), ParamRange::new(0.0, 2.0)],
        )
        .unwrap();
        let mesh = tessellate(&face, &TessOptions::default()).unwrap();
        assert_eq!(mesh.triangles.len(), 2);
        assert_eq!(mesh.positions.len(), 4);
        assert!((mesh_area(&mesh) - 6.0).abs() < 1e-12);
        assert_watertight(&mesh);
    }

    #[test]
    fn boundary_refinement_limit_is_an_error() {
        let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
        let ctx = RefineCtx {
            surface: &cylinder,
            tol: 1e-12,
            max_len: f64::INFINITY,
        };
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            FaceTessellationBudgetProfile::v1_defaults(),
            PolicyVersion::V1,
        );
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let mut scope = OperationScope::new(&context);
        let a_uv = Vec2::new(0.0, 0.0);
        let b_uv = Vec2::new(core::f64::consts::PI, 0.0);
        let mut out = Vec::new();
        let expected = kcore::operation::LimitSnapshot {
            stage: FACE_TESSELLATION_BOUNDARY_DEPTH,
            resource: ResourceKind::Depth,
            consumed: 17,
            allowed: 16,
        };
        assert_eq!(
            refine_edge(
                &ctx,
                (a_uv, cylinder.eval([a_uv.x, a_uv.y])),
                (b_uv, cylinder.eval([b_uv.x, b_uv.y])),
                MAX_BOUNDARY_DEPTH as u64,
                &mut out,
                &mut scope,
            ),
            Err(Error::ResourceLimit { snapshot: expected })
        );
        assert!(out.is_empty());
        assert_eq!(scope.ledger().limit_events(), &[expected]);
    }

    #[test]
    fn planar_face_with_hole() {
        let plane = Plane::new(Frame::world());
        let outer = TrimLoop::new(vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(4.0, 0.0),
            Vec2::new(4.0, 4.0),
            Vec2::new(0.0, 4.0),
        ])
        .unwrap();
        // Clockwise hole.
        let hole = TrimLoop::new(vec![
            Vec2::new(1.0, 1.0),
            Vec2::new(1.0, 3.0),
            Vec2::new(3.0, 3.0),
            Vec2::new(3.0, 1.0),
        ])
        .unwrap();
        let face = TrimmedSurface::new(&plane, vec![outer, hole]).unwrap();
        let mesh = tessellate(&face, &TessOptions::default()).unwrap();
        assert!((mesh_area(&mesh) - (16.0 - 4.0)).abs() < 1e-9);
        assert_eq!(mesh.boundary.len(), 2);
        assert_eq!(mesh.boundary[0].len(), 4);
        assert_eq!(mesh.boundary[1].len(), 4);
        assert_watertight(&mesh);
    }

    #[test]
    fn streaming_bridge_selection_preserves_the_hard_multi_hole_output_golden() {
        let plane = Plane::new(Frame::world());
        let outer = TrimLoop::new(vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(12.0, 0.0),
            Vec2::new(12.0, 10.0),
            Vec2::new(0.0, 10.0),
        ])
        .unwrap();
        let holes = [
            vec![
                Vec2::new(1.0, 2.0),
                Vec2::new(1.0, 4.0),
                Vec2::new(3.0, 4.0),
                Vec2::new(3.0, 2.0),
            ],
            vec![
                Vec2::new(4.0, 6.0),
                Vec2::new(4.0, 9.0),
                Vec2::new(7.0, 9.0),
                Vec2::new(7.0, 6.0),
            ],
            vec![
                Vec2::new(8.0, 1.0),
                Vec2::new(8.0, 5.0),
                Vec2::new(11.0, 5.0),
                Vec2::new(11.0, 1.0),
            ],
        ];
        let mut loops = vec![outer];
        loops.extend(
            holes
                .into_iter()
                .map(|points| TrimLoop::new(points).unwrap()),
        );
        let face = TrimmedSurface::new(&plane, loops).unwrap();
        let mesh = tessellate(&face, &TessOptions::default()).unwrap();

        assert_eq!(
            mesh.boundary,
            [
                vec![0, 1, 2, 3],
                vec![4, 5, 6, 7],
                vec![8, 9, 10, 11],
                vec![12, 13, 14, 15],
            ]
        );
        assert_eq!(
            mesh.triangles,
            [
                [3, 0, 4],
                [3, 4, 5],
                [3, 5, 6],
                [3, 6, 8],
                [3, 8, 9],
                [2, 3, 9],
                [2, 9, 10],
                [2, 10, 11],
                [2, 11, 13],
                [2, 13, 14],
                [1, 2, 14],
                [1, 14, 15],
                [0, 1, 15],
                [0, 15, 12],
                [4, 0, 12],
                [7, 4, 12],
                [6, 7, 12],
                [8, 6, 12],
                [11, 8, 12],
                [11, 12, 13],
            ]
        );
        assert_watertight(&mesh);
        assert!((mesh_area(&mesh) - 95.0).abs() < 1e-12);
    }

    #[test]
    fn cylinder_patch_meets_chord_tolerance() {
        let cyl = Cylinder::new(Frame::world(), 2.0).unwrap();
        let face = TrimmedSurface::rectangle(
            &cyl,
            [
                ParamRange::new(0.0, core::f64::consts::PI),
                ParamRange::new(0.0, 2.0),
            ],
        )
        .unwrap();
        let opts = TessOptions {
            chord_tol: 1e-3,
            max_edge_len: None,
        };
        let mesh = tessellate(&face, &opts).unwrap();
        assert!(mesh.triangles.len() > 50, "refinement must kick in");
        assert_watertight(&mesh);

        // Every vertex lies exactly on the cylinder (vertices are evaluated).
        for p in &mesh.positions {
            let r = (p.x * p.x + p.y * p.y).sqrt();
            assert!((r - 2.0).abs() < 1e-12);
        }

        // Chordal deviation at pseudo-random interior barycenters.
        let mut state = 0x1234_5678_9ABC_DEF0_u64;
        let mut rand01 = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state as f64 / u64::MAX as f64
        };
        for k in 0..500 {
            let tri = mesh.triangles[k * 7919 % mesh.triangles.len()];
            let (mut w0, mut w1) = (rand01(), rand01());
            if w0 + w1 > 1.0 {
                (w0, w1) = (1.0 - w0, 1.0 - w1);
            }
            let w = [w0, w1, 1.0 - w0 - w1];
            let [a, b, c] = tri.map(|i| i as usize);
            let uv = mesh.uvs[a] * w[0] + mesh.uvs[b] * w[1] + mesh.uvs[c] * w[2];
            let lin =
                mesh.positions[a] * w[0] + mesh.positions[b] * w[1] + mesh.positions[c] * w[2];
            let dev = cyl.eval([uv.x, uv.y]).dist(lin);
            assert!(dev <= 2.0 * opts.chord_tol, "deviation {dev:.3e} at {uv:?}");
        }

        // Lateral area r·Δu·Δv within 1%.
        let exact = 2.0 * core::f64::consts::PI * 2.0;
        assert!((mesh_area(&mesh) - exact).abs() / exact < 0.01);
    }

    #[test]
    fn orientation_validation() {
        let plane = Plane::new(Frame::world());
        // Clockwise outer loop rejected.
        let cw = TrimLoop::new(vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(0.0, 1.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(1.0, 0.0),
        ])
        .unwrap();
        assert!(TrimmedSurface::new(&plane, vec![cw]).is_err());
        // Counterclockwise hole rejected.
        let outer = TrimLoop::new(vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(4.0, 0.0),
            Vec2::new(4.0, 4.0),
            Vec2::new(0.0, 4.0),
        ])
        .unwrap();
        let ccw_hole = TrimLoop::new(vec![
            Vec2::new(1.0, 1.0),
            Vec2::new(3.0, 1.0),
            Vec2::new(3.0, 3.0),
            Vec2::new(1.0, 3.0),
        ])
        .unwrap();
        assert!(TrimmedSurface::new(&plane, vec![outer, ccw_hole]).is_err());
        // Degenerate loops rejected outright.
        assert!(TrimLoop::new(vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)]).is_err());
        assert!(
            TrimLoop::new(vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(1.0, 0.0),
                Vec2::new(2.0, 0.0),
            ])
            .is_err()
        );
    }

    #[test]
    fn trim_loop_cleaned_count_matches_the_exact_allocated_copy() {
        let points = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(1.0, 0.0),
            Vec2::new(1.0, 0.0),
            Vec2::new(0.0, 1.0),
            Vec2::new(0.0, 0.0),
        ];
        assert_eq!(TrimLoop::cleaned_point_count(&points), Ok(3));
        let loop_ = TrimLoop::new(points).unwrap();
        assert_eq!(loop_.points.len(), 3);
        assert_eq!(
            loop_.points,
            [
                Vec2::new(0.0, 0.0),
                Vec2::new(1.0, 0.0),
                Vec2::new(0.0, 1.0)
            ]
        );

        let nonfinite = [Vec2::new(0.0, 0.0), Vec2::new(f64::NAN, 1.0)];
        assert_eq!(
            TrimLoop::cleaned_point_count(&nonfinite),
            Err(Error::InvalidGeometry {
                reason: "trim loop vertex is not finite"
            })
        );
    }

    #[test]
    fn tessellation_is_deterministic() {
        let cyl = Cylinder::new(Frame::world(), 1.0).unwrap();
        let outer = TrimLoop::new(vec![
            Vec2::new(0.2, 0.1),
            Vec2::new(2.8, 0.3),
            Vec2::new(2.5, 1.9),
            Vec2::new(0.4, 1.7),
        ])
        .unwrap();
        let hole = TrimLoop::new(vec![
            Vec2::new(1.2, 0.8),
            Vec2::new(1.2, 1.2),
            Vec2::new(1.8, 1.2),
            Vec2::new(1.8, 0.8),
        ])
        .unwrap();
        let opts = TessOptions {
            chord_tol: 5e-3,
            max_edge_len: None,
        };
        let build = || {
            let face = TrimmedSurface::new(&cyl, vec![outer.clone(), hole.clone()]).unwrap();
            tessellate(&face, &opts).unwrap()
        };
        let (m1, m2) = (build(), build());
        assert_eq!(m1.triangles, m2.triangles);
        assert_eq!(m1.uvs, m2.uvs);
        assert_eq!(m1.boundary, m2.boundary);
        let bits = |m: &FaceMesh| -> Vec<[u64; 3]> {
            m.positions
                .iter()
                .map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
                .collect()
        };
        assert_eq!(bits(&m1), bits(&m2), "positions must be bit-identical");
        assert_watertight(&m1);
    }

    #[test]
    fn contextual_tessellation_preserves_bits_and_enforces_all_five_stage_boundaries() {
        let cylinder = Cylinder::new(Frame::world(), 2.0).unwrap();
        let face = TrimmedSurface::rectangle(
            &cylinder,
            [
                ParamRange::new(0.0, core::f64::consts::PI),
                ParamRange::new(0.0, 2.0),
            ],
        )
        .unwrap();
        let opts = TessOptions {
            chord_tol: 1e-3,
            max_edge_len: None,
        };
        let legacy = tessellate(&face, &opts).unwrap();
        let session = tessellation_session(ExecutionPolicy::Serial);
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let baseline = tessellate_with_context(&face, &opts, &context).unwrap();
        assert_eq!(baseline.result(), Ok(&legacy));

        let boundary = usage_for(baseline.report(), FACE_TESSELLATION_BOUNDARY_DEPTH).consumed;
        let boundary_splits =
            usage_for(baseline.report(), FACE_TESSELLATION_BOUNDARY_SPLITS).consumed;
        let passes = usage_for(baseline.report(), FACE_TESSELLATION_REFINEMENT_PASSES).consumed;
        let triangles = usage_for(baseline.report(), FACE_TESSELLATION_MESH_TRIANGLES).consumed;
        let vertices = usage_for(baseline.report(), FACE_TESSELLATION_MESH_VERTICES).consumed;
        assert!(boundary > 0 && boundary < MAX_BOUNDARY_DEPTH as u64);
        assert!(boundary_splits > 0 && boundary_splits < FACE_TESSELLATION_U32_ITEM_LIMIT);
        assert!(passes > 0 && passes < MAX_REFINE_PASSES as u64);
        assert!(triangles > 0 && triangles < MAX_TRIANGLES as u64);
        assert!(vertices > 0 && vertices < FACE_TESSELLATION_U32_ITEM_LIMIT);

        let run = |stage, resource, mode, allowed, diagnostics| {
            let context = OperationContext::new(&session, Tolerances::default())
                .unwrap()
                .with_budget_overrides(override_limit(stage, resource, mode, allowed))
                .with_diagnostics(diagnostics, 4);
            tessellate_with_context(&face, &opts, &context).unwrap()
        };

        for (stage, resource, mode, needed, diagnostic) in [
            (
                FACE_TESSELLATION_BOUNDARY_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                boundary,
                FACE_TESSELLATION_BOUNDARY_DEPTH_LIMIT,
            ),
            (
                FACE_TESSELLATION_REFINEMENT_PASSES,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                passes,
                FACE_TESSELLATION_REFINEMENT_PASS_LIMIT,
            ),
            (
                FACE_TESSELLATION_BOUNDARY_SPLITS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                boundary_splits,
                FACE_TESSELLATION_BOUNDARY_SPLIT_LIMIT,
            ),
            (
                FACE_TESSELLATION_MESH_TRIANGLES,
                ResourceKind::Items,
                AccountingMode::HighWater,
                triangles,
                FACE_TESSELLATION_MESH_TRIANGLE_LIMIT,
            ),
            (
                FACE_TESSELLATION_MESH_VERTICES,
                ResourceKind::Items,
                AccountingMode::Cumulative,
                vertices,
                FACE_TESSELLATION_MESH_VERTEX_LIMIT,
            ),
        ] {
            let below = run(
                stage,
                resource,
                mode,
                needed - 1,
                kcore::operation::DiagnosticLevel::Off,
            );
            let snapshot = kcore::operation::LimitSnapshot {
                stage,
                resource,
                consumed: needed,
                allowed: needed - 1,
            };
            assert_eq!(below.result(), Err(&Error::ResourceLimit { snapshot }));
            assert_eq!(below.report().limit_events(), &[snapshot]);
            assert!(below.report().diagnostics().is_empty());

            let diagnosed = run(
                stage,
                resource,
                mode,
                needed - 1,
                kcore::operation::DiagnosticLevel::Summary,
            );
            assert_eq!(diagnosed.result(), Err(&Error::ResourceLimit { snapshot }));
            assert_eq!(diagnosed.report().limit_events(), &[snapshot]);
            assert!(diagnosed.report().diagnostics().iter().any(|entry| {
                entry.stage == stage
                    && entry.code == diagnostic
                    && entry.kind == DiagnosticKind::LimitReached(snapshot)
            }));

            let exact = run(
                stage,
                resource,
                mode,
                needed,
                kcore::operation::DiagnosticLevel::Off,
            );
            let above = run(
                stage,
                resource,
                mode,
                needed + 1,
                kcore::operation::DiagnosticLevel::Off,
            );
            assert_eq!(exact.result(), Ok(&legacy));
            assert_eq!(above.result(), Ok(&legacy));
            assert_eq!(usage_for(exact.report(), stage).consumed, needed);
            assert_eq!(usage_for(above.report(), stage).consumed, needed);
            assert!(exact.report().limit_events().is_empty());
            assert!(above.report().limit_events().is_empty());
        }

        for execution in [
            ExecutionPolicy::AtMost(NonZeroUsize::new(1).unwrap()),
            ExecutionPolicy::AtMost(NonZeroUsize::new(2).unwrap()),
            ExecutionPolicy::Available,
        ] {
            let parallel_session = tessellation_session(execution);
            let parallel_context =
                OperationContext::new(&parallel_session, Tolerances::default()).unwrap();
            let parallel = tessellate_with_context(&face, &opts, &parallel_context).unwrap();
            assert_eq!(parallel, baseline);
        }

        let shared_budget = FaceTessellationBudgetProfile::v1_defaults().overlaid(&override_limit(
            FACE_TESSELLATION_REFINEMENT_PASSES,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            passes * 2,
        ));
        let shared_session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            shared_budget,
            PolicyVersion::V1,
        );
        let shared_context = OperationContext::new(&shared_session, Tolerances::default()).unwrap();
        let mut scope = OperationScope::new(&shared_context);
        assert_eq!(
            tessellate_in_scope(&face, &opts, &mut scope),
            Ok(legacy.clone())
        );
        assert_eq!(tessellate_in_scope(&face, &opts, &mut scope), Ok(legacy));
        assert_eq!(
            scope
                .ledger()
                .snapshots()
                .iter()
                .find(|snapshot| snapshot.stage == FACE_TESSELLATION_BOUNDARY_DEPTH)
                .unwrap()
                .consumed,
            boundary
        );
        assert_eq!(
            scope
                .ledger()
                .snapshots()
                .iter()
                .find(|snapshot| snapshot.stage == FACE_TESSELLATION_REFINEMENT_PASSES)
                .unwrap()
                .consumed,
            passes * 2
        );
    }

    #[test]
    fn initial_mesh_vertex_denial_precedes_every_surface_evaluation() {
        let plane = CountingSurface::new(Plane::new(Frame::world()));
        let face = TrimmedSurface::rectangle(
            &plane,
            [ParamRange::new(0.0, 1.0), ParamRange::new(0.0, 1.0)],
        )
        .unwrap();
        let budget = FaceTessellationBudgetProfile::v1_defaults().overlaid(&override_limit(
            FACE_TESSELLATION_MESH_VERTICES,
            ResourceKind::Items,
            AccountingMode::Cumulative,
            3,
        ));
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            budget,
            PolicyVersion::V1,
        );
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let outcome = tessellate_with_context(&face, &TessOptions::default(), &context).unwrap();
        let snapshot = LimitSnapshot {
            stage: FACE_TESSELLATION_MESH_VERTICES,
            resource: ResourceKind::Items,
            consumed: 4,
            allowed: 3,
        };

        assert_eq!(outcome.result(), Err(&Error::ResourceLimit { snapshot }));
        assert_eq!(plane.evaluations(), 0);
        assert_eq!(
            usage_for(outcome.report(), FACE_TESSELLATION_MESH_VERTICES).consumed,
            0
        );
        assert_eq!(outcome.report().limit_events(), &[snapshot]);
    }

    #[test]
    fn raised_policy_cannot_cross_the_u32_mesh_vertex_cap_without_allocating() {
        let budget = FaceTessellationBudgetProfile::v1_defaults().overlaid(&override_limit(
            FACE_TESSELLATION_MESH_VERTICES,
            ResourceKind::Items,
            AccountingMode::Cumulative,
            FACE_TESSELLATION_U32_ITEM_LIMIT + 10,
        ));
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            budget,
            PolicyVersion::V1,
        );
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let mut scope = OperationScope::new(&context);
        scope
            .ledger_mut()
            .charge_resource(
                FACE_TESSELLATION_MESH_VERTICES,
                ResourceKind::Items,
                FACE_TESSELLATION_U32_ITEM_LIMIT,
            )
            .unwrap();
        let snapshot = LimitSnapshot {
            stage: FACE_TESSELLATION_MESH_VERTICES,
            resource: ResourceKind::Items,
            consumed: FACE_TESSELLATION_U32_ITEM_LIMIT + 1,
            allowed: FACE_TESSELLATION_U32_ITEM_LIMIT,
        };

        assert_eq!(
            preflight_mesh_vertices(&mut scope, 1),
            Err(Error::ResourceLimit { snapshot })
        );
        assert_eq!(
            snapshot_for(
                scope.ledger().snapshots().as_slice(),
                FACE_TESSELLATION_MESH_VERTICES
            )
            .consumed,
            FACE_TESSELLATION_U32_ITEM_LIMIT
        );
        assert!(scope.ledger().limit_events().is_empty());
    }

    #[test]
    fn initial_earclip_triangle_limit_is_exact_and_preallocation() {
        let plane = Plane::new(Frame::world());
        let face = TrimmedSurface::rectangle(
            &plane,
            [ParamRange::new(0.0, 1.0), ParamRange::new(0.0, 1.0)],
        )
        .unwrap();
        let budget = FaceTessellationBudgetProfile::v1_defaults().overlaid(&override_limit(
            FACE_TESSELLATION_MESH_TRIANGLES,
            ResourceKind::Items,
            AccountingMode::HighWater,
            1,
        ));
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            budget,
            PolicyVersion::V1,
        );
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let outcome = tessellate_with_context(&face, &TessOptions::default(), &context).unwrap();
        let snapshot = LimitSnapshot {
            stage: FACE_TESSELLATION_MESH_TRIANGLES,
            resource: ResourceKind::Items,
            consumed: 2,
            allowed: 1,
        };

        assert_eq!(outcome.result(), Err(&Error::ResourceLimit { snapshot }));
        assert_eq!(
            usage_for(outcome.report(), FACE_TESSELLATION_MESH_TRIANGLES).consumed,
            1
        );
        assert_eq!(outcome.report().limit_events(), &[snapshot]);
    }

    #[test]
    fn coupled_allocation_preflights_have_stable_precedence_and_no_partial_usage() {
        let run_boundary = |split_limit, vertex_limit| {
            let budget = FaceTessellationBudgetProfile::v1_defaults().overlaid(
                &BudgetPlan::new([
                    LimitSpec::new(
                        FACE_TESSELLATION_BOUNDARY_SPLITS,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        split_limit,
                    ),
                    LimitSpec::new(
                        FACE_TESSELLATION_MESH_VERTICES,
                        ResourceKind::Items,
                        AccountingMode::Cumulative,
                        vertex_limit,
                    ),
                ])
                .unwrap(),
            );
            let session = SessionPolicy::new(
                SessionPrecision::parasolid(),
                NumericalPolicy::v1(),
                ExecutionPolicy::Serial,
                budget,
                PolicyVersion::V1,
            );
            let context = OperationContext::new(&session, Tolerances::default()).unwrap();
            let mut scope = OperationScope::new(&context);
            let error = charge_boundary_split_and_vertex(&mut scope).unwrap_err();
            let snapshots = scope.ledger().snapshots();
            let usage = (
                snapshot_for(&snapshots, FACE_TESSELLATION_BOUNDARY_SPLITS).consumed,
                snapshot_for(&snapshots, FACE_TESSELLATION_MESH_VERTICES).consumed,
            );
            (error, usage, scope.ledger().limit_events().to_vec())
        };

        let split_snapshot = LimitSnapshot {
            stage: FACE_TESSELLATION_BOUNDARY_SPLITS,
            resource: ResourceKind::Work,
            consumed: 1,
            allowed: 0,
        };
        assert_eq!(
            run_boundary(0, 0),
            (
                Error::ResourceLimit {
                    snapshot: split_snapshot
                },
                (0, 0),
                vec![split_snapshot],
            )
        );

        let vertex_snapshot = LimitSnapshot {
            stage: FACE_TESSELLATION_MESH_VERTICES,
            resource: ResourceKind::Items,
            consumed: 1,
            allowed: 0,
        };
        assert_eq!(
            run_boundary(1, 0),
            (
                Error::ResourceLimit {
                    snapshot: vertex_snapshot
                },
                (0, 0),
                vec![vertex_snapshot],
            )
        );

        let run_refinement = |vertex_limit, triangle_limit| {
            let budget = FaceTessellationBudgetProfile::v1_defaults().overlaid(
                &BudgetPlan::new([
                    LimitSpec::new(
                        FACE_TESSELLATION_MESH_TRIANGLES,
                        ResourceKind::Items,
                        AccountingMode::HighWater,
                        triangle_limit,
                    ),
                    LimitSpec::new(
                        FACE_TESSELLATION_MESH_VERTICES,
                        ResourceKind::Items,
                        AccountingMode::Cumulative,
                        vertex_limit,
                    ),
                ])
                .unwrap(),
            );
            let session = SessionPolicy::new(
                SessionPrecision::parasolid(),
                NumericalPolicy::v1(),
                ExecutionPolicy::Serial,
                budget,
                PolicyVersion::V1,
            );
            let context = OperationContext::new(&session, Tolerances::default()).unwrap();
            let mut scope = OperationScope::new(&context);
            let error = preflight_refinement_generation(&mut scope, 1, 1).unwrap_err();
            let snapshots = scope.ledger().snapshots();
            let usage = (
                snapshot_for(&snapshots, FACE_TESSELLATION_REFINEMENT_PASSES).consumed,
                snapshot_for(&snapshots, FACE_TESSELLATION_MESH_VERTICES).consumed,
                snapshot_for(&snapshots, FACE_TESSELLATION_MESH_TRIANGLES).consumed,
            );
            (error, usage, scope.ledger().limit_events().to_vec())
        };

        assert_eq!(
            run_refinement(0, 0),
            (
                Error::ResourceLimit {
                    snapshot: vertex_snapshot
                },
                (0, 0, 0),
                vec![vertex_snapshot],
            )
        );
        let triangle_snapshot = LimitSnapshot {
            stage: FACE_TESSELLATION_MESH_TRIANGLES,
            resource: ResourceKind::Items,
            consumed: 1,
            allowed: 0,
        };
        assert_eq!(
            run_refinement(1, 0),
            (
                Error::ResourceLimit {
                    snapshot: triangle_snapshot
                },
                (0, 0, 0),
                vec![triangle_snapshot],
            )
        );
    }

    #[test]
    fn child_ledgers_preserve_per_face_v1_caps_and_mesh_bits() {
        let cylinder = Cylinder::new(Frame::world(), 2.0).unwrap();
        let face = TrimmedSurface::rectangle(
            &cylinder,
            [
                ParamRange::new(0.0, core::f64::consts::PI),
                ParamRange::new(0.0, 2.0),
            ],
        )
        .unwrap();
        let opts = TessOptions {
            chord_tol: 1e-3,
            max_edge_len: None,
        };
        let baseline_session = tessellation_session(ExecutionPolicy::Serial);
        let baseline_context =
            OperationContext::new(&baseline_session, Tolerances::default()).unwrap();
        let baseline = tessellate_with_context(&face, &opts, &baseline_context).unwrap();
        let expected_mesh = baseline.result().unwrap();
        let expected_boundary =
            usage_for(baseline.report(), FACE_TESSELLATION_BOUNDARY_DEPTH).consumed;
        let expected_splits =
            usage_for(baseline.report(), FACE_TESSELLATION_BOUNDARY_SPLITS).consumed;
        let expected_passes =
            usage_for(baseline.report(), FACE_TESSELLATION_REFINEMENT_PASSES).consumed;
        let expected_triangles =
            usage_for(baseline.report(), FACE_TESSELLATION_MESH_TRIANGLES).consumed;
        let expected_vertices =
            usage_for(baseline.report(), FACE_TESSELLATION_MESH_VERTICES).consumed;

        let parent_budget = FaceTessellationBudgetProfile::v1_defaults().overlaid(
            &kcore::operation::BudgetPlan::new([
                kcore::operation::LimitSpec::new(
                    FACE_TESSELLATION_BOUNDARY_SPLITS,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    FACE_TESSELLATION_U32_ITEM_LIMIT * 2,
                ),
                kcore::operation::LimitSpec::new(
                    FACE_TESSELLATION_REFINEMENT_PASSES,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    (MAX_REFINE_PASSES * 2) as u64,
                ),
                // Concurrent child triangle buffers coexist, so reservable
                // Items/HighWater capacity is additive even though merged
                // usage retains the maximum observation.
                kcore::operation::LimitSpec::new(
                    FACE_TESSELLATION_MESH_TRIANGLES,
                    ResourceKind::Items,
                    AccountingMode::HighWater,
                    (MAX_TRIANGLES * 2) as u64,
                ),
                kcore::operation::LimitSpec::new(
                    FACE_TESSELLATION_MESH_VERTICES,
                    ResourceKind::Items,
                    AccountingMode::Cumulative,
                    FACE_TESSELLATION_U32_ITEM_LIMIT * 2,
                ),
            ])
            .unwrap(),
        );
        let parent_session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            parent_budget,
            PolicyVersion::V1,
        );
        let parent_context = OperationContext::new(&parent_session, Tolerances::default()).unwrap();
        let mut parent = OperationScope::new(&parent_context);
        let mut first = parent
            .ledger_mut()
            .reserve_child(0, FaceTessellationBudgetProfile::v1_defaults())
            .unwrap();
        let mut second = parent
            .ledger_mut()
            .reserve_child(1, FaceTessellationBudgetProfile::v1_defaults())
            .unwrap();

        let first_mesh = tessellate_in_child_ledger(&face, &opts, &mut first).unwrap();
        let second_mesh = tessellate_in_child_ledger(&face, &opts, &mut second).unwrap();
        assert_eq!(&first_mesh, expected_mesh);
        assert_eq!(&second_mesh, expected_mesh);
        for child in [&first, &second] {
            let snapshots = child.ledger().snapshots();
            assert_eq!(
                snapshot_for(&snapshots, FACE_TESSELLATION_BOUNDARY_DEPTH).consumed,
                expected_boundary
            );
            assert_eq!(
                snapshot_for(&snapshots, FACE_TESSELLATION_REFINEMENT_PASSES).consumed,
                expected_passes
            );
            assert_eq!(
                snapshot_for(&snapshots, FACE_TESSELLATION_BOUNDARY_SPLITS).consumed,
                expected_splits
            );
            assert_eq!(
                snapshot_for(&snapshots, FACE_TESSELLATION_MESH_TRIANGLES).consumed,
                expected_triangles
            );
            assert_eq!(
                snapshot_for(&snapshots, FACE_TESSELLATION_MESH_VERTICES).consumed,
                expected_vertices
            );
            assert_eq!(
                snapshot_for(&snapshots, FACE_TESSELLATION_BOUNDARY_DEPTH).allowed,
                MAX_BOUNDARY_DEPTH as u64
            );
            assert_eq!(
                snapshot_for(&snapshots, FACE_TESSELLATION_REFINEMENT_PASSES).allowed,
                MAX_REFINE_PASSES as u64
            );
        }

        parent
            .ledger_mut()
            .merge_children(vec![second, first])
            .unwrap();
        let report = parent.finish(Ok(())).report().clone();
        assert_eq!(
            usage_for(&report, FACE_TESSELLATION_BOUNDARY_DEPTH).consumed,
            expected_boundary
        );
        assert_eq!(
            usage_for(&report, FACE_TESSELLATION_REFINEMENT_PASSES).consumed,
            expected_passes * 2
        );
        assert_eq!(
            usage_for(&report, FACE_TESSELLATION_BOUNDARY_SPLITS).consumed,
            expected_splits * 2
        );
        assert_eq!(
            usage_for(&report, FACE_TESSELLATION_MESH_TRIANGLES).consumed,
            expected_triangles
        );
        assert_eq!(
            usage_for(&report, FACE_TESSELLATION_MESH_VERTICES).consumed,
            expected_vertices * 2
        );
        assert!(report.limit_events().is_empty());
    }

    #[test]
    fn sequential_ledgers_keep_each_patch_cap_while_parent_usage_accumulates() {
        let cylinder = Cylinder::new(Frame::world(), 2.0).unwrap();
        let face = TrimmedSurface::rectangle(
            &cylinder,
            [
                ParamRange::new(0.0, core::f64::consts::PI),
                ParamRange::new(0.0, 2.0),
            ],
        )
        .unwrap();
        let opts = TessOptions {
            chord_tol: 1e-3,
            max_edge_len: None,
        };
        let baseline_session = tessellation_session(ExecutionPolicy::Serial);
        let baseline_context =
            OperationContext::new(&baseline_session, Tolerances::default()).unwrap();
        let baseline = tessellate_with_context(&face, &opts, &baseline_context).unwrap();
        let expected_mesh = baseline.result().unwrap();
        let expected_boundary =
            usage_for(baseline.report(), FACE_TESSELLATION_BOUNDARY_DEPTH).consumed;
        let expected_splits =
            usage_for(baseline.report(), FACE_TESSELLATION_BOUNDARY_SPLITS).consumed;
        let expected_passes =
            usage_for(baseline.report(), FACE_TESSELLATION_REFINEMENT_PASSES).consumed;
        let expected_triangles =
            usage_for(baseline.report(), FACE_TESSELLATION_MESH_TRIANGLES).consumed;
        let expected_vertices =
            usage_for(baseline.report(), FACE_TESSELLATION_MESH_VERTICES).consumed;
        let parent_budget = FaceTessellationBudgetProfile::v1_defaults().overlaid(&override_limit(
            FACE_TESSELLATION_REFINEMENT_PASSES,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            (MAX_REFINE_PASSES * 2) as u64,
        ));
        let parent_session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            parent_budget,
            PolicyVersion::V1,
        );
        let parent_context = OperationContext::new(&parent_session, Tolerances::default()).unwrap();
        let mut parent = OperationScope::new(&parent_context);

        for _ in 0..2 {
            let mut patch = parent
                .ledger_mut()
                .sequential(FaceTessellationBudgetProfile::v1_defaults())
                .unwrap();
            assert_eq!(
                tessellate_in_sequential_ledger(&face, &opts, &mut patch).as_ref(),
                Ok(expected_mesh)
            );
            let snapshots = patch.snapshots();
            assert_eq!(
                snapshot_for(&snapshots, FACE_TESSELLATION_BOUNDARY_DEPTH),
                LimitSnapshot {
                    stage: FACE_TESSELLATION_BOUNDARY_DEPTH,
                    resource: ResourceKind::Depth,
                    consumed: expected_boundary,
                    allowed: MAX_BOUNDARY_DEPTH as u64,
                }
            );
            assert_eq!(
                snapshot_for(&snapshots, FACE_TESSELLATION_REFINEMENT_PASSES),
                LimitSnapshot {
                    stage: FACE_TESSELLATION_REFINEMENT_PASSES,
                    resource: ResourceKind::Work,
                    consumed: expected_passes,
                    allowed: MAX_REFINE_PASSES as u64,
                }
            );
            assert_eq!(
                snapshot_for(&snapshots, FACE_TESSELLATION_BOUNDARY_SPLITS).consumed,
                expected_splits
            );
            assert_eq!(
                snapshot_for(&snapshots, FACE_TESSELLATION_MESH_TRIANGLES).consumed,
                expected_triangles
            );
            assert_eq!(
                snapshot_for(&snapshots, FACE_TESSELLATION_MESH_VERTICES).consumed,
                expected_vertices
            );
        }

        let report = parent.finish(Ok(())).report().clone();
        assert_eq!(
            usage_for(&report, FACE_TESSELLATION_BOUNDARY_DEPTH).consumed,
            expected_boundary
        );
        assert_eq!(
            usage_for(&report, FACE_TESSELLATION_REFINEMENT_PASSES).consumed,
            expected_passes * 2
        );
        assert_eq!(
            usage_for(&report, FACE_TESSELLATION_BOUNDARY_SPLITS).consumed,
            expected_splits * 2
        );
        assert_eq!(
            usage_for(&report, FACE_TESSELLATION_MESH_TRIANGLES).consumed,
            expected_triangles
        );
        assert_eq!(
            usage_for(&report, FACE_TESSELLATION_MESH_VERTICES).consumed,
            expected_vertices * 2
        );
        assert!(report.limit_events().is_empty());
    }

    #[test]
    fn sequential_tessellation_reports_tighter_parent_stage_and_root_coordinates() {
        let cylinder = Cylinder::new(Frame::world(), 2.0).unwrap();
        let face = TrimmedSurface::rectangle(
            &cylinder,
            [
                ParamRange::new(0.0, core::f64::consts::PI),
                ParamRange::new(0.0, 2.0),
            ],
        )
        .unwrap();
        let opts = TessOptions {
            chord_tol: 1e-3,
            max_edge_len: None,
        };
        let baseline_session = tessellation_session(ExecutionPolicy::Serial);
        let baseline_context =
            OperationContext::new(&baseline_session, Tolerances::default()).unwrap();
        let baseline = tessellate_with_context(&face, &opts, &baseline_context).unwrap();
        let expected_mesh = baseline.result().unwrap();
        let expected_passes =
            usage_for(baseline.report(), FACE_TESSELLATION_REFINEMENT_PASSES).consumed;
        let expected_splits =
            usage_for(baseline.report(), FACE_TESSELLATION_BOUNDARY_SPLITS).consumed;
        let expected_total_work = expected_splits + expected_passes;
        assert!(expected_passes > 0);

        for (parent_budget, expected_stage) in [
            (
                FaceTessellationBudgetProfile::v1_defaults().overlaid(&override_limit(
                    FACE_TESSELLATION_REFINEMENT_PASSES,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    expected_passes,
                )),
                FACE_TESSELLATION_REFINEMENT_PASSES,
            ),
            (
                FaceTessellationBudgetProfile::v1_defaults()
                    .overlaid(&override_limit(
                        FACE_TESSELLATION_REFINEMENT_PASSES,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        (MAX_REFINE_PASSES * 2) as u64,
                    ))
                    .with_total_work_limit(expected_total_work),
                kcore::operation::TOTAL_WORK_STAGE,
            ),
        ] {
            let parent_session = SessionPolicy::new(
                SessionPrecision::parasolid(),
                NumericalPolicy::v1(),
                ExecutionPolicy::Serial,
                parent_budget,
                PolicyVersion::V1,
            );
            let parent_context =
                OperationContext::new(&parent_session, Tolerances::default()).unwrap();
            let mut parent = OperationScope::new(&parent_context);
            {
                let mut first = parent
                    .ledger_mut()
                    .sequential(FaceTessellationBudgetProfile::v1_defaults())
                    .unwrap();
                assert_eq!(
                    tessellate_in_sequential_ledger(&face, &opts, &mut first).as_ref(),
                    Ok(expected_mesh)
                );
            }

            let snapshot = {
                let mut second = parent
                    .ledger_mut()
                    .sequential(FaceTessellationBudgetProfile::v1_defaults())
                    .unwrap();
                let snapshot = match tessellate_in_sequential_ledger(&face, &opts, &mut second) {
                    Err(Error::ResourceLimit { snapshot }) => snapshot,
                    other => panic!("unexpected tighter-parent result: {other:?}"),
                };
                assert_eq!(
                    snapshot_for(&second.snapshots(), FACE_TESSELLATION_REFINEMENT_PASSES).consumed,
                    0,
                    "the parent rejection must roll back the attempted local pass"
                );
                snapshot
            };
            assert_eq!(
                snapshot,
                LimitSnapshot {
                    stage: expected_stage,
                    resource: ResourceKind::Work,
                    consumed: if expected_stage == kcore::operation::TOTAL_WORK_STAGE {
                        expected_total_work + 1
                    } else {
                        expected_passes + 1
                    },
                    allowed: if expected_stage == kcore::operation::TOTAL_WORK_STAGE {
                        expected_total_work
                    } else {
                        expected_passes
                    },
                }
            );
            let report = parent.finish(Ok(())).report().clone();
            assert_eq!(
                usage_for(&report, FACE_TESSELLATION_REFINEMENT_PASSES).consumed,
                expected_passes
            );
            if expected_stage == kcore::operation::TOTAL_WORK_STAGE {
                assert_eq!(
                    usage_for(&report, kcore::operation::TOTAL_WORK_STAGE).consumed,
                    expected_splits + expected_passes,
                    "root work must include accepted boundary splits and refinement passes"
                );
            }
            assert_eq!(report.limit_events(), &[snapshot]);
        }
    }

    #[test]
    fn child_ledger_retains_exact_boundary_limit_event_for_parent_merge() {
        let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
        let face = TrimmedSurface::rectangle(
            &cylinder,
            [
                ParamRange::new(0.0, core::f64::consts::PI),
                ParamRange::new(0.0, 1.0),
            ],
        )
        .unwrap();
        let session = tessellation_session(ExecutionPolicy::Serial);
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let mut parent = OperationScope::new(&context);
        let mut child = parent
            .ledger_mut()
            .reserve_child(7, FaceTessellationBudgetProfile::v1_defaults())
            .unwrap();
        let snapshot = LimitSnapshot {
            stage: FACE_TESSELLATION_BOUNDARY_DEPTH,
            resource: ResourceKind::Depth,
            consumed: 17,
            allowed: 16,
        };

        assert_eq!(
            tessellate_in_child_ledger(
                &face,
                &TessOptions {
                    chord_tol: 1e-12,
                    max_edge_len: None,
                },
                &mut child,
            ),
            Err(Error::ResourceLimit { snapshot })
        );
        assert_eq!(child.ledger().limit_events(), &[snapshot]);

        parent.ledger_mut().merge_children(vec![child]).unwrap();
        let report = parent.finish(Ok(())).report().clone();
        assert_eq!(report.limit_events(), &[snapshot]);
        assert!(report.diagnostics().is_empty());
    }

    #[test]
    fn legacy_boundary_limit_keeps_the_algorithm_limit_shape() {
        let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
        let face = TrimmedSurface::rectangle(
            &cylinder,
            [
                ParamRange::new(0.0, core::f64::consts::PI),
                ParamRange::new(0.0, 1.0),
            ],
        )
        .unwrap();
        assert_eq!(
            tessellate(
                &face,
                &TessOptions {
                    chord_tol: 1e-12,
                    max_edge_len: None,
                },
            ),
            Err(Error::AlgorithmLimit {
                operation: "tessellation boundary refinement depth",
                limit: MAX_BOUNDARY_DEPTH,
            })
        );

        assert_eq!(
            legacy_tessellation_error(Error::ResourceLimit {
                snapshot: kcore::operation::LimitSnapshot {
                    stage: FACE_TESSELLATION_REFINEMENT_PASSES,
                    resource: ResourceKind::Work,
                    consumed: 25,
                    allowed: 24,
                },
            }),
            Error::AlgorithmLimit {
                operation: "tessellation interior refinement passes",
                limit: MAX_REFINE_PASSES,
            }
        );
        assert_eq!(
            legacy_tessellation_error(Error::ResourceLimit {
                snapshot: LimitSnapshot {
                    stage: FACE_TESSELLATION_MESH_TRIANGLES,
                    resource: ResourceKind::Items,
                    consumed: MAX_TRIANGLES as u64 + 1,
                    allowed: MAX_TRIANGLES as u64,
                },
            }),
            Error::AlgorithmLimit {
                operation: "tessellation triangle count",
                limit: MAX_TRIANGLES,
            }
        );
    }

    #[test]
    fn pass_preflight_preserves_legacy_precedence_and_completed_usage() {
        let denied_budget = FaceTessellationBudgetProfile::v1_defaults().overlaid(&override_limit(
            FACE_TESSELLATION_REFINEMENT_PASSES,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            0,
        ));
        let denied_session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            denied_budget,
            PolicyVersion::V1,
        );
        let denied_context = OperationContext::new(&denied_session, Tolerances::default()).unwrap();
        let mut denied_scope = OperationScope::new(&denied_context);
        let snapshot = kcore::operation::LimitSnapshot {
            stage: FACE_TESSELLATION_REFINEMENT_PASSES,
            resource: ResourceKind::Work,
            consumed: 1,
            allowed: 0,
        };
        assert_eq!(
            preflight_refinement_pass(&mut denied_scope),
            Err(Error::ResourceLimit { snapshot }),
            "pass exhaustion retains precedence over the triangle backstop"
        );
        assert_eq!(denied_scope.ledger().limit_events(), &[snapshot]);

        let triangle_session = tessellation_session(ExecutionPolicy::Serial);
        let triangle_context =
            OperationContext::new(&triangle_session, Tolerances::default()).unwrap();
        let mut triangle_scope = OperationScope::new(&triangle_context);
        let triangle_snapshot = LimitSnapshot {
            stage: FACE_TESSELLATION_MESH_TRIANGLES,
            resource: ResourceKind::Items,
            consumed: MAX_TRIANGLES as u64 + 1,
            allowed: MAX_TRIANGLES as u64,
        };
        assert_eq!(
            observe_mesh_triangles(&mut triangle_scope, MAX_TRIANGLES as u64 + 1),
            Err(Error::ResourceLimit {
                snapshot: triangle_snapshot,
            })
        );
        assert_eq!(
            usage_for(
                &triangle_scope.finish(Ok(())).report().clone(),
                FACE_TESSELLATION_REFINEMENT_PASSES,
            )
            .consumed,
            0,
            "a triangle-backstop failure did not complete a pass"
        );
    }

    #[test]
    fn contextual_entry_fills_missing_family_budget_before_tessellation() {
        let plane = Plane::new(Frame::world());
        let face = TrimmedSurface::rectangle(
            &plane,
            [ParamRange::new(0.0, 1.0), ParamRange::new(0.0, 1.0)],
        )
        .unwrap();
        let session = SessionPolicy::v1();
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let outcome = tessellate_with_context(&face, &TessOptions::default(), &context).unwrap();
        assert!(outcome.result().is_ok());
        assert_eq!(outcome.report().usage().len(), 5);

        let invalid = tessellate_with_context(
            &face,
            &TessOptions {
                chord_tol: 0.0,
                max_edge_len: None,
            },
            &context,
        )
        .unwrap();
        assert_eq!(
            invalid.result(),
            Err(&Error::InvalidTolerance { value: 0.0 })
        );
    }

    #[test]
    fn shared_scope_rejects_missing_budget_before_planar_or_curved_work() {
        let session = SessionPolicy::v1();
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let expected = Error::OperationPolicy {
            source: OperationPolicyError::UnknownLimit {
                stage: FACE_TESSELLATION_BOUNDARY_DEPTH,
                resource: ResourceKind::Depth,
            },
        };

        let plane = CountingSurface::new(Plane::new(Frame::world()));
        let planar_face = TrimmedSurface::rectangle(
            &plane,
            [ParamRange::new(0.0, 1.0), ParamRange::new(0.0, 1.0)],
        )
        .unwrap();
        let mut planar_scope = OperationScope::new(&context);
        let planar_ledger = planar_scope.ledger().clone();
        assert_eq!(
            tessellate_in_scope(&planar_face, &TessOptions::default(), &mut planar_scope,),
            Err(expected.clone())
        );
        assert_eq!(plane.evaluations(), 0);
        assert_eq!(planar_scope.ledger(), &planar_ledger);

        let cylinder = CountingSurface::new(Cylinder::new(Frame::world(), 1.0).unwrap());
        let curved_face = TrimmedSurface::rectangle(
            &cylinder,
            [
                ParamRange::new(0.0, core::f64::consts::PI),
                ParamRange::new(0.0, 1.0),
            ],
        )
        .unwrap();
        let mut curved_scope = OperationScope::new(&context);
        let curved_ledger = curved_scope.ledger().clone();
        assert_eq!(
            tessellate_in_scope(&curved_face, &TessOptions::default(), &mut curved_scope,),
            Err(expected)
        );
        assert_eq!(cylinder.evaluations(), 0);
        assert_eq!(curved_scope.ledger(), &curved_ledger);
    }

    #[test]
    fn invalid_options_rejected() {
        let plane = Plane::new(Frame::world());
        let face = TrimmedSurface::rectangle(
            &plane,
            [ParamRange::new(0.0, 1.0), ParamRange::new(0.0, 1.0)],
        )
        .unwrap();
        for bad in [
            TessOptions {
                chord_tol: 0.0,
                max_edge_len: None,
            },
            TessOptions {
                chord_tol: f64::NAN,
                max_edge_len: None,
            },
            TessOptions {
                chord_tol: 1e-4,
                max_edge_len: Some(-1.0),
            },
        ] {
            assert!(tessellate(&face, &bad).is_err());
        }
    }

    #[test]
    fn max_edge_len_limits_triangle_size() {
        let plane = Plane::new(Frame::world());
        let face = TrimmedSurface::rectangle(
            &plane,
            [ParamRange::new(0.0, 4.0), ParamRange::new(0.0, 4.0)],
        )
        .unwrap();
        let mesh = tessellate(
            &face,
            &TessOptions {
                chord_tol: 1e-4,
                max_edge_len: Some(1.0),
            },
        )
        .unwrap();
        assert_watertight(&mesh);
        assert!((mesh_area(&mesh) - 16.0).abs() < 1e-9);
        for tri in &mesh.triangles {
            for (i, j) in tri_edges(tri) {
                let len = mesh.positions[i as usize].dist(mesh.positions[j as usize]);
                assert!(len <= 1.0 + 1e-9, "edge length {len} exceeds cap");
            }
        }
    }

    #[test]
    fn boundary_vertices_freeze_across_refinement() {
        // The refined boundary of a cylinder patch must survive interior
        // refinement untouched: boundary indices form a prefix of the vertex
        // array and their uvs lie exactly on the trim rectangle.
        let cyl = Cylinder::new(Frame::world(), 2.0).unwrap();
        let face = TrimmedSurface::rectangle(
            &cyl,
            [
                ParamRange::new(0.0, core::f64::consts::PI),
                ParamRange::new(0.0, 2.0),
            ],
        )
        .unwrap();
        let mesh = tessellate(
            &face,
            &TessOptions {
                chord_tol: 1e-3,
                max_edge_len: None,
            },
        )
        .unwrap();
        for &vi in &mesh.boundary[0] {
            let uv = mesh.uvs[vi as usize];
            let on_rect = uv.x.abs() < 1e-15
                || (uv.x - core::f64::consts::PI).abs() < 1e-15
                || uv.y.abs() < 1e-15
                || (uv.y - 2.0).abs() < 1e-15;
            assert!(on_rect, "boundary vertex {uv:?} off the trim rectangle");
        }
        assert_watertight(&mesh);
    }
}
