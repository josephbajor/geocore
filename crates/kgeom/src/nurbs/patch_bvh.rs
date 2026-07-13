//! Conservative BVH, implicit certificates, and source-provenanced adaptive
//! isolation for NURBS patches.

use super::NurbsSurface;
use crate::aabb::Aabb3;
use crate::bvh::AabbBvh;
use crate::implicit::{ImplicitBoxRelation, ImplicitSurface, classify_implicit_box};
use crate::param::ParamRange;
use crate::surface::{Dir, Surface};
use crate::vec::{Point3, Vec3};
use kcore::error::{Error, Result};
use kcore::interval::Interval;
use kcore::operation::{
    DiagnosticCode, LimitSnapshot, OperationPolicyError, OperationScope, ResourceKind, StageId,
};
use kcore::tolerance::LINEAR_RESOLUTION;

/// Stable work stage for NURBS implicit-isolation setup and subdivision attempts.
pub const NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS: StageId =
    match StageId::new("kgeom.nurbs.implicit-isolation-subdivisions") {
        Ok(stage) => stage,
        Err(_) => panic!("valid NURBS implicit-isolation subdivision stage"),
    };

/// Stable high-water stage for the conservative implicit-isolation candidate cover.
pub const NURBS_IMPLICIT_ISOLATION_CANDIDATES: StageId =
    match StageId::new("kgeom.nurbs.implicit-isolation-candidates") {
        Ok(stage) => stage,
        Err(_) => panic!("valid NURBS implicit-isolation candidate stage"),
    };

/// Stable high-water stage for exact implicit-isolation subdivision depth.
pub const NURBS_IMPLICIT_ISOLATION_DEPTH: StageId =
    match StageId::new("kgeom.nurbs.implicit-isolation-depth") {
        Ok(stage) => stage,
        Err(_) => panic!("valid NURBS implicit-isolation depth stage"),
    };

/// Diagnostic for subdivision-work exhaustion during implicit isolation.
pub const NURBS_IMPLICIT_ISOLATION_SUBDIVISION_LIMIT: DiagnosticCode =
    match DiagnosticCode::new("kgeom.nurbs.implicit-isolation-subdivision-limit") {
        Ok(code) => code,
        Err(_) => panic!("valid NURBS implicit-isolation subdivision diagnostic"),
    };

/// Diagnostic for candidate-cover exhaustion during implicit isolation.
pub const NURBS_IMPLICIT_ISOLATION_CANDIDATE_LIMIT: DiagnosticCode =
    match DiagnosticCode::new("kgeom.nurbs.implicit-isolation-candidate-limit") {
        Ok(code) => code,
        Err(_) => panic!("valid NURBS implicit-isolation candidate diagnostic"),
    };

/// Diagnostic for depth exhaustion during implicit isolation.
pub const NURBS_IMPLICIT_ISOLATION_DEPTH_LIMIT: DiagnosticCode =
    match DiagnosticCode::new("kgeom.nurbs.implicit-isolation-depth-limit") {
        Ok(code) => code,
        Err(_) => panic!("valid NURBS implicit-isolation depth diagnostic"),
    };

/// Diagnostic for floating-point parameter resolution during implicit isolation.
pub const NURBS_IMPLICIT_ISOLATION_NUMERIC_RESOLUTION: DiagnosticCode =
    match DiagnosticCode::new("kgeom.nurbs.implicit-isolation-numeric-resolution") {
        Ok(code) => code,
        Err(_) => panic!("valid NURBS implicit-isolation numeric diagnostic"),
    };

/// Certified relation of a positive-weight Bezier patch's control hull to a
/// plane tolerance slab.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlanePatchRelation {
    /// The complete patch lies on the negative side beyond the slab.
    Negative,
    /// The control hull meets the slab; the patch requires further work.
    Candidate,
    /// The complete patch lies on the positive side beyond the slab.
    Positive,
}

/// One rounded working NURBS subpatch whose source-range enclosure could not
/// be excluded from an implicit zero set.
#[derive(Debug, Clone, PartialEq)]
pub struct ImplicitCandidateCell {
    source_patch: usize,
    patch: NurbsSurface,
    bounds: Aabb3,
    depth: u32,
}

impl ImplicitCandidateCell {
    /// Index of the source Bezier patch in [`NurbsSurfaceBvh`].
    pub fn source_patch(&self) -> usize {
        self.source_patch
    }

    /// Clamped working subpatch used for deterministic partitioning and seeds.
    pub fn patch(&self) -> &NurbsSurface {
        &self.patch
    }

    /// Source parameter rectangle covered by the subpatch.
    pub fn parameter_range(&self) -> [ParamRange; 2] {
        self.patch.param_range()
    }

    /// Conservative outward interval box evaluated from the original source.
    pub fn bounds(&self) -> Aabb3 {
        self.bounds
    }

    /// Number of deterministic binary subdivisions from the source Bezier patch.
    pub fn depth(&self) -> u32 {
        self.depth
    }
}

/// Structured reasons why recursive implicit isolation stopped before every
/// retained cell reached the requested depth.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ImplicitIsolationLimits {
    candidate_budget: Option<usize>,
    parameter_resolution: bool,
    subdivision_work: Option<LimitSnapshot>,
    candidate_cells: Option<LimitSnapshot>,
    subdivision_depth: Option<LimitSnapshot>,
}

impl ImplicitIsolationLimits {
    /// Candidate-cell budget that prevented further subdivision, if any.
    pub fn candidate_budget(self) -> Option<usize> {
        self.candidate_budget
    }

    /// Whether an exact parameter midpoint rounded to an existing endpoint.
    pub fn parameter_resolution(self) -> bool {
        self.parameter_resolution
    }

    /// Structured subdivision-work limit reached by a contextual isolation.
    pub fn subdivision_work(self) -> Option<LimitSnapshot> {
        self.subdivision_work
    }

    /// Structured candidate-cover limit reached by a contextual isolation.
    pub fn candidate_cells(self) -> Option<LimitSnapshot> {
        self.candidate_cells
    }

    /// Structured subdivision-depth limit reached by a contextual isolation.
    pub fn subdivision_depth(self) -> Option<LimitSnapshot> {
        self.subdivision_depth
    }

    /// True when no configured or numeric limit interrupted isolation.
    pub fn is_empty(self) -> bool {
        self.candidate_budget.is_none()
            && !self.parameter_resolution
            && self.subdivision_work.is_none()
            && self.candidate_cells.is_none()
            && self.subdivision_depth.is_none()
    }
}

/// Failure to run contextual implicit isolation.
#[derive(Debug, Clone, PartialEq)]
pub enum ContextImplicitIsolationError {
    /// Geometry validation or exact NURBS processing failed.
    Kernel(Error),
    /// The operation scope did not provide a valid accounting contract.
    Policy(OperationPolicyError),
}

impl From<Error> for ContextImplicitIsolationError {
    fn from(error: Error) -> Self {
        Self::Kernel(error)
    }
}

impl From<OperationPolicyError> for ContextImplicitIsolationError {
    fn from(error: OperationPolicyError) -> Self {
        Self::Policy(error)
    }
}

/// Certified cover of every possible implicit-surface contact on a NURBS
/// surface after deterministic subdivision and source-range interval pruning.
#[derive(Debug, Clone, PartialEq)]
pub struct ImplicitPatchIsolation {
    candidates: Vec<ImplicitCandidateCell>,
    requested_depth: u32,
    limits: ImplicitIsolationLimits,
}

impl ImplicitPatchIsolation {
    /// Retained candidate cells in deterministic source/range order.
    pub fn candidates(&self) -> &[ImplicitCandidateCell] {
        &self.candidates
    }

    /// Requested binary subdivision depth.
    pub fn requested_depth(&self) -> u32 {
        self.requested_depth
    }

    /// Structured limits encountered while refining the cover.
    pub fn limits(&self) -> ImplicitIsolationLimits {
        self.limits
    }

    /// True when isolation finished without a configured or numeric limit.
    /// Every retained cell then reached the requested depth.
    pub fn is_complete(&self) -> bool {
        self.limits.is_empty()
    }

    /// True when complete isolation excluded the entire represented surface.
    pub fn is_proven_empty(&self) -> bool {
        self.is_complete() && self.candidates.is_empty()
    }
}

/// Deterministic hierarchy over the Bezier decomposition of one
/// clamped NURBS surface.
#[derive(Debug, Clone, PartialEq)]
pub struct NurbsSurfaceBvh {
    source: NurbsSurface,
    patches: Vec<NurbsSurface>,
    hierarchy: AabbBvh,
}

#[derive(Debug, Clone, Copy)]
struct PlaneFilter {
    normal: Vec3,
    scaled_half_width: f64,
}

impl NurbsSurfaceBvh {
    /// Extract the surface's tensor-product Bezier patches and build a
    /// deterministic hierarchy over source-provenanced range enclosures.
    pub fn build(surface: &NurbsSurface) -> Result<Self> {
        Self::build_partition(surface, surface)
    }

    fn build_partition(source: &NurbsSurface, partition: &NurbsSurface) -> Result<Self> {
        let patches = partition.to_bezier_patches()?;
        let bounds: Vec<_> = patches
            .iter()
            .map(|patch| {
                super::surface_range_interval::position_range_aabb(source, patch.param_range())
            })
            .collect();
        let hierarchy = AabbBvh::build(&bounds)?;
        Ok(Self {
            source: source.clone(),
            patches,
            hierarchy,
        })
    }

    /// Contextual hierarchy construction with source-rectangle Work admitted
    /// before Bezier extraction, interval evaluation, or hierarchy allocation.
    ///
    /// The preflight conservatively charges every source tensor-span slot as a
    /// potential Bezier patch, including repeated/empty slots. Each potential
    /// patch is charged at the complete source-range evaluation cost returned
    /// by the interval implementation.
    pub fn build_in_scope(
        surface: &NurbsSurface,
        scope: &mut OperationScope<'_, '_>,
    ) -> core::result::Result<Self, ContextImplicitIsolationError> {
        let work = source_bvh_build_work_units(surface)?;
        scope
            .ledger_mut()
            .charge(NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS, work)?;
        Self::build_partition(surface, surface).map_err(ContextImplicitIsolationError::Kernel)
    }

    /// Contextual hierarchy construction over a positive source subrectangle.
    ///
    /// Work is conservatively admitted from the original source before the
    /// working restriction, Bezier extraction, interval evaluation, or any of
    /// their allocations. The rounded restricted surface supplies only patch
    /// partitions; all hierarchy bounds retain `source` provenance.
    pub fn build_range_in_scope(
        source: &NurbsSurface,
        range: [ParamRange; 2],
        scope: &mut OperationScope<'_, '_>,
    ) -> core::result::Result<Self, ContextImplicitIsolationError> {
        let work = source_bvh_build_work_units(source)?;
        scope
            .ledger_mut()
            .charge(NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS, work)?;
        let partition = source.restricted_to(range)?;
        Self::build_partition(source, &partition).map_err(ContextImplicitIsolationError::Kernel)
    }

    /// Number of Bezier patches, in deterministic source `u`/`v` order.
    pub fn patch_count(&self) -> usize {
        self.patches.len()
    }

    /// Number of nodes in the balanced spatial hierarchy.
    pub fn node_count(&self) -> usize {
        self.hierarchy.node_count()
    }

    /// One extracted patch by deterministic source index.
    pub fn patch(&self, index: usize) -> Option<&NurbsSurface> {
        self.patches.get(index)
    }

    /// Conservative source-provenanced range bound for one patch.
    pub fn patch_bounds(&self, index: usize) -> Option<Aabb3> {
        self.hierarchy.primitive_bounds(index)
    }

    /// Conservative bound containing the whole represented surface.
    pub fn root_bounds(&self) -> Aabb3 {
        self.hierarchy.root_bounds().unwrap_or_else(Aabb3::empty)
    }

    /// Patch indices whose source-range boxes meet `query` after outward
    /// growth by `margin`.
    pub fn query_aabb(&self, query: Aabb3, margin: f64) -> Result<Vec<usize>> {
        self.hierarchy.query_aabb(query, margin)
    }

    /// Candidate patch pairs whose source-range boxes are separated by no
    /// more than `max_separation`. Empty is a certified broad-phase miss.
    pub fn overlapping_patch_pairs(
        &self,
        other: &NurbsSurfaceBvh,
        max_separation: f64,
    ) -> Result<Vec<(usize, usize)>> {
        self.hierarchy
            .overlapping_pairs(&other.hierarchy, max_separation)
    }

    /// Classify one patch's complete source-range box against an implicit
    /// surface after outward growth by a model-space `margin`.
    ///
    /// A non-candidate result certifies that the entire rational patch is
    /// farther than `margin` from the implicit zero set. `Candidate` does not
    /// assert that an intersection exists; it requests subdivision or a
    /// narrower solver.
    pub fn classify_patch_against_implicit(
        &self,
        patch: usize,
        surface: &dyn ImplicitSurface,
        margin: f64,
    ) -> Result<ImplicitBoxRelation> {
        let bounds = self
            .hierarchy
            .primitive_bounds(patch)
            .ok_or(Error::InvalidGeometry {
                reason: "NURBS BVH patch index is out of range",
            })?;
        classify_implicit_box(surface, bounds, margin)
    }

    /// Patches whose complete source-range boxes cannot be excluded from an
    /// implicit zero set within model-space `margin`.
    ///
    /// Hierarchy nodes are pruned by interval field signs before leaf boxes
    /// are classified. The hierarchy and leaves use outward interval
    /// evaluation of the original surface, not rounded extracted patch hulls.
    /// An empty result is therefore a certificate that the complete represented
    /// NURBS surface misses the implicit surface by more than the requested
    /// margin.
    pub fn implicit_candidates(
        &self,
        surface: &dyn ImplicitSurface,
        margin: f64,
    ) -> Result<Vec<usize>> {
        validate_margin(margin)?;
        Ok(self.hierarchy.query_pruned(|bounds| {
            match classify_implicit_box(surface, bounds, margin) {
                Ok(ImplicitBoxRelation::Candidate) | Err(_) => true,
                Ok(ImplicitBoxRelation::Negative | ImplicitBoxRelation::Positive) => false,
            }
        }))
    }

    /// Recursively isolate an implicit zero set with deterministic binary NURBS
    /// subdivision and source-range interval pruning.
    ///
    /// The returned cells conservatively cover every possible contact within
    /// `margin`. `requested_depth` controls spatial refinement without
    /// pretending that a retained box contains a root. `max_candidate_cells`
    /// is a soft memory bound: if the initial Bezier candidate cover already
    /// exceeds it, that cover is preserved and reported as limited rather
    /// than dropping geometry.
    pub fn isolate_implicit_candidates(
        &self,
        surface: &dyn ImplicitSurface,
        margin: f64,
        requested_depth: u32,
        max_candidate_cells: usize,
    ) -> Result<ImplicitPatchIsolation> {
        if max_candidate_cells == 0 {
            return Err(Error::InvalidGeometry {
                reason: "NURBS implicit isolation candidate budget must be positive",
            });
        }
        let mut limiter = LegacyIsolationLimiter {
            max_candidate_cells,
        };
        match isolate_implicit_candidates_engine(
            self,
            surface,
            margin,
            requested_depth,
            &mut limiter,
        ) {
            Ok(isolation) => Ok(isolation),
            Err(IsolationEngineError::Kernel(error)) => Err(error),
            Err(IsolationEngineError::Limiter(error)) => match error {},
        }
    }

    /// Contextual source-provenanced implicit isolation with deterministic work, candidate,
    /// and depth accounting.
    ///
    /// The caller charges its setup unit before contextual hierarchy
    /// construction; [`NurbsSurfaceBvh::build_in_scope`] or
    /// [`NurbsSurfaceBvh::build_range_in_scope`] then admits source-BVH range
    /// evaluation. This method pre-admits each candidate subdivision together
    /// with all four possible child source-range enclosures. Candidate cells
    /// and subdivision depth are observed as high-water resources. Reaching
    /// any configured allowance retains a conservative parent cover and records
    /// the exact attempted snapshot in [`ImplicitIsolationLimits`].
    pub fn isolate_implicit_candidates_in_scope(
        &self,
        surface: &dyn ImplicitSurface,
        margin: f64,
        requested_depth: u32,
        scope: &mut OperationScope<'_, '_>,
    ) -> core::result::Result<ImplicitPatchIsolation, ContextImplicitIsolationError> {
        let mut limiter = ContextIsolationLimiter { scope };
        match isolate_implicit_candidates_engine(
            self,
            surface,
            margin,
            requested_depth,
            &mut limiter,
        ) {
            Ok(isolation) => Ok(isolation),
            Err(IsolationEngineError::Kernel(error)) => {
                Err(ContextImplicitIsolationError::Kernel(error))
            }
            Err(IsolationEngineError::Limiter(error)) => {
                Err(ContextImplicitIsolationError::Policy(error))
            }
        }
    }

    /// Classify one patch against the plane through `origin` with the given
    /// normal. `half_width` is a model-space tolerance on each side of the
    /// plane. Normal scale and signed-distance arithmetic are enclosed with
    /// outward-rounded intervals before a side is certified.
    pub fn classify_patch_against_plane(
        &self,
        patch: usize,
        origin: Point3,
        normal: Vec3,
        half_width: f64,
    ) -> Result<PlanePatchRelation> {
        let plane = validate_plane(origin, normal, half_width)?;
        let bounds = self
            .hierarchy
            .primitive_bounds(patch)
            .ok_or(Error::InvalidGeometry {
                reason: "NURBS BVH patch index is out of range",
            })?;
        Ok(classify_box(
            bounds,
            origin,
            plane.normal,
            plane.scaled_half_width,
        ))
    }

    /// Patches whose source-range boxes meet a plane tolerance slab. Empty
    /// proves the complete surface misses the slab.
    pub fn plane_candidates(
        &self,
        origin: Point3,
        normal: Vec3,
        half_width: f64,
    ) -> Result<Vec<usize>> {
        let plane = validate_plane(origin, normal, half_width)?;
        Ok(self.hierarchy.query_pruned(|bounds| {
            classify_box(bounds, origin, plane.normal, plane.scaled_half_width)
                == PlanePatchRelation::Candidate
        }))
    }
}

fn validate_plane(origin: Point3, normal: Vec3, half_width: f64) -> Result<PlaneFilter> {
    if !finite_point(origin) || !finite_point(normal) {
        return Err(Error::InvalidGeometry {
            reason: "NURBS patch plane must have finite origin and normal",
        });
    }
    if !half_width.is_finite() || half_width < 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "NURBS patch plane half-width must be finite and non-negative",
        });
    }
    let length = normal.norm();
    if !length.is_finite() || length <= LINEAR_RESOLUTION {
        return Err(Error::InvalidGeometry {
            reason: "NURBS patch plane normal is degenerate",
        });
    }
    let nx = Interval::point(normal.x);
    let ny = Interval::point(normal.y);
    let nz = Interval::point(normal.z);
    let norm_squared = nx.square() + ny.square() + nz.square();
    let norm_upper = norm_squared.hi().sqrt().next_up();
    let scaled_half_width = if half_width == 0.0 {
        0.0
    } else {
        (half_width * norm_upper).next_up()
    };
    Ok(PlaneFilter {
        normal,
        scaled_half_width,
    })
}

fn validate_margin(margin: f64) -> Result<()> {
    if !margin.is_finite() || margin < 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "NURBS implicit-surface margin must be finite and non-negative",
        });
    }
    Ok(())
}

enum IsolationEngineError<E> {
    Kernel(Error),
    Limiter(E),
}

trait IsolationLimiter {
    type Error;

    fn observe_candidates(
        &mut self,
        value: usize,
        limits: &mut ImplicitIsolationLimits,
    ) -> core::result::Result<bool, Self::Error>;

    fn observe_depth(
        &mut self,
        value: u64,
        limits: &mut ImplicitIsolationLimits,
    ) -> core::result::Result<bool, Self::Error>;

    fn charge_subdivision(
        &mut self,
        amount: u64,
        limits: &mut ImplicitIsolationLimits,
    ) -> core::result::Result<bool, Self::Error>;

    fn attempted_candidate_count(
        &mut self,
        retained: usize,
        children: usize,
        remaining: usize,
    ) -> core::result::Result<usize, Self::Error>;

    fn record_numeric_resolution(&mut self) -> core::result::Result<(), Self::Error>;
}

struct LegacyIsolationLimiter {
    max_candidate_cells: usize,
}

impl IsolationLimiter for LegacyIsolationLimiter {
    type Error = core::convert::Infallible;

    fn observe_candidates(
        &mut self,
        value: usize,
        limits: &mut ImplicitIsolationLimits,
    ) -> core::result::Result<bool, Self::Error> {
        if value <= self.max_candidate_cells {
            Ok(true)
        } else {
            limits.candidate_budget = Some(self.max_candidate_cells);
            Ok(false)
        }
    }

    fn observe_depth(
        &mut self,
        _value: u64,
        _limits: &mut ImplicitIsolationLimits,
    ) -> core::result::Result<bool, Self::Error> {
        Ok(true)
    }

    fn charge_subdivision(
        &mut self,
        _amount: u64,
        _limits: &mut ImplicitIsolationLimits,
    ) -> core::result::Result<bool, Self::Error> {
        Ok(true)
    }

    fn attempted_candidate_count(
        &mut self,
        retained: usize,
        children: usize,
        remaining: usize,
    ) -> core::result::Result<usize, Self::Error> {
        Ok(retained
            .checked_add(children)
            .and_then(|count| count.checked_add(remaining))
            .expect("allocated candidate cover count fits usize"))
    }

    fn record_numeric_resolution(&mut self) -> core::result::Result<(), Self::Error> {
        Ok(())
    }
}

struct ContextIsolationLimiter<'scope, 'context, 'session> {
    scope: &'scope mut OperationScope<'context, 'session>,
}

impl IsolationLimiter for ContextIsolationLimiter<'_, '_, '_> {
    type Error = OperationPolicyError;

    fn observe_candidates(
        &mut self,
        value: usize,
        limits: &mut ImplicitIsolationLimits,
    ) -> core::result::Result<bool, Self::Error> {
        let value = usize_to_u64(
            value,
            NURBS_IMPLICIT_ISOLATION_CANDIDATES,
            ResourceKind::Items,
        )?;
        let Some(snapshot) = observe_context_limit(
            self.scope,
            NURBS_IMPLICIT_ISOLATION_CANDIDATES,
            ResourceKind::Items,
            value,
        )?
        else {
            return Ok(true);
        };
        if limits.candidate_cells.is_none() {
            limits.candidate_budget = usize::try_from(snapshot.allowed).ok();
            limits.candidate_cells = Some(snapshot);
        }
        Ok(false)
    }

    fn observe_depth(
        &mut self,
        value: u64,
        limits: &mut ImplicitIsolationLimits,
    ) -> core::result::Result<bool, Self::Error> {
        let Some(snapshot) = observe_context_limit(
            self.scope,
            NURBS_IMPLICIT_ISOLATION_DEPTH,
            ResourceKind::Depth,
            value,
        )?
        else {
            return Ok(true);
        };
        if limits.subdivision_depth.is_none() {
            limits.subdivision_depth = Some(snapshot);
        }
        Ok(false)
    }

    fn charge_subdivision(
        &mut self,
        amount: u64,
        limits: &mut ImplicitIsolationLimits,
    ) -> core::result::Result<bool, Self::Error> {
        match self
            .scope
            .ledger_mut()
            .charge(NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS, amount)
        {
            Ok(()) => Ok(true),
            Err(OperationPolicyError::LimitReached(snapshot)) => {
                if limits.subdivision_work.is_none() {
                    limits.subdivision_work = Some(snapshot);
                }
                Ok(false)
            }
            Err(error) => Err(error),
        }
    }

    fn attempted_candidate_count(
        &mut self,
        retained: usize,
        children: usize,
        remaining: usize,
    ) -> core::result::Result<usize, Self::Error> {
        retained
            .checked_add(children)
            .and_then(|count| count.checked_add(remaining))
            .ok_or(OperationPolicyError::AccountingOverflow {
                stage: NURBS_IMPLICIT_ISOLATION_CANDIDATES,
                resource: ResourceKind::Items,
            })
    }

    fn record_numeric_resolution(&mut self) -> core::result::Result<(), Self::Error> {
        self.scope
            .record_numeric_resolution(NURBS_IMPLICIT_ISOLATION_DEPTH);
        Ok(())
    }
}

fn isolate_implicit_candidates_engine<L: IsolationLimiter>(
    hierarchy: &NurbsSurfaceBvh,
    surface: &dyn ImplicitSurface,
    margin: f64,
    requested_depth: u32,
    limiter: &mut L,
) -> core::result::Result<ImplicitPatchIsolation, IsolationEngineError<L::Error>> {
    let mut limits = ImplicitIsolationLimits::default();
    let mut cells: Vec<_> = hierarchy
        .implicit_candidates(surface, margin)
        .map_err(IsolationEngineError::Kernel)?
        .into_iter()
        .map(|source_patch| WorkCell {
            cell: candidate_cell_with_bounds(
                source_patch,
                hierarchy.patches[source_patch].clone(),
                hierarchy
                    .hierarchy
                    .primitive_bounds(source_patch)
                    .expect("candidate index came from this hierarchy"),
                0,
            ),
            blocked: false,
        })
        .collect();

    if !limiter
        .observe_candidates(cells.len(), &mut limits)
        .map_err(IsolationEngineError::Limiter)?
        || !limiter
            .observe_depth(0, &mut limits)
            .map_err(IsolationEngineError::Limiter)?
    {
        return Ok(isolation_result(cells, requested_depth, limits));
    }

    for _ in 0..requested_depth {
        if cells.is_empty() || cells.iter().all(|work| work.blocked) {
            break;
        }
        let previous = core::mem::take(&mut cells);
        let previous_len = previous.len();
        let mut next = Vec::with_capacity(previous_len.saturating_mul(2));
        for (position, mut work) in previous.into_iter().enumerate() {
            if work.blocked || limits.subdivision_work.is_some() {
                work.blocked = true;
                next.push(work);
                continue;
            }

            let attempted_depth = u64::from(work.cell.depth).saturating_add(1);
            if !limiter
                .observe_depth(attempted_depth, &mut limits)
                .map_err(IsolationEngineError::Limiter)?
            {
                work.blocked = true;
                next.push(work);
                continue;
            }
            let subdivision_work = source_child_work_units(&hierarchy.source);
            if !limiter
                .charge_subdivision(subdivision_work, &mut limits)
                .map_err(IsolationEngineError::Limiter)?
            {
                work.blocked = true;
                next.push(work);
                continue;
            }

            let Some(children) = candidate_children(&hierarchy.source, &work.cell, surface, margin)
                .map_err(IsolationEngineError::Kernel)?
            else {
                work.blocked = true;
                limits.parameter_resolution = true;
                limiter
                    .record_numeric_resolution()
                    .map_err(IsolationEngineError::Limiter)?;
                next.push(work);
                continue;
            };

            let remaining = previous_len - position - 1;
            let attempted_candidates = limiter
                .attempted_candidate_count(next.len(), children.len(), remaining)
                .map_err(IsolationEngineError::Limiter)?;
            if limiter
                .observe_candidates(attempted_candidates, &mut limits)
                .map_err(IsolationEngineError::Limiter)?
            {
                next.extend(children.into_iter().map(|cell| WorkCell {
                    cell,
                    blocked: false,
                }));
            } else {
                work.blocked = true;
                next.push(work);
            }
        }
        cells = next;
    }

    Ok(isolation_result(cells, requested_depth, limits))
}

fn observe_context_limit(
    scope: &mut OperationScope<'_, '_>,
    stage: StageId,
    resource: ResourceKind,
    value: u64,
) -> core::result::Result<Option<LimitSnapshot>, OperationPolicyError> {
    match scope.ledger_mut().observe(stage, resource, value) {
        Ok(()) => Ok(None),
        Err(OperationPolicyError::LimitReached(snapshot)) => Ok(Some(snapshot)),
        Err(error) => Err(error),
    }
}

fn usize_to_u64(
    value: usize,
    stage: StageId,
    resource: ResourceKind,
) -> core::result::Result<u64, OperationPolicyError> {
    u64::try_from(value).map_err(|_| OperationPolicyError::AccountingOverflow { stage, resource })
}

fn source_bvh_build_work_units(
    surface: &NurbsSurface,
) -> core::result::Result<u64, OperationPolicyError> {
    let rectangle_slots = source_work_value(
        super::surface_range_interval::source_tensor_span_slots(surface),
    )?;
    let range_work = source_work_value(super::surface_range_interval::position_range_work_units(
        surface,
    ))?;
    rectangle_slots
        .checked_mul(range_work)
        .ok_or(OperationPolicyError::AccountingOverflow {
            stage: NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
            resource: ResourceKind::Work,
        })
}

fn source_child_work_units(surface: &NurbsSurface) -> u64 {
    let range_work = super::surface_range_interval::position_range_work_units(surface)
        .and_then(|work| u64::try_from(work).ok())
        .unwrap_or(u64::MAX);
    range_work.saturating_mul(4).saturating_add(1)
}

fn source_work_value(value: Option<usize>) -> core::result::Result<u64, OperationPolicyError> {
    value.and_then(|value| u64::try_from(value).ok()).ok_or(
        OperationPolicyError::AccountingOverflow {
            stage: NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
            resource: ResourceKind::Work,
        },
    )
}

#[derive(Debug)]
struct WorkCell {
    cell: ImplicitCandidateCell,
    blocked: bool,
}

fn candidate_cell(
    source: &NurbsSurface,
    source_patch: usize,
    patch: NurbsSurface,
    depth: u32,
) -> ImplicitCandidateCell {
    let bounds = super::surface_range_interval::position_range_aabb(source, patch.param_range());
    candidate_cell_with_bounds(source_patch, patch, bounds, depth)
}

fn candidate_cell_with_bounds(
    source_patch: usize,
    patch: NurbsSurface,
    bounds: Aabb3,
    depth: u32,
) -> ImplicitCandidateCell {
    ImplicitCandidateCell {
        source_patch,
        patch,
        bounds,
        depth,
    }
}

fn candidate_children(
    source: &NurbsSurface,
    parent: &ImplicitCandidateCell,
    surface: &dyn ImplicitSurface,
    margin: f64,
) -> Result<Option<Vec<ImplicitCandidateCell>>> {
    let mut choices = Vec::with_capacity(2);
    for (axis, direction) in [Dir::U, Dir::V].into_iter().enumerate() {
        let range = parent.patch.param_range()[axis];
        let midpoint = range.lo + 0.5 * range.width();
        if !(range.lo < midpoint && midpoint < range.hi) {
            continue;
        }
        let (left, right) = parent.patch.split_at(direction, midpoint)?;
        let mut children = Vec::with_capacity(2);
        let mut uncertainty = 0.0;
        for patch in [left, right] {
            let child = candidate_cell(source, parent.source_patch, patch, parent.depth + 1);
            if classify_implicit_box(surface, child.bounds, margin)?
                == ImplicitBoxRelation::Candidate
            {
                let expanded = child.bounds.inflated(margin);
                let width = if expanded.is_finite() {
                    surface.implicit_interval(expanded).width()
                } else {
                    f64::INFINITY
                };
                uncertainty += if width.is_finite() {
                    width.max(0.0)
                } else {
                    f64::INFINITY
                };
                children.push(child);
            }
        }
        choices.push((axis, children, uncertainty));
    }
    choices.sort_by(|a, b| {
        a.1.len()
            .cmp(&b.1.len())
            .then(a.2.total_cmp(&b.2))
            .then(a.0.cmp(&b.0))
    });
    Ok(choices.into_iter().next().map(|(_, children, _)| children))
}

fn isolation_result(
    cells: Vec<WorkCell>,
    requested_depth: u32,
    limits: ImplicitIsolationLimits,
) -> ImplicitPatchIsolation {
    let mut candidates: Vec<_> = cells.into_iter().map(|work| work.cell).collect();
    candidates.sort_by(|a, b| {
        let ar = a.parameter_range();
        let br = b.parameter_range();
        a.source_patch
            .cmp(&b.source_patch)
            .then(ar[0].lo.total_cmp(&br[0].lo))
            .then(ar[1].lo.total_cmp(&br[1].lo))
            .then(ar[0].hi.total_cmp(&br[0].hi))
            .then(ar[1].hi.total_cmp(&br[1].hi))
    });
    ImplicitPatchIsolation {
        candidates,
        requested_depth,
        limits,
    }
}

fn finite_point(point: Vec3) -> bool {
    point.x.is_finite() && point.y.is_finite() && point.z.is_finite()
}

fn classify_box(
    bounds: Aabb3,
    origin: Point3,
    normal: Vec3,
    half_width: f64,
) -> PlanePatchRelation {
    let minimum = Vec3::new(
        if normal.x >= 0.0 {
            bounds.min.x
        } else {
            bounds.max.x
        },
        if normal.y >= 0.0 {
            bounds.min.y
        } else {
            bounds.max.y
        },
        if normal.z >= 0.0 {
            bounds.min.z
        } else {
            bounds.max.z
        },
    );
    let maximum = Vec3::new(
        if normal.x >= 0.0 {
            bounds.max.x
        } else {
            bounds.min.x
        },
        if normal.y >= 0.0 {
            bounds.max.y
        } else {
            bounds.min.y
        },
        if normal.z >= 0.0 {
            bounds.max.z
        } else {
            bounds.min.z
        },
    );
    let lo = signed_distance_interval(minimum, origin, normal);
    let hi = signed_distance_interval(maximum, origin, normal);
    if !lo.lo().is_finite() || !lo.hi().is_finite() || !hi.lo().is_finite() || !hi.hi().is_finite()
    {
        return PlanePatchRelation::Candidate;
    }
    classify_interval(lo.lo(), hi.hi(), half_width)
}

fn signed_distance_interval(point: Point3, origin: Point3, normal: Vec3) -> Interval {
    let dx = Interval::point(point.x) - Interval::point(origin.x);
    let dy = Interval::point(point.y) - Interval::point(origin.y);
    let dz = Interval::point(point.z) - Interval::point(origin.z);
    Interval::point(normal.x) * dx + Interval::point(normal.y) * dy + Interval::point(normal.z) * dz
}

fn classify_interval(minimum: f64, maximum: f64, half_width: f64) -> PlanePatchRelation {
    if maximum < -half_width {
        PlanePatchRelation::Negative
    } else if minimum > half_width {
        PlanePatchRelation::Positive
    } else {
        PlanePatchRelation::Candidate
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::param::ParamRange;
    use crate::surface::{Plane, Sphere, Surface};
    use kcore::operation::{
        AccountingMode, BudgetPlan, ExecutionPolicy, LimitSpec, NumericalPolicy, OperationContext,
        OperationReport, OperationScope, PolicyVersion, SessionPolicy, SessionPrecision,
    };
    use kcore::tolerance::Tolerances;

    fn rational_multi_patch(offset: Vec3) -> NurbsSurface {
        let knots = vec![0.0, 0.0, 0.5, 1.0, 1.0];
        let mut points = Vec::new();
        let mut weights = Vec::new();
        for u in 0..3 {
            for v in 0..3 {
                points
                    .push(Point3::new(f64::from(u), f64::from(v), 0.1 * f64::from(u * v)) + offset);
                weights.push(0.75 + 0.125 * f64::from((u * 3 + v) % 5));
            }
        }
        NurbsSurface::new(1, 1, knots.clone(), knots, points, Some(weights)).unwrap()
    }

    fn positive_quadratic_height(center: f64, epsilon: f64) -> NurbsSurface {
        let z0 = center * center + epsilon;
        let z1 = center * center - center + epsilon;
        let z2 = (1.0 - center) * (1.0 - center) + epsilon;
        NurbsSurface::new(
            2,
            1,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, z0),
                Point3::new(0.0, 1.0, z0),
                Point3::new(0.5, 0.0, z1),
                Point3::new(0.5, 1.0, z1),
                Point3::new(1.0, 0.0, z2),
                Point3::new(1.0, 1.0, z2),
            ],
            None,
        )
        .unwrap()
    }

    fn coincident_patch() -> NurbsSurface {
        NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
            ],
            None,
        )
        .unwrap()
    }

    fn repeated_knot_multi_patch() -> NurbsSurface {
        let mut points = Vec::new();
        for u in 0..6 {
            points.push(Point3::new(f64::from(u), 0.0, 0.0));
            points.push(Point3::new(f64::from(u), 1.0, 0.0));
        }
        NurbsSurface::new(
            2,
            1,
            vec![0.0, 0.0, 0.0, 0.4, 0.4, 0.7, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            points,
            None,
        )
        .unwrap()
    }

    fn contextual_isolation(
        hierarchy: &NurbsSurfaceBvh,
        surface: &dyn ImplicitSurface,
        requested_depth: u32,
        work_allowed: u64,
        candidates_allowed: u64,
        depth_allowed: u64,
    ) -> (ImplicitPatchIsolation, OperationReport) {
        let budget = BudgetPlan::new([
            LimitSpec::new(
                NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                work_allowed,
            ),
            LimitSpec::new(
                NURBS_IMPLICIT_ISOLATION_CANDIDATES,
                ResourceKind::Items,
                AccountingMode::HighWater,
                candidates_allowed,
            ),
            LimitSpec::new(
                NURBS_IMPLICIT_ISOLATION_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                depth_allowed,
            ),
        ])
        .unwrap();
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            budget,
            PolicyVersion::V1,
        );
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let mut scope = OperationScope::new(&context);
        let isolation = hierarchy
            .isolate_implicit_candidates_in_scope(surface, 0.0, requested_depth, &mut scope)
            .unwrap();
        let (result, report) = scope.finish(Ok(isolation)).into_parts();
        (result.unwrap(), report)
    }

    fn usage(report: &OperationReport, stage: StageId) -> LimitSnapshot {
        *report
            .usage()
            .iter()
            .find(|snapshot| snapshot.stage == stage)
            .unwrap()
    }

    fn contextual_build(
        surface: &NurbsSurface,
        work_allowed: u64,
    ) -> (
        core::result::Result<NurbsSurfaceBvh, ContextImplicitIsolationError>,
        OperationReport,
    ) {
        let budget = BudgetPlan::new([LimitSpec::new(
            NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            work_allowed,
        )])
        .unwrap();
        let session = SessionPolicy::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Serial,
            budget,
            PolicyVersion::V1,
        );
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let mut scope = OperationScope::new(&context);
        let result = NurbsSurfaceBvh::build_in_scope(surface, &mut scope);
        let (_, report) = scope.finish(Ok(())).into_parts();
        (result, report)
    }

    #[test]
    fn source_rectangle_build_is_pre_admitted_at_exact_multi_span_boundary() {
        let surface = repeated_knot_multi_patch();
        assert_eq!(
            super::super::surface_range_interval::source_tensor_span_slots(&surface),
            Some(4),
            "the repeated u knot contributes one inspected empty tensor slot"
        );
        assert_eq!(
            super::super::surface_range_interval::position_range_work_units(&surface),
            Some(25),
        );
        let exact_work = source_bvh_build_work_units(&surface).unwrap();
        assert_eq!(exact_work, 100);

        let (exact, exact_report) = contextual_build(&surface, exact_work);
        assert_eq!(exact.unwrap().patch_count(), 3);
        assert_eq!(
            usage(&exact_report, NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS).consumed,
            exact_work
        );
        assert!(exact_report.limit_events().is_empty());

        let (low, low_report) = contextual_build(&surface, exact_work - 1);
        assert_eq!(
            low.unwrap_err(),
            ContextImplicitIsolationError::Policy(OperationPolicyError::LimitReached(
                LimitSnapshot {
                    stage: NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
                    resource: ResourceKind::Work,
                    consumed: exact_work,
                    allowed: exact_work - 1,
                }
            ))
        );
        assert_eq!(
            low_report.limit_events(),
            &[LimitSnapshot {
                stage: NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
                resource: ResourceKind::Work,
                consumed: exact_work,
                allowed: exact_work - 1,
            }]
        );
        assert_eq!(
            usage(&low_report, NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS).consumed,
            0,
            "denied build preflight cannot partially consume Work"
        );
    }

    #[test]
    fn hierarchy_preserves_patch_order_and_conservatively_queries() {
        let surface = rational_multi_patch(Vec3::default());
        let first = NurbsSurfaceBvh::build(&surface).unwrap();
        let second = NurbsSurfaceBvh::build(&surface).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.patch_count(), 4);
        assert_eq!(first.node_count(), 7);
        let expected = [
            [ParamRange::new(0.0, 0.5), ParamRange::new(0.0, 0.5)],
            [ParamRange::new(0.0, 0.5), ParamRange::new(0.5, 1.0)],
            [ParamRange::new(0.5, 1.0), ParamRange::new(0.0, 0.5)],
            [ParamRange::new(0.5, 1.0), ParamRange::new(0.5, 1.0)],
        ];
        for (index, range) in expected.into_iter().enumerate() {
            assert_eq!(first.patch(index).unwrap().param_range(), range);
        }

        let query =
            Aabb3::from_points(&[Point3::new(-0.1, -0.1, -1.0), Point3::new(0.9, 0.9, 1.0)]);
        assert_eq!(first.query_aabb(query, 0.0).unwrap(), vec![0]);
        assert!(
            first
                .query_aabb(Aabb3::from_point(Point3::new(20.0, 20.0, 20.0)), 0.0)
                .unwrap()
                .is_empty()
        );
        for i in 0..=20 {
            for j in 0..=20 {
                let point = surface.eval([f64::from(i) / 20.0, f64::from(j) / 20.0]);
                assert!(first.root_bounds().inflated(1.0e-12).contains(point));
            }
        }
    }

    #[test]
    fn pair_broad_phase_has_no_false_negative_and_proves_far_misses() {
        let surface = rational_multi_patch(Vec3::default());
        let a = NurbsSurfaceBvh::build(&surface).unwrap();
        let same = NurbsSurfaceBvh::build(&surface).unwrap();
        let pairs = a.overlapping_patch_pairs(&same, 0.0).unwrap();
        for index in 0..a.patch_count() {
            assert!(pairs.contains(&(index, index)));
        }

        let far = NurbsSurfaceBvh::build(&rational_multi_patch(Vec3::new(0.0, 0.0, 10.0))).unwrap();
        assert!(a.overlapping_patch_pairs(&far, 0.0).unwrap().is_empty());
    }

    #[test]
    fn implicit_filter_prunes_analytic_zero_sets_without_false_negatives() {
        let hierarchy = NurbsSurfaceBvh::build(&rational_multi_patch(Vec3::default())).unwrap();
        let crossing = Sphere::new(crate::frame::Frame::world(), 1.0).unwrap();
        let candidates = hierarchy.implicit_candidates(&crossing, 0.0).unwrap();
        assert!(!candidates.is_empty());
        assert_eq!(
            candidates,
            hierarchy.implicit_candidates(&crossing, 0.0).unwrap()
        );

        // The patch boundary at (u, v) = (0.5, 0) evaluates to (1, 0, 0),
        // so every patch owning that boundary must survive the interval
        // filter. This checks a real zero, not merely broad box overlap.
        for index in 0..hierarchy.patch_count() {
            let patch = hierarchy.patch(index).unwrap();
            let range = patch.param_range();
            if range[0].contains(0.5) && range[1].contains(0.0) {
                assert!(candidates.contains(&index));
                assert_eq!(
                    hierarchy
                        .classify_patch_against_implicit(index, &crossing, 0.0)
                        .unwrap(),
                    ImplicitBoxRelation::Candidate
                );
            }
        }

        let isolation = hierarchy
            .isolate_implicit_candidates(&crossing, 0.0, 6, 256)
            .unwrap();
        assert!(isolation.is_complete());
        assert!(!isolation.candidates().is_empty());
        assert!(isolation.candidates().iter().all(|cell| cell.depth() == 6));
        let zero = hierarchy.patch(0).unwrap().eval([0.5, 0.0]);
        assert!(isolation.candidates().iter().any(|cell| {
            let range = cell.parameter_range();
            range[0].contains(0.5)
                && range[1].contains(0.0)
                && cell.bounds().inflated(1.0e-12).contains(zero)
        }));

        let far_frame =
            crate::frame::Frame::from_z(Point3::new(0.0, 0.0, 10.0), Vec3::new(0.0, 0.0, 1.0))
                .unwrap();
        let far = Sphere::new(far_frame, 1.0).unwrap();
        assert!(hierarchy.implicit_candidates(&far, 0.0).unwrap().is_empty());
        let far_isolation = hierarchy
            .isolate_implicit_candidates(&far, 0.0, u32::MAX, 1)
            .unwrap();
        assert!(far_isolation.is_proven_empty());
        assert_eq!(far_isolation.requested_depth(), u32::MAX);
    }

    #[test]
    fn implicit_filter_margin_and_invalid_inputs_are_explicit() {
        let hierarchy = NurbsSurfaceBvh::build(&rational_multi_patch(Vec3::default())).unwrap();
        let sphere = Sphere::new(
            crate::frame::Frame::from_z(Point3::new(0.0, 0.0, 3.0), Vec3::new(0.0, 0.0, 1.0))
                .unwrap(),
            1.0,
        )
        .unwrap();
        assert!(
            hierarchy
                .implicit_candidates(&sphere, 0.0)
                .unwrap()
                .is_empty()
        );
        assert!(
            !hierarchy
                .implicit_candidates(&sphere, 2.0)
                .unwrap()
                .is_empty()
        );
        assert!(hierarchy.implicit_candidates(&sphere, -1.0).is_err());
        assert!(hierarchy.implicit_candidates(&sphere, f64::NAN).is_err());
        assert!(
            hierarchy
                .classify_patch_against_implicit(usize::MAX, &sphere, 0.0)
                .is_err()
        );
    }

    #[test]
    fn adaptive_isolation_proves_a_miss_hidden_by_the_source_control_hull() {
        let surface = positive_quadratic_height(0.37, 1.0e-5);
        let hierarchy = NurbsSurfaceBvh::build(&surface).unwrap();
        let plane = Plane::new(crate::frame::Frame::world());
        assert_eq!(
            hierarchy.implicit_candidates(&plane, 1.0e-6).unwrap(),
            vec![0]
        );

        let isolation = hierarchy
            .isolate_implicit_candidates(&plane, 1.0e-6, 16, 64)
            .unwrap();
        assert!(isolation.is_complete());
        assert!(isolation.is_proven_empty());
        assert!(isolation.candidates().is_empty());
        assert_eq!(isolation.requested_depth(), 16);
    }

    #[test]
    fn adaptive_isolation_is_deterministic_and_preserves_budget_limited_cover() {
        let coincident = coincident_patch();
        let hierarchy = NurbsSurfaceBvh::build(&coincident).unwrap();
        let plane = Plane::new(crate::frame::Frame::world());
        let first = hierarchy
            .isolate_implicit_candidates(&plane, 0.0, 5, 2)
            .unwrap();
        let second = hierarchy
            .isolate_implicit_candidates(&plane, 0.0, 5, 2)
            .unwrap();
        assert_eq!(first, second);
        assert!(!first.is_complete());
        assert_eq!(first.limits().candidate_budget(), Some(2));
        assert!(!first.limits().parameter_resolution());
        assert_eq!(first.candidates().len(), 2);
        assert!(first.candidates().iter().all(|cell| cell.depth() == 1));
        assert!(
            first
                .candidates()
                .iter()
                .all(
                    |cell| cell.bounds().inflated(1.0e-12).contains(cell.patch().eval([
                        cell.parameter_range()[0].lerp(0.5),
                        cell.parameter_range()[1].lerp(0.5),
                    ]))
                )
        );
        assert!(
            hierarchy
                .isolate_implicit_candidates(&plane, 0.0, 1, 0)
                .is_err()
        );
    }

    #[test]
    fn contextual_isolation_stage_boundaries_are_exact_and_failure_atomic() {
        let coincident = coincident_patch();
        let hierarchy = NurbsSurfaceBvh::build(&coincident).unwrap();
        let plane = Plane::new(crate::frame::Frame::world());
        let child_work = source_child_work_units(&coincident);

        let (work_low, work_low_report) = contextual_isolation(&hierarchy, &plane, 1, 0, 4, 1);
        let work_limit = LimitSnapshot {
            stage: NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
            resource: ResourceKind::Work,
            consumed: child_work,
            allowed: 0,
        };
        assert_eq!(work_low.limits().subdivision_work(), Some(work_limit));
        assert_eq!(work_low_report.limit_events(), &[work_limit]);
        assert_eq!(
            usage(&work_low_report, NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS).consumed,
            0
        );
        assert_eq!(work_low.candidates().len(), 1);
        assert_eq!(work_low.candidates()[0].depth(), 0);

        let (work_exact, work_exact_report) =
            contextual_isolation(&hierarchy, &plane, 1, child_work, 4, 1);
        let (work_plus, work_plus_report) =
            contextual_isolation(&hierarchy, &plane, 1, child_work + 1, 4, 1);
        assert!(work_exact.limits().subdivision_work().is_none());
        assert_eq!(work_exact.candidates().len(), 2);
        assert_eq!(work_exact, work_plus);
        assert_eq!(
            usage(&work_exact_report, NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS).consumed,
            child_work
        );
        assert_eq!(
            usage(&work_plus_report, NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS).consumed,
            child_work
        );

        let (candidate_low, candidate_low_report) =
            contextual_isolation(&hierarchy, &plane, 1, child_work, 1, 1);
        let candidate_limit = LimitSnapshot {
            stage: NURBS_IMPLICIT_ISOLATION_CANDIDATES,
            resource: ResourceKind::Items,
            consumed: 2,
            allowed: 1,
        };
        assert_eq!(
            candidate_low.limits().candidate_cells(),
            Some(candidate_limit)
        );
        assert_eq!(candidate_low_report.limit_events(), &[candidate_limit]);
        assert_eq!(
            usage(&candidate_low_report, NURBS_IMPLICIT_ISOLATION_CANDIDATES).consumed,
            1
        );
        let (candidate_exact, candidate_exact_report) =
            contextual_isolation(&hierarchy, &plane, 1, child_work, 2, 1);
        let (candidate_plus, candidate_plus_report) =
            contextual_isolation(&hierarchy, &plane, 1, child_work, 3, 1);
        let legacy_exact = hierarchy
            .isolate_implicit_candidates(&plane, 0.0, 1, 2)
            .unwrap();
        let legacy_limited = hierarchy
            .isolate_implicit_candidates(&plane, 0.0, 1, 1)
            .unwrap();
        assert_eq!(legacy_exact, candidate_exact);
        assert_eq!(legacy_limited.candidates(), candidate_low.candidates());
        assert_eq!(
            legacy_limited.limits().candidate_budget(),
            candidate_low.limits().candidate_budget()
        );
        assert_eq!(
            legacy_limited.limits().parameter_resolution(),
            candidate_low.limits().parameter_resolution()
        );
        assert_eq!(candidate_exact, candidate_plus);
        assert_eq!(
            usage(&candidate_exact_report, NURBS_IMPLICIT_ISOLATION_CANDIDATES).consumed,
            2
        );
        assert_eq!(
            usage(&candidate_plus_report, NURBS_IMPLICIT_ISOLATION_CANDIDATES).consumed,
            2
        );

        let (depth_low, depth_low_report) =
            contextual_isolation(&hierarchy, &plane, 1, child_work, 4, 0);
        let depth_limit = LimitSnapshot {
            stage: NURBS_IMPLICIT_ISOLATION_DEPTH,
            resource: ResourceKind::Depth,
            consumed: 1,
            allowed: 0,
        };
        assert_eq!(depth_low.limits().subdivision_depth(), Some(depth_limit));
        assert_eq!(depth_low_report.limit_events(), &[depth_limit]);
        assert_eq!(
            usage(&depth_low_report, NURBS_IMPLICIT_ISOLATION_DEPTH).consumed,
            0
        );
        let (depth_exact, depth_exact_report) =
            contextual_isolation(&hierarchy, &plane, 1, child_work, 4, 1);
        let (depth_plus, depth_plus_report) =
            contextual_isolation(&hierarchy, &plane, 1, child_work + 1, 4, 2);
        assert_eq!(depth_exact, depth_plus);
        assert_eq!(
            usage(&depth_exact_report, NURBS_IMPLICIT_ISOLATION_DEPTH).consumed,
            1
        );
        assert_eq!(
            usage(&depth_plus_report, NURBS_IMPLICIT_ISOLATION_DEPTH).consumed,
            1
        );
    }

    #[test]
    fn contextual_limited_cover_is_deterministic_ordered_and_conservative() {
        let coincident = coincident_patch();
        let hierarchy = NurbsSurfaceBvh::build(&coincident).unwrap();
        let plane = Plane::new(crate::frame::Frame::world());
        let child_work = source_child_work_units(&coincident);
        let (first, first_report) = contextual_isolation(&hierarchy, &plane, 1, child_work, 1, 1);
        let (second, second_report) = contextual_isolation(&hierarchy, &plane, 1, child_work, 1, 1);
        assert_eq!(first, second);
        assert_eq!(first_report.limit_events(), second_report.limit_events());
        assert_eq!(first.candidates().len(), 1);
        let candidate = &first.candidates()[0];
        assert_eq!(candidate.parameter_range(), coincident.param_range());
        for i in 0..=4 {
            for j in 0..=4 {
                let uv = [f64::from(i) / 4.0, f64::from(j) / 4.0];
                assert!(candidate.bounds().contains(coincident.eval(uv)));
            }
        }
    }

    #[test]
    fn contextual_numeric_resolution_is_not_reported_as_a_resource_limit() {
        let lo = 1.0_f64;
        let hi = lo.next_up();
        let surface = NurbsSurface::new(
            1,
            1,
            vec![lo, lo, hi, hi],
            vec![lo, lo, hi, hi],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
            ],
            None,
        )
        .unwrap();
        let hierarchy = NurbsSurfaceBvh::build(&surface).unwrap();
        let plane = Plane::new(crate::frame::Frame::world());
        let (isolation, report) = contextual_isolation(
            &hierarchy,
            &plane,
            1,
            source_child_work_units(&surface),
            4,
            1,
        );
        assert!(isolation.limits().parameter_resolution());
        assert!(isolation.limits().subdivision_work().is_none());
        assert!(isolation.limits().candidate_cells().is_none());
        assert!(isolation.limits().subdivision_depth().is_none());
        assert!(report.limit_events().is_empty());
        assert_eq!(
            report.numeric_resolution_stages(),
            &[NURBS_IMPLICIT_ISOLATION_DEPTH]
        );
    }

    #[test]
    fn adaptive_isolation_reports_parameter_resolution_without_dropping_cover() {
        let lo = 1.0_f64;
        let hi = lo.next_up();
        let surface = NurbsSurface::new(
            1,
            1,
            vec![lo, lo, hi, hi],
            vec![lo, lo, hi, hi],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
            ],
            None,
        )
        .unwrap();
        let hierarchy = NurbsSurfaceBvh::build(&surface).unwrap();
        let plane = Plane::new(crate::frame::Frame::world());
        let isolation = hierarchy
            .isolate_implicit_candidates(&plane, 0.0, 1, 4)
            .unwrap();
        assert!(!isolation.is_complete());
        assert!(isolation.limits().parameter_resolution());
        assert_eq!(isolation.limits().candidate_budget(), None);
        assert_eq!(isolation.candidates().len(), 1);
        assert_eq!(
            isolation.candidates()[0].parameter_range(),
            [ParamRange::new(lo, hi), ParamRange::new(lo, hi),]
        );
    }

    #[test]
    fn plane_control_hulls_certify_sides_without_sampled_proof() {
        let hierarchy = NurbsSurfaceBvh::build(&rational_multi_patch(Vec3::default())).unwrap();
        let normal = Vec3::new(1.0, 0.0, 0.0);
        assert_eq!(
            hierarchy
                .plane_candidates(Point3::new(0.75, 0.0, 0.0), normal, 0.0)
                .unwrap(),
            vec![0, 1]
        );
        assert_eq!(
            hierarchy
                .plane_candidates(Point3::new(0.75, 0.0, 0.0), normal * 10.0, 0.0)
                .unwrap(),
            vec![0, 1]
        );
        assert_eq!(
            hierarchy
                .classify_patch_against_plane(0, Point3::new(0.75, 0.0, 0.0), normal, 0.0)
                .unwrap(),
            PlanePatchRelation::Candidate
        );
        assert_eq!(
            hierarchy
                .classify_patch_against_plane(2, Point3::new(0.75, 0.0, 0.0), normal, 0.0)
                .unwrap(),
            PlanePatchRelation::Positive
        );
        assert!(
            hierarchy
                .plane_candidates(Point3::new(-1.0, 0.0, 0.0), normal, 0.0)
                .unwrap()
                .is_empty()
        );
        assert!(
            hierarchy
                .plane_candidates(Point3::new(3.0, 0.0, 0.0), normal, 0.0)
                .unwrap()
                .is_empty()
        );
        assert_eq!(
            hierarchy
                .plane_candidates(Point3::new(0.9, 0.0, 0.0), normal, 0.2)
                .unwrap(),
            vec![0, 1, 2, 3]
        );
    }

    #[test]
    fn certified_plane_sides_contain_every_evaluated_patch_point() {
        let hierarchy = NurbsSurfaceBvh::build(&rational_multi_patch(Vec3::default())).unwrap();
        let origin = Point3::new(0.8, 0.7, 0.05);
        let normal = Vec3::new(1.0, -0.3, 0.2).normalized().unwrap();
        let half_width = 0.01;
        for index in 0..hierarchy.patch_count() {
            let patch = hierarchy.patch(index).unwrap();
            let relation = hierarchy
                .classify_patch_against_plane(index, origin, normal, half_width)
                .unwrap();
            let range = patch.param_range();
            for i in 0..=20 {
                for j in 0..=20 {
                    let point = patch.eval([
                        range[0].lerp(f64::from(i) / 20.0),
                        range[1].lerp(f64::from(j) / 20.0),
                    ]);
                    let distance = normal.dot(point - origin);
                    match relation {
                        PlanePatchRelation::Negative => assert!(distance < -half_width),
                        PlanePatchRelation::Positive => assert!(distance > half_width),
                        PlanePatchRelation::Candidate => {}
                    }
                }
            }
        }
    }

    #[test]
    fn invalid_plane_and_unclamped_surface_are_rejected() {
        let hierarchy = NurbsSurfaceBvh::build(&rational_multi_patch(Vec3::default())).unwrap();
        assert!(
            hierarchy
                .plane_candidates(Point3::default(), Vec3::default(), 0.0)
                .is_err()
        );
        assert!(
            hierarchy
                .plane_candidates(Point3::default(), Vec3::new(1.0, 0.0, 0.0), f64::NAN)
                .is_err()
        );
        assert!(hierarchy
            .classify_patch_against_plane(100, Point3::default(), Vec3::new(1.0, 0.0, 0.0), 0.0,)
            .is_err());

        let unclamped = NurbsSurface::new(
            1,
            1,
            vec![0.0, 1.0, 2.0, 3.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point3::default(); 4],
            None,
        )
        .unwrap();
        assert!(NurbsSurfaceBvh::build(&unclamped).is_err());
    }
}
