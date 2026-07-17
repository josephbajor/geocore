//! Certified point classification against faces and solid bodies.
//!
//! First rung of the boolean ladder: `unite`/`subtract`/`intersect` need to
//! decide where fragment witness points sit relative to the operand bodies,
//! and that decision must be certified or honestly refused — never guessed.
//!
//! The algorithm is general over topology (any loop structure, any number of
//! shells, holes, non-convex boundaries). Metric decisions use conservative
//! interval filters; combinatorial decisions (trim containment, ray-crossing
//! parity) use the exact `orient2d`/`orient3d` predicates on stored vertex
//! coordinates, never on derived intersection points. The certified slice
//! covers faces on planar surfaces bounded by straight line edges — exactly
//! the face class the first boolean rungs produce and consume. Every other
//! configuration returns [`PointFaceVerdict::Indeterminate`] /
//! [`PointBodyVerdict::Indeterminate`] with a stable reason instead of an
//! uncertified answer.
//!
//! Near-boundary honesty: points within the session linear resolution of a
//! face are boundary sites; points inside a face's *guard band* — a
//! conservative widening covering vertex near-coplanarity and entity
//! tolerances — are `Indeterminate` by design, because no verdict about them
//! is certifiable from the stored geometry.

use kcore::interval::Interval;
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationScope, ResourceKind, StageId,
};
use kcore::predicates::{Orientation, orient2d, orient3d, polygon_orientation2d_iter};
use kgeom::vec::Point3;
use ktopo::entity::{
    BodyKind, EdgeId as RawEdgeId, FaceId as RawFaceId, Loop, RegionKind, VertexId as RawVertexId,
};
use ktopo::geom::{CurveGeom, SurfaceGeom};
use ktopo::store::Store;

use crate::error::{Error, Result};
use crate::operation::{OperationOutcome, OperationSettings};
use crate::session::Part;
use crate::{BodyId, EdgeId, EntityKind, FaceId, PartId, VertexId};

/// Cumulative predicate/scan work performed by one classification query.
pub const POINT_CLASSIFICATION_WORK: StageId = known_stage("kernel.classify.point-work");
/// High-water count of certification ray candidates attempted by one query.
pub const POINT_CLASSIFICATION_RAYS: StageId = known_stage("kernel.classify.point-rays");

const fn known_stage(value: &'static str) -> StageId {
    match StageId::new(value) {
        Ok(stage) => stage,
        Err(_) => panic!("invalid built-in classification stage identifier"),
    }
}

/// Deterministic certification ray directions, tried in order until one
/// avoids every degenerate contact. The components are fixed non-axis
/// non-rational-ratio constants so rays are generic for axis-aligned models
/// while remaining bit-identical across platforms.
const RAY_DIRECTIONS: [[f64; 3]; 12] = [
    [
        0.540_302_305_868_139_7,
        0.454_648_713_412_841_4,
        0.708_073_418_273_571_2,
    ],
    [
        -0.614_160_275_663_524_9,
        0.573_519_986_072_457_3,
        0.542_432_360_954_101_2,
    ],
    [
        0.657_591_465_871_960_8,
        -0.526_432_162_877_356,
        0.538_809_045_103_547_8,
    ],
    [
        0.512_320_949_045_461_7,
        0.590_929_742_636_949_2,
        -0.622_646_386_970_952_2,
    ],
    [
        -0.577_326_150_859_446_7,
        -0.516_837_559_871_251_1,
        0.632_057_887_465_960_6,
    ],
    [
        -0.539_007_373_734_243_9,
        0.628_910_684_142_918_6,
        -0.560_236_496_412_824_9,
    ],
    [
        0.601_486_251_643_218_4,
        -0.549_618_262_363_559_1,
        -0.579_770_762_559_874_3,
    ],
    [
        -0.582_195_978_871_223_6,
        -0.567_924_314_712_268_5,
        -0.581_837_121_425_346_9,
    ],
    [
        0.804_338_916_236_711_2,
        0.348_129_534_921_175_8,
        0.481_282_614_973_869_4,
    ],
    [
        0.331_265_268_543_186_2,
        0.812_573_684_293_764_1,
        0.479_463_301_293_856_7,
    ],
    [
        0.343_746_581_756_912_4,
        0.469_852_374_615_298_3,
        0.813_074_926_385_617_2,
    ],
    [
        -0.769_412_385_617_293_4,
        0.352_986_417_293_854_6,
        -0.532_814_926_371_594_8,
    ],
];

/// Built-in accounting ceilings for one point-classification query.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PointClassificationBudgetProfile;

impl PointClassificationBudgetProfile {
    /// Returns generous exact ceilings for one classification query.
    pub fn v1_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                POINT_CLASSIFICATION_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                16_000_000,
            ),
            LimitSpec::new(
                POINT_CLASSIFICATION_RAYS,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                RAY_DIRECTIONS.len() as u64,
            ),
        ])
        .expect("built-in point-classification budget is valid")
    }
}

/// Typed request to classify one model-space point against one face.
#[derive(Debug, Clone, PartialEq)]
pub struct ClassifyPointOnFaceRequest {
    pub(crate) face: FaceId,
    pub(crate) point: Point3,
    pub(crate) settings: OperationSettings,
}

impl ClassifyPointOnFaceRequest {
    /// Construct a request with default operation settings.
    pub fn new(face: FaceId, point: Point3) -> Self {
        Self {
            face,
            point,
            settings: OperationSettings::default(),
        }
    }

    /// Replace contextual operation settings.
    pub fn with_settings(mut self, settings: OperationSettings) -> Self {
        self.settings = settings;
        self
    }

    /// Face receiving the query.
    pub fn face(&self) -> FaceId {
        self.face.clone()
    }

    /// Model-space query point.
    pub const fn point(&self) -> Point3 {
        self.point
    }

    /// Contextual operation settings.
    pub const fn settings(&self) -> &OperationSettings {
        &self.settings
    }
}

/// Typed request to classify one model-space point against one solid body.
#[derive(Debug, Clone, PartialEq)]
pub struct ClassifyPointInBodyRequest {
    pub(crate) body: BodyId,
    pub(crate) point: Point3,
    pub(crate) settings: OperationSettings,
}

impl ClassifyPointInBodyRequest {
    /// Construct a request with default operation settings.
    pub fn new(body: BodyId, point: Point3) -> Self {
        Self {
            body,
            point,
            settings: OperationSettings::default(),
        }
    }

    /// Replace contextual operation settings.
    pub fn with_settings(mut self, settings: OperationSettings) -> Self {
        self.settings = settings;
        self
    }

    /// Body receiving the query.
    pub fn body(&self) -> BodyId {
        self.body.clone()
    }

    /// Model-space query point.
    pub const fn point(&self) -> Point3 {
        self.point
    }

    /// Contextual operation settings.
    pub const fn settings(&self) -> &OperationSettings {
        &self.settings
    }
}

/// Where a point certified as on-face sits within the face's closed set.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum PointFaceSite {
    /// Strictly inside the trimmed face.
    Interior,
    /// Within tolerance of one bounding edge, away from its vertices.
    EdgeInterior(EdgeId),
    /// Within tolerance of one bounding vertex.
    AtVertex(VertexId),
}

/// Certified relation of a point to one face, or an honest refusal.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum PointFaceVerdict {
    /// The point is certified on the face at the reported site.
    On(PointFaceSite),
    /// The point is certified off the face.
    Off,
    /// No verdict is certifiable from the stored geometry.
    Indeterminate {
        /// Stable explanation for the refused verdict.
        reason: &'static str,
    },
}

/// Point/face classification evidence tied to exact facade identity.
#[derive(Debug, Clone, PartialEq)]
pub struct PointFaceClassification {
    pub(crate) face: FaceId,
    pub(crate) verdict: PointFaceVerdict,
}

impl PointFaceClassification {
    /// Queried face identity.
    pub fn face(&self) -> FaceId {
        self.face.clone()
    }

    /// Certified verdict or honest refusal.
    pub const fn verdict(&self) -> &PointFaceVerdict {
        &self.verdict
    }
}

/// Certified relation of a point to one solid body, or an honest refusal.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum PointBodyVerdict {
    /// Certified strictly inside the body's material.
    Interior,
    /// Certified strictly outside the body's material.
    Exterior,
    /// Certified on the body boundary at the reported face site.
    Boundary {
        /// Boundary face carrying the contact.
        face: FaceId,
        /// Site of the contact within that face.
        site: PointFaceSite,
    },
    /// No verdict is certifiable from the stored geometry.
    Indeterminate {
        /// Stable explanation for the refused verdict.
        reason: &'static str,
    },
}

/// Re-checkable evidence for one certified interior/exterior verdict.
///
/// The verdict was proven by counting transversal crossings of the open
/// segment from the query point to `far_point` against the exact
/// vertex-polygon triangulation of every boundary face; every crossing sign
/// was certified by exact `orient3d` evaluations.
#[derive(Debug, Clone, PartialEq)]
pub struct RayParityWitness {
    pub(crate) far_point: Point3,
    pub(crate) crossings: u32,
    pub(crate) crossed_faces: Vec<FaceId>,
}

impl RayParityWitness {
    /// Segment endpoint certified outside the body's bounding box.
    pub const fn far_point(&self) -> Point3 {
        self.far_point
    }

    /// Number of certified transversal boundary crossings.
    pub const fn crossings(&self) -> u32 {
        self.crossings
    }

    /// Faces crossed, one entry per crossing in traversal order.
    pub fn crossed_faces(&self) -> &[FaceId] {
        &self.crossed_faces
    }
}

/// Point/body classification evidence tied to exact facade identity.
#[derive(Debug, Clone, PartialEq)]
pub struct PointBodyClassification {
    pub(crate) body: BodyId,
    pub(crate) verdict: PointBodyVerdict,
    pub(crate) witness: Option<RayParityWitness>,
}

impl PointBodyClassification {
    /// Queried body identity.
    pub fn body(&self) -> BodyId {
        self.body.clone()
    }

    /// Certified verdict or honest refusal.
    pub const fn verdict(&self) -> &PointBodyVerdict {
        &self.verdict
    }

    /// Parity evidence backing an interior/exterior verdict.
    pub const fn witness(&self) -> Option<&RayParityWitness> {
        self.witness.as_ref()
    }
}

const GAP_PLANAR_ONLY: &str =
    "point classification is certified only for faces on planar surfaces in this slice";
const GAP_LINE_EDGES_ONLY: &str =
    "point classification is certified only for faces bounded by straight line edges";
const GAP_BOUNDED_EDGES_ONLY: &str =
    "point classification requires bounded edges with vertices at both ends";
const GAP_NO_LOOPS: &str = "point classification requires at least one bounding loop";
const GAP_SHORT_LOOP: &str = "a face boundary loop has fewer than three vertices";
const GAP_GUARD_BAND: &str = "the point lies in the classification guard band of a face";
const GAP_DEGENERATE_PROJECTION: &str =
    "a face boundary loop projects degenerately on every coordinate plane";
const GAP_PROJECTED_CONTACT: &str =
    "the point projects onto a face boundary without a certified metric contact";
const GAP_EAR_SEARCH: &str = "face triangulation found no certified ear in a boundary loop";
const GAP_RAYS_EXHAUSTED: &str =
    "every certification ray candidate met a degenerate boundary contact";

impl Part<'_> {
    /// Classify one model-space point against one face through a facade-owned
    /// operation scope.
    ///
    /// The verdict is certified (exact predicates plus conservative interval
    /// filters) or [`PointFaceVerdict::Indeterminate`] with a stable reason.
    /// Wrong-part and stale identities and non-finite query points are
    /// rejected before the scope starts.
    pub fn classify_point_on_face(
        &self,
        request: ClassifyPointOnFaceRequest,
    ) -> Result<OperationOutcome<PointFaceClassification>> {
        let ClassifyPointOnFaceRequest {
            face,
            point,
            settings,
        } = request;
        self.face(face.clone())?;
        require_finite_point(point)?;

        let context = settings
            .context(self.policy)?
            .with_family_budget_defaults(PointClassificationBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        let linear = settings.tolerances().linear();
        let result = classify_on_face_impl(self, &face, as_coords(point), linear, &mut scope);
        Ok(scope.finish_typed(result))
    }

    /// Classify one model-space point against one solid body through a
    /// facade-owned operation scope.
    ///
    /// Boundary contacts are certified per face first; interior/exterior
    /// verdicts are then proven by exact ray-crossing parity and carry a
    /// re-checkable [`RayParityWitness`]. Any face outside the certified
    /// slice, guard-band contact, or exhausted ray budget yields
    /// [`PointBodyVerdict::Indeterminate`] instead of a guess.
    pub fn classify_point_in_body(
        &self,
        request: ClassifyPointInBodyRequest,
    ) -> Result<OperationOutcome<PointBodyClassification>> {
        let ClassifyPointInBodyRequest {
            body,
            point,
            settings,
        } = request;
        self.body(body.clone())?;
        require_finite_point(point)?;

        let context = settings
            .context(self.policy)?
            .with_family_budget_defaults(PointClassificationBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        let linear = settings.tolerances().linear();
        let result = classify_in_body_impl(self, &body, as_coords(point), linear, &mut scope);
        Ok(scope.finish_typed(result))
    }
}

fn require_finite_point(point: Point3) -> Result<()> {
    if [point.x, point.y, point.z].iter().all(|c| c.is_finite()) {
        Ok(())
    } else {
        Err(Error::Core {
            source: kcore::error::Error::InvalidGeometry {
                reason: "classification query point must be finite",
            },
        })
    }
}

fn as_coords(point: Point3) -> [f64; 3] {
    [point.x, point.y, point.z]
}

fn read<T>(result: kcore::error::Result<T>) -> Result<T> {
    result.map_err(|source| Error::InconsistentTopology { source })
}

fn charge(scope: &mut OperationScope<'_, '_>, amount: u64) -> Result<()> {
    scope
        .ledger_mut()
        .charge(POINT_CLASSIFICATION_WORK, amount)
        .map_err(Error::from)
}

/// One boundary loop of a supported face as an ordered exact vertex ring.
struct PreparedLoop {
    vertices: Vec<PreparedVertex>,
}

struct PreparedVertex {
    vertex: RawVertexId,
    point: [f64; 3],
    /// Edge from this vertex to the ring successor (the owning fin's edge).
    edge: RawEdgeId,
    vertex_tol: f64,
    edge_tol: f64,
}

/// A face admitted to the certified planar slice, with its metric bands.
struct PreparedFace {
    raw: RawFaceId,
    origin: [f64; 3],
    normal: [f64; 3],
    normal_sq: Interval,
    loops: Vec<PreparedLoop>,
    /// Metric half-width of the on-surface band (session linear resolution).
    on_tol: f64,
    /// Conservative half-width outside which off-face is certified: covers
    /// vertex/edge deviation from the exact surface plus entity tolerances.
    guard: f64,
    /// Coordinate axis dropped by the exact trim-classification projection.
    drop_axis: usize,
}

enum PrepOutcome {
    Ready(PreparedFace),
    Gap(&'static str),
}

fn prepare_face(store: &Store, raw: RawFaceId, linear: f64) -> Result<PrepOutcome> {
    let face = read(store.get(raw))?;
    let SurfaceGeom::Plane(plane) = read(store.surface(face.surface))? else {
        return Ok(PrepOutcome::Gap(GAP_PLANAR_ONLY));
    };
    let origin = as_coords(plane.frame().origin());
    let z = plane.frame().z();
    let normal = [z.x, z.y, z.z];
    let normal_sq = Interval::point(normal[0]).square()
        + Interval::point(normal[1]).square()
        + Interval::point(normal[2]).square();
    if normal_sq.lo() <= 0.0 || normal_sq.lo().is_nan() {
        return Ok(PrepOutcome::Gap(GAP_DEGENERATE_PROJECTION));
    }
    if face.loops().is_empty() {
        return Ok(PrepOutcome::Gap(GAP_NO_LOOPS));
    }

    let mut loops = Vec::with_capacity(face.loops().len());
    let mut max_elem_tol = linear;
    let mut deviation_sq_hi: f64 = 0.0;
    for &loop_id in face.loops() {
        let ring = read(store.get::<Loop>(loop_id))?;
        let mut vertices = Vec::with_capacity(ring.fins().len());
        for &fin_id in ring.fins() {
            let fin = read(store.get(fin_id))?;
            let edge = read(store.get(fin.edge))?;
            let Some(curve_id) = edge.curve else {
                return Ok(PrepOutcome::Gap(GAP_LINE_EDGES_ONLY));
            };
            if !matches!(read(store.curve(curve_id))?, CurveGeom::Line(_)) {
                return Ok(PrepOutcome::Gap(GAP_LINE_EDGES_ONLY));
            }
            let tail = if fin.sense.is_forward() {
                edge.vertices[0]
            } else {
                edge.vertices[1]
            };
            let Some(tail) = tail else {
                return Ok(PrepOutcome::Gap(GAP_BOUNDED_EDGES_ONLY));
            };
            let point = as_coords(read(store.vertex_position(tail))?);
            let vertex_tol =
                linear.max(read(store.get(tail))?.tolerance.map_or(0.0, |t| t.value()));
            let edge_tol = linear.max(edge.tolerance.map_or(0.0, |t| t.value()));
            max_elem_tol = max_elem_tol.max(vertex_tol).max(edge_tol);
            let offset = dot_offset(point, origin, normal);
            if let Some(dev_sq) = offset.square().checked_div(normal_sq) {
                deviation_sq_hi = deviation_sq_hi.max(dev_sq.hi());
            } else {
                return Ok(PrepOutcome::Gap(GAP_DEGENERATE_PROJECTION));
            }
            vertices.push(PreparedVertex {
                vertex: tail,
                point,
                edge: fin.edge,
                vertex_tol,
                edge_tol,
            });
        }
        if vertices.len() < 3 {
            return Ok(PrepOutcome::Gap(GAP_SHORT_LOOP));
        }
        loops.push(PreparedLoop { vertices });
    }

    let deviation = deviation_sq_hi.max(0.0).sqrt().next_up();
    let guard = 4.0 * (deviation + max_elem_tol);
    if !guard.is_finite() {
        return Ok(PrepOutcome::Gap(GAP_DEGENERATE_PROJECTION));
    }
    let Some(drop_axis) = choose_drop_axis(&loops) else {
        return Ok(PrepOutcome::Gap(GAP_DEGENERATE_PROJECTION));
    };
    Ok(PrepOutcome::Ready(PreparedFace {
        raw,
        origin,
        normal,
        normal_sq,
        loops,
        on_tol: linear,
        guard,
        drop_axis,
    }))
}

/// Conservative interval enclosure of `normal · (point - origin)`.
fn dot_offset(point: [f64; 3], origin: [f64; 3], normal: [f64; 3]) -> Interval {
    let mut sum = Interval::point(0.0);
    for axis in 0..3 {
        sum = sum
            + Interval::point(normal[axis])
                * (Interval::point(point[axis]) - Interval::point(origin[axis]));
    }
    sum
}

/// Conservative interval enclosure of `|a - b|²`.
fn distance_sq(a: [f64; 3], b: [f64; 3]) -> Interval {
    let mut sum = Interval::point(0.0);
    for axis in 0..3 {
        sum = sum + (Interval::point(a[axis]) - Interval::point(b[axis])).square();
    }
    sum
}

fn project(point: [f64; 3], drop_axis: usize) -> [f64; 2] {
    match drop_axis {
        0 => [point[1], point[2]],
        1 => [point[0], point[2]],
        _ => [point[0], point[1]],
    }
}

/// Pick the dropped axis with the largest projected outer-loop area whose
/// projection keeps every loop's exact polygon orientation nonzero.
fn choose_drop_axis(loops: &[PreparedLoop]) -> Option<usize> {
    let outer = &loops[0].vertices;
    let mut scored: [(usize, f64); 3] = [(0, 0.0), (1, 0.0), (2, 0.0)];
    for (axis, score) in scored.iter_mut() {
        let mut area = 0.0_f64;
        for (i, v) in outer.iter().enumerate() {
            let a = project(v.point, *axis);
            let b = project(outer[(i + 1) % outer.len()].point, *axis);
            area += a[0] * b[1] - b[0] * a[1];
        }
        *score = area.abs();
    }
    scored.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(core::cmp::Ordering::Equal));
    for (axis, _) in scored {
        let all_nonzero = loops.iter().all(|ring| {
            polygon_orientation2d_iter(ring.vertices.iter().map(|v| project(v.point, axis)))
                != Orientation::Zero
        });
        if all_nonzero {
            return Some(axis);
        }
    }
    None
}

enum BandOutcome {
    OffMargin,
    OnBand,
    Gap,
}

/// Certified three-zone distance test of the point against the face plane.
fn plane_band(face: &PreparedFace, point: [f64; 3]) -> BandOutcome {
    let offset_sq = dot_offset(point, face.origin, face.normal).square();
    let on = offset_sq - Interval::point(face.on_tol).square() * face.normal_sq;
    if on.hi() <= 0.0 {
        return BandOutcome::OnBand;
    }
    let off = offset_sq - Interval::point(face.guard).square() * face.normal_sq;
    if off.lo() >= 0.0 {
        return BandOutcome::OffMargin;
    }
    BandOutcome::Gap
}

enum ScanOutcome<T> {
    Hit(T),
    Clear,
    Gap,
}

/// Certified vertex contact scan: a hit is within the vertex tolerance and a
/// clear pass certifies at least the guard clearance from every vertex.
fn vertex_scan(face: &PreparedFace, point: [f64; 3]) -> ScanOutcome<RawVertexId> {
    let guard_sq = Interval::point(face.guard).square();
    let mut undecided = false;
    for ring in &face.loops {
        for v in &ring.vertices {
            let d_sq = distance_sq(point, v.point);
            if (d_sq - Interval::point(v.vertex_tol).square()).hi() <= 0.0 {
                return ScanOutcome::Hit(v.vertex);
            }
            if (d_sq - guard_sq).lo() >= 0.0 {
                continue;
            }
            undecided = true;
        }
    }
    if undecided {
        ScanOutcome::Gap
    } else {
        ScanOutcome::Clear
    }
}

/// Certified edge contact scan, run after a clear vertex scan so endpoint
/// regions already carry a guard-clearance certificate.
fn edge_scan(face: &PreparedFace, point: [f64; 3]) -> ScanOutcome<RawEdgeId> {
    let guard_sq = Interval::point(face.guard).square();
    let mut undecided = false;
    for ring in &face.loops {
        let count = ring.vertices.len();
        for i in 0..count {
            let v0 = &ring.vertices[i];
            let v1 = &ring.vertices[(i + 1) % count];
            let mut w = [Interval::point(0.0); 3];
            let mut u = [Interval::point(0.0); 3];
            let mut w_sq = Interval::point(0.0);
            let mut c = Interval::point(0.0);
            for axis in 0..3 {
                w[axis] = Interval::point(v1.point[axis]) - Interval::point(v0.point[axis]);
                u[axis] = Interval::point(point[axis]) - Interval::point(v0.point[axis]);
                w_sq = w_sq + w[axis].square();
                c = c + u[axis] * w[axis];
            }
            // dist(line)² ≤ t² ⇔ |u×w|² ≤ t²|w|² for |w|² > 0. The cross
            // product keeps cancellation at coordinate level, so squaring a
            // near-zero enclosure stays far below t² instead of drowning it
            // in the rounding noise of large canceling products.
            let line_num = (u[1] * w[2] - u[2] * w[1]).square()
                + (u[2] * w[0] - u[0] * w[2]).square()
                + (u[0] * w[1] - u[1] * w[0]).square();
            if (line_num - guard_sq * w_sq).lo() >= 0.0 {
                continue;
            }
            let span_inside = c.lo() >= 0.0 && (w_sq - c).lo() >= 0.0;
            let span_outside = c.hi() < 0.0 || (w_sq - c).hi() < 0.0;
            if span_outside {
                // The nearest segment point is an endpoint, and the vertex
                // scan already certified guard clearance from it.
                continue;
            }
            let within_tol = (line_num - Interval::point(v0.edge_tol).square() * w_sq).hi() <= 0.0;
            if within_tol && span_inside {
                return ScanOutcome::Hit(v0.edge);
            }
            undecided = true;
        }
    }
    if undecided {
        ScanOutcome::Gap
    } else {
        ScanOutcome::Clear
    }
}

enum WindingOutcome {
    Inside,
    Outside,
    Gap,
}

/// Exact even-odd trim containment of the projected point against every
/// projected boundary loop (half-open crossing rule, exact `orient2d`).
fn winding_parity(face: &PreparedFace, point: [f64; 3]) -> WindingOutcome {
    let p = project(point, face.drop_axis);
    let mut crossings: u64 = 0;
    for ring in &face.loops {
        let count = ring.vertices.len();
        for i in 0..count {
            let a = project(ring.vertices[i].point, face.drop_axis);
            let b = project(ring.vertices[(i + 1) % count].point, face.drop_axis);
            let a_above = a[1] > p[1];
            let b_above = b[1] > p[1];
            if a_above == b_above {
                continue;
            }
            match orient2d(a, b, p) {
                Orientation::Zero => return WindingOutcome::Gap,
                side => {
                    let crossed = if a_above {
                        side == Orientation::Negative
                    } else {
                        side == Orientation::Positive
                    };
                    if crossed {
                        crossings += 1;
                    }
                }
            }
        }
    }
    if crossings % 2 == 1 {
        WindingOutcome::Inside
    } else {
        WindingOutcome::Outside
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum RawSite {
    Interior,
    EdgeInterior(RawEdgeId),
    AtVertex(RawVertexId),
}

enum SiteOutcome {
    On(RawSite),
    Off,
    Gap(&'static str),
}

/// Full certified point-vs-face test for one prepared face.
fn face_site(
    face: &PreparedFace,
    point: [f64; 3],
    scope: &mut OperationScope<'_, '_>,
) -> Result<SiteOutcome> {
    let vertex_count: u64 = face.loops.iter().map(|l| l.vertices.len() as u64).sum();
    charge(scope, 1 + 3 * vertex_count)?;
    match plane_band(face, point) {
        BandOutcome::OffMargin => return Ok(SiteOutcome::Off),
        BandOutcome::Gap => return Ok(SiteOutcome::Gap(GAP_GUARD_BAND)),
        BandOutcome::OnBand => {}
    }
    match vertex_scan(face, point) {
        ScanOutcome::Hit(vertex) => return Ok(SiteOutcome::On(RawSite::AtVertex(vertex))),
        ScanOutcome::Gap => return Ok(SiteOutcome::Gap(GAP_GUARD_BAND)),
        ScanOutcome::Clear => {}
    }
    match edge_scan(face, point) {
        ScanOutcome::Hit(edge) => return Ok(SiteOutcome::On(RawSite::EdgeInterior(edge))),
        ScanOutcome::Gap => return Ok(SiteOutcome::Gap(GAP_GUARD_BAND)),
        ScanOutcome::Clear => {}
    }
    match winding_parity(face, point) {
        WindingOutcome::Inside => Ok(SiteOutcome::On(RawSite::Interior)),
        WindingOutcome::Outside => Ok(SiteOutcome::Off),
        WindingOutcome::Gap => Ok(SiteOutcome::Gap(GAP_PROJECTED_CONTACT)),
    }
}

/// Exact ear-clipping triangulation of one loop in its face projection.
///
/// Exactly collinear corners are dropped first (their zero-area triangles
/// cannot carry a transversal crossing). Returns `None` when no certified
/// ear exists, which routes the query to an honest refusal.
fn triangulate_loop(
    ring: &PreparedLoop,
    drop_axis: usize,
    scope: &mut OperationScope<'_, '_>,
) -> Result<Option<Vec<Triangle>>> {
    let points2: Vec<[f64; 2]> = ring
        .vertices
        .iter()
        .map(|v| project(v.point, drop_axis))
        .collect();
    let mut idx: Vec<usize> = (0..ring.vertices.len()).collect();

    loop {
        let mut removed = false;
        for k in 0..idx.len() {
            let count = idx.len();
            let a = points2[idx[(k + count - 1) % count]];
            let b = points2[idx[k]];
            let c = points2[idx[(k + 1) % count]];
            if orient2d(a, b, c) == Orientation::Zero {
                idx.remove(k);
                removed = true;
                break;
            }
        }
        if idx.len() < 3 {
            return Ok(None);
        }
        if !removed {
            break;
        }
    }

    let orientation = polygon_orientation2d_iter(idx.iter().map(|&i| points2[i]));
    if orientation == Orientation::Zero {
        return Ok(None);
    }
    let opposite = -orientation.as_i8();

    let mut triangles = Vec::with_capacity(idx.len() - 2);
    while idx.len() > 3 {
        let count = idx.len();
        charge(scope, (count * count) as u64)?;
        let mut clipped = false;
        for k in 0..count {
            let ia = idx[(k + count - 1) % count];
            let ib = idx[k];
            let ic = idx[(k + 1) % count];
            let corner = orient2d(points2[ia], points2[ib], points2[ic]);
            if corner == Orientation::Zero {
                idx.remove(k);
                clipped = true;
                break;
            }
            if corner != orientation {
                continue;
            }
            let blocked = idx.iter().any(|&j| {
                if j == ia || j == ib || j == ic {
                    return false;
                }
                let p = points2[j];
                let strictly_outside = orient2d(points2[ia], points2[ib], p).as_i8() == opposite
                    || orient2d(points2[ib], points2[ic], p).as_i8() == opposite
                    || orient2d(points2[ic], points2[ia], p).as_i8() == opposite;
                !strictly_outside
            });
            if blocked {
                continue;
            }
            triangles.push([
                ring.vertices[ia].point,
                ring.vertices[ib].point,
                ring.vertices[ic].point,
            ]);
            idx.remove(k);
            clipped = true;
            break;
        }
        if !clipped {
            return Ok(None);
        }
    }
    triangles.push([
        ring.vertices[idx[0]].point,
        ring.vertices[idx[1]].point,
        ring.vertices[idx[2]].point,
    ]);
    Ok(Some(triangles))
}

/// One exact vertex-polygon triangle in model space.
type Triangle = [[f64; 3]; 3];

/// One boundary face admitted to the parity phase with its exact triangles.
struct ParityFace {
    face: PreparedFace,
    triangles: Vec<Triangle>,
}

struct RawParityWitness {
    far_point: [f64; 3],
    crossings: u32,
    crossed_faces: Vec<RawFaceId>,
}

enum ParityOutcome {
    Decided {
        inside: bool,
        witness: RawParityWitness,
    },
    Gap,
}

/// Exact crossing count of the open segment `point → far` against one
/// face's triangles. `None` reports a degenerate contact for this ray.
fn count_face_crossings(
    triangles: &[Triangle],
    point: [f64; 3],
    far: [f64; 3],
    scope: &mut OperationScope<'_, '_>,
) -> Result<Option<u32>> {
    let mut crossings = 0_u32;
    for triangle in triangles {
        charge(scope, 1)?;
        let [a, b, c] = *triangle;
        let side_point = orient3d(a, b, c, point);
        let side_far = orient3d(a, b, c, far);
        if side_point == Orientation::Zero && side_far == Orientation::Zero {
            return Ok(None);
        }
        if side_point == Orientation::Zero {
            // The segment meets this triangle's plane only at the query
            // point, which the boundary phase certified off the face.
            continue;
        }
        if side_far == Orientation::Zero || side_point == side_far {
            if side_far == Orientation::Zero {
                return Ok(None);
            }
            continue;
        }
        let t_ab = orient3d(point, far, a, b);
        let t_bc = orient3d(point, far, b, c);
        let t_ca = orient3d(point, far, c, a);
        if t_ab == Orientation::Zero || t_bc == Orientation::Zero || t_ca == Orientation::Zero {
            return Ok(None);
        }
        if t_ab == t_bc && t_bc == t_ca {
            crossings += 1;
        }
    }
    Ok(Some(crossings))
}

/// Certified interior/exterior parity over every prepared boundary face.
///
/// Deterministic ray candidates are tried in order; a candidate is abandoned
/// on any exactly-degenerate contact (through an edge, vertex, artificial
/// triangulation diagonal, or coplanar with the segment), so an accepted
/// count has every crossing certified transversal by exact predicates.
fn ray_parity(
    faces: &[ParityFace],
    point: [f64; 3],
    scope: &mut OperationScope<'_, '_>,
) -> Result<ParityOutcome> {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for entry in faces {
        for ring in &entry.face.loops {
            for v in &ring.vertices {
                for axis in 0..3 {
                    lo[axis] = lo[axis].min(v.point[axis]);
                    hi[axis] = hi[axis].max(v.point[axis]);
                }
            }
        }
    }
    let mut reach = 1.0_f64;
    for axis in 0..3 {
        reach = reach
            .max((point[axis] - lo[axis]).abs())
            .max((point[axis] - hi[axis]).abs());
    }
    let scale = 8.0 * reach;

    for (attempt, direction) in RAY_DIRECTIONS.iter().enumerate() {
        scope
            .ledger_mut()
            .observe(
                POINT_CLASSIFICATION_RAYS,
                ResourceKind::Depth,
                (attempt + 1) as u64,
            )
            .map_err(Error::from)?;
        let far = [
            point[0] + scale * direction[0],
            point[1] + scale * direction[1],
            point[2] + scale * direction[2],
        ];
        let outside = (0..3).any(|axis| far[axis] < lo[axis] || far[axis] > hi[axis]);
        if !outside || far.iter().any(|c| !c.is_finite()) {
            continue;
        }
        let mut crossings = 0_u32;
        let mut crossed_faces = Vec::new();
        let mut degenerate = false;
        for entry in faces {
            match count_face_crossings(&entry.triangles, point, far, scope)? {
                None => {
                    degenerate = true;
                    break;
                }
                Some(count) => {
                    if count > 0 {
                        crossings += count;
                        for _ in 0..count {
                            crossed_faces.push(entry.face.raw);
                        }
                    }
                }
            }
        }
        if degenerate {
            continue;
        }
        return Ok(ParityOutcome::Decided {
            inside: crossings % 2 == 1,
            witness: RawParityWitness {
                far_point: far,
                crossings,
                crossed_faces,
            },
        });
    }
    Ok(ParityOutcome::Gap)
}

fn wrap_site(part: &PartId, site: RawSite) -> PointFaceSite {
    match site {
        RawSite::Interior => PointFaceSite::Interior,
        RawSite::EdgeInterior(edge) => PointFaceSite::EdgeInterior(EdgeId::new(part.clone(), edge)),
        RawSite::AtVertex(vertex) => PointFaceSite::AtVertex(VertexId::new(part.clone(), vertex)),
    }
}

fn classify_on_face_impl(
    part: &Part<'_>,
    face: &FaceId,
    point: [f64; 3],
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<PointFaceClassification> {
    let store = &part.state.store;
    let verdict = match prepare_face(store, face.raw(), linear)? {
        PrepOutcome::Gap(reason) => PointFaceVerdict::Indeterminate { reason },
        PrepOutcome::Ready(prepared) => match face_site(&prepared, point, scope)? {
            SiteOutcome::On(site) => PointFaceVerdict::On(wrap_site(face.part(), site)),
            SiteOutcome::Off => PointFaceVerdict::Off,
            SiteOutcome::Gap(reason) => PointFaceVerdict::Indeterminate { reason },
        },
    };
    Ok(PointFaceClassification {
        face: face.clone(),
        verdict,
    })
}

fn classify_in_body_impl(
    part: &Part<'_>,
    body: &BodyId,
    point: [f64; 3],
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<PointBodyClassification> {
    let store = &part.state.store;
    let raw_body = store.get(body.raw()).map_err(|_| Error::StaleEntity {
        kind: EntityKind::Body,
    })?;
    if raw_body.kind() != BodyKind::Solid {
        return Err(Error::Core {
            source: kcore::error::Error::InvalidGeometry {
                reason: "point/body classification requires a solid body",
            },
        });
    }

    let mut face_ids: Vec<RawFaceId> = Vec::new();
    for &region_id in raw_body.regions() {
        let region = read(store.get(region_id))?;
        if region.kind() != RegionKind::Solid {
            continue;
        }
        for &shell_id in region.shells() {
            let shell = read(store.get(shell_id))?;
            for &face_id in shell.faces() {
                if !face_ids.contains(&face_id) {
                    face_ids.push(face_id);
                }
            }
        }
    }
    if face_ids.is_empty() {
        return Err(Error::Core {
            source: kcore::error::Error::InvalidGeometry {
                reason: "solid body classification found no material boundary faces",
            },
        });
    }
    charge(scope, face_ids.len() as u64)?;

    // Boundary phase: any certified face contact decides the query; every
    // capability gap or guard-band contact blocks an interior/exterior claim.
    let mut prepared: Vec<PreparedFace> = Vec::with_capacity(face_ids.len());
    let mut first_gap: Option<&'static str> = None;
    for &face_id in face_ids.iter() {
        match prepare_face(store, face_id, linear)? {
            PrepOutcome::Gap(reason) => {
                first_gap.get_or_insert(reason);
            }
            PrepOutcome::Ready(face) => match face_site(&face, point, scope)? {
                SiteOutcome::On(site) => {
                    return Ok(PointBodyClassification {
                        body: body.clone(),
                        verdict: PointBodyVerdict::Boundary {
                            face: FaceId::new(body.part().clone(), face_id),
                            site: wrap_site(body.part(), site),
                        },
                        witness: None,
                    });
                }
                SiteOutcome::Off => prepared.push(face),
                SiteOutcome::Gap(reason) => {
                    first_gap.get_or_insert(reason);
                }
            },
        }
    }
    if let Some(reason) = first_gap {
        return Ok(PointBodyClassification {
            body: body.clone(),
            verdict: PointBodyVerdict::Indeterminate { reason },
            witness: None,
        });
    }

    // Parity phase: triangulate every face exactly, then count certified
    // transversal crossings along a deterministic generic segment.
    let mut triangulated: Vec<ParityFace> = Vec::with_capacity(prepared.len());
    for face in prepared {
        let mut triangles = Vec::new();
        for ring in &face.loops {
            match triangulate_loop(ring, face.drop_axis, scope)? {
                Some(mut ring_triangles) => triangles.append(&mut ring_triangles),
                None => {
                    return Ok(PointBodyClassification {
                        body: body.clone(),
                        verdict: PointBodyVerdict::Indeterminate {
                            reason: GAP_EAR_SEARCH,
                        },
                        witness: None,
                    });
                }
            }
        }
        triangulated.push(ParityFace { face, triangles });
    }
    match ray_parity(&triangulated, point, scope)? {
        ParityOutcome::Decided { inside, witness } => {
            let far = witness.far_point;
            Ok(PointBodyClassification {
                body: body.clone(),
                verdict: if inside {
                    PointBodyVerdict::Interior
                } else {
                    PointBodyVerdict::Exterior
                },
                witness: Some(RayParityWitness {
                    far_point: Point3::new(far[0], far[1], far[2]),
                    crossings: witness.crossings,
                    crossed_faces: witness
                        .crossed_faces
                        .into_iter()
                        .map(|raw| FaceId::new(body.part().clone(), raw))
                        .collect(),
                }),
            })
        }
        ParityOutcome::Gap => Ok(PointBodyClassification {
            body: body.clone(),
            verdict: PointBodyVerdict::Indeterminate {
                reason: GAP_RAYS_EXHAUSTED,
            },
            witness: None,
        }),
    }
}

#[cfg(test)]
mod tests {
    use kcore::operation::{AccountingMode, BudgetPlan, LimitSpec, ResourceKind};
    use kgeom::frame::Frame;
    use kgeom::vec::{Point2, Point3};
    use ktopo::profile::PlanarProfile;
    use ktopo::store::Store;

    use super::*;
    use crate::{Kernel, KernelError, OperationSettings, Session};

    fn solid_part<F>(build: F) -> (Session, PartId, BodyId)
    where
        F: FnOnce(&mut Store) -> ktopo::entity::BodyId,
    {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let raw = {
            let mut edit = session.edit_part(part_id.clone()).unwrap();
            build(edit.store_mut_for_test())
        };
        (session, part_id.clone(), BodyId::new(part_id, raw))
    }

    fn body_verdict(
        session: &Session,
        part_id: &PartId,
        body: &BodyId,
        point: Point3,
    ) -> PointBodyClassification {
        session
            .part(part_id.clone())
            .unwrap()
            .classify_point_in_body(ClassifyPointInBodyRequest::new(body.clone(), point))
            .unwrap()
            .into_result()
            .unwrap()
    }

    #[test]
    fn block_interior_exterior_and_witness_parity_are_certified() {
        let (session, part_id, body) = solid_part(|store| {
            ktopo::make::block(store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap()
        });

        let inside = body_verdict(&session, &part_id, &body, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(inside.verdict(), &PointBodyVerdict::Interior);
        let witness = inside.witness().unwrap();
        assert_eq!(witness.crossings() % 2, 1);
        assert_eq!(witness.crossed_faces().len(), witness.crossings() as usize);

        // A skewed interior point exercises non-central crossings.
        let skewed = body_verdict(&session, &part_id, &body, Point3::new(0.7, -0.6, 0.3));
        assert_eq!(skewed.verdict(), &PointBodyVerdict::Interior);

        for point in [
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, -5.0),
            Point3::new(1.5, 1.5, 1.5),
        ] {
            let outside = body_verdict(&session, &part_id, &body, point);
            assert_eq!(outside.verdict(), &PointBodyVerdict::Exterior);
            assert_eq!(outside.witness().unwrap().crossings() % 2, 0);
        }
    }

    #[test]
    fn block_boundary_sites_are_certified_by_kind() {
        let (session, part_id, body) = solid_part(|store| {
            ktopo::make::block(store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap()
        });

        let face_center = body_verdict(&session, &part_id, &body, Point3::new(0.0, 0.0, 1.0));
        assert!(
            matches!(
                face_center.verdict(),
                PointBodyVerdict::Boundary {
                    site: PointFaceSite::Interior,
                    ..
                }
            ),
            "expected face-interior boundary, got {:?}",
            face_center.verdict()
        );

        let edge_midpoint = body_verdict(&session, &part_id, &body, Point3::new(1.0, 0.0, 1.0));
        assert!(
            matches!(
                edge_midpoint.verdict(),
                PointBodyVerdict::Boundary {
                    site: PointFaceSite::EdgeInterior(_),
                    ..
                }
            ),
            "expected edge boundary, got {:?}",
            edge_midpoint.verdict()
        );

        let corner = body_verdict(&session, &part_id, &body, Point3::new(1.0, 1.0, 1.0));
        assert!(
            matches!(
                corner.verdict(),
                PointBodyVerdict::Boundary {
                    site: PointFaceSite::AtVertex(_),
                    ..
                }
            ),
            "expected vertex boundary, got {:?}",
            corner.verdict()
        );
    }

    #[test]
    fn block_guard_band_is_honestly_indeterminate() {
        let (session, part_id, body) = solid_part(|store| {
            ktopo::make::block(store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap()
        });

        // Within linear resolution of the top face: a certified boundary.
        let on = body_verdict(&session, &part_id, &body, Point3::new(0.0, 0.0, 1.0 + 1e-9));
        assert!(matches!(on.verdict(), PointBodyVerdict::Boundary { .. }));

        // Inside the guard band (res < d < 4·res): honest refusal.
        let banded = body_verdict(&session, &part_id, &body, Point3::new(0.0, 0.0, 1.0 + 2e-8));
        assert!(
            matches!(banded.verdict(), PointBodyVerdict::Indeterminate { .. }),
            "expected a guard-band refusal, got {:?}",
            banded.verdict()
        );

        // Beyond the guard band: certified exterior.
        let clear = body_verdict(&session, &part_id, &body, Point3::new(0.0, 0.0, 1.0 + 1e-7));
        assert_eq!(clear.verdict(), &PointBodyVerdict::Exterior);
    }

    #[test]
    fn face_sites_and_off_face_verdicts_are_certified() {
        let (session, part_id, body) = solid_part(|store| {
            ktopo::make::block(store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap()
        });
        let part = session.part(part_id.clone()).unwrap();
        let store = &part.state.store;
        let face_ids = store.faces_of_body(body.raw()).unwrap();

        let classify = |face: FaceId, point: Point3| -> PointFaceVerdict {
            part.classify_point_on_face(ClassifyPointOnFaceRequest::new(face, point))
                .unwrap()
                .into_result()
                .unwrap()
                .verdict()
                .clone()
        };

        // Exactly one block face carries (0,0,1) in its interior: z = +1.
        let mut top = None;
        for &raw in &face_ids {
            let verdict = classify(
                FaceId::new(part_id.clone(), raw),
                Point3::new(0.0, 0.0, 1.0),
            );
            match verdict {
                PointFaceVerdict::On(PointFaceSite::Interior) => {
                    assert!(top.replace(raw).is_none(), "two faces claimed the point");
                }
                PointFaceVerdict::Off => {}
                other => panic!("unexpected verdict {other:?}"),
            }
        }
        let top = FaceId::new(part_id.clone(), top.expect("no face claimed the point"));

        assert!(matches!(
            classify(top.clone(), Point3::new(1.0, 0.0, 1.0)),
            PointFaceVerdict::On(PointFaceSite::EdgeInterior(_))
        ));
        assert!(matches!(
            classify(top.clone(), Point3::new(-1.0, 1.0, 1.0)),
            PointFaceVerdict::On(PointFaceSite::AtVertex(_))
        ));
        // On the surface plane but laterally outside the trim: certified off.
        assert_eq!(
            classify(top.clone(), Point3::new(5.0, 0.0, 1.0)),
            PointFaceVerdict::Off
        );
        // Off the surface entirely.
        assert_eq!(
            classify(top.clone(), Point3::new(0.0, 0.0, 3.0)),
            PointFaceVerdict::Off
        );
        // Guard band: honest refusal.
        assert!(matches!(
            classify(top, Point3::new(0.0, 0.0, 1.0 + 2e-8)),
            PointFaceVerdict::Indeterminate { .. }
        ));
    }

    #[test]
    fn holed_extrusion_shaft_is_exterior_and_ring_material_is_interior() {
        let (session, part_id, body) = solid_part(|store| {
            let outer = [
                Point2::new(-2.0, -2.0),
                Point2::new(2.0, -2.0),
                Point2::new(2.0, 2.0),
                Point2::new(-2.0, 2.0),
            ];
            let hole = [
                Point2::new(-0.5, -0.5),
                Point2::new(0.5, -0.5),
                Point2::new(0.5, 0.5),
                Point2::new(-0.5, 0.5),
            ];
            let profile =
                PlanarProfile::from_polygon_with_holes(Frame::world(), &outer, &[&hole]).unwrap();
            ktopo::make::extrude_profile(store, &profile, 1.0).unwrap()
        });

        // Inside the hole shaft: not material, certified through crossings
        // that enter and leave the ring material.
        let shaft = body_verdict(&session, &part_id, &body, Point3::new(0.0, 0.0, 0.5));
        assert_eq!(shaft.verdict(), &PointBodyVerdict::Exterior);

        let material = body_verdict(&session, &part_id, &body, Point3::new(1.25, 0.6, 0.5));
        assert_eq!(material.verdict(), &PointBodyVerdict::Interior);

        let above = body_verdict(&session, &part_id, &body, Point3::new(0.0, 0.0, 2.0));
        assert_eq!(above.verdict(), &PointBodyVerdict::Exterior);

        let hole_wall = body_verdict(&session, &part_id, &body, Point3::new(0.5, 0.0, 0.5));
        assert!(matches!(
            hole_wall.verdict(),
            PointBodyVerdict::Boundary { .. }
        ));
    }

    #[test]
    fn nonconvex_extrusion_notch_is_exterior() {
        let (session, part_id, body) = solid_part(|store| {
            let outer = [
                Point2::new(0.0, 0.0),
                Point2::new(3.0, 0.0),
                Point2::new(3.0, 1.0),
                Point2::new(1.0, 1.0),
                Point2::new(1.0, 3.0),
                Point2::new(0.0, 3.0),
            ];
            let profile = PlanarProfile::from_polygon(Frame::world(), &outer).unwrap();
            ktopo::make::extrude_profile(store, &profile, 1.0).unwrap()
        });

        // The notch of the L lies outside the material.
        let notch = body_verdict(&session, &part_id, &body, Point3::new(2.0, 2.0, 0.5));
        assert_eq!(notch.verdict(), &PointBodyVerdict::Exterior);

        let material = body_verdict(&session, &part_id, &body, Point3::new(0.5, 0.5, 0.5));
        assert_eq!(material.verdict(), &PointBodyVerdict::Interior);

        // The reflex-corner vertical edge is a certified boundary.
        let reflex_edge = body_verdict(&session, &part_id, &body, Point3::new(1.0, 1.0, 0.5));
        assert!(matches!(
            reflex_edge.verdict(),
            PointBodyVerdict::Boundary {
                site: PointFaceSite::EdgeInterior(_),
                ..
            }
        ));
    }

    #[test]
    fn curved_bodies_are_an_honest_capability_gap() {
        let (session, part_id, body) =
            solid_part(|store| ktopo::make::sphere(store, &Frame::world(), 1.0).unwrap());
        let center = body_verdict(&session, &part_id, &body, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(
            center.verdict(),
            &PointBodyVerdict::Indeterminate {
                reason: super::GAP_PLANAR_ONLY
            }
        );
    }

    #[test]
    fn work_budget_limits_are_reported_not_ignored() {
        let (session, part_id, body) = solid_part(|store| {
            ktopo::make::block(store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap()
        });
        let plan = BudgetPlan::new([LimitSpec::new(
            POINT_CLASSIFICATION_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            2,
        )])
        .unwrap();
        let outcome = session
            .part(part_id)
            .unwrap()
            .classify_point_in_body(
                ClassifyPointInBodyRequest::new(body, Point3::new(0.0, 0.0, 0.0))
                    .with_settings(OperationSettings::new().with_budget_overrides(plan)),
            )
            .unwrap();
        let result = outcome.into_result();
        let error = result.unwrap_err();
        let crossing = error.limit().expect("limit evidence must be preserved");
        assert_eq!(crossing.stage, POINT_CLASSIFICATION_WORK);
    }

    #[test]
    fn invalid_inputs_are_rejected_before_the_scope_starts() {
        let (session, part_id, body) = solid_part(|store| {
            ktopo::make::block(store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap()
        });
        let part = session.part(part_id).unwrap();
        let result = part.classify_point_in_body(ClassifyPointInBodyRequest::new(
            body,
            Point3::new(f64::NAN, 0.0, 0.0),
        ));
        assert!(matches!(result, Err(KernelError::Core { .. })));
    }
}
