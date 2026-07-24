//! Certified material-set distance enclosures for exact analytic solids.
//!
//! This lower-kernel query proves an interval containing the minimum
//! Euclidean distance between two closed material sets.  Face domains provide
//! conservative support projections for the lower bound; active fin pcurves
//! lifted through their owning surfaces provide the upper bound. Consequently
//! contact, overlap, and containment legitimately produce enclosures whose
//! lower endpoint is zero without confusing material distance with boundary
//! distance. The retained feasible point pair proves only the upper endpoint;
//! it is not asserted to be a closest pair. Both bounds carry outward the
//! fixed incidence envelope admitted by Full validation, even when stored
//! topology has no explicit tolerance.

use crate::body_properties::{Point3Enclosure, ScalarEnclosure};
use crate::check::{
    CheckBudgetProfile, CheckLevel, CheckOutcome, CheckReport, check_body_report_in_scope,
};
use crate::entity::{Body, BodyId, BodyKind, EdgeId, FaceId, FinId, VertexId};
use crate::geom::{Curve2dGeom, SurfaceGeom};
use crate::store::Store;
use kcore::error::{Error, Result};
use kcore::interval::Interval;
use kcore::math;
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationScope, ResourceKind, StageId,
};
use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::vec::{Point3, Vec2, Vec3};

const fn known_stage(value: &'static str) -> StageId {
    match StageId::new(value) {
        Ok(stage) => stage,
        Err(_) => panic!("invalid built-in body-distance stage"),
    }
}

/// Cumulative exact structural and fixed-degree interval work.
pub const BODY_DISTANCE_ANALYTIC_WORK: StageId =
    known_stage("ktopo.interrogate.body-distance-analytic-work");

const DEFAULT_ANALYTIC_WORK: u64 = 4_194_304;
const EDGE_REALIZATION_RADIUS: f64 = LINEAR_RESOLUTION;
const VERTEX_REALIZATION_RADIUS: f64 = 2.0 * LINEAR_RESOLUTION;
const BODY_SCAN_WEIGHT: u64 = 2;
const REGION_WEIGHT: u64 = 4;
const SHELL_WEIGHT: u64 = 6;
const FACE_WEIGHT: u64 = 16;
const LOOP_WEIGHT: u64 = 8;
const FIN_WEIGHT: u64 = 8;
const EDGE_USE_WEIGHT: u64 = 12;
const VERTEX_USE_WEIGHT: u64 = 4;
const WITNESS_WEIGHT: u64 = 16;
const PROJECTION_WEIGHT: u64 = 24;
const WITNESS_PAIR_WEIGHT: u64 = 20;

/// Aggregate v1 policy for two Full checks followed by analytic distance.
#[derive(Debug, Clone, Copy, Default)]
pub struct BodyDistanceBudgetProfile;

impl BodyDistanceBudgetProfile {
    /// Canonical family defaults for one certified two-body query.
    ///
    /// Cumulative one-body checker allowances are doubled; high-water
    /// allowances are shared unchanged by the serial pair query.
    pub fn v1_defaults() -> BudgetPlan {
        doubled_full_check_defaults().overlaid(
            &BudgetPlan::new([LimitSpec::new(
                BODY_DISTANCE_ANALYTIC_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                DEFAULT_ANALYTIC_WORK,
            )])
            .expect("valid body-distance budget"),
        )
    }
}

fn doubled_full_check_defaults() -> BudgetPlan {
    let single = CheckBudgetProfile::v1_defaults(CheckLevel::Full);
    let mut pair = BudgetPlan::new(single.limits().iter().map(|limit| {
        let allowed = if limit.mode == AccountingMode::Cumulative {
            limit
                .allowed
                .checked_mul(2)
                .expect("built-in pair checker budget fits u64")
        } else {
            limit.allowed
        };
        LimitSpec::new(limit.stage, limit.resource, limit.mode, allowed)
    }))
    .expect("built-in pair checker budget is valid");
    if let Some(total) = single.total_work_limit() {
        pair = pair.with_total_work_limit(
            total
                .checked_mul(2)
                .expect("built-in pair total-work budget fits u64"),
        );
    }
    pair
}

/// Request-relative operand named by a typed refusal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyDistanceOperand {
    /// The first body supplied by the caller.
    First,
    /// The second body supplied by the caller.
    Second,
}

/// Why a valid request did not produce a certified material distance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum BodyDistanceRefusal {
    /// Full validation found faults or unresolved proof obligations.
    BodyNotFullValid {
        /// Request-relative body.
        operand: BodyDistanceOperand,
    },
    /// The operand is not a three-dimensional solid material set.
    NonSolidBody {
        /// Request-relative body.
        operand: BodyDistanceOperand,
    },
    /// Exact distance does not consume a tolerant face.
    TolerantFace {
        /// Request-relative body.
        operand: BodyDistanceOperand,
        /// Raw lower-kernel face.
        face: FaceId,
    },
    /// Exact distance does not consume a tolerant edge.
    TolerantEdge {
        /// Request-relative body.
        operand: BodyDistanceOperand,
        /// Raw lower-kernel edge.
        edge: EdgeId,
    },
    /// Exact distance does not consume a tolerant vertex.
    TolerantVertex {
        /// Request-relative body.
        operand: BodyDistanceOperand,
        /// Raw lower-kernel vertex.
        vertex: VertexId,
    },
    /// A face lacks the finite conservative domain required by the theorem.
    MissingFaceDomain {
        /// Request-relative body.
        operand: BodyDistanceOperand,
        /// Raw lower-kernel face.
        face: FaceId,
    },
    /// A supporting surface is outside the exact Plane/Cylinder slice.
    UnsupportedSurface {
        /// Request-relative body.
        operand: BodyDistanceOperand,
        /// Raw lower-kernel face.
        face: FaceId,
    },
    /// A face trim is outside the exact Line2d/Circle2d proof slice.
    UnsupportedPcurve {
        /// Request-relative body.
        operand: BodyDistanceOperand,
        /// Raw lower-kernel face.
        face: FaceId,
    },
    /// A nominal solid carries lower-dimensional shell attachments.
    MixedDimensionalBody {
        /// Request-relative body.
        operand: BodyDistanceOperand,
    },
    /// No topology-owned point was available to prove a finite upper bound.
    NoUpperWitness {
        /// Request-relative body.
        operand: BodyDistanceOperand,
    },
    /// Outward arithmetic did not produce an ordered finite enclosure.
    IndeterminateEnclosure,
}

/// One topology-owned feasible boundary point used by the upper-bound proof.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyDistanceBoundaryWitness {
    face: FaceId,
    fin: FinId,
    edge: EdgeId,
    pcurve_parameter: ScalarEnclosure,
    point: Point3Enclosure,
}

impl BodyDistanceBoundaryWitness {
    /// Face whose trim boundary owns this point.
    pub const fn face(self) -> FaceId {
        self.face
    }

    /// Fin whose active pcurve defines the lifted witness trace.
    pub const fn fin(self) -> FinId {
        self.fin
    }

    /// Edge used by the active fin pcurve.
    pub const fn edge(self) -> EdgeId {
        self.edge
    }

    /// Certified parameter enclosure on the fin's pcurve.
    pub const fn pcurve_parameter(self) -> ScalarEnclosure {
        self.pcurve_parameter
    }

    /// Certified model-space enclosure of the lifted pcurve point, expanded
    /// by the fixed Full-validation edge-incidence envelope.
    pub const fn point(self) -> Point3Enclosure {
        self.point
    }
}

/// Feasible boundary-point pair proving the certified distance upper bound.
///
/// The pair is deterministic and topology-owned, but is not claimed to be a
/// minimizer or a stationary closest-point pair.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyDistanceUpperWitness {
    points: [BodyDistanceBoundaryWitness; 2],
    distance: ScalarEnclosure,
}

impl BodyDistanceUpperWitness {
    /// Boundary points in caller request order.
    pub const fn points(self) -> [BodyDistanceBoundaryWitness; 2] {
        self.points
    }

    /// Enclosure of the Euclidean distance between the feasible points.
    pub const fn distance(self) -> ScalarEnclosure {
        self.distance
    }
}

/// Ordered Full-check evidence paired with certification or typed refusal.
#[derive(Debug, Clone, PartialEq)]
pub enum BodyDistanceOutcome {
    /// The support and witness bounds certified the material distance.
    Certified {
        /// Certified nonnegative distance enclosure.
        distance: ScalarEnclosure,
        /// Feasible point pair proving the enclosure's upper endpoint.
        upper_witness: BodyDistanceUpperWitness,
        /// Full reports in caller request order.
        full_checks: [CheckReport; 2],
    },
    /// The request is valid but outside the current proof boundary.
    Refused {
        /// Typed refusal reason.
        reason: BodyDistanceRefusal,
        /// Full reports in caller request order.
        full_checks: [CheckReport; 2],
    },
}

impl BodyDistanceOutcome {
    /// Certified enclosure, when the query succeeded.
    pub const fn distance(&self) -> Option<ScalarEnclosure> {
        match self {
            Self::Certified { distance, .. } => Some(*distance),
            Self::Refused { .. } => None,
        }
    }

    /// Feasible point-pair evidence, when the query succeeded.
    pub const fn upper_witness(&self) -> Option<BodyDistanceUpperWitness> {
        match self {
            Self::Certified { upper_witness, .. } => Some(*upper_witness),
            Self::Refused { .. } => None,
        }
    }

    /// Typed refusal, when the proof boundary rejected the request.
    pub const fn refusal(&self) -> Option<BodyDistanceRefusal> {
        match self {
            Self::Certified { .. } => None,
            Self::Refused { reason, .. } => Some(*reason),
        }
    }

    /// Full reports in caller request order.
    pub const fn full_checks(&self) -> &[CheckReport; 2] {
        match self {
            Self::Certified { full_checks, .. } | Self::Refused { full_checks, .. } => full_checks,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct CanonicalBodies {
    ids: [BodyId; 2],
    operands: [BodyDistanceOperand; 2],
    swapped: bool,
    body_scan: u64,
}

impl CanonicalBodies {
    fn reports(self, canonical: [CheckReport; 2]) -> [CheckReport; 2] {
        let [first, second] = canonical;
        if self.swapped {
            [second, first]
        } else {
            [first, second]
        }
    }

    fn upper_witness(self, canonical: BodyDistanceUpperWitness) -> BodyDistanceUpperWitness {
        if self.swapped {
            let [first, second] = canonical.points;
            BodyDistanceUpperWitness {
                points: [second, first],
                distance: canonical.distance,
            }
        } else {
            canonical
        }
    }
}

fn canonical_bodies(store: &Store, first: BodyId, second: BodyId) -> Result<CanonicalBodies> {
    store.get(first)?;
    store.get(second)?;
    if first == second {
        return Err(Error::InvalidGeometry {
            reason: "body distance requires two distinct bodies",
        });
    }
    let mut ordinals = [None, None];
    let mut body_scan = 0_u64;
    for (ordinal, (candidate, _)) in store.iter::<Body>().enumerate() {
        body_scan = body_scan.checked_add(1).ok_or(Error::InvalidGeometry {
            reason: "body-distance body count overflow",
        })?;
        if candidate == first {
            ordinals[0] = Some(ordinal);
        }
        if candidate == second {
            ordinals[1] = Some(ordinal);
        }
    }
    let [Some(first_ordinal), Some(second_ordinal)] = ordinals else {
        return Err(Error::StaleHandle);
    };
    if first_ordinal < second_ordinal {
        Ok(CanonicalBodies {
            ids: [first, second],
            operands: [BodyDistanceOperand::First, BodyDistanceOperand::Second],
            swapped: false,
            body_scan,
        })
    } else {
        Ok(CanonicalBodies {
            ids: [second, first],
            operands: [BodyDistanceOperand::Second, BodyDistanceOperand::First],
            swapped: true,
            body_scan,
        })
    }
}

#[derive(Debug, Clone, Copy)]
struct BodyStructure {
    regions: u64,
    shells: u64,
    faces: u64,
    loops: u64,
    fins: u64,
    edge_uses: u64,
    vertex_uses: u64,
    witnesses: u64,
}

fn structural_count(value: usize, reason: &'static str) -> Result<u64> {
    u64::try_from(value).map_err(|_| Error::InvalidGeometry { reason })
}

fn checked_add<F>(
    total: &mut u64,
    count: usize,
    reason: &'static str,
    weight: u64,
    charge: &mut F,
) -> Result<()>
where
    F: FnMut(u64, u64) -> Result<()>,
{
    let count = structural_count(count, reason)?;
    charge(count, weight)?;
    *total = total
        .checked_add(count)
        .ok_or(Error::InvalidGeometry { reason })?;
    Ok(())
}

fn body_structure<F>(store: &Store, body: BodyId, mut charge: F) -> Result<BodyStructure>
where
    F: FnMut(u64, u64) -> Result<()>,
{
    let mut structure = BodyStructure {
        regions: 0,
        shells: 0,
        faces: 0,
        loops: 0,
        fins: 0,
        edge_uses: 0,
        vertex_uses: 0,
        witnesses: 0,
    };
    let body = store.get(body)?;
    checked_add(
        &mut structure.regions,
        body.regions.len(),
        "body-distance region count overflow",
        REGION_WEIGHT,
        &mut charge,
    )?;
    for &region_id in &body.regions {
        let region = store.get(region_id)?;
        checked_add(
            &mut structure.shells,
            region.shells.len(),
            "body-distance shell count overflow",
            SHELL_WEIGHT,
            &mut charge,
        )?;
        for &shell_id in &region.shells {
            let shell = store.get(shell_id)?;
            checked_add(
                &mut structure.faces,
                shell.faces.len(),
                "body-distance face count overflow",
                FACE_WEIGHT,
                &mut charge,
            )?;
            checked_add(
                &mut structure.edge_uses,
                shell.edges.len(),
                "body-distance edge-use count overflow",
                EDGE_USE_WEIGHT,
                &mut charge,
            )?;
            if shell.vertex.is_some() {
                checked_add(
                    &mut structure.vertex_uses,
                    1,
                    "body-distance vertex-use count overflow",
                    VERTEX_USE_WEIGHT,
                    &mut charge,
                )?;
            }
            for &edge_id in &shell.edges {
                let edge = store.get(edge_id)?;
                checked_add(
                    &mut structure.vertex_uses,
                    edge.vertices.into_iter().flatten().count(),
                    "body-distance vertex-use count overflow",
                    VERTEX_USE_WEIGHT,
                    &mut charge,
                )?;
            }
            for &face_id in &shell.faces {
                let face = store.get(face_id)?;
                checked_add(
                    &mut structure.loops,
                    face.loops.len(),
                    "body-distance loop count overflow",
                    LOOP_WEIGHT,
                    &mut charge,
                )?;
                for &loop_id in &face.loops {
                    let loop_value = store.get(loop_id)?;
                    checked_add(
                        &mut structure.fins,
                        loop_value.fins.len(),
                        "body-distance fin count overflow",
                        FIN_WEIGHT,
                        &mut charge,
                    )?;
                    checked_add(
                        &mut structure.witnesses,
                        loop_value.fins.len(),
                        "body-distance witness count overflow",
                        WITNESS_WEIGHT,
                        &mut charge,
                    )?;
                    for &fin_id in &loop_value.fins {
                        let fin = store.get(fin_id)?;
                        let edge = store.get(fin.edge)?;
                        checked_add(
                            &mut structure.edge_uses,
                            1,
                            "body-distance edge-use count overflow",
                            EDGE_USE_WEIGHT,
                            &mut charge,
                        )?;
                        checked_add(
                            &mut structure.vertex_uses,
                            edge.vertices.into_iter().flatten().count(),
                            "body-distance vertex-use count overflow",
                            VERTEX_USE_WEIGHT,
                            &mut charge,
                        )?;
                    }
                }
            }
        }
    }
    Ok(structure)
}

fn add_work(total: &mut u64, count: u64, weight: u64) -> Result<()> {
    *total = total
        .checked_add(count.checked_mul(weight).ok_or(Error::InvalidGeometry {
            reason: "body-distance analytic work overflow",
        })?)
        .ok_or(Error::InvalidGeometry {
            reason: "body-distance analytic work overflow",
        })?;
    Ok(())
}

fn analytic_numeric_work(first: BodyStructure, second: BodyStructure) -> Result<u64> {
    let faces = first
        .faces
        .checked_add(second.faces)
        .ok_or(Error::InvalidGeometry {
            reason: "body-distance analytic work overflow",
        })?;
    let surface_axes = faces.checked_mul(3).ok_or(Error::InvalidGeometry {
        reason: "body-distance analytic work overflow",
    })?;
    let world_projections = faces.checked_mul(3).ok_or(Error::InvalidGeometry {
        reason: "body-distance analytic work overflow",
    })?;
    let support_projections = surface_axes
        .checked_mul(faces)
        .ok_or(Error::InvalidGeometry {
            reason: "body-distance analytic work overflow",
        })?;
    let projections =
        world_projections
            .checked_add(support_projections)
            .ok_or(Error::InvalidGeometry {
                reason: "body-distance analytic work overflow",
            })?;
    let witness_pairs =
        first
            .witnesses
            .checked_mul(second.witnesses)
            .ok_or(Error::InvalidGeometry {
                reason: "body-distance analytic work overflow",
            })?;
    let mut work = 0_u64;
    add_work(&mut work, projections, PROJECTION_WEIGHT)?;
    add_work(&mut work, witness_pairs, WITNESS_PAIR_WEIGHT)?;
    Ok(work)
}

fn charge_analytic_work(scope: &mut OperationScope<'_, '_>, count: u64, weight: u64) -> Result<()> {
    let mut work = 0_u64;
    add_work(&mut work, count, weight)?;
    charge_analytic_amount(scope, work)
}

fn charge_analytic_amount(scope: &mut OperationScope<'_, '_>, work: u64) -> Result<()> {
    scope
        .ledger_mut()
        .charge(BODY_DISTANCE_ANALYTIC_WORK, work)
        .map_err(Error::from)
}

fn analytic_preflight(
    store: &Store,
    canonical: CanonicalBodies,
) -> Result<(u64, [BodyStructure; 2])> {
    let first = body_structure(store, canonical.ids[0], |_, _| Ok(()))?;
    let second = body_structure(store, canonical.ids[1], |_, _| Ok(()))?;
    let regions = first
        .regions
        .checked_add(second.regions)
        .ok_or(Error::InvalidGeometry {
            reason: "body-distance analytic work overflow",
        })?;
    let shells = first
        .shells
        .checked_add(second.shells)
        .ok_or(Error::InvalidGeometry {
            reason: "body-distance analytic work overflow",
        })?;
    let faces = first
        .faces
        .checked_add(second.faces)
        .ok_or(Error::InvalidGeometry {
            reason: "body-distance analytic work overflow",
        })?;
    let loops = first
        .loops
        .checked_add(second.loops)
        .ok_or(Error::InvalidGeometry {
            reason: "body-distance analytic work overflow",
        })?;
    let edge_uses =
        first
            .edge_uses
            .checked_add(second.edge_uses)
            .ok_or(Error::InvalidGeometry {
                reason: "body-distance analytic work overflow",
            })?;
    let vertex_uses =
        first
            .vertex_uses
            .checked_add(second.vertex_uses)
            .ok_or(Error::InvalidGeometry {
                reason: "body-distance analytic work overflow",
            })?;
    let fins = first
        .fins
        .checked_add(second.fins)
        .ok_or(Error::InvalidGeometry {
            reason: "body-distance analytic work overflow",
        })?;
    let witnesses =
        first
            .witnesses
            .checked_add(second.witnesses)
            .ok_or(Error::InvalidGeometry {
                reason: "body-distance analytic work overflow",
            })?;

    let mut work = 1_u64;
    add_work(&mut work, canonical.body_scan, BODY_SCAN_WEIGHT)?;
    add_work(&mut work, regions, REGION_WEIGHT)?;
    add_work(&mut work, shells, SHELL_WEIGHT)?;
    add_work(&mut work, faces, FACE_WEIGHT)?;
    add_work(&mut work, loops, LOOP_WEIGHT)?;
    add_work(&mut work, fins, FIN_WEIGHT)?;
    add_work(&mut work, edge_uses, EDGE_USE_WEIGHT)?;
    add_work(&mut work, vertex_uses, VERTEX_USE_WEIGHT)?;
    add_work(&mut work, witnesses, WITNESS_WEIGHT)?;
    work = work
        .checked_add(analytic_numeric_work(first, second)?)
        .ok_or(Error::InvalidGeometry {
            reason: "body-distance analytic work overflow",
        })?;
    Ok((work, [first, second]))
}

/// Exact structural charge for one two-body distance query.
///
/// The count covers allocation-free representation preflight, every face
/// projection on every candidate axis, active-pcurve witness construction,
/// and the complete witness cross product. It is independent of numeric
/// conditioning and is computed before numeric scratch is allocated.
pub fn body_distance_analytic_work(store: &Store, body_a: BodyId, body_b: BodyId) -> Result<u64> {
    let canonical = canonical_bodies(store, body_a, body_b)?;
    analytic_preflight(store, canonical).map(|(work, _)| work)
}

type IntervalPoint = [Interval; 3];

#[derive(Debug, Clone, Copy)]
enum SurfacePatch {
    Plane {
        frame: Frame,
        domain: [Interval; 2],
    },
    Cylinder {
        frame: Frame,
        radius: f64,
        domain: [Interval; 2],
    },
}

impl SurfacePatch {
    fn frame(self) -> Frame {
        match self {
            Self::Plane { frame, .. } | Self::Cylinder { frame, .. } => frame,
        }
    }

    fn projection(self, axis: Vec3) -> Option<Interval> {
        let projection = match self {
            Self::Plane { frame, domain } => finite_interval(
                interval_point_dot(frame.origin(), axis)
                    + interval_vec_dot(frame.x(), axis) * domain[0]
                    + interval_vec_dot(frame.y(), axis) * domain[1],
            ),
            Self::Cylinder {
                frame,
                radius,
                domain,
            } => {
                let center = interval_point_dot(frame.origin(), axis)
                    + interval_vec_dot(frame.z(), axis) * domain[1];
                let x = interval_vec_dot(frame.x(), axis);
                let y = interval_vec_dot(frame.y(), axis);
                let amplitude = (x.square() + y.square()).sqrt()? * Interval::point(radius);
                finite_interval(amplitude)?;
                finite_interval(center + Interval::new(-amplitude.hi().abs(), amplitude.hi().abs()))
            }
        }?;
        let norm = axis_norm(axis)?;
        let padding = norm * Interval::point(VERTEX_REALIZATION_RADIUS);
        finite_interval(projection + Interval::new(-padding.hi().abs(), padding.hi().abs()))
    }

    fn periods(self) -> [Option<f64>; 2] {
        match self {
            Self::Plane { .. } => [None, None],
            Self::Cylinder { .. } => [Some(core::f64::consts::TAU), None],
        }
    }

    fn evaluate(self, uv: [Interval; 2]) -> Option<IntervalPoint> {
        let point = match self {
            Self::Plane { frame, .. } => frame_point(frame, uv[0], uv[1], Interval::point(0.0)),
            Self::Cylinder { frame, radius, .. } => {
                let (sine, cosine) = interval_sincos(uv[0])?;
                frame_point(
                    frame,
                    Interval::point(radius) * cosine,
                    Interval::point(radius) * sine,
                    uv[1],
                )
            }
        };
        point
            .iter()
            .all(|value| finite_interval(*value).is_some())
            .then_some(point)
    }

    fn include_uv_bounds(&mut self, bounds: [Interval; 2]) {
        let domain = match self {
            Self::Plane { domain, .. } | Self::Cylinder { domain, .. } => domain,
        };
        for direction in 0..2 {
            domain[direction] = Interval::new(
                domain[direction].lo().min(bounds[direction].lo()),
                domain[direction].hi().max(bounds[direction].hi()),
            );
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct BoundaryWitness {
    face: FaceId,
    fin: FinId,
    edge: EdgeId,
    pcurve_parameter: Interval,
    point: IntervalPoint,
}

#[derive(Debug)]
struct SupportedBody {
    patches: Vec<SurfacePatch>,
    witnesses: Vec<BoundaryWitness>,
}

impl SupportedBody {
    fn projection(&self, axis: Vec3) -> Option<Interval> {
        let mut projection: Option<Interval> = None;
        for patch in &self.patches {
            let current = patch.projection(axis)?;
            projection = Some(match projection {
                None => current,
                Some(accumulated) => Interval::new(
                    accumulated.lo().min(current.lo()),
                    accumulated.hi().max(current.hi()),
                ),
            });
        }
        projection
    }

    fn append_axes(&self, axes: &mut Vec<Vec3>) {
        for patch in &self.patches {
            let frame = patch.frame();
            axes.extend([frame.x(), frame.y(), frame.z()]);
        }
    }
}

fn invalid_body(operand: BodyDistanceOperand) -> BodyDistanceRefusal {
    BodyDistanceRefusal::BodyNotFullValid { operand }
}

fn prepare_body(
    store: &Store,
    body: BodyId,
    operand: BodyDistanceOperand,
    structure: BodyStructure,
) -> core::result::Result<SupportedBody, BodyDistanceRefusal> {
    let body_value = store.get(body).map_err(|_| invalid_body(operand))?;
    for &region_id in &body_value.regions {
        let region = store.get(region_id).map_err(|_| invalid_body(operand))?;
        for &shell_id in &region.shells {
            let shell = store.get(shell_id).map_err(|_| invalid_body(operand))?;
            if !shell.edges.is_empty() || shell.vertex.is_some() {
                return Err(BodyDistanceRefusal::MixedDimensionalBody { operand });
            }
        }
    }
    let faces = store
        .faces_of_body(body)
        .map_err(|_| invalid_body(operand))?;
    let patch_capacity = usize::try_from(structure.faces)
        .map_err(|_| BodyDistanceRefusal::IndeterminateEnclosure)?;
    let witness_capacity = usize::try_from(structure.witnesses)
        .map_err(|_| BodyDistanceRefusal::IndeterminateEnclosure)?;
    let mut patches = Vec::with_capacity(patch_capacity);
    let mut witnesses = Vec::with_capacity(witness_capacity);

    for face_id in faces {
        let face = store.get(face_id).map_err(|_| invalid_body(operand))?;
        if face.tolerance.is_some() {
            return Err(BodyDistanceRefusal::TolerantFace {
                operand,
                face: face_id,
            });
        }
        let domain = face.domain.ok_or(BodyDistanceRefusal::MissingFaceDomain {
            operand,
            face: face_id,
        })?;
        let domain = [
            Interval::new(domain.u.lo, domain.u.hi),
            Interval::new(domain.v.lo, domain.v.hi),
        ];
        let mut patch = match store.get(face.surface).map_err(|_| invalid_body(operand))? {
            SurfaceGeom::Plane(plane) => SurfacePatch::Plane {
                frame: *plane.frame(),
                domain,
            },
            SurfaceGeom::Cylinder(cylinder) => SurfacePatch::Cylinder {
                frame: *cylinder.frame(),
                radius: cylinder.radius(),
                domain,
            },
            _ => {
                return Err(BodyDistanceRefusal::UnsupportedSurface {
                    operand,
                    face: face_id,
                });
            }
        };
        for &loop_id in &face.loops {
            let loop_value = store.get(loop_id).map_err(|_| invalid_body(operand))?;
            for &fin_id in &loop_value.fins {
                let fin = store.get(fin_id).map_err(|_| invalid_body(operand))?;
                let Some(pcurve) = fin.pcurve else {
                    return Err(BodyDistanceRefusal::UnsupportedPcurve {
                        operand,
                        face: face_id,
                    });
                };
                let edge = store.get(fin.edge).map_err(|_| invalid_body(operand))?;
                if edge.tolerance.is_some() {
                    return Err(BodyDistanceRefusal::TolerantEdge {
                        operand,
                        edge: fin.edge,
                    });
                }
                for vertex_id in edge.vertices.into_iter().flatten() {
                    if store
                        .get(vertex_id)
                        .map_err(|_| invalid_body(operand))?
                        .tolerance
                        .is_some()
                    {
                        return Err(BodyDistanceRefusal::TolerantVertex {
                            operand,
                            vertex: vertex_id,
                        });
                    }
                }
                let edge_range = edge_parameter_range(store, edge)
                    .ok_or(BodyDistanceRefusal::IndeterminateEnclosure)?;
                let curve = store
                    .get(pcurve.curve())
                    .map_err(|_| invalid_body(operand))?;
                let parameter = pcurve_midpoint_parameter(pcurve);
                let uv = match curve {
                    Curve2dGeom::Line(_) | Curve2dGeom::Circle(_) => {
                        if !pcurve_parameter_has_edge_preimage(pcurve, parameter, edge_range) {
                            return Err(BodyDistanceRefusal::IndeterminateEnclosure);
                        }
                        let effective_range = effective_pcurve_range(pcurve, edge_range)
                            .ok_or(BodyDistanceRefusal::IndeterminateEnclosure)?;
                        let bounds = pcurve_bounds(curve, pcurve, effective_range, patch.periods())
                            .ok_or(BodyDistanceRefusal::IndeterminateEnclosure)?;
                        patch.include_uv_bounds(bounds);
                        pcurve_point(curve, parameter)
                            .and_then(|point| {
                                apply_chart(point, pcurve.chart().period_shifts(), patch.periods())
                            })
                            .ok_or(BodyDistanceRefusal::IndeterminateEnclosure)?
                    }
                    _ => {
                        return Err(BodyDistanceRefusal::UnsupportedPcurve {
                            operand,
                            face: face_id,
                        });
                    }
                };
                let point = expand_point(
                    patch
                        .evaluate(uv)
                        .ok_or(BodyDistanceRefusal::IndeterminateEnclosure)?,
                    EDGE_REALIZATION_RADIUS,
                )
                .ok_or(BodyDistanceRefusal::IndeterminateEnclosure)?;
                witnesses.push(BoundaryWitness {
                    face: face_id,
                    fin: fin_id,
                    edge: fin.edge,
                    pcurve_parameter: parameter,
                    point,
                });
            }
        }
        patches.push(patch);
    }
    if witnesses.is_empty() {
        return Err(BodyDistanceRefusal::NoUpperWitness { operand });
    }
    if patches.is_empty() {
        return Err(BodyDistanceRefusal::IndeterminateEnclosure);
    }
    Ok(SupportedBody { patches, witnesses })
}

fn finite_interval(value: Interval) -> Option<Interval> {
    (value.lo().is_finite() && value.hi().is_finite()).then_some(value)
}

fn frame_point(frame: Frame, x: Interval, y: Interval, z: Interval) -> IntervalPoint {
    let origin = frame.origin();
    let x_axis = frame.x();
    let y_axis = frame.y();
    let z_axis = frame.z();
    [
        Interval::point(origin.x)
            + Interval::point(x_axis.x) * x
            + Interval::point(y_axis.x) * y
            + Interval::point(z_axis.x) * z,
        Interval::point(origin.y)
            + Interval::point(x_axis.y) * x
            + Interval::point(y_axis.y) * y
            + Interval::point(z_axis.y) * z,
        Interval::point(origin.z)
            + Interval::point(x_axis.z) * x
            + Interval::point(y_axis.z) * y
            + Interval::point(z_axis.z) * z,
    ]
}

fn edge_parameter_range(store: &Store, edge: &crate::entity::Edge) -> Option<ParamRange> {
    match edge.bounds {
        Some((lo, hi)) if lo.is_finite() && hi.is_finite() && lo < hi => {
            Some(ParamRange::new(lo, hi))
        }
        Some(_) => None,
        None => {
            let curve = store.get(edge.curve?).ok()?;
            let range = curve.as_curve().param_range();
            (range.is_finite() && range.lo < range.hi).then_some(range)
        }
    }
}

fn pcurve_midpoint_parameter(pcurve: crate::entity::FinPcurve) -> Interval {
    let range = pcurve.range();
    (Interval::point(range.lo) + Interval::point(range.hi)) * Interval::point(0.5)
}

fn pcurve_parameter_has_edge_preimage(
    pcurve: crate::entity::FinPcurve,
    parameter: Interval,
    edge_range: ParamRange,
) -> bool {
    let map = pcurve.edge_to_pcurve();
    let Some(edge_parameter) =
        (parameter - Interval::point(map.offset())).checked_div(Interval::point(map.scale()))
    else {
        return false;
    };
    edge_parameter.lo() >= edge_range.lo && edge_parameter.hi() <= edge_range.hi
}

fn effective_pcurve_range(
    pcurve: crate::entity::FinPcurve,
    edge_range: ParamRange,
) -> Option<ParamRange> {
    let map = pcurve.edge_to_pcurve();
    let mapped = [edge_range.lo, edge_range.hi].map(|parameter| {
        Interval::point(map.scale()) * Interval::point(parameter) + Interval::point(map.offset())
    });
    let active = pcurve.range();
    let lo = active.lo.min(mapped[0].lo()).min(mapped[1].lo());
    let hi = active.hi.max(mapped[0].hi()).max(mapped[1].hi());
    (lo.is_finite() && hi.is_finite() && lo < hi).then_some(ParamRange::new(lo, hi))
}

fn pcurve_bounds(
    curve: &Curve2dGeom,
    pcurve: crate::entity::FinPcurve,
    range: ParamRange,
    periods: [Option<f64>; 2],
) -> Option<[Interval; 2]> {
    let curve = curve.as_curve();
    let bounds = [
        curve.source_affine_range(range, Vec2::new(1.0, 0.0), 0.0)?,
        curve.source_affine_range(range, Vec2::new(0.0, 1.0), 0.0)?,
    ];
    apply_chart(bounds, pcurve.chart().period_shifts(), periods)
}

fn pcurve_point(curve: &Curve2dGeom, parameter: Interval) -> Option<[Interval; 2]> {
    let point = match curve {
        Curve2dGeom::Line(line) => {
            let origin = line.origin();
            let direction = line.dir();
            [
                Interval::point(origin.x) + Interval::point(direction.x) * parameter,
                Interval::point(origin.y) + Interval::point(direction.y) * parameter,
            ]
        }
        Curve2dGeom::Circle(circle) => {
            let center = circle.center();
            let x = circle.x_dir();
            let y = x.perp();
            let (sine, cosine) = interval_sincos(parameter)?;
            let radius = Interval::point(circle.radius());
            [
                Interval::point(center.x)
                    + radius * (Interval::point(x.x) * cosine + Interval::point(y.x) * sine),
                Interval::point(center.y)
                    + radius * (Interval::point(x.y) * cosine + Interval::point(y.y) * sine),
            ]
        }
        _ => return None,
    };
    point
        .iter()
        .all(|value| finite_interval(*value).is_some())
        .then_some(point)
}

fn expand_point(point: IntervalPoint, radius: f64) -> Option<IntervalPoint> {
    let padding = Interval::new(-radius, radius);
    let expanded = point.map(|coordinate| coordinate + padding);
    expanded
        .iter()
        .all(|coordinate| finite_interval(*coordinate).is_some())
        .then_some(expanded)
}

fn apply_chart(
    mut uv: [Interval; 2],
    shifts: [i32; 2],
    periods: [Option<f64>; 2],
) -> Option<[Interval; 2]> {
    for direction in 0..2 {
        if shifts[direction] == 0 {
            continue;
        }
        let period = periods[direction]?;
        uv[direction] =
            uv[direction] + Interval::point(f64::from(shifts[direction])) * Interval::point(period);
        finite_interval(uv[direction])?;
    }
    Some(uv)
}

fn interval_sincos(angle: Interval) -> Option<(Interval, Interval)> {
    finite_interval(angle)?;
    let midpoint = 0.5 * angle.lo() + 0.5 * angle.hi();
    let radius = (midpoint - angle.lo())
        .abs()
        .max((angle.hi() - midpoint).abs())
        .next_up();
    if !midpoint.is_finite() || !radius.is_finite() {
        return None;
    }
    if radius >= core::f64::consts::TAU {
        return Some((Interval::new(-1.0, 1.0), Interval::new(-1.0, 1.0)));
    }
    let (sine, cosine) = math::sincos(midpoint);
    let cover = |value: f64| {
        let approximation = Interval::new(value.next_down(), value.next_up());
        let result = approximation + Interval::new(-radius, radius);
        Interval::new(result.lo().max(-1.0), result.hi().min(1.0))
    };
    Some((cover(sine), cover(cosine)))
}

fn interval_point_dot(point: Point3, axis: Vec3) -> Interval {
    Interval::point(point.x) * Interval::point(axis.x)
        + Interval::point(point.y) * Interval::point(axis.y)
        + Interval::point(point.z) * Interval::point(axis.z)
}

fn interval_vec_dot(vector: Vec3, axis: Vec3) -> Interval {
    Interval::point(vector.x) * Interval::point(axis.x)
        + Interval::point(vector.y) * Interval::point(axis.y)
        + Interval::point(vector.z) * Interval::point(axis.z)
}

fn interval_separation(first: Interval, second: Interval) -> f64 {
    if first.hi() < second.lo() {
        (Interval::point(second.lo()) - Interval::point(first.hi()))
            .lo()
            .max(0.0)
    } else if second.hi() < first.lo() {
        (Interval::point(first.lo()) - Interval::point(second.hi()))
            .lo()
            .max(0.0)
    } else {
        0.0
    }
}

fn axis_norm(axis: Vec3) -> Option<Interval> {
    finite_interval(
        Interval::point(axis.x).square()
            + Interval::point(axis.y).square()
            + Interval::point(axis.z).square(),
    )?
    .sqrt()
    .and_then(finite_interval)
}

fn world_box_lower(first: &SupportedBody, second: &SupportedBody) -> Option<f64> {
    let world = [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    ];
    let mut squared = Interval::point(0.0);
    for axis in world {
        let gap = interval_separation(first.projection(axis)?, second.projection(axis)?);
        squared = squared + Interval::point(gap).square();
    }
    Some(finite_interval(squared)?.sqrt()?.lo().max(0.0))
}

fn support_lower(first: &SupportedBody, second: &SupportedBody, axes: &[Vec3]) -> Option<f64> {
    let mut lower = world_box_lower(first, second)?;
    for &axis in axes {
        let gap = interval_separation(first.projection(axis)?, second.projection(axis)?);
        if gap == 0.0 {
            continue;
        }
        let norm = axis_norm(axis)?;
        if norm.lo() <= 0.0 || !norm.hi().is_finite() {
            return None;
        }
        let normalized = Interval::point(gap).checked_div(Interval::point(norm.hi()))?;
        lower = lower.max(normalized.lo().max(0.0));
    }
    lower.is_finite().then_some(lower)
}

fn point_distance(first: IntervalPoint, second: IntervalPoint) -> Option<Interval> {
    let mut squared = Interval::point(0.0);
    for coordinate in 0..3 {
        squared = squared + (first[coordinate] - second[coordinate]).square();
    }
    finite_interval(squared)?.sqrt().and_then(finite_interval)
}

fn witness_upper(
    first: &SupportedBody,
    second: &SupportedBody,
) -> Option<BodyDistanceUpperWitness> {
    let mut best: Option<BodyDistanceUpperWitness> = None;
    for &first_point in &first.witnesses {
        for &second_point in &second.witnesses {
            let point_distance = ScalarEnclosure::from_interval(point_distance(
                first_point.point,
                second_point.point,
            )?)?;
            let candidate = BodyDistanceUpperWitness {
                points: [
                    BodyDistanceBoundaryWitness {
                        face: first_point.face,
                        fin: first_point.fin,
                        edge: first_point.edge,
                        pcurve_parameter: ScalarEnclosure::from_interval(
                            first_point.pcurve_parameter,
                        )?,
                        point: Point3Enclosure::from_intervals(first_point.point)?,
                    },
                    BodyDistanceBoundaryWitness {
                        face: second_point.face,
                        fin: second_point.fin,
                        edge: second_point.edge,
                        pcurve_parameter: ScalarEnclosure::from_interval(
                            second_point.pcurve_parameter,
                        )?,
                        point: Point3Enclosure::from_intervals(second_point.point)?,
                    },
                ],
                distance: point_distance,
            };
            if best.is_none_or(|current| candidate.distance.upper() < current.distance.upper()) {
                best = Some(candidate);
            }
        }
    }
    best
}

fn refusal(
    canonical: CanonicalBodies,
    checks: &[CheckReport; 2],
    reason: BodyDistanceRefusal,
) -> BodyDistanceOutcome {
    BodyDistanceOutcome::Refused {
        reason,
        full_checks: canonical.reports(checks.clone()),
    }
}

/// Full-validate and certify a material-distance enclosure in one scope.
///
/// Raw operands are canonicalized by live body iteration ordinal before both
/// Full checks and every numeric operation. Reports and refusal operands are
/// mapped back to caller request order at the outcome boundary. The analytic
/// work ledger is charged before canonical discovery, during structural
/// discovery, and before fixed-degree interval allocation.
pub fn certify_body_distance_in_scope(
    store: &Store,
    body_a: BodyId,
    body_b: BodyId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<BodyDistanceOutcome> {
    store.get(body_a)?;
    store.get(body_b)?;
    if body_a == body_b {
        return Err(Error::InvalidGeometry {
            reason: "body distance requires two distinct bodies",
        });
    }
    let body_scan = structural_count(store.count::<Body>(), "body-distance body count overflow")?;
    let mut initial_work = 1_u64;
    add_work(&mut initial_work, body_scan, BODY_SCAN_WEIGHT)?;
    charge_analytic_amount(scope, initial_work)?;
    let canonical = canonical_bodies(store, body_a, body_b)?;
    debug_assert_eq!(canonical.body_scan, body_scan);
    let checks = [
        check_body_report_in_scope(store, canonical.ids[0], CheckLevel::Full, scope)?,
        check_body_report_in_scope(store, canonical.ids[1], CheckLevel::Full, scope)?,
    ];
    for index in 0..2 {
        if checks[index].outcome() != CheckOutcome::Valid {
            return Ok(refusal(
                canonical,
                &checks,
                BodyDistanceRefusal::BodyNotFullValid {
                    operand: canonical.operands[index],
                },
            ));
        }
    }
    for index in 0..2 {
        if store.get(canonical.ids[index])?.kind != BodyKind::Solid {
            return Ok(refusal(
                canonical,
                &checks,
                BodyDistanceRefusal::NonSolidBody {
                    operand: canonical.operands[index],
                },
            ));
        }
    }

    let first_structure = body_structure(store, canonical.ids[0], |count, weight| {
        charge_analytic_work(scope, count, weight)
    })?;
    let second_structure = body_structure(store, canonical.ids[1], |count, weight| {
        charge_analytic_work(scope, count, weight)
    })?;
    let structures = [first_structure, second_structure];
    charge_analytic_amount(
        scope,
        analytic_numeric_work(first_structure, second_structure)?,
    )?;

    let first = match prepare_body(
        store,
        canonical.ids[0],
        canonical.operands[0],
        structures[0],
    ) {
        Ok(body) => body,
        Err(reason) => return Ok(refusal(canonical, &checks, reason)),
    };
    let second = match prepare_body(
        store,
        canonical.ids[1],
        canonical.operands[1],
        structures[1],
    ) {
        Ok(body) => body,
        Err(reason) => return Ok(refusal(canonical, &checks, reason)),
    };
    let mut axes = Vec::new();
    first.append_axes(&mut axes);
    second.append_axes(&mut axes);

    let Some(lower) = support_lower(&first, &second, &axes) else {
        return Ok(refusal(
            canonical,
            &checks,
            BodyDistanceRefusal::IndeterminateEnclosure,
        ));
    };
    let Some(upper_witness) = witness_upper(&first, &second) else {
        return Ok(refusal(
            canonical,
            &checks,
            BodyDistanceRefusal::IndeterminateEnclosure,
        ));
    };
    let upper = upper_witness.distance().upper();
    if !lower.is_finite() || !upper.is_finite() || lower < 0.0 || lower > upper {
        return Ok(refusal(
            canonical,
            &checks,
            BodyDistanceRefusal::IndeterminateEnclosure,
        ));
    }
    let Some(distance) = ScalarEnclosure::from_interval(Interval::new(lower, upper)) else {
        return Ok(refusal(
            canonical,
            &checks,
            BodyDistanceRefusal::IndeterminateEnclosure,
        ));
    };
    Ok(BodyDistanceOutcome::Certified {
        distance,
        upper_witness: canonical.upper_witness(upper_witness),
        full_checks: canonical.reports(checks),
    })
}

#[cfg(test)]
mod tests;
