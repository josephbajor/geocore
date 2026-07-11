use super::result::{
    ContactKind, SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
};
use kcore::error::{CapabilityId, Error, Result};
use kcore::operation::{
    AccountingMode, BudgetPlan, DiagnosticCode, DiagnosticKind, ExecutionPolicy, LimitSnapshot,
    LimitSpec, NumericalPolicy, OperationContext, OperationPolicyError, OperationScope,
    PolicyVersion, ResourceKind, SessionPolicy, SessionPrecision, StageId,
};
use kcore::proof::{IncompleteCause, IncompleteEvidence};
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::implicit::ImplicitSurface;
use kgeom::nurbs::{
    ContextImplicitIsolationError, ImplicitIsolationLimits,
    NURBS_IMPLICIT_ISOLATION_CANDIDATE_LIMIT, NURBS_IMPLICIT_ISOLATION_CANDIDATES,
    NURBS_IMPLICIT_ISOLATION_DEPTH, NURBS_IMPLICIT_ISOLATION_DEPTH_LIMIT,
    NURBS_IMPLICIT_ISOLATION_NUMERIC_RESOLUTION, NURBS_IMPLICIT_ISOLATION_SUBDIVISION_LIMIT,
    NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS, NurbsCurve, NurbsSurface, NurbsSurfaceBvh,
};
use kgeom::param::ParamRange;
use kgeom::surface::{Dir, Surface};
use kgeom::vec::Point3;

const MIN_GRID_STEPS: usize = 24;
const MAX_GRID_STEPS: usize = 96;
const MAX_BISECTION_STEPS: usize = 80;
const PROOF_SUBDIVISION_DEPTH: u32 = 12;
const PROOF_CANDIDATE_BUDGET: usize = 4_096;
const PROOF_SUBDIVISION_WORK: u64 =
    1 + PROOF_SUBDIVISION_DEPTH as u64 * PROOF_CANDIDATE_BUDGET as u64;
const COMPLETION_REASON: &str =
    "fixed-grid NURBS surface marching does not prove complete coverage";

/// Stable stage for one signed-distance evaluation at a marching-grid sample.
pub const NURBS_SURFACE_MARCH_SAMPLES: StageId =
    match StageId::new("kops.intersect.ssi-grid-samples") {
        Ok(stage) => stage,
        Err(_) => panic!("valid NURBS surface marching stage"),
    };

/// Diagnostic emitted when the marching-grid sample allowance is exhausted.
pub const NURBS_SURFACE_MARCH_SAMPLE_LIMIT: DiagnosticCode =
    match DiagnosticCode::new("kops.intersect.ssi-grid-sample-limit") {
        Ok(code) => code,
        Err(_) => panic!("valid NURBS surface marching diagnostic code"),
    };

/// Stable incomplete-proof observation for the fixed-grid marching bridge.
pub const NURBS_SURFACE_MARCH_INCOMPLETE: DiagnosticCode =
    match DiagnosticCode::new("kops.intersect.ssi-fixed-grid-incomplete") {
        Ok(code) => code,
        Err(_) => panic!("valid NURBS surface marching diagnostic code"),
    };

/// Complete-domain exclusion proof unavailable from the fixed-grid marcher.
pub const NURBS_SURFACE_MARCH_COMPLETE_COVERAGE: CapabilityId =
    match CapabilityId::new("kops.intersect.ssi-fixed-grid-complete-coverage") {
        Ok(capability) => capability,
        Err(_) => panic!("valid NURBS surface marching capability"),
    };

/// Every diagnostic identity owned by the NURBS surface marcher, in stable
/// deterministic order.
///
/// Implicit-isolation proof diagnostics remain inventoried by `kgeom`; the
/// marcher references those identities without duplicating their ownership.
pub const NURBS_SURFACE_MARCH_DIAGNOSTICS: &[DiagnosticCode] = &[
    NURBS_SURFACE_MARCH_SAMPLE_LIMIT,
    NURBS_SURFACE_MARCH_INCOMPLETE,
];

/// Every capability identity owned by the NURBS surface marcher, in stable
/// deterministic order.
pub const NURBS_SURFACE_MARCH_CAPABILITIES: &[CapabilityId] =
    &[NURBS_SURFACE_MARCH_COMPLETE_COVERAGE];

/// Version-1 deterministic budget profile for the shared NURBS-surface marcher.
pub struct NurbsSurfaceMarchBudgetProfile;

impl NurbsSurfaceMarchBudgetProfile {
    /// Preserves the prior proof depth, candidate cover, and maximum `97 × 97`
    /// grid without earlier exhaustion.
    pub fn v1_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                PROOF_SUBDIVISION_WORK,
            ),
            LimitSpec::new(
                NURBS_IMPLICIT_ISOLATION_CANDIDATES,
                ResourceKind::Items,
                AccountingMode::HighWater,
                PROOF_CANDIDATE_BUDGET as u64,
            ),
            LimitSpec::new(
                NURBS_IMPLICIT_ISOLATION_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                u64::from(PROOF_SUBDIVISION_DEPTH),
            ),
            LimitSpec::new(
                NURBS_SURFACE_MARCH_SAMPLES,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                ((MAX_GRID_STEPS + 1) * (MAX_GRID_STEPS + 1)) as u64,
            ),
        ])
        .expect("built-in NURBS surface marching budget is valid")
    }
}

#[derive(Debug, Clone, PartialEq)]
pub(super) enum ContextMarchError {
    Kernel(Error),
    Limit(LimitSnapshot),
    Policy(OperationPolicyError),
}

impl From<Error> for ContextMarchError {
    fn from(error: Error) -> Self {
        Self::Kernel(error)
    }
}

#[derive(Clone, Copy)]
pub(super) struct MarchConfig<'a> {
    pub surface: &'a NurbsSurface,
    pub surface_range: [ParamRange; 2],
    pub tolerances: Tolerances,
    pub implicit_surface: &'a dyn ImplicitSurface,
    pub signed_distance: &'a dyn Fn(Point3) -> f64,
    pub other_uv: &'a dyn Fn(Point3) -> Option<[f64; 2]>,
    pub branch_kind: &'a dyn Fn(&[MarchPoint]) -> ContactKind,
    pub overlap_reason: &'static str,
    pub non_finite_reason: &'static str,
    pub finite_range_reason: &'static str,
    pub clamped_surface_reason: &'static str,
    pub domain_range_reason: &'static str,
}

#[derive(Debug, Clone, Copy)]
pub(super) struct MarchPoint {
    pub surface_uv: [f64; 2],
    pub other_uv: [f64; 2],
    pub point: Point3,
}

#[derive(Debug, Clone, Copy)]
struct Sample {
    uv: [f64; 2],
    distance: f64,
}

#[derive(Debug, Clone, Copy)]
struct CellHit {
    edge: usize,
    point: MarchPoint,
}

#[derive(Debug, Clone, Copy)]
struct Segment {
    a: MarchPoint,
    b: MarchPoint,
}

enum ProofCoverage {
    ProvenEmpty,
    Incomplete(Vec<IncompleteEvidence>),
}

fn fixed_grid_incomplete_evidence() -> IncompleteEvidence {
    IncompleteEvidence {
        code: NURBS_SURFACE_MARCH_INCOMPLETE,
        stage: NURBS_SURFACE_MARCH_SAMPLES,
        cause: IncompleteCause::ProofMethodUnavailable {
            capability: NURBS_SURFACE_MARCH_COMPLETE_COVERAGE,
        },
        message: COMPLETION_REASON,
    }
}

fn provisional_result(
    curves: Vec<SurfaceSurfaceCurve>,
    mut incomplete_evidence: Vec<IncompleteEvidence>,
) -> Result<SurfaceSurfaceIntersections> {
    incomplete_evidence.push(fixed_grid_incomplete_evidence());
    SurfaceSurfaceIntersections::canonicalized_with_incomplete_evidence(
        Vec::new(),
        curves,
        COMPLETION_REASON,
        incomplete_evidence,
    )
}

pub(super) fn march_nurbs_surface_intersection(
    config: MarchConfig<'_>,
) -> Result<SurfaceSurfaceIntersections> {
    let session = SessionPolicy::new(
        SessionPrecision::parasolid(),
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        NurbsSurfaceMarchBudgetProfile::v1_defaults(),
        PolicyVersion::V1,
    );
    let context = OperationContext::new(&session, config.tolerances)
        .expect("validated Tolerances always satisfy v1 session precision");
    let mut scope = OperationScope::new(&context);
    match march_nurbs_surface_intersection_in_scope(config, &mut scope) {
        Ok(result) => scope.finish(Ok(result)).into_result(),
        Err(ContextMarchError::Kernel(error)) => scope.finish(Err(error)).into_result(),
        Err(ContextMarchError::Limit(snapshot)) => scope
            .finish(Err(Error::ResourceLimit { snapshot }))
            .into_result(),
        Err(ContextMarchError::Policy(error)) => {
            panic!("built-in v1 NURBS surface marching policy is invalid: {error:?}")
        }
    }
}

pub(super) fn march_nurbs_surface_intersection_in_scope(
    config: MarchConfig<'_>,
    scope: &mut OperationScope<'_, '_>,
) -> core::result::Result<SurfaceSurfaceIntersections, ContextMarchError> {
    validate_nurbs_surface_range(config)?;
    let mut incomplete_evidence = match contextual_proof_coverage(config, scope)? {
        ProofCoverage::ProvenEmpty => return Ok(SurfaceSurfaceIntersections::complete_empty()),
        ProofCoverage::Incomplete(evidence) => evidence,
    };

    let parameter_tol = surface_parameter_tolerance(config.surface_range, config.tolerances);
    if parameter_window_is_tiny(config.surface_range, parameter_tol) {
        incomplete_evidence.push(fixed_grid_incomplete_evidence());
        return Ok(
            SurfaceSurfaceIntersections::indeterminate_empty_with_evidence(
                COMPLETION_REASON,
                incomplete_evidence,
            ),
        );
    }

    let (u_steps, v_steps) = marching_steps(config.surface);
    let samples = sample_grid_in_scope(config, u_steps, v_steps, scope)?;
    finish_sampled_march(
        config,
        parameter_tol,
        u_steps,
        v_steps,
        samples,
        incomplete_evidence,
    )
    .map_err(ContextMarchError::Kernel)
}

fn proof_range(config: MarchConfig<'_>) -> [ParamRange; 2] {
    let domain = config.surface.param_range();
    [
        ParamRange::new(
            config.surface_range[0].lo.max(domain[0].lo),
            config.surface_range[0].hi.min(domain[0].hi),
        ),
        ParamRange::new(
            config.surface_range[1].lo.max(domain[1].lo),
            config.surface_range[1].hi.min(domain[1].hi),
        ),
    ]
}

fn contextual_proof_coverage(
    config: MarchConfig<'_>,
    scope: &mut OperationScope<'_, '_>,
) -> core::result::Result<ProofCoverage, ContextMarchError> {
    // Charge before restriction/BVH construction so a zero root-work budget
    // cannot execute a proof and then report a complete result with zero work.
    match scope
        .ledger_mut()
        .charge(NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS, 1)
    {
        Ok(()) => {}
        Err(OperationPolicyError::LimitReached(snapshot)) => {
            let evidence = diagnose_limit(
                scope,
                snapshot,
                NURBS_IMPLICIT_ISOLATION_SUBDIVISION_LIMIT,
                "NURBS implicit-isolation proof setup limit reached",
            );
            return Ok(ProofCoverage::Incomplete(vec![evidence]));
        }
        Err(error) => return Err(ContextMarchError::Policy(error)),
    }

    let Ok(active_surface) = config.surface.restricted_to(proof_range(config)) else {
        return Ok(ProofCoverage::Incomplete(Vec::new()));
    };
    let Ok(hierarchy) = NurbsSurfaceBvh::build(&active_surface) else {
        return Ok(ProofCoverage::Incomplete(Vec::new()));
    };
    let isolation = hierarchy
        .isolate_implicit_candidates_in_scope(
            config.implicit_surface,
            config.tolerances.linear(),
            PROOF_SUBDIVISION_DEPTH,
            scope,
        )
        .map_err(|error| match error {
            ContextImplicitIsolationError::Kernel(error) => ContextMarchError::Kernel(error),
            ContextImplicitIsolationError::Policy(error) => ContextMarchError::Policy(error),
        })?;
    let incomplete_evidence = diagnose_isolation_limits(scope, isolation.limits());
    if isolation.is_proven_empty() && incomplete_evidence.is_empty() {
        Ok(ProofCoverage::ProvenEmpty)
    } else {
        Ok(ProofCoverage::Incomplete(incomplete_evidence))
    }
}

/// Evidence order is the deterministic proof-obligation order: subdivision
/// work, retained candidates, depth, then numerical resolution. The caller
/// appends the fixed-grid proof-method gap after these proof-stage stops.
fn diagnose_isolation_limits(
    scope: &mut OperationScope<'_, '_>,
    limits: ImplicitIsolationLimits,
) -> Vec<IncompleteEvidence> {
    let mut evidence = Vec::new();
    if let Some(snapshot) = limits.subdivision_work() {
        evidence.push(diagnose_limit(
            scope,
            snapshot,
            NURBS_IMPLICIT_ISOLATION_SUBDIVISION_LIMIT,
            "NURBS implicit-isolation subdivision limit reached",
        ));
    }
    if let Some(snapshot) = limits.candidate_cells() {
        evidence.push(diagnose_limit(
            scope,
            snapshot,
            NURBS_IMPLICIT_ISOLATION_CANDIDATE_LIMIT,
            "NURBS implicit-isolation candidate-cover limit reached",
        ));
    }
    if let Some(snapshot) = limits.subdivision_depth() {
        evidence.push(diagnose_limit(
            scope,
            snapshot,
            NURBS_IMPLICIT_ISOLATION_DEPTH_LIMIT,
            "NURBS implicit-isolation depth limit reached",
        ));
    }
    if limits.parameter_resolution() {
        const MESSAGE: &str =
            "NURBS implicit isolation stopped at floating-point parameter resolution";
        scope.diagnose(
            NURBS_IMPLICIT_ISOLATION_DEPTH,
            NURBS_IMPLICIT_ISOLATION_NUMERIC_RESOLUTION,
            DiagnosticKind::NumericResolution,
            MESSAGE,
        );
        evidence.push(IncompleteEvidence {
            code: NURBS_IMPLICIT_ISOLATION_NUMERIC_RESOLUTION,
            stage: NURBS_IMPLICIT_ISOLATION_DEPTH,
            cause: IncompleteCause::NumericResolution,
            message: MESSAGE,
        });
    }
    evidence
}

fn diagnose_limit(
    scope: &mut OperationScope<'_, '_>,
    snapshot: LimitSnapshot,
    code: DiagnosticCode,
    message: &'static str,
) -> IncompleteEvidence {
    scope.diagnose(
        snapshot.stage,
        code,
        DiagnosticKind::LimitReached(snapshot),
        message,
    );
    IncompleteEvidence {
        code,
        stage: snapshot.stage,
        cause: IncompleteCause::Limit { snapshot },
        message,
    }
}

fn parameter_window_is_tiny(range: [ParamRange; 2], parameter_tol: f64) -> bool {
    range[0].width() <= parameter_tol || range[1].width() <= parameter_tol
}

fn finish_sampled_march(
    config: MarchConfig<'_>,
    parameter_tol: f64,
    u_steps: usize,
    v_steps: usize,
    samples: Vec<Sample>,
    incomplete_evidence: Vec<IncompleteEvidence>,
) -> Result<SurfaceSurfaceIntersections> {
    if samples
        .iter()
        .all(|sample| sample.distance.abs() <= config.tolerances.linear())
    {
        return Err(Error::InvalidGeometry {
            reason: config.overlap_reason,
        });
    }

    let mut segments = Vec::new();
    for i in 0..u_steps {
        for j in 0..v_steps {
            collect_cell_segments(
                config,
                [
                    sample_at(&samples, v_steps, i, j),
                    sample_at(&samples, v_steps, i + 1, j),
                    sample_at(&samples, v_steps, i + 1, j + 1),
                    sample_at(&samples, v_steps, i, j + 1),
                ],
                parameter_tol,
                &mut segments,
            );
        }
    }

    let polylines = join_segments(segments, parameter_tol, config.tolerances);
    let mut curves = Vec::new();
    for polyline in polylines {
        if let Some(curve) = branch_from_polyline(config, polyline, parameter_tol)? {
            curves.push(curve);
        }
    }
    provisional_result(curves, incomplete_evidence)
}

fn collect_cell_segments(
    config: MarchConfig<'_>,
    corners: [Sample; 4],
    parameter_tol: f64,
    segments: &mut Vec<Segment>,
) {
    if corners
        .iter()
        .all(|corner| corner.distance.abs() <= config.tolerances.linear())
    {
        return;
    }

    let edge_corners = [(0, 1), (1, 2), (2, 3), (3, 0)];
    let mut hits = Vec::new();
    for (edge, (a, b)) in edge_corners.into_iter().enumerate() {
        for point in edge_roots(config, corners[a], corners[b], parameter_tol) {
            push_cell_hit(
                &mut hits,
                CellHit { edge, point },
                parameter_tol,
                config.tolerances,
            );
        }
    }

    hits.sort_by(|a, b| {
        a.edge
            .cmp(&b.edge)
            .then(a.point.surface_uv[0].total_cmp(&b.point.surface_uv[0]))
            .then(a.point.surface_uv[1].total_cmp(&b.point.surface_uv[1]))
    });

    match hits.len() {
        0 | 1 => {}
        2 => push_segment(
            segments,
            Segment {
                a: hits[0].point,
                b: hits[1].point,
            },
            parameter_tol,
            config.tolerances,
        ),
        _ => {
            for pair in hits.chunks_exact(2) {
                push_segment(
                    segments,
                    Segment {
                        a: pair[0].point,
                        b: pair[1].point,
                    },
                    parameter_tol,
                    config.tolerances,
                );
            }
        }
    }
}

fn edge_roots(
    config: MarchConfig<'_>,
    a: Sample,
    b: Sample,
    parameter_tol: f64,
) -> Vec<MarchPoint> {
    let a_zero = a.distance.abs() <= config.tolerances.linear();
    let b_zero = b.distance.abs() <= config.tolerances.linear();
    if a_zero && b_zero {
        return [a.uv, b.uv]
            .into_iter()
            .filter_map(|uv| march_point(config, uv, parameter_tol))
            .collect();
    }
    if a_zero {
        return march_point(config, a.uv, parameter_tol)
            .into_iter()
            .collect();
    }
    if b_zero {
        return march_point(config, b.uv, parameter_tol)
            .into_iter()
            .collect();
    }
    if same_sign(a.distance, b.distance) {
        return Vec::new();
    }

    let mut lo_uv = a.uv;
    let mut hi_uv = b.uv;
    let mut f_lo = a.distance;
    let mut root_uv = midpoint_uv(lo_uv, hi_uv);
    for _ in 0..MAX_BISECTION_STEPS {
        root_uv = midpoint_uv(lo_uv, hi_uv);
        let f_mid = (config.signed_distance)(config.surface.eval(root_uv));
        if f_mid.abs() <= config.tolerances.linear() || uv_distance(lo_uv, hi_uv) <= parameter_tol {
            break;
        }
        if same_sign(f_lo, f_mid) {
            lo_uv = root_uv;
            f_lo = f_mid;
        } else {
            hi_uv = root_uv;
        }
    }

    march_point(config, root_uv, parameter_tol)
        .into_iter()
        .collect()
}

fn branch_from_polyline(
    config: MarchConfig<'_>,
    polyline: Vec<MarchPoint>,
    parameter_tol: f64,
) -> Result<Option<SurfaceSurfaceCurve>> {
    let points = distinct_polyline_points(polyline, config.tolerances);
    if points.len() < 2 {
        return Ok(None);
    }
    let Some(nurbs) = polyline_nurbs(&points, config.tolerances)? else {
        return Ok(None);
    };
    let range = nurbs.param_range();
    let start = points[0];
    let end = points[points.len() - 1];
    Ok(Some(SurfaceSurfaceCurve {
        curve: SurfaceIntersectionCurve::Nurbs(nurbs),
        curve_range: range,
        uv_a_start: start.other_uv,
        uv_a_end: end.other_uv,
        uv_b_start: fit_uv(start.surface_uv, config.surface_range, parameter_tol)
            .unwrap_or(start.surface_uv),
        uv_b_end: fit_uv(end.surface_uv, config.surface_range, parameter_tol)
            .unwrap_or(end.surface_uv),
        kind: (config.branch_kind)(&points),
    }))
}

fn polyline_nurbs(points: &[MarchPoint], tolerances: Tolerances) -> Result<Option<NurbsCurve>> {
    let mut controls = Vec::with_capacity(points.len());
    let mut cumulative = Vec::with_capacity(points.len());
    let mut length = 0.0;
    controls.push(points[0].point);
    cumulative.push(0.0);
    for point in &points[1..] {
        let step = controls[controls.len() - 1].dist(point.point);
        if step <= tolerances.linear() {
            continue;
        }
        length += step;
        controls.push(point.point);
        cumulative.push(length);
    }
    if controls.len() < 2 || length <= tolerances.linear() {
        return Ok(None);
    }

    let mut knots = vec![0.0, 0.0];
    for s in cumulative
        .iter()
        .skip(1)
        .take(cumulative.len().saturating_sub(2))
    {
        knots.push(*s / length);
    }
    knots.push(1.0);
    knots.push(1.0);
    Ok(Some(NurbsCurve::new(1, knots, controls, None)?))
}

fn distinct_polyline_points(polyline: Vec<MarchPoint>, tolerances: Tolerances) -> Vec<MarchPoint> {
    let mut points = Vec::new();
    for point in polyline {
        if points.last().is_none_or(|last: &MarchPoint| {
            !points_match(last, &point, tolerances.angular(), tolerances)
        }) {
            points.push(point);
        }
    }
    points
}

fn join_segments(
    mut segments: Vec<Segment>,
    parameter_tol: f64,
    tolerances: Tolerances,
) -> Vec<Vec<MarchPoint>> {
    let mut polylines = Vec::new();
    while let Some(segment) = segments.pop() {
        let mut polyline = vec![segment.a, segment.b];
        let mut changed = true;
        while changed {
            changed = false;
            let mut index = 0;
            while index < segments.len() {
                if attach_segment(&mut polyline, segments[index], parameter_tol, tolerances) {
                    segments.swap_remove(index);
                    changed = true;
                } else {
                    index += 1;
                }
            }
        }
        polylines.push(polyline);
    }
    polylines
}

fn attach_segment(
    polyline: &mut Vec<MarchPoint>,
    segment: Segment,
    parameter_tol: f64,
    tolerances: Tolerances,
) -> bool {
    let front = polyline[0];
    let back = polyline[polyline.len() - 1];
    if points_match(&back, &segment.a, parameter_tol, tolerances) {
        polyline.push(segment.b);
        true
    } else if points_match(&back, &segment.b, parameter_tol, tolerances) {
        polyline.push(segment.a);
        true
    } else if points_match(&front, &segment.b, parameter_tol, tolerances) {
        polyline.insert(0, segment.a);
        true
    } else if points_match(&front, &segment.a, parameter_tol, tolerances) {
        polyline.insert(0, segment.b);
        true
    } else {
        false
    }
}

fn march_point(config: MarchConfig<'_>, uv: [f64; 2], parameter_tol: f64) -> Option<MarchPoint> {
    let surface_uv = fit_uv(uv, config.surface_range, parameter_tol)?;
    let point = config.surface.eval(surface_uv);
    if (config.signed_distance)(point).abs() > 4.0 * config.tolerances.linear() {
        return None;
    }
    let other_uv = (config.other_uv)(point)?;
    Some(MarchPoint {
        surface_uv,
        other_uv,
        point,
    })
}

fn sample_grid_in_scope(
    config: MarchConfig<'_>,
    u_steps: usize,
    v_steps: usize,
    scope: &mut OperationScope<'_, '_>,
) -> core::result::Result<Vec<Sample>, ContextMarchError> {
    let mut samples = Vec::with_capacity((u_steps + 1) * (v_steps + 1));
    for i in 0..=u_steps {
        for j in 0..=v_steps {
            charge_grid_sample(scope)?;
            samples.push(
                evaluate_grid_sample(config, u_steps, v_steps, i, j)
                    .map_err(ContextMarchError::Kernel)?,
            );
        }
    }
    Ok(samples)
}

fn evaluate_grid_sample(
    config: MarchConfig<'_>,
    u_steps: usize,
    v_steps: usize,
    i: usize,
    j: usize,
) -> Result<Sample> {
    let uv = [
        config.surface_range[0].lerp(i as f64 / u_steps as f64),
        config.surface_range[1].lerp(j as f64 / v_steps as f64),
    ];
    let distance = (config.signed_distance)(config.surface.eval(uv));
    if !distance.is_finite() {
        return Err(Error::InvalidGeometry {
            reason: config.non_finite_reason,
        });
    }
    Ok(Sample { uv, distance })
}

fn charge_grid_sample(
    scope: &mut OperationScope<'_, '_>,
) -> core::result::Result<(), ContextMarchError> {
    match scope.ledger_mut().charge(NURBS_SURFACE_MARCH_SAMPLES, 1) {
        Ok(()) => Ok(()),
        Err(OperationPolicyError::LimitReached(snapshot)) => {
            Err(ContextMarchError::Limit(snapshot))
        }
        Err(error) => Err(ContextMarchError::Policy(error)),
    }
}

fn sample_at(samples: &[Sample], v_steps: usize, i: usize, j: usize) -> Sample {
    samples[i * (v_steps + 1) + j]
}

fn marching_steps(surface: &NurbsSurface) -> (usize, usize) {
    let (nu, nv) = surface.net_size();
    (
        ((nu + surface.degree_u()) * 8).clamp(MIN_GRID_STEPS, MAX_GRID_STEPS),
        ((nv + surface.degree_v()) * 8).clamp(MIN_GRID_STEPS, MAX_GRID_STEPS),
    )
}

fn midpoint_uv(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]
}

fn uv_distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    let du = a[0] - b[0];
    let dv = a[1] - b[1];
    (du * du + dv * dv).sqrt()
}

fn same_sign(a: f64, b: f64) -> bool {
    (a < 0.0 && b < 0.0) || (a > 0.0 && b > 0.0)
}

fn fit_uv(candidate: [f64; 2], range: [ParamRange; 2], tolerance: f64) -> Option<[f64; 2]> {
    let mut uv = [0.0; 2];
    for axis in 0..2 {
        if candidate[axis] < range[axis].lo - tolerance
            || candidate[axis] > range[axis].hi + tolerance
        {
            return None;
        }
        uv[axis] = candidate[axis].clamp(range[axis].lo, range[axis].hi);
    }
    Some(uv)
}

fn points_match(
    a: &MarchPoint,
    b: &MarchPoint,
    parameter_tol: f64,
    tolerances: Tolerances,
) -> bool {
    uv_distance(a.surface_uv, b.surface_uv) <= parameter_tol
        || a.point.dist(b.point) <= tolerances.linear()
}

fn push_cell_hit(
    hits: &mut Vec<CellHit>,
    candidate: CellHit,
    parameter_tol: f64,
    tolerances: Tolerances,
) {
    if !hits
        .iter()
        .any(|hit| points_match(&hit.point, &candidate.point, parameter_tol, tolerances))
    {
        hits.push(candidate);
    }
}

fn push_segment(
    segments: &mut Vec<Segment>,
    candidate: Segment,
    parameter_tol: f64,
    tolerances: Tolerances,
) {
    if points_match(&candidate.a, &candidate.b, parameter_tol, tolerances) {
        return;
    }
    if !segments.iter().any(|segment| {
        (points_match(&segment.a, &candidate.a, parameter_tol, tolerances)
            && points_match(&segment.b, &candidate.b, parameter_tol, tolerances))
            || (points_match(&segment.a, &candidate.b, parameter_tol, tolerances)
                && points_match(&segment.b, &candidate.a, parameter_tol, tolerances))
    }) {
        segments.push(candidate);
    }
}

fn surface_parameter_tolerance(range: [ParamRange; 2], tolerances: Tolerances) -> f64 {
    (range[0].width().abs().max(range[1].width().abs()) * 1e-10)
        .max(tolerances.angular())
        .max(1e-12)
}

fn validate_nurbs_surface_range(config: MarchConfig<'_>) -> Result<()> {
    if config
        .surface_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: config.finite_range_reason,
        });
    }
    if !config.surface.knots(Dir::U).is_clamped() || !config.surface.knots(Dir::V).is_clamped() {
        return Err(Error::InvalidGeometry {
            reason: config.clamped_surface_reason,
        });
    }
    let domain = config.surface.param_range();
    let parameter_tol = surface_parameter_tolerance(domain, config.tolerances);
    for (axis, domain_axis) in domain.iter().enumerate() {
        if config.surface_range[axis].lo < domain_axis.lo - parameter_tol
            || config.surface_range[axis].hi > domain_axis.hi + parameter_tol
        {
            return Err(Error::InvalidGeometry {
                reason: config.domain_range_reason,
            });
        }
    }
    Ok(())
}
