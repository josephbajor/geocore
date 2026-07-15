//! Compatible direct NURBS/NURBS surface marching.
//!
//! This first paired-surface slice accepts two finite-open clamped sources
//! with the same quadratic-linear unit-square basis, constant shared weights,
//! and exact identity `x/y` control fields. Direct and constant-normal
//! Offset(NURBS)/NURBS pairs accept distinct windows with a positive-area
//! overlap and clip discovery to that shared rectangle. Both
//! sources therefore have the injective chart `(x,y) = (u,v)`, and their
//! spatial difference is confined to `z`, so a zero contour of the scalar `z`
//! difference is a surface/surface contact.
//! The derived scalar surface is discovery-only: complete misses are proved
//! from outward original-control differences, and promoted branches are
//! independently certified against both original sources.

use super::nurbs_surface_march::{
    ContextMarchError, MarchConfig, MarchOutput, MarchPoint, NURBS_SURFACE_MARCH_SAMPLE_LIMIT,
    NurbsSurfaceMarchBudgetProfile, march_nurbs_surface_intersection_with_traces_in_scope,
};
use super::result::{ContactKind, SurfaceIntersectionCurve, SurfaceSurfaceIntersections};
use kcore::error::{Error, Result};
use kcore::interval::Interval;
use kcore::operation::{
    AccountingMode, DiagnosticKind, ExecutionPolicy, NumericalPolicy, OperationContext,
    OperationOutcome, OperationPolicyError, OperationScope, PolicyVersion, ResourceKind,
    SessionPolicy, SessionPrecision,
};
use kcore::tolerance::{LINEAR_RESOLUTION, Tolerances};
use kgeom::curve::Curve;
use kgeom::curve2d::NurbsCurve2d;
use kgeom::frame::Frame;
use kgeom::nurbs::{
    NURBS_IMPLICIT_ISOLATION_CANDIDATES, NURBS_IMPLICIT_ISOLATION_DEPTH,
    NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS, NurbsCurve, NurbsSurface,
};
use kgeom::param::ParamRange;
use kgeom::surface::{Dir, Plane, Surface};
use kgeom::vec::Point3;

const INCOMPATIBLE_REASON: &str = "direct NURBS/NURBS surface intersection requires the identical finite-open quadratic-linear unit chart, constant weights, and positive-area parameter-window overlap";

/// Intersect one compatible pair of direct clamped NURBS surfaces over their
/// positive-area shared unit-chart window.
pub fn intersect_bounded_nurbs_nurbs_surfaces(
    surface_a: &NurbsSurface,
    range_a: [ParamRange; 2],
    surface_b: &NurbsSurface,
    range_b: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let session = SessionPolicy::new(
        SessionPrecision::parasolid(),
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        NurbsSurfaceMarchBudgetProfile::v1_defaults(),
        PolicyVersion::V1,
    );
    let context = OperationContext::new(&session, tolerances)
        .expect("validated Tolerances always satisfy v1 session precision");
    intersect_bounded_nurbs_nurbs_surfaces_with_context(
        surface_a, range_a, surface_b, range_b, &context,
    )
    .expect("built-in v1 NURBS/NURBS surface policy is valid")
    .into_result()
}

/// Context-aware compatible direct NURBS/NURBS surface intersection.
pub fn intersect_bounded_nurbs_nurbs_surfaces_with_context(
    surface_a: &NurbsSurface,
    range_a: [ParamRange; 2],
    surface_b: &NurbsSurface,
    range_b: [ParamRange; 2],
    context: &OperationContext<'_>,
) -> core::result::Result<OperationOutcome<SurfaceSurfaceIntersections>, OperationPolicyError> {
    validate_context_budget(context)?;
    let mut scope = OperationScope::new(context);
    let result = intersect_bounded_nurbs_nurbs_surfaces_with_traces_in_scope(
        surface_a,
        range_a,
        surface_b,
        range_b,
        context.tolerances(),
        &mut scope,
    );
    match result {
        Ok(output) => Ok(scope.finish(Ok(output.result))),
        Err(ContextMarchError::Kernel(error)) => Ok(scope.finish(Err(error))),
        Err(ContextMarchError::Limit(snapshot)) => {
            scope.diagnose(
                snapshot.stage,
                NURBS_SURFACE_MARCH_SAMPLE_LIMIT,
                DiagnosticKind::LimitReached(snapshot),
                "NURBS surface marching grid sample limit reached",
            );
            Ok(scope.finish(Err(Error::ResourceLimit { snapshot })))
        }
        Err(ContextMarchError::Policy(error)) => Err(error),
    }
}

/// Intersect one constant-normal offset of a compatible planar NURBS basis
/// with one direct compatible NURBS surface over their positive-area shared
/// unit-chart window.
pub fn intersect_bounded_offset_nurbs_nurbs_surfaces(
    basis: &NurbsSurface,
    signed_distance: f64,
    offset_range: [ParamRange; 2],
    direct: &NurbsSurface,
    direct_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let session = SessionPolicy::new(
        SessionPrecision::parasolid(),
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        NurbsSurfaceMarchBudgetProfile::v1_defaults(),
        PolicyVersion::V1,
    );
    let context = OperationContext::new(&session, tolerances)
        .expect("validated Tolerances always satisfy v1 session precision");
    intersect_bounded_offset_nurbs_nurbs_surfaces_with_context(
        basis,
        signed_distance,
        offset_range,
        direct,
        direct_range,
        &context,
    )
    .expect("built-in v1 Offset(NURBS)/NURBS surface policy is valid")
    .into_result()
}

/// Context-aware constant-normal Offset(NURBS)/NURBS surface intersection
/// over the positive-area overlap of the two requested unit-chart windows.
pub fn intersect_bounded_offset_nurbs_nurbs_surfaces_with_context(
    basis: &NurbsSurface,
    signed_distance: f64,
    offset_range: [ParamRange; 2],
    direct: &NurbsSurface,
    direct_range: [ParamRange; 2],
    context: &OperationContext<'_>,
) -> core::result::Result<OperationOutcome<SurfaceSurfaceIntersections>, OperationPolicyError> {
    validate_context_budget(context)?;
    let mut scope = OperationScope::new(context);
    let result = intersect_bounded_offset_nurbs_nurbs_surfaces_with_traces_in_scope(
        basis,
        signed_distance,
        offset_range,
        direct,
        direct_range,
        context.tolerances(),
        &mut scope,
    );
    match result {
        Ok(output) => Ok(scope.finish(Ok(output.result))),
        Err(ContextMarchError::Kernel(error)) => Ok(scope.finish(Err(error))),
        Err(ContextMarchError::Limit(snapshot)) => {
            scope.diagnose(
                snapshot.stage,
                NURBS_SURFACE_MARCH_SAMPLE_LIMIT,
                DiagnosticKind::LimitReached(snapshot),
                "NURBS surface marching grid sample limit reached",
            );
            Ok(scope.finish(Err(Error::ResourceLimit { snapshot })))
        }
        Err(ContextMarchError::Policy(error)) => Err(error),
    }
}

pub(super) fn supports_direct_nurbs_nurbs_surface_pair(
    surface_a: &NurbsSurface,
    range_a: [ParamRange; 2],
    surface_b: &NurbsSurface,
    range_b: [ParamRange; 2],
) -> bool {
    supports_compatible_nurbs_unit_chart_pair(surface_a, surface_b)
        && shared_positive_unit_chart_window(range_a, range_b).is_some()
}

fn supports_compatible_nurbs_unit_chart_pair(
    surface_a: &NurbsSurface,
    surface_b: &NurbsSurface,
) -> bool {
    let domain = [ParamRange::new(0.0, 1.0), ParamRange::new(0.0, 1.0)];
    surface_a.param_range() == domain
        && surface_b.param_range() == domain
        && surface_a.periodicity() == [None, None]
        && surface_b.periodicity() == [None, None]
        && surface_a.degree_u() == 2
        && surface_a.degree_v() == 1
        && surface_b.degree_u() == 2
        && surface_b.degree_v() == 1
        && surface_a.knots(Dir::U).is_clamped()
        && surface_a.knots(Dir::V).is_clamped()
        && surface_b.knots(Dir::U).is_clamped()
        && surface_b.knots(Dir::V).is_clamped()
        && surface_a.knots(Dir::U).as_slice() == [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]
        && surface_b.knots(Dir::U).as_slice() == [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]
        && surface_a.knots(Dir::V).as_slice() == [0.0, 0.0, 1.0, 1.0]
        && surface_b.knots(Dir::V).as_slice() == [0.0, 0.0, 1.0, 1.0]
        && surface_a.weights() == surface_b.weights()
        && surface_a
            .weights()
            .is_none_or(|weights| weights.iter().all(|weight| *weight == weights[0]))
        && exact_unit_xy_controls(surface_a)
        && exact_unit_xy_controls(surface_b)
}

fn positive_unit_chart_window(window: [ParamRange; 2]) -> bool {
    let domain = [ParamRange::new(0.0, 1.0), ParamRange::new(0.0, 1.0)];
    window.iter().zip(domain).all(|(range, domain)| {
        range.is_finite() && range.width() > 0.0 && range.lo >= domain.lo && range.hi <= domain.hi
    })
}

fn shared_positive_unit_chart_window(
    window_a: [ParamRange; 2],
    window_b: [ParamRange; 2],
) -> Option<[ParamRange; 2]> {
    if !positive_unit_chart_window(window_a) || !positive_unit_chart_window(window_b) {
        return None;
    }
    let shared_axis = |axis: usize| {
        let lo = window_a[axis].lo.max(window_b[axis].lo);
        let hi = window_a[axis].hi.min(window_b[axis].hi);
        (hi > lo).then(|| ParamRange::new(lo, hi))
    };
    Some([shared_axis(0)?, shared_axis(1)?])
}

pub(super) fn supports_offset_nurbs_nurbs_surface_pair(
    basis: &NurbsSurface,
    signed_distance: f64,
    offset_range: [ParamRange; 2],
    direct: &NurbsSurface,
    direct_range: [ParamRange; 2],
) -> bool {
    signed_distance.is_finite()
        && supports_compatible_nurbs_unit_chart_pair(basis, direct)
        && shared_positive_unit_chart_window(offset_range, direct_range).is_some()
        && supports_constant_positive_normal_offset(basis, signed_distance)
}

pub(super) fn supports_strictly_separated_constant_normal_offset_nurbs_pair(
    basis_a: &NurbsSurface,
    signed_distance_a: f64,
    range_a: [ParamRange; 2],
    basis_b: &NurbsSurface,
    signed_distance_b: f64,
    range_b: [ParamRange; 2],
) -> bool {
    range_a == range_b
        && positive_unit_chart_window(range_a)
        && supports_compatible_nurbs_unit_chart_pair(basis_a, basis_b)
        && supports_constant_positive_normal_offset(basis_a, signed_distance_a)
        && supports_constant_positive_normal_offset(basis_b, signed_distance_b)
        && constant_normal_offset_nurbs_pair_proves_empty(
            basis_a,
            signed_distance_a,
            basis_b,
            signed_distance_b,
        )
}

fn supports_constant_positive_normal_offset(basis: &NurbsSurface, signed_distance: f64) -> bool {
    signed_distance.is_finite()
        && basis
            .points()
            .first()
            .is_some_and(|first| basis.points().iter().all(|point| point.z == first.z))
        && basis.points().iter().all(|point| {
            let lifted = Interval::point(point.z) + Interval::point(signed_distance);
            lifted.lo().is_finite() && lifted.hi().is_finite()
        })
}

fn exact_unit_xy_controls(surface: &NurbsSurface) -> bool {
    let expected_u = [0.0, 0.5, 1.0];
    let expected_v = [0.0, 1.0];
    surface.points().len() == 6
        && surface.points().iter().enumerate().all(|(index, point)| {
            point.x == expected_u[index / 2] && point.y == expected_v[index % 2]
        })
}

pub(super) fn intersect_bounded_nurbs_nurbs_surfaces_with_traces_in_scope(
    surface_a: &NurbsSurface,
    range_a: [ParamRange; 2],
    surface_b: &NurbsSurface,
    range_b: [ParamRange; 2],
    tolerances: Tolerances,
    scope: &mut OperationScope<'_, '_>,
) -> core::result::Result<MarchOutput, ContextMarchError> {
    if !supports_direct_nurbs_nurbs_surface_pair(surface_a, range_a, surface_b, range_b) {
        return Err(ContextMarchError::Kernel(Error::InvalidGeometry {
            reason: INCOMPATIBLE_REASON,
        }));
    }
    intersect_compatible_nurbs_pair_with_traces_in_scope(
        surface_a, range_a, surface_b, range_b, tolerances, true, scope,
    )
}

pub(super) fn intersect_bounded_offset_nurbs_nurbs_surfaces_with_traces_in_scope(
    basis: &NurbsSurface,
    signed_distance: f64,
    offset_range: [ParamRange; 2],
    direct: &NurbsSurface,
    direct_range: [ParamRange; 2],
    tolerances: Tolerances,
    scope: &mut OperationScope<'_, '_>,
) -> core::result::Result<MarchOutput, ContextMarchError> {
    if !supports_offset_nurbs_nurbs_surface_pair(
        basis,
        signed_distance,
        offset_range,
        direct,
        direct_range,
    ) {
        return Err(ContextMarchError::Kernel(Error::InvalidGeometry {
            reason: "Offset(NURBS)/NURBS surface intersection requires a direct constant-positive-normal unit-chart basis, compatible direct source, and positive-area window overlap",
        }));
    }
    if offset_control_difference_proves_empty(basis, signed_distance, direct) {
        return Ok(MarchOutput {
            result: SurfaceSurfaceIntersections::complete_empty(),
            traces: Vec::new(),
        });
    }
    let effective = constant_normal_offset_surface(basis, signed_distance)?;
    intersect_compatible_nurbs_pair_with_traces_in_scope(
        &effective,
        offset_range,
        direct,
        direct_range,
        tolerances,
        false,
        scope,
    )
}

#[allow(clippy::too_many_arguments)]
fn intersect_compatible_nurbs_pair_with_traces_in_scope(
    surface_a: &NurbsSurface,
    range_a: [ParamRange; 2],
    surface_b: &NurbsSurface,
    range_b: [ParamRange; 2],
    tolerances: Tolerances,
    original_control_miss_is_authoritative: bool,
    scope: &mut OperationScope<'_, '_>,
) -> core::result::Result<MarchOutput, ContextMarchError> {
    let Some(shared_range) = shared_positive_unit_chart_window(range_a, range_b) else {
        return Err(ContextMarchError::Kernel(Error::InvalidGeometry {
            reason: INCOMPATIBLE_REASON,
        }));
    };
    if !supports_compatible_nurbs_unit_chart_pair(surface_a, surface_b) {
        return Err(ContextMarchError::Kernel(Error::InvalidGeometry {
            reason: INCOMPATIBLE_REASON,
        }));
    }
    if original_control_miss_is_authoritative
        && original_control_difference_proves_empty(surface_a, surface_b)
    {
        return Ok(MarchOutput {
            result: SurfaceSurfaceIntersections::complete_empty(),
            traces: Vec::new(),
        });
    }

    let difference = scalar_difference_surface(surface_a, surface_b)?;
    let plane = Plane::new(Frame::world());
    let signed_distance = |point: Point3| point.z;
    let other_uv = |point: Point3| Some([point.x, point.y]);
    let branch_kind =
        |points: &[MarchPoint]| paired_branch_kind(surface_a, surface_b, points, tolerances);
    let output = march_nurbs_surface_intersection_with_traces_in_scope(
        MarchConfig {
            surface: &difference,
            surface_range: shared_range,
            // A tighter discovery tolerance keeps sub-tolerance cell edges
            // available for joining into a positive-length branch. The
            // caller tolerance still owns final whole-range certification.
            tolerances: Tolerances::with_linear(
                (tolerances.linear() / 1_024.0).max(LINEAR_RESOLUTION),
            )
            .expect("derived discovery tolerance respects the session floor"),
            implicit_surface: &plane,
            // `difference` contains rounded control-point subtractions, so
            // its interval isolation may guide discovery but cannot prove an
            // original-source miss. The outward control proof above owns the
            // only complete-empty exit for this paired arm.
            implicit_empty_is_authoritative: false,
            signed_distance: &signed_distance,
            other_uv: &other_uv,
            branch_kind: &branch_kind,
            overlap_reason: "coincident direct NURBS/NURBS intersection is a surface overlap",
            non_finite_reason: "direct NURBS/NURBS intersection sampled non-finite geometry",
            finite_range_reason: "direct NURBS/NURBS intersection requires finite non-reversed ranges",
            clamped_surface_reason: "direct NURBS/NURBS intersection requires clamped surfaces",
            domain_range_reason: "direct NURBS/NURBS ranges must lie within the shared domain",
        },
        scope,
    )?;
    lift_output_to_original_sources(output, surface_a, surface_b)
}

fn constant_normal_offset_surface(
    basis: &NurbsSurface,
    signed_distance: f64,
) -> Result<NurbsSurface> {
    let points = basis
        .points()
        .iter()
        .map(|point| Point3::new(point.x, point.y, point.z + signed_distance))
        .collect();
    NurbsSurface::new(
        basis.degree_u(),
        basis.degree_v(),
        basis.knots(Dir::U).as_slice().to_vec(),
        basis.knots(Dir::V).as_slice().to_vec(),
        points,
        basis.weights().map(<[f64]>::to_vec),
    )
}

fn offset_control_difference_proves_empty(
    basis: &NurbsSurface,
    signed_distance: f64,
    direct: &NurbsSurface,
) -> bool {
    let mut lower = f64::INFINITY;
    let mut upper = f64::NEG_INFINITY;
    for (basis, direct) in basis.points().iter().zip(direct.points()) {
        let difference =
            Interval::point(basis.z) + Interval::point(signed_distance) - Interval::point(direct.z);
        lower = lower.min(difference.lo());
        upper = upper.max(difference.hi());
    }
    lower > 0.0 || upper < 0.0
}

fn constant_normal_offset_nurbs_pair_proves_empty(
    basis_a: &NurbsSurface,
    signed_distance_a: f64,
    basis_b: &NurbsSurface,
    signed_distance_b: f64,
) -> bool {
    let mut lower = f64::INFINITY;
    let mut upper = f64::NEG_INFINITY;
    for (basis_a, basis_b) in basis_a.points().iter().zip(basis_b.points()) {
        let difference = Interval::point(basis_a.z) + Interval::point(signed_distance_a)
            - Interval::point(basis_b.z)
            - Interval::point(signed_distance_b);
        lower = lower.min(difference.lo());
        upper = upper.max(difference.hi());
    }
    lower > 0.0 || upper < 0.0
}

fn scalar_difference_surface(
    surface_a: &NurbsSurface,
    surface_b: &NurbsSurface,
) -> Result<NurbsSurface> {
    let points = surface_a
        .points()
        .iter()
        .zip(surface_b.points())
        .map(|(a, b)| Point3::new(a.x, a.y, a.z - b.z))
        .collect();
    NurbsSurface::new(
        surface_a.degree_u(),
        surface_a.degree_v(),
        surface_a.knots(Dir::U).as_slice().to_vec(),
        surface_a.knots(Dir::V).as_slice().to_vec(),
        points,
        surface_a.weights().map(<[f64]>::to_vec),
    )
}

/// Positive shared weights make the rational scalar difference a convex
/// combination of its control differences. Outward subtraction prevents a
/// rounded derived coefficient from becoming miss evidence.
fn original_control_difference_proves_empty(
    surface_a: &NurbsSurface,
    surface_b: &NurbsSurface,
) -> bool {
    let mut lower = f64::INFINITY;
    let mut upper = f64::NEG_INFINITY;
    for (a, b) in surface_a.points().iter().zip(surface_b.points()) {
        let difference = Interval::point(a.z) - Interval::point(b.z);
        lower = lower.min(difference.lo());
        upper = upper.max(difference.hi());
    }
    lower > 0.0 || upper < 0.0
}

fn paired_branch_kind(
    surface_a: &NurbsSurface,
    surface_b: &NurbsSurface,
    points: &[MarchPoint],
    tolerances: Tolerances,
) -> ContactKind {
    let uv = points[points.len() / 2].surface_uv;
    let (Some(normal_a), Some(normal_b)) = (surface_a.normal(uv), surface_b.normal(uv)) else {
        return ContactKind::Singular;
    };
    if normal_a.cross(normal_b).norm() <= tolerances.angular() {
        ContactKind::Tangent
    } else {
        ContactKind::Transverse
    }
}

fn lift_output_to_original_sources(
    mut output: MarchOutput,
    surface_a: &NurbsSurface,
    surface_b: &NurbsSurface,
) -> core::result::Result<MarchOutput, ContextMarchError> {
    if output.result.curves.len() != output.traces.len() {
        return Err(ContextMarchError::Kernel(Error::InvalidGeometry {
            reason: "NURBS/NURBS march trace count does not match discovered branches",
        }));
    }
    for (branch, trace) in output.result.curves.iter_mut().zip(&mut output.traces) {
        let pcurve = simplify_axis_aligned_pcurve(&trace.surface_pcurve)?;
        let controls = pcurve
            .points()
            .iter()
            .map(|uv| {
                let a = surface_a.eval([uv.x, uv.y]);
                let b = surface_b.eval([uv.x, uv.y]);
                (a + b) * 0.5
            })
            .collect();
        let carrier = NurbsCurve::new(1, pcurve.knots().as_slice().to_vec(), controls, None)?;
        let start = pcurve.points()[0];
        let end = pcurve.points()[pcurve.points().len() - 1];
        branch.curve = SurfaceIntersectionCurve::Nurbs(carrier.clone());
        branch.curve_range = carrier.param_range();
        branch.uv_a_start = [start.x, start.y];
        branch.uv_a_end = [end.x, end.y];
        branch.uv_b_start = [start.x, start.y];
        branch.uv_b_end = [end.x, end.y];
        trace.carrier = carrier;
        trace.other_pcurve = pcurve.clone();
        trace.surface_pcurve = pcurve;
    }
    Ok(output)
}

/// Collapse an exactly axis-aligned monotone discovery polyline to its two
/// endpoints. This is candidate conditioning only; the original-source
/// interval certificate remains authoritative for the resulting chord.
fn simplify_axis_aligned_pcurve(pcurve: &NurbsCurve2d) -> Result<NurbsCurve2d> {
    let points = pcurve.points();
    let first = points[0];
    let last = points[points.len() - 1];
    let between =
        |value: f64, first: f64, last: f64| value >= first.min(last) && value <= first.max(last);
    let constant_u = points
        .iter()
        .all(|point| point.x == first.x && between(point.y, first.y, last.y));
    let constant_v = points
        .iter()
        .all(|point| point.y == first.y && between(point.x, first.x, last.x));
    if points.len() > 2 && (constant_u || constant_v) {
        NurbsCurve2d::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![first, last], None)
    } else {
        Ok(pcurve.clone())
    }
}

fn validate_context_budget(
    context: &OperationContext<'_>,
) -> core::result::Result<(), OperationPolicyError> {
    let budget = context.effective_budget();
    for (stage, resource, mode) in [
        (
            NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        ),
        (
            NURBS_IMPLICIT_ISOLATION_CANDIDATES,
            ResourceKind::Items,
            AccountingMode::HighWater,
        ),
        (
            NURBS_IMPLICIT_ISOLATION_DEPTH,
            ResourceKind::Depth,
            AccountingMode::HighWater,
        ),
        (
            super::NURBS_SURFACE_MARCH_SAMPLES,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        ),
    ] {
        let Some(limit) = budget
            .limits()
            .iter()
            .find(|limit| limit.stage == stage && limit.resource == resource)
        else {
            return Err(OperationPolicyError::UnknownLimit { stage, resource });
        };
        if limit.mode != mode {
            return Err(OperationPolicyError::AccountingModeMismatch { stage, resource });
        }
    }
    Ok(())
}
