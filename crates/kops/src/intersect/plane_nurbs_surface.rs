use super::nurbs_surface_march::{
    ContextMarchError, MarchConfig, MarchOutput, MarchPoint, NURBS_SURFACE_MARCH_SAMPLE_LIMIT,
    NurbsSurfaceMarchBudgetProfile, march_nurbs_surface_intersection_with_traces_in_scope,
};
use super::result::{ContactKind, SurfaceSurfaceIntersections};
use kcore::error::{Error, Result};
use kcore::operation::{
    AccountingMode, DiagnosticKind, ExecutionPolicy, NumericalPolicy, OperationContext,
    OperationOutcome, OperationPolicyError, OperationScope, PolicyVersion, ResourceKind,
    SessionPolicy, SessionPrecision,
};
use kcore::tolerance::Tolerances;
use kgeom::nurbs::NurbsSurface;
use kgeom::nurbs::{
    NURBS_IMPLICIT_ISOLATION_CANDIDATES, NURBS_IMPLICIT_ISOLATION_DEPTH,
    NURBS_IMPLICIT_ISOLATION_SUBDIVISIONS,
};
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Surface};
use kgeom::vec::Point3;

/// Intersect a finite plane window with a clamped NURBS surface over a finite
/// parameter window.
///
/// This is a fixed-grid marching bridge for the broader SSI path: it samples
/// the plane's signed-distance field over the NURBS parameter rectangle and
/// returns joined zero-contour branches as degree-1 NURBS polylines.
pub fn intersect_bounded_plane_nurbs_surface(
    plane: &Plane,
    plane_range: [ParamRange; 2],
    surface: &NurbsSurface,
    surface_range: [ParamRange; 2],
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
    intersect_bounded_plane_nurbs_surface_with_context(
        plane,
        plane_range,
        surface,
        surface_range,
        &context,
    )
    .expect("built-in v1 plane/NURBS policy is valid")
    .into_result()
}

/// Context-aware plane/NURBS-surface intersection with deterministic work
/// accounting and a retained operation report.
///
/// The context's effective budget must contain the proof subdivision,
/// candidate, and depth stages plus [`super::NURBS_SURFACE_MARCH_SAMPLES`],
/// normally via [`super::NurbsSurfaceMarchBudgetProfile::v1_defaults`].
/// Configuration errors are returned separately because they are not
/// geometric failures.
pub fn intersect_bounded_plane_nurbs_surface_with_context(
    plane: &Plane,
    plane_range: [ParamRange; 2],
    surface: &NurbsSurface,
    surface_range: [ParamRange; 2],
    context: &OperationContext<'_>,
) -> core::result::Result<OperationOutcome<SurfaceSurfaceIntersections>, OperationPolicyError> {
    validate_context_budget(context)?;
    let mut scope = OperationScope::new(context);
    let result = intersect_bounded_plane_nurbs_surface_impl(
        plane,
        plane_range,
        surface,
        surface_range,
        context.tolerances(),
        &mut scope,
    );
    match result {
        Ok(result) => Ok(scope.finish(Ok(result))),
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

fn intersect_bounded_plane_nurbs_surface_impl(
    plane: &Plane,
    plane_range: [ParamRange; 2],
    surface: &NurbsSurface,
    surface_range: [ParamRange; 2],
    tolerances: Tolerances,
    scope: &mut OperationScope<'_, '_>,
) -> core::result::Result<SurfaceSurfaceIntersections, ContextMarchError> {
    intersect_bounded_plane_nurbs_surface_with_traces_in_scope(
        plane,
        plane_range,
        surface,
        surface_range,
        tolerances,
        scope,
    )
    .map(|output| output.result)
}

pub(super) fn intersect_bounded_plane_nurbs_surface_with_traces_in_scope(
    plane: &Plane,
    plane_range: [ParamRange; 2],
    surface: &NurbsSurface,
    surface_range: [ParamRange; 2],
    tolerances: Tolerances,
    scope: &mut OperationScope<'_, '_>,
) -> core::result::Result<MarchOutput, ContextMarchError> {
    validate_plane_range(plane_range)?;

    let signed_distance = |point| plane.frame().to_local(point).z;
    let other_uv = |point| plane_uv_at(point, plane, plane_range, tolerances);
    let branch_kind = |points: &[MarchPoint]| plane_branch_kind(surface, plane, points, tolerances);
    let config = MarchConfig {
        surface,
        surface_range,
        tolerances,
        implicit_surface: plane,
        signed_distance: &signed_distance,
        other_uv: &other_uv,
        branch_kind: &branch_kind,
        overlap_reason: "coincident plane/nurbs-surface intersection is a surface overlap",
        non_finite_reason: "plane/nurbs-surface intersection sampled non-finite geometry",
        finite_range_reason: "plane/nurbs-surface intersection requires finite non-reversed NURBS surface ranges",
        clamped_surface_reason: "plane/nurbs-surface intersection requires a clamped NURBS surface",
        domain_range_reason: "plane/nurbs-surface intersection surface range must lie within the NURBS domain",
    };
    march_nurbs_surface_intersection_with_traces_in_scope(config, scope)
}

fn plane_branch_kind(
    surface: &NurbsSurface,
    plane: &Plane,
    points: &[MarchPoint],
    tolerances: Tolerances,
) -> ContactKind {
    let mid = points[points.len() / 2].surface_uv;
    let Some(surface_normal) = surface.normal(mid) else {
        return ContactKind::Singular;
    };
    if surface_normal.cross(plane.frame().z()).norm() <= tolerances.angular() {
        ContactKind::Tangent
    } else {
        ContactKind::Transverse
    }
}

fn plane_uv_at(
    point: Point3,
    plane: &Plane,
    range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let local = plane.frame().to_local(point);
    Some([
        fit_scalar_parameter(local.x, range[0], tolerances.linear())?,
        fit_scalar_parameter(local.y, range[1], tolerances.linear())?,
    ])
}

fn fit_scalar_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
}

fn validate_plane_range(plane_range: [ParamRange; 2]) -> Result<()> {
    if plane_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "plane/nurbs-surface intersection requires finite non-reversed plane ranges",
        });
    }
    Ok(())
}
