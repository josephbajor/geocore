//! Point-to-surface parameter mapping and distance queries.
//!
//! Analytic surface classes use their closed-form parameterizations and
//! distance fields. Other finite-domain surfaces, including NURBS, use the
//! deterministic closest-point projector. Analytic inversion deliberately
//! returns its raw trigonometric branch; callers that need the surface's
//! canonical periodic branch can apply [`normalize_surface_uv`]. Keeping
//! those operations separate lets algorithms unwrap a continuous chart
//! without first introducing a seam discontinuity.

use crate::param::wrap_periodic;
use crate::project::{
    ProjectionError, compose_surface_projection_context, project_to_surface_in_scope,
};
use crate::surface::{Cone, Cylinder, Plane, Sphere, Surface, Torus};
use crate::vec::Point3;
use kcore::error::{CapabilityId, ClassifiedError, ErrorClass, ErrorCode};
use kcore::math;
use kcore::operation::{OperationContext, OperationOutcome, OperationPolicyError, OperationScope};

/// How a point-to-surface query obtained its result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfacePointMethod {
    /// A closed-form expression for the concrete analytic surface class.
    Analytic,
    /// Deterministic closest-point projection over the natural finite domain.
    Projected,
}

/// Parameters recovered for a model-space point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfacePointUv {
    /// Surface parameters. Analytic periodic parameters retain the raw
    /// `atan2` branch; use [`normalize_surface_uv`] for the base chart.
    pub uv: [f64; 2],
    /// Whether the parameters were closed-form or numerically projected.
    pub method: SurfacePointMethod,
}

/// Distance from a model-space point to a surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfacePointDistance {
    /// Non-negative Euclidean distance to the surface.
    pub distance: f64,
    /// Whether the distance was closed-form or numerically projected.
    pub method: SurfacePointMethod,
}

/// Why a point-to-surface query could not produce a result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum SurfacePointError {
    /// Numerical projection needs a finite search window, but at least one
    /// natural parameter direction is unbounded.
    UnboundedProjectionWindow,
    /// Deterministic closest-point projection did not produce a candidate.
    ProjectionFailed,
}

/// Why a contextual point-to-surface query could not produce a result.
///
/// The compatibility API intentionally collapses every projector failure to
/// [`SurfacePointError::ProjectionFailed`]. Contextual callers retain the
/// complete projection failure, including policy limit snapshots.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum SurfacePointContextError {
    /// Numerical projection needs a finite search window, but at least one
    /// natural parameter direction is unbounded.
    UnboundedProjectionWindow,
    /// Deterministic closest-point projection failed.
    Projection(ProjectionError),
}

const fn known_error_code(value: &'static str) -> ErrorCode {
    match ErrorCode::new(value) {
        Ok(code) => code,
        Err(_) => panic!("invalid built-in surface-point error code"),
    }
}

const fn known_capability(value: &'static str) -> CapabilityId {
    match CapabilityId::new(value) {
        Ok(capability) => capability,
        Err(_) => panic!("invalid built-in surface-point capability identifier"),
    }
}

/// Stable machine-readable identities owned by [`SurfacePointContextError`].
pub mod error_code {
    use super::{ErrorCode, known_error_code};

    /// The fallback surface has no finite natural projection window.
    pub const UNBOUNDED_PROJECTION_WINDOW: ErrorCode =
        known_error_code("kgeom.surface-point.unbounded-projection-window");
}

/// Stable finite support-matrix features owned by surface-point queries.
pub mod capability {
    use super::{CapabilityId, known_capability};

    /// A finite natural window for numerical closest-point projection.
    pub const FINITE_PROJECTION_WINDOW: CapabilityId =
        known_capability("kgeom.surface-point.finite-projection-window");
}

impl SurfacePointContextError {
    /// Returns the broad semantic class of this failure.
    pub fn class(&self) -> ErrorClass {
        match self {
            Self::UnboundedProjectionWindow => ErrorClass::Unsupported,
            Self::Projection(error) => error.class(),
        }
    }

    /// Returns the stable machine-readable identity of this failure.
    pub fn code(&self) -> ErrorCode {
        match self {
            Self::UnboundedProjectionWindow => error_code::UNBOUNDED_PROJECTION_WINDOW,
            Self::Projection(error) => error.code(),
        }
    }

    /// Returns the unsupported capability supplied by a nested failure.
    pub fn capability(&self) -> Option<CapabilityId> {
        match self {
            Self::UnboundedProjectionWindow => Some(capability::FINITE_PROJECTION_WINDOW),
            Self::Projection(error) => error.capability(),
        }
    }

    /// Returns structured deterministic-limit data from projection policy.
    pub fn limit(&self) -> Option<kcore::operation::LimitSnapshot> {
        match self {
            Self::UnboundedProjectionWindow => None,
            Self::Projection(error) => error.limit(),
        }
    }
}

impl core::fmt::Display for SurfacePointContextError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::UnboundedProjectionWindow => {
                formatter.write_str("surface has no finite natural projection window")
            }
            Self::Projection(error) => {
                write!(formatter, "surface-point projection failed: {error}")
            }
        }
    }
}

impl std::error::Error for SurfacePointContextError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::UnboundedProjectionWindow => None,
            Self::Projection(error) => Some(error),
        }
    }
}

impl ClassifiedError for SurfacePointContextError {
    fn class(&self) -> ErrorClass {
        self.class()
    }

    fn code(&self) -> ErrorCode {
        self.code()
    }

    fn capability(&self) -> Option<CapabilityId> {
        self.capability()
    }

    fn limit(&self) -> Option<kcore::operation::LimitSnapshot> {
        self.limit()
    }
}

impl From<ProjectionError> for SurfacePointContextError {
    fn from(error: ProjectionError) -> Self {
        Self::Projection(error)
    }
}

fn analytic_surface_uv(surface: &dyn Surface, point: Point3) -> Option<[f64; 2]> {
    if let Some(surface) = surface.as_any().downcast_ref::<Plane>() {
        let local = surface.frame().to_local(point);
        Some([local.x, local.y])
    } else if let Some(surface) = surface.as_any().downcast_ref::<Cylinder>() {
        let local = surface.frame().to_local(point);
        Some([math::atan2(local.y, local.x), local.z])
    } else if let Some(surface) = surface.as_any().downcast_ref::<Cone>() {
        let local = surface.frame().to_local(point);
        Some([
            math::atan2(local.y, local.x),
            local.z / math::cos(surface.half_angle()),
        ])
    } else if let Some(surface) = surface.as_any().downcast_ref::<Sphere>() {
        let local = surface.frame().to_local(point);
        Some([
            math::atan2(local.y, local.x),
            math::atan2(local.z, (local.x * local.x + local.y * local.y).sqrt()),
        ])
    } else if let Some(surface) = surface.as_any().downcast_ref::<Torus>() {
        let local = surface.frame().to_local(point);
        let rho = (local.x * local.x + local.y * local.y).sqrt();
        Some([
            math::atan2(local.y, local.x),
            math::atan2(local.z, rho - surface.major_radius()),
        ])
    } else {
        None
    }
}

fn analytic_surface_distance(surface: &dyn Surface, point: Point3) -> Option<f64> {
    if let Some(surface) = surface.as_any().downcast_ref::<Plane>() {
        Some(surface.frame().to_local(point).z.abs())
    } else if let Some(surface) = surface.as_any().downcast_ref::<Cylinder>() {
        let local = surface.frame().to_local(point);
        Some(((local.x * local.x + local.y * local.y).sqrt() - surface.radius()).abs())
    } else if let Some(surface) = surface.as_any().downcast_ref::<Cone>() {
        // In the (rho, z) half-plane the cone is the line through (r, 0)
        // with unit direction (sin alpha, cos alpha).
        let local = surface.frame().to_local(point);
        let rho = (local.x * local.x + local.y * local.y).sqrt();
        let (sin_alpha, cos_alpha) = math::sincos(surface.half_angle());
        Some(((rho - surface.radius()) * cos_alpha - local.z * sin_alpha).abs())
    } else if let Some(surface) = surface.as_any().downcast_ref::<Sphere>() {
        Some((surface.frame().to_local(point).norm() - surface.radius()).abs())
    } else if let Some(surface) = surface.as_any().downcast_ref::<Torus>() {
        let local = surface.frame().to_local(point);
        let ring = (local.x * local.x + local.y * local.y).sqrt() - surface.major_radius();
        Some(((ring * ring + local.z * local.z).sqrt() - surface.minor_radius()).abs())
    } else {
        None
    }
}

fn validate_contextual_point(point: Point3) -> core::result::Result<(), SurfacePointContextError> {
    if point.x.is_finite() && point.y.is_finite() && point.z.is_finite() {
        Ok(())
    } else {
        Err(ProjectionError::InvalidQueryPoint.into())
    }
}

/// Recover parameters for a point associated with `surface`.
///
/// Plane, cylinder, cone, sphere, and torus mappings are closed-form. Their
/// periodic angles retain the raw `atan2` branch so continuity-sensitive
/// callers can unwrap them. Other surface classes are projected over their
/// natural finite parameter ranges; this is the path used by NURBS.
pub fn invert_surface_point(
    surface: &dyn Surface,
    point: Point3,
) -> Result<SurfacePointUv, SurfacePointError> {
    if let Some(uv) = analytic_surface_uv(surface, point) {
        return Ok(SurfacePointUv {
            uv,
            method: SurfacePointMethod::Analytic,
        });
    }

    let window = surface.param_range();
    if !window[0].is_finite() || !window[1].is_finite() {
        return Err(SurfacePointError::UnboundedProjectionWindow);
    }
    let projection = crate::project::project_to_surface(surface, point, window)
        .ok_or(SurfacePointError::ProjectionFailed)?;
    Ok(SurfacePointUv {
        uv: projection.uv,
        method: SurfacePointMethod::Projected,
    })
}

/// Recover surface parameters with deterministic projection accounting.
///
/// Surface-projection family defaults are composed below matching session
/// entries and explicit request overrides. Analytic classes return without
/// consuming projection work.
pub fn invert_surface_point_with_context(
    surface: &dyn Surface,
    point: Point3,
    context: &OperationContext<'_>,
) -> core::result::Result<
    OperationOutcome<SurfacePointUv, SurfacePointContextError>,
    OperationPolicyError,
> {
    if analytic_surface_uv(surface, point).is_some() {
        let mut scope = OperationScope::new(context);
        let result = invert_surface_point_in_scope(surface, point, &mut scope);
        return Ok(scope.finish_typed(result));
    }
    let context = compose_surface_projection_context(context)?;
    let mut scope = OperationScope::new(&context);
    let result = invert_surface_point_in_scope(surface, point, &mut scope);
    Ok(scope.finish_typed(result))
}

/// Recover surface parameters using the caller's existing operation scope.
///
/// Analytic classes do not inspect or consume projection limits. Fallback
/// classes validate and charge the active ledger through the contextual
/// closest-point projector.
pub fn invert_surface_point_in_scope(
    surface: &dyn Surface,
    point: Point3,
    scope: &mut OperationScope<'_, '_>,
) -> core::result::Result<SurfacePointUv, SurfacePointContextError> {
    validate_contextual_point(point)?;
    if let Some(uv) = analytic_surface_uv(surface, point) {
        return Ok(SurfacePointUv {
            uv,
            method: SurfacePointMethod::Analytic,
        });
    }

    let window = surface.param_range();
    if !window[0].is_finite() || !window[1].is_finite() {
        return Err(SurfacePointContextError::UnboundedProjectionWindow);
    }
    let projection = project_to_surface_in_scope(surface, point, window, scope)?;
    Ok(SurfacePointUv {
        uv: projection.uv,
        method: SurfacePointMethod::Projected,
    })
}

/// Wrap periodic components of `uv` into the surface's natural base chart.
///
/// Non-periodic components are returned unchanged. This operation is kept
/// separate from [`invert_surface_point`] because loop algorithms often need
/// the raw branch for continuity-preserving unwrapping.
pub fn normalize_surface_uv(surface: &dyn Surface, mut uv: [f64; 2]) -> [f64; 2] {
    let base = surface.param_range();
    for (direction, period) in surface.periodicity().into_iter().enumerate() {
        if let Some(period) = period {
            uv[direction] = wrap_periodic(uv[direction], base[direction].lo, period);
        }
    }
    uv
}

/// Compute the distance from `point` to `surface`.
///
/// Current analytic classes use exact distance formulas. Other surface
/// classes, including NURBS, use deterministic projection over the natural
/// finite parameter ranges.
pub fn distance_to_surface(
    surface: &dyn Surface,
    point: Point3,
) -> Result<SurfacePointDistance, SurfacePointError> {
    if let Some(distance) = analytic_surface_distance(surface, point) {
        return Ok(SurfacePointDistance {
            distance,
            method: SurfacePointMethod::Analytic,
        });
    }

    let window = surface.param_range();
    if !window[0].is_finite() || !window[1].is_finite() {
        return Err(SurfacePointError::UnboundedProjectionWindow);
    }
    let projection = crate::project::project_to_surface(surface, point, window)
        .ok_or(SurfacePointError::ProjectionFailed)?;
    Ok(SurfacePointDistance {
        distance: projection.dist,
        method: SurfacePointMethod::Projected,
    })
}

/// Compute point-to-surface distance with deterministic projection accounting.
///
/// Surface-projection family defaults are composed below matching session
/// entries and explicit request overrides. Analytic classes return without
/// consuming projection work.
pub fn distance_to_surface_with_context(
    surface: &dyn Surface,
    point: Point3,
    context: &OperationContext<'_>,
) -> core::result::Result<
    OperationOutcome<SurfacePointDistance, SurfacePointContextError>,
    OperationPolicyError,
> {
    if analytic_surface_distance(surface, point).is_some() {
        let mut scope = OperationScope::new(context);
        let result = distance_to_surface_in_scope(surface, point, &mut scope);
        return Ok(scope.finish_typed(result));
    }
    let context = compose_surface_projection_context(context)?;
    let mut scope = OperationScope::new(&context);
    let result = distance_to_surface_in_scope(surface, point, &mut scope);
    Ok(scope.finish_typed(result))
}

/// Compute point-to-surface distance using the caller's operation scope.
pub fn distance_to_surface_in_scope(
    surface: &dyn Surface,
    point: Point3,
    scope: &mut OperationScope<'_, '_>,
) -> core::result::Result<SurfacePointDistance, SurfacePointContextError> {
    validate_contextual_point(point)?;
    if let Some(distance) = analytic_surface_distance(surface, point) {
        return Ok(SurfacePointDistance {
            distance,
            method: SurfacePointMethod::Analytic,
        });
    }

    let window = surface.param_range();
    if !window[0].is_finite() || !window[1].is_finite() {
        return Err(SurfacePointContextError::UnboundedProjectionWindow);
    }
    let projection = project_to_surface_in_scope(surface, point, window, scope)?;
    Ok(SurfacePointDistance {
        distance: projection.dist,
        method: SurfacePointMethod::Projected,
    })
}
