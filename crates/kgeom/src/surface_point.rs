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
use crate::project::project_to_surface;
use crate::surface::{Cone, Cylinder, Plane, Sphere, Surface, Torus};
use crate::vec::Point3;
use kcore::math;

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
    let analytic = if let Some(surface) = surface.as_any().downcast_ref::<Plane>() {
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
    };

    if let Some(uv) = analytic {
        return Ok(SurfacePointUv {
            uv,
            method: SurfacePointMethod::Analytic,
        });
    }

    let window = surface.param_range();
    if !window[0].is_finite() || !window[1].is_finite() {
        return Err(SurfacePointError::UnboundedProjectionWindow);
    }
    let projection =
        project_to_surface(surface, point, window).ok_or(SurfacePointError::ProjectionFailed)?;
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
    let analytic = if let Some(surface) = surface.as_any().downcast_ref::<Plane>() {
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
    };

    if let Some(distance) = analytic {
        return Ok(SurfacePointDistance {
            distance,
            method: SurfacePointMethod::Analytic,
        });
    }

    let window = surface.param_range();
    if !window[0].is_finite() || !window[1].is_finite() {
        return Err(SurfacePointError::UnboundedProjectionWindow);
    }
    let projection =
        project_to_surface(surface, point, window).ok_or(SurfacePointError::ProjectionFailed)?;
    Ok(SurfacePointDistance {
        distance: projection.dist,
        method: SurfacePointMethod::Projected,
    })
}
