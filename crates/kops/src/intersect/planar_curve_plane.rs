use super::conic::{
    ExactTrigLinearKind, HARMONIC_ROOT_CLASSIFICATION_REASON, HarmonicCutAccumulator,
    TrigLinearSolution, exact_trig_linear_kind, strict_cut_midpoint, trig_linear_roots,
    trig_linear_solution,
};
use super::parameter::fit_parameter_pair;
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint,
    accept_curve_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::predicates::{Orientation, affine_dot3};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::Point3;

/// Intersect a circle restricted to a finite range with a finite plane
/// parameter window.
pub fn intersect_bounded_circle_plane(
    circle: &Circle,
    circle_range: ParamRange,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    validate_ranges(circle_range, circle.radius(), plane_range, tolerances)?;
    let conic = PlanarConic {
        curve: circle,
        frame: circle.frame(),
        radius_x: circle.radius(),
        radius_y: circle.radius(),
    };
    intersect_planar_conic_plane(conic, circle_range, plane, plane_range, tolerances)
}

/// Clip a circle that its caller has already constructed in `plane`.
///
/// This internal seam preserves that construction proof instead of requiring a
/// re-normalized derived frame to reproduce bit-identical plane normals.
pub(super) fn clip_bounded_circle_on_plane(
    circle: &Circle,
    circle_range: ParamRange,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    validate_ranges(circle_range, circle.radius(), plane_range, tolerances)?;
    contained_planar_conic(
        PlanarConic {
            curve: circle,
            frame: circle.frame(),
            radius_x: circle.radius(),
            radius_y: circle.radius(),
        },
        circle_range,
        plane,
        plane_range,
        tolerances,
    )
}

/// Intersect an ellipse restricted to a finite range with a finite plane
/// parameter window.
pub fn intersect_bounded_ellipse_plane(
    ellipse: &Ellipse,
    ellipse_range: ParamRange,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    validate_ranges(
        ellipse_range,
        ellipse.minor_radius(),
        plane_range,
        tolerances,
    )?;
    let conic = PlanarConic {
        curve: ellipse,
        frame: ellipse.frame(),
        radius_x: ellipse.major_radius(),
        radius_y: ellipse.minor_radius(),
    };
    intersect_planar_conic_plane(conic, ellipse_range, plane, plane_range, tolerances)
}

#[derive(Clone, Copy)]
struct PlanarConic<'a> {
    curve: &'a dyn Curve,
    frame: &'a Frame,
    radius_x: f64,
    radius_y: f64,
}

fn intersect_planar_conic_plane(
    conic: PlanarConic<'_>,
    curve_range: ParamRange,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    let normal = plane.frame().z();
    let offset = conic.frame.origin() - plane.frame().origin();
    let c = offset.dot(normal);
    let a = conic.frame.x().dot(normal) * conic.radius_x;
    let b = conic.frame.y().dot(normal) * conic.radius_y;
    let Some(exact_kind) = exact_planar_harmonic_kind(conic, plane) else {
        return Ok(CurveSurfaceIntersections::indeterminate_empty(
            HARMONIC_ROOT_CLASSIFICATION_REASON,
        ));
    };
    match exact_kind {
        ExactTrigLinearKind::Identity => {
            return contained_planar_conic(conic, curve_range, plane, plane_range, tolerances);
        }
        ExactTrigLinearKind::ConstantNonzero => {
            return Ok(CurveSurfaceIntersections::complete_empty());
        }
        ExactTrigLinearKind::General => {}
    }
    let Some(solution) = trig_linear_solution(a, b, c, curve_range, tolerances.linear()) else {
        return Ok(CurveSurfaceIntersections::indeterminate_empty(
            HARMONIC_ROOT_CLASSIFICATION_REASON,
        ));
    };
    let roots = match solution {
        TrigLinearSolution::Identity => {
            return Ok(CurveSurfaceIntersections::indeterminate_empty(
                HARMONIC_ROOT_CLASSIFICATION_REASON,
            ));
        }
        TrigLinearSolution::Roots(roots) => roots,
    };
    let mut points = Vec::new();
    for (t_curve, tangent) in roots {
        let Some(uv) = fit_parameter_pair(
            plane_uv(conic.curve.eval(t_curve), plane),
            plane_range,
            tolerances.linear(),
        ) else {
            continue;
        };
        if let Some(point) = accept_curve_surface_candidate(
            conic.curve,
            t_curve,
            plane,
            uv,
            if tangent {
                ContactKind::Tangent
            } else {
                ContactKind::Transverse
            },
            tolerances,
        ) {
            push_distinct(&mut points, point, tolerances);
        }
    }

    CurveSurfaceIntersections::canonicalized_complete(points, Vec::new())
}

fn exact_planar_harmonic_kind(
    conic: PlanarConic<'_>,
    plane: &Plane,
) -> Option<ExactTrigLinearKind> {
    let normal = plane.frame().z().to_array();
    let conic_normal = conic.frame.z();
    let plane_normal = plane.frame().z();
    let frame_normals_match = plane_normal == conic_normal || plane_normal == -conic_normal;
    let (cosine, sine) = if frame_normals_match {
        // Frame's public contract is orthonormal even when normalization leaves
        // the stored dyadic components with a tiny nonzero dot product.
        (Orientation::Zero, Orientation::Zero)
    } else {
        let zero = [0.0; 3];
        (
            affine_dot3(normal, conic.frame.x().to_array(), zero, 0.0)?.sign(),
            affine_dot3(normal, conic.frame.y().to_array(), zero, 0.0)?.sign(),
        )
    };
    let constant = affine_dot3(
        normal,
        conic.frame.origin().to_array(),
        plane.frame().origin().to_array(),
        0.0,
    )?
    .sign();
    Some(exact_trig_linear_kind(cosine, sine, constant))
}

fn contained_planar_conic(
    conic: PlanarConic<'_>,
    curve_range: ParamRange,
    plane: &Plane,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    if curve_range.width() == 0.0 {
        let t_curve = curve_range.lo;
        let Some(uv) = fit_parameter_pair(
            plane_uv(conic.curve.eval(t_curve), plane),
            plane_range,
            tolerances.linear(),
        ) else {
            return Ok(CurveSurfaceIntersections::complete_empty());
        };
        let points = accept_curve_surface_candidate(
            conic.curve,
            t_curve,
            plane,
            uv,
            ContactKind::Tangent,
            tolerances,
        )
        .into_iter()
        .collect::<Vec<CurveSurfacePoint>>();
        return CurveSurfaceIntersections::canonicalized_complete(points, Vec::new());
    }

    let mut cuts = HarmonicCutAccumulator::new(curve_range);
    for (axis, axis_range) in plane_range.iter().enumerate() {
        let (c0, a, b) = plane_axis_coefficients(conic, plane, axis);
        for bound in [axis_range.lo, axis_range.hi] {
            let Some(roots) = trig_linear_roots(a, b, c0 - bound, curve_range, tolerances.linear())
            else {
                return Ok(CurveSurfaceIntersections::indeterminate_empty(
                    HARMONIC_ROOT_CLASSIFICATION_REASON,
                ));
            };
            if cuts
                .push_harmonic_roots([a, b, c0 - bound], &roots, |_| true)
                .is_none()
            {
                return Ok(CurveSurfaceIntersections::indeterminate_empty(
                    HARMONIC_ROOT_CLASSIFICATION_REASON,
                ));
            }
        }
    }
    let cuts = cuts.into_parameters();

    let mut points = Vec::new();
    let mut overlaps = Vec::new();
    for interval in cuts.windows(2) {
        let lo = interval[0];
        let hi = interval[1];
        let Some(mid) = strict_cut_midpoint(lo, hi) else {
            return Ok(CurveSurfaceIntersections::indeterminate_empty(
                HARMONIC_ROOT_CLASSIFICATION_REASON,
            ));
        };
        if fit_parameter_pair(
            plane_uv(conic.curve.eval(mid), plane),
            plane_range,
            tolerances.linear(),
        )
        .is_none()
        {
            continue;
        }
        let Some(uv_start) = fit_parameter_pair(
            plane_uv(conic.curve.eval(lo), plane),
            plane_range,
            tolerances.linear(),
        ) else {
            continue;
        };
        let Some(uv_end) = fit_parameter_pair(
            plane_uv(conic.curve.eval(hi), plane),
            plane_range,
            tolerances.linear(),
        ) else {
            continue;
        };
        overlaps.push(CurveSurfaceOverlap {
            curve: ParamRange::new(lo, hi),
            uv_start,
            uv_end,
        });
    }

    for &cut in &cuts {
        let cut_point = conic.curve.eval(cut);
        if overlaps.iter().any(|overlap| {
            (cut >= overlap.curve.lo - tolerances.linear()
                && cut <= overlap.curve.hi + tolerances.linear())
                || cut_point.dist(conic.curve.eval(overlap.curve.lo)) <= tolerances.linear()
                || cut_point.dist(conic.curve.eval(overlap.curve.hi)) <= tolerances.linear()
        }) {
            continue;
        }
        let Some(uv) =
            fit_parameter_pair(plane_uv(cut_point, plane), plane_range, tolerances.linear())
        else {
            continue;
        };
        if let Some(point) = accept_curve_surface_candidate(
            conic.curve,
            cut,
            plane,
            uv,
            ContactKind::Tangent,
            tolerances,
        ) {
            push_distinct(&mut points, point, tolerances);
        }
    }

    CurveSurfaceIntersections::canonicalized_complete(points, overlaps)
}

fn plane_axis_coefficients(conic: PlanarConic<'_>, plane: &Plane, axis: usize) -> (f64, f64, f64) {
    let plane_axis = if axis == 0 {
        plane.frame().x()
    } else {
        plane.frame().y()
    };
    let offset = conic.frame.origin() - plane.frame().origin();
    (
        offset.dot(plane_axis),
        conic.frame.x().dot(plane_axis) * conic.radius_x,
        conic.frame.y().dot(plane_axis) * conic.radius_y,
    )
}

fn plane_uv(point: Point3, plane: &Plane) -> [f64; 2] {
    let local = plane.frame().to_local(point);
    [local.x, local.y]
}

fn push_distinct(
    points: &mut Vec<CurveSurfacePoint>,
    candidate: CurveSurfacePoint,
    tolerances: Tolerances,
) {
    if !points
        .iter()
        .any(|point| point.point.dist(candidate.point) <= tolerances.linear())
    {
        points.push(candidate);
    }
}

fn parameter_tolerance(radius: f64, tolerances: Tolerances) -> f64 {
    (tolerances.linear() / radius).max(tolerances.angular())
}

fn validate_ranges(
    curve_range: ParamRange,
    curve_radius: f64,
    plane_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<()> {
    if !curve_range.is_finite() || curve_range.width() < 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "curve/plane intersection requires a finite non-reversed curve range",
        });
    }
    if curve_range.width() > core::f64::consts::TAU + parameter_tolerance(curve_radius, tolerances)
    {
        return Err(Error::InvalidGeometry {
            reason: "bounded curve range cannot span more than one period",
        });
    }
    if plane_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "curve/plane intersection requires finite non-reversed surface ranges",
        });
    }
    Ok(())
}
