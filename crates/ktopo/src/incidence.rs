//! Shared validation for edge/pcurve/surface incidence.
//!
//! Both checked topology edits and the body checker use this implementation
//! so an operation cannot accept a pcurve that the checker later rejects.
//! Fast validation uses deterministic samples; Full validation promotes only
//! conservative whole-interval analytic residual bounds to certificates.

use crate::entity::{
    CurveId, Edge, EdgeId, FaceDomain, FinPcurve, PcurveEndpointKind, SeamSide, SurfaceId,
    SurfaceParameter,
};
use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use crate::store::Store;
use kcore::error::Result as KernelResult;
use kcore::math;
use kgeom::curve::Curve;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Dir, Surface};
use kgeom::vec::{Point2, Point3, Vec2, Vec3};
use kgraph::{EvalLimits, SurfaceDerivativeOrder};

const INCIDENCE_SAMPLES: usize = 5;

/// Classification used by topology operations and checker diagnostics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PcurveIssue {
    StaleReference,
    BadRange,
    BadChart,
    BadClosure,
    BadSingularity,
    BadSeam,
    OffSurface,
}

/// Whether a whole-interval incidence proof is currently available.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum IncidenceCertification {
    /// A conservative residual bound is within the requested tolerance.
    Certified,
    /// No violation is asserted, but this representation pair is not yet
    /// covered by a whole-interval proof.
    Indeterminate,
}

#[derive(Debug, Clone, Copy)]
enum AnalyticTrace {
    /// `origin + direction * t`.
    Affine { origin: Point3, direction: Vec3 },
    /// `center + cosine * cos(t) + sine * sin(t)`.
    Harmonic {
        center: Point3,
        cosine: Vec3,
        sine: Vec3,
    },
}

fn edge_range(bounds: Option<(f64, f64)>) -> core::result::Result<ParamRange, PcurveIssue> {
    match bounds {
        Some((lo, hi)) if lo.is_finite() && hi.is_finite() && lo < hi => {
            Ok(ParamRange::new(lo, hi))
        }
        _ => Err(PcurveIssue::BadRange),
    }
}

fn parameter_slack(a: f64, b: f64) -> f64 {
    256.0 * f64::EPSILON * (1.0 + a.abs().max(b.abs()))
}

fn parameter_close(a: f64, b: f64) -> bool {
    (a - b).abs() <= parameter_slack(a, b)
}

fn graph_surface_periodicity(
    store: &Store,
    surface: SurfaceId,
) -> core::result::Result<[Option<f64>; 2], PcurveIssue> {
    store
        .eval_context(
            EvalLimits::default(),
            kcore::tolerance::Tolerances::default(),
        )
        .surface_periodicity(surface)
        .map_err(|_| PcurveIssue::BadChart)
}

/// Validate the pcurve handle and its active range against its own natural
/// parameter domain. Periodic curves may use an unwrapped interval no wider
/// than one period.
pub(crate) fn check_pcurve_definition(
    store: &Store,
    pcurve_use: FinPcurve,
) -> core::result::Result<(), PcurveIssue> {
    let geometry = store
        .get(pcurve_use.curve())
        .map_err(|_| PcurveIssue::StaleReference)?;
    let curve = geometry.as_curve();
    let range = pcurve_use.range();
    let valid = match curve.periodicity() {
        Some(period) => {
            period.is_finite()
                && period > 0.0
                && range.width() <= period + parameter_slack(range.width(), period)
        }
        None => {
            let natural = curve.param_range();
            natural.contains(range.lo) && natural.contains(range.hi)
        }
    };
    if valid {
        Ok(())
    } else {
        Err(PcurveIssue::BadRange)
    }
}

/// Validate that a pcurve use covers a bounded logical edge domain exactly.
/// This is shared by exact and curve-less tolerant edges.
pub(crate) fn check_pcurve_parameterization(
    store: &Store,
    bounds: Option<(f64, f64)>,
    pcurve_use: FinPcurve,
) -> core::result::Result<(), PcurveIssue> {
    check_pcurve_definition(store, pcurve_use)?;
    let edge_range = edge_range(bounds)?;
    let pcurve_range = pcurve_use.range();
    let q0 = pcurve_use.parameter_at_edge(edge_range.lo);
    let q1 = pcurve_use.parameter_at_edge(edge_range.hi);
    if q0.is_finite()
        && q1.is_finite()
        && parameter_close(q0.min(q1), pcurve_range.lo)
        && parameter_close(q0.max(q1), pcurve_range.hi)
    {
        Ok(())
    } else {
        Err(PcurveIssue::BadRange)
    }
}

/// Validate that a fin's integer-period chart is meaningful on its surface.
pub(crate) fn check_pcurve_chart(
    store: &Store,
    surface_id: SurfaceId,
    pcurve_use: FinPcurve,
) -> core::result::Result<(), PcurveIssue> {
    check_pcurve_definition(store, pcurve_use)?;
    let pcurve = store
        .get(pcurve_use.curve())
        .map_err(|_| PcurveIssue::StaleReference)?
        .as_curve();
    let periods = graph_surface_periodicity(store, surface_id)?;
    for q in [pcurve_use.range().lo, pcurve_use.range().hi] {
        pcurve_use
            .chart()
            .apply(pcurve.eval(q), periods)
            .map_err(|_| PcurveIssue::BadChart)?;
    }
    Ok(())
}

/// Validate optional closed-use winding and singular endpoint metadata.
pub(crate) fn check_pcurve_metadata(
    store: &Store,
    edge: &Edge,
    surface_id: SurfaceId,
    face_domain: Option<FaceDomain>,
    pcurve_use: FinPcurve,
) -> core::result::Result<(), PcurveIssue> {
    check_pcurve_definition(store, pcurve_use)?;
    check_pcurve_chart(store, surface_id, pcurve_use)?;
    let pcurve = store
        .get(pcurve_use.curve())
        .map_err(|_| PcurveIssue::StaleReference)?
        .as_curve();
    store
        .get(surface_id)
        .map_err(|_| PcurveIssue::StaleReference)?;
    let periods = graph_surface_periodicity(store, surface_id)?;
    let degeneracies = store
        .eval_context(
            EvalLimits::default(),
            kcore::tolerance::Tolerances::default(),
        )
        .surface_degeneracies(surface_id)
        .map_err(|_| PcurveIssue::BadSingularity)?;
    let closed =
        edge.bounds.is_none() || edge.vertices[0].is_some() && edge.vertices[0] == edge.vertices[1];

    let edge_range = match edge.bounds {
        Some((lo, hi)) if lo.is_finite() && hi.is_finite() && lo < hi => ParamRange::new(lo, hi),
        Some(_) => return Err(PcurveIssue::BadRange),
        None => {
            let curve_id = edge.curve.ok_or(PcurveIssue::BadRange)?;
            let range = store
                .get(curve_id)
                .map_err(|_| PcurveIssue::StaleReference)?
                .as_curve()
                .param_range();
            if !range.is_finite() || range.lo >= range.hi {
                return Err(PcurveIssue::BadRange);
            }
            range
        }
    };
    let endpoints = [
        pcurve_use
            .evaluate_uv(pcurve, edge_range.lo, periods)
            .map_err(|_| PcurveIssue::BadChart)?,
        pcurve_use
            .evaluate_uv(pcurve, edge_range.hi, periods)
            .map_err(|_| PcurveIssue::BadChart)?,
    ];

    let endpoint_kinds = pcurve_use.endpoint_kinds();
    if edge.bounds.is_none()
        && endpoint_kinds
            .iter()
            .any(|kind| *kind != PcurveEndpointKind::Regular)
    {
        return Err(PcurveIssue::BadSingularity);
    }
    for (&kind, uv) in endpoint_kinds.iter().zip(endpoints) {
        if kind == PcurveEndpointKind::SurfaceSingularity
            && !degeneracies.iter().any(|degeneracy| {
                let value = match degeneracy.dir {
                    Dir::U => uv.x,
                    Dir::V => uv.y,
                };
                parameter_close(value, degeneracy.at)
            })
        {
            return Err(PcurveIssue::BadSingularity);
        }
    }

    if let Some(winding) = pcurve_use.closure_winding() {
        if !closed {
            return Err(PcurveIssue::BadClosure);
        }
        for direction in 0..2 {
            let delta = if direction == 0 {
                endpoints[1].x - endpoints[0].x
            } else {
                endpoints[1].y - endpoints[0].y
            };
            let expected = match periods[direction] {
                Some(period) if period.is_finite() && period > 0.0 => {
                    f64::from(winding[direction]) * period
                }
                Some(_) => return Err(PcurveIssue::BadClosure),
                None if winding[direction] == 0 => 0.0,
                None => return Err(PcurveIssue::BadClosure),
            };
            if !parameter_close(delta, expected) {
                return Err(PcurveIssue::BadClosure);
            }
        }
    }
    if let Some(seam) = pcurve_use.seam() {
        let domain = face_domain.ok_or(PcurveIssue::BadSeam)?;
        let direction = match seam.direction() {
            SurfaceParameter::U => 0,
            SurfaceParameter::V => 1,
        };
        let period = periods[direction].ok_or(PcurveIssue::BadSeam)?;
        let domain_range = if direction == 0 { domain.u } else { domain.v };
        if !period.is_finite() || period <= 0.0 || !parameter_close(domain_range.width(), period) {
            return Err(PcurveIssue::BadSeam);
        }
        let natural = pcurve.param_range();
        let range = if pcurve.periodicity().is_some() && natural.is_finite() {
            natural
        } else {
            pcurve_use.range()
        };
        let bounds = pcurve.bounding_box(range);
        let min = pcurve_use
            .chart()
            .apply(bounds.min, periods)
            .map_err(|_| PcurveIssue::BadChart)?;
        let max = pcurve_use
            .chart()
            .apply(bounds.max, periods)
            .map_err(|_| PcurveIssue::BadChart)?;
        let (min, max) = if direction == 0 {
            (min.x, max.x)
        } else {
            (min.y, max.y)
        };
        let boundary = match seam.side() {
            SeamSide::Lower => domain_range.lo,
            SeamSide::Upper => domain_range.hi,
        };
        if !parameter_close(min, boundary) || !parameter_close(max, boundary) {
            return Err(PcurveIssue::BadSeam);
        }
    }
    Ok(())
}

/// Validate a complete `(3D curve, edge range, 2D pcurve, surface)` tuple.
pub(crate) fn check_pcurve_incidence(
    store: &Store,
    curve_id: CurveId,
    bounds: Option<(f64, f64)>,
    surface_id: SurfaceId,
    pcurve_use: FinPcurve,
    tolerance: f64,
) -> core::result::Result<(), PcurveIssue> {
    check_pcurve_definition(store, pcurve_use)?;
    check_pcurve_chart(store, surface_id, pcurve_use)?;
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(PcurveIssue::BadRange);
    }
    let curve_geometry = store
        .get(curve_id)
        .map_err(|_| PcurveIssue::StaleReference)?;
    let curve = curve_geometry.as_curve();
    let edge_range = match bounds {
        Some(_) => {
            check_pcurve_parameterization(store, bounds, pcurve_use)?;
            edge_range(bounds)?
        }
        None => {
            let range = curve.param_range();
            if !range.is_finite() || range.lo >= range.hi {
                return Err(PcurveIssue::BadRange);
            }
            range
        }
    };

    let pcurve_range = pcurve_use.range();
    if bounds.is_none() {
        let q0 = pcurve_use.parameter_at_edge(edge_range.lo);
        let q1 = pcurve_use.parameter_at_edge(edge_range.hi);
        if !q0.is_finite()
            || !q1.is_finite()
            || !parameter_close(q0.min(q1), pcurve_range.lo)
            || !parameter_close(q0.max(q1), pcurve_range.hi)
        {
            return Err(PcurveIssue::BadRange);
        }
    }

    let pcurve_geometry = store
        .get(pcurve_use.curve())
        .map_err(|_| PcurveIssue::StaleReference)?;
    let pcurve = pcurve_geometry.as_curve();
    let periods = graph_surface_periodicity(store, surface_id)?;
    let mut evaluator = store.eval_context(
        EvalLimits::default(),
        kcore::tolerance::Tolerances::with_linear(
            tolerance.max(kcore::tolerance::LINEAR_RESOLUTION),
        )
        .map_err(|_| PcurveIssue::BadRange)?,
    );
    for i in 0..=INCIDENCE_SAMPLES {
        let t = edge_range.lerp(i as f64 / INCIDENCE_SAMPLES as f64);
        let q = pcurve_use.parameter_at_edge(t);
        if q < pcurve_range.lo - parameter_slack(q, pcurve_range.lo)
            || q > pcurve_range.hi + parameter_slack(q, pcurve_range.hi)
        {
            return Err(PcurveIssue::BadRange);
        }
        let uv = pcurve_use
            .evaluate_uv(pcurve, t, periods)
            .map_err(|_| PcurveIssue::BadChart)?;
        let point = evaluator
            .eval_surface(surface_id, [uv.x, uv.y], SurfaceDerivativeOrder::Position)
            .map_err(|_| PcurveIssue::OffSurface)?
            .p;
        if point.dist(curve.eval(t)) > tolerance {
            return Err(PcurveIssue::OffSurface);
        }
    }
    Ok(())
}

/// Prove that an exact edge curve lies on one supporting surface for its
/// entire active parameter interval.
///
/// The current proof slice covers every stored curve class on a plane,
/// lines and harmonic curves on cylinders, and harmonic curves on spheres.
/// Unsupported pairs remain indeterminate; sampling is never promoted to a
/// certificate.
pub(crate) fn certify_edge_surface_incidence(
    store: &Store,
    edge_id: EdgeId,
    surface_id: SurfaceId,
    tolerance: f64,
) -> KernelResult<IncidenceCertification> {
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Ok(IncidenceCertification::Indeterminate);
    }
    let edge = store.get(edge_id)?;
    let Some(curve_id) = edge.curve else {
        return Ok(IncidenceCertification::Indeterminate);
    };
    let curve = store.get(curve_id)?;
    let surface = store.get(surface_id)?;
    let Some(range) = active_edge_range(edge, curve) else {
        return Ok(IncidenceCertification::Indeterminate);
    };
    let Some(bound) = direct_incidence_bound(curve, surface, range) else {
        return Ok(IncidenceCertification::Indeterminate);
    };
    Ok(certification_from_bound(
        bound,
        curve_coordinate_scale(curve, range),
        tolerance,
    ))
}

/// Prove equality of a 3D edge trace and the trace produced by lifting one
/// pcurve through its supporting surface.
///
/// Exact affine and single-frequency harmonic traces are compared using
/// whole-interval residual bounds. General NURBS compositions and pcurves
/// that vary both parameters of a nonlinear surface remain indeterminate.
pub(crate) fn certify_pcurve_incidence(
    store: &Store,
    edge_id: EdgeId,
    surface_id: SurfaceId,
    pcurve_use: FinPcurve,
    tolerance: f64,
) -> KernelResult<IncidenceCertification> {
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Ok(IncidenceCertification::Indeterminate);
    }
    let edge = store.get(edge_id)?;
    let Some(curve_id) = edge.curve else {
        return Ok(IncidenceCertification::Indeterminate);
    };
    let curve = store.get(curve_id)?;
    let surface = store.get(surface_id)?;
    let pcurve = store.get(pcurve_use.curve())?;
    let Some(range) = active_edge_range(edge, curve) else {
        return Ok(IncidenceCertification::Indeterminate);
    };
    let Some(edge_trace) = attached_curve_trace(curve) else {
        return Ok(IncidenceCertification::Indeterminate);
    };
    let Some(lifted_trace) = lifted_pcurve_trace(pcurve, surface, pcurve_use) else {
        return Ok(IncidenceCertification::Indeterminate);
    };
    let Some(bound) = trace_difference_bound(edge_trace, lifted_trace, range) else {
        return Ok(IncidenceCertification::Indeterminate);
    };
    let scale = trace_scale(edge_trace, range).max(trace_scale(lifted_trace, range));
    Ok(certification_from_bound(bound, scale, tolerance))
}

fn certification_from_bound(
    residual_upper_bound: f64,
    coordinate_scale: f64,
    tolerance: f64,
) -> IncidenceCertification {
    // The analytic formulas below are conservative in exact arithmetic.
    // Reserve a scale-aware floating-point guard so rounding cannot turn a
    // borderline computation into a certificate.
    let guard = 4096.0 * f64::EPSILON * (1.0 + coordinate_scale.abs());
    if residual_upper_bound.is_finite()
        && coordinate_scale.is_finite()
        && residual_upper_bound + guard <= tolerance
    {
        IncidenceCertification::Certified
    } else {
        IncidenceCertification::Indeterminate
    }
}

fn active_edge_range(edge: &Edge, curve: &CurveGeom) -> Option<ParamRange> {
    match edge.bounds {
        Some((lo, hi)) if lo.is_finite() && hi.is_finite() && lo < hi => {
            Some(ParamRange::new(lo, hi))
        }
        Some(_) => None,
        None => {
            let range = curve.as_curve().param_range();
            (range.is_finite() && range.lo < range.hi).then_some(range)
        }
    }
}

fn direct_incidence_bound(
    curve: &CurveGeom,
    surface: &SurfaceGeom,
    range: ParamRange,
) -> Option<f64> {
    match surface {
        SurfaceGeom::Plane(plane) => plane_curve_residual_bound(curve, plane.frame(), range),
        SurfaceGeom::Cylinder(cylinder) => {
            cylinder_curve_residual_bound(curve, cylinder.frame(), cylinder.radius(), range)
        }
        SurfaceGeom::Sphere(sphere) => {
            sphere_curve_residual_bound(curve, sphere.frame().origin(), sphere.radius())
        }
        SurfaceGeom::Cone(_) | SurfaceGeom::Torus(_) | SurfaceGeom::Nurbs(_) => None,
        _ => None,
    }
}

fn signed_plane_distance(frame: &Frame, point: Point3) -> f64 {
    (point - frame.origin()).dot(frame.z())
}

fn plane_curve_residual_bound(curve: &CurveGeom, frame: &Frame, range: ParamRange) -> Option<f64> {
    match curve {
        CurveGeom::Line(line) => Some(
            signed_plane_distance(frame, line.eval(range.lo))
                .abs()
                .max(signed_plane_distance(frame, line.eval(range.hi)).abs()),
        ),
        CurveGeom::Circle(_) | CurveGeom::Ellipse(_) => {
            let Some(AnalyticTrace::Harmonic {
                center,
                cosine,
                sine,
            }) = attached_curve_trace(curve)
            else {
                unreachable!("circle and ellipse always have harmonic traces");
            };
            Some(
                signed_plane_distance(frame, center).abs()
                    + cosine.dot(frame.z()).abs()
                    + sine.dot(frame.z()).abs(),
            )
        }
        CurveGeom::Nurbs(curve) => Some(
            curve
                .points()
                .iter()
                .map(|&point| signed_plane_distance(frame, point).abs())
                .fold(0.0, f64::max),
        ),
        _ => None,
    }
}

fn cylinder_curve_residual_bound(
    curve: &CurveGeom,
    frame: &Frame,
    radius: f64,
    range: ParamRange,
) -> Option<f64> {
    match curve {
        CurveGeom::Line(line) => {
            let origin = frame.to_local(line.origin());
            let direction = local_vector(frame, line.dir());
            let a = direction.x * direction.x + direction.y * direction.y;
            let b = 2.0 * (origin.x * direction.x + origin.y * direction.y);
            let c = origin.x * origin.x + origin.y * origin.y - radius * radius;
            Some(max_abs_quadratic(a, b, c, range) / radius)
        }
        CurveGeom::Circle(_) | CurveGeom::Ellipse(_) => {
            let trace = attached_curve_trace(curve)?;
            let AnalyticTrace::Harmonic {
                center,
                cosine,
                sine,
            } = trace
            else {
                return None;
            };
            let center = frame.to_local(center);
            let cosine = local_vector(frame, cosine);
            let sine = local_vector(frame, sine);
            let center = Vec3::new(center.x, center.y, 0.0);
            let cosine = Vec3::new(cosine.x, cosine.y, 0.0);
            let sine = Vec3::new(sine.x, sine.y, 0.0);
            Some(squared_radius_residual_bound(center, cosine, sine, radius) / radius)
        }
        CurveGeom::Nurbs(_) => None,
        _ => None,
    }
}

fn sphere_curve_residual_bound(curve: &CurveGeom, center: Point3, radius: f64) -> Option<f64> {
    let AnalyticTrace::Harmonic {
        center: curve_center,
        cosine,
        sine,
    } = attached_curve_trace(curve)?
    else {
        return None;
    };
    Some(squared_radius_residual_bound(curve_center - center, cosine, sine, radius) / radius)
}

fn squared_radius_residual_bound(center: Vec3, cosine: Vec3, sine: Vec3, radius: f64) -> f64 {
    let constant = center.norm_sq() + 0.5 * (cosine.norm_sq() + sine.norm_sq()) - radius * radius;
    let cos_one = 2.0 * center.dot(cosine);
    let sin_one = 2.0 * center.dot(sine);
    let cos_two = 0.5 * (cosine.norm_sq() - sine.norm_sq());
    let sin_two = cosine.dot(sine);
    constant.abs() + cos_one.abs() + sin_one.abs() + cos_two.abs() + sin_two.abs()
}

fn max_abs_quadratic(a: f64, b: f64, c: f64, range: ParamRange) -> f64 {
    let value = |t: f64| (a * t + b) * t + c;
    let mut out = value(range.lo).abs().max(value(range.hi).abs());
    if a != 0.0 {
        let critical = -b / (2.0 * a);
        if range.contains(critical) {
            out = out.max(value(critical).abs());
        }
    }
    out
}

fn attached_curve_trace(curve: &CurveGeom) -> Option<AnalyticTrace> {
    match curve {
        CurveGeom::Line(line) => Some(AnalyticTrace::Affine {
            origin: line.origin(),
            direction: line.dir(),
        }),
        CurveGeom::Circle(circle) => Some(AnalyticTrace::Harmonic {
            center: circle.frame().origin(),
            cosine: circle.frame().x() * circle.radius(),
            sine: circle.frame().y() * circle.radius(),
        }),
        CurveGeom::Ellipse(ellipse) => Some(AnalyticTrace::Harmonic {
            center: ellipse.frame().origin(),
            cosine: ellipse.frame().x() * ellipse.major_radius(),
            sine: ellipse.frame().y() * ellipse.minor_radius(),
        }),
        CurveGeom::Nurbs(_) => None,
        _ => None,
    }
}

fn lifted_pcurve_trace(
    pcurve: &Curve2dGeom,
    surface: &SurfaceGeom,
    use_: FinPcurve,
) -> Option<AnalyticTrace> {
    let periods = surface.as_leaf_surface()?.periodicity();
    let chart_offset = use_.chart().apply(Point2::default(), periods).ok()?;
    let map = use_.edge_to_pcurve();
    match pcurve {
        Curve2dGeom::Line(line) => {
            let uv0 = line.origin() + line.dir() * map.offset() + chart_offset;
            let rate = line.dir() * map.scale();
            lifted_line_trace(surface, uv0, rate)
        }
        Curve2dGeom::Circle(circle) => {
            let SurfaceGeom::Plane(plane) = surface else {
                return None;
            };
            let orientation = unit_angular_rate(map.scale())?;
            let center = circle.center() + chart_offset;
            let x = circle.x_dir() * circle.radius();
            let y = circle.x_dir().perp() * circle.radius();
            Some(phase_harmonic(
                plane.frame().point_at(center.x, center.y, 0.0),
                uv_vector(plane.frame(), x),
                uv_vector(plane.frame(), y),
                map.offset(),
                orientation,
            ))
        }
        Curve2dGeom::Nurbs(_) => None,
        _ => None,
    }
}

fn lifted_line_trace(surface: &SurfaceGeom, uv0: Point2, rate: Vec2) -> Option<AnalyticTrace> {
    match surface {
        SurfaceGeom::Plane(plane) => Some(AnalyticTrace::Affine {
            origin: plane.frame().point_at(uv0.x, uv0.y, 0.0),
            direction: uv_vector(plane.frame(), rate),
        }),
        SurfaceGeom::Cylinder(cylinder) if rate.x == 0.0 => Some(AnalyticTrace::Affine {
            origin: cylinder.eval([uv0.x, uv0.y]),
            direction: cylinder.frame().z() * rate.y,
        }),
        SurfaceGeom::Cylinder(cylinder) if rate.y == 0.0 => {
            let orientation = unit_angular_rate(rate.x)?;
            Some(phase_harmonic(
                cylinder.frame().origin() + cylinder.frame().z() * uv0.y,
                cylinder.frame().x() * cylinder.radius(),
                cylinder.frame().y() * cylinder.radius(),
                uv0.x,
                orientation,
            ))
        }
        SurfaceGeom::Cone(cone) if rate.x == 0.0 => {
            let (sin_angle, cos_angle) = math::sincos(cone.half_angle());
            let radial = frame_radial(cone.frame(), uv0.x);
            Some(AnalyticTrace::Affine {
                origin: cone.frame().origin() + radial * cone.radius(),
                direction: (radial * sin_angle + cone.frame().z() * cos_angle) * rate.y,
            })
        }
        SurfaceGeom::Cone(cone) if rate.y == 0.0 => {
            let orientation = unit_angular_rate(rate.x)?;
            let (sin_angle, cos_angle) = math::sincos(cone.half_angle());
            let radius = cone.radius() + uv0.y * sin_angle;
            Some(phase_harmonic(
                cone.frame().origin() + cone.frame().z() * (uv0.y * cos_angle),
                cone.frame().x() * radius,
                cone.frame().y() * radius,
                uv0.x,
                orientation,
            ))
        }
        SurfaceGeom::Sphere(sphere) if rate.y == 0.0 => {
            let orientation = unit_angular_rate(rate.x)?;
            let (sin_v, cos_v) = math::sincos(uv0.y);
            Some(phase_harmonic(
                sphere.frame().origin() + sphere.frame().z() * (sphere.radius() * sin_v),
                sphere.frame().x() * (sphere.radius() * cos_v),
                sphere.frame().y() * (sphere.radius() * cos_v),
                uv0.x,
                orientation,
            ))
        }
        SurfaceGeom::Sphere(sphere) if rate.x == 0.0 => {
            let orientation = unit_angular_rate(rate.y)?;
            let radial = frame_radial(sphere.frame(), uv0.x);
            Some(phase_harmonic(
                sphere.frame().origin(),
                radial * sphere.radius(),
                sphere.frame().z() * sphere.radius(),
                uv0.y,
                orientation,
            ))
        }
        SurfaceGeom::Torus(torus) if rate.y == 0.0 => {
            let orientation = unit_angular_rate(rate.x)?;
            let (sin_v, cos_v) = math::sincos(uv0.y);
            let radius = torus.major_radius() + torus.minor_radius() * cos_v;
            Some(phase_harmonic(
                torus.frame().origin() + torus.frame().z() * (torus.minor_radius() * sin_v),
                torus.frame().x() * radius,
                torus.frame().y() * radius,
                uv0.x,
                orientation,
            ))
        }
        SurfaceGeom::Torus(torus) if rate.x == 0.0 => {
            let orientation = unit_angular_rate(rate.y)?;
            let radial = frame_radial(torus.frame(), uv0.x);
            Some(phase_harmonic(
                torus.frame().origin() + radial * torus.major_radius(),
                radial * torus.minor_radius(),
                torus.frame().z() * torus.minor_radius(),
                uv0.y,
                orientation,
            ))
        }
        SurfaceGeom::Nurbs(_)
        | SurfaceGeom::Cylinder(_)
        | SurfaceGeom::Cone(_)
        | SurfaceGeom::Sphere(_)
        | SurfaceGeom::Torus(_) => None,
        _ => None,
    }
}

fn unit_angular_rate(rate: f64) -> Option<f64> {
    if rate == 1.0 || rate == -1.0 {
        Some(rate)
    } else {
        None
    }
}

fn phase_harmonic(center: Point3, x: Vec3, y: Vec3, phase: f64, orientation: f64) -> AnalyticTrace {
    let (sin_phase, cos_phase) = math::sincos(phase);
    AnalyticTrace::Harmonic {
        center,
        cosine: x * cos_phase + y * sin_phase,
        sine: (y * cos_phase - x * sin_phase) * orientation,
    }
}

fn frame_radial(frame: &Frame, u: f64) -> Vec3 {
    let (sin_u, cos_u) = math::sincos(u);
    frame.x() * cos_u + frame.y() * sin_u
}

fn uv_vector(frame: &Frame, vector: Vec2) -> Vec3 {
    frame.x() * vector.x + frame.y() * vector.y
}

fn local_vector(frame: &Frame, vector: Vec3) -> Vec3 {
    Vec3::new(
        vector.dot(frame.x()),
        vector.dot(frame.y()),
        vector.dot(frame.z()),
    )
}

fn trace_difference_bound(
    left: AnalyticTrace,
    right: AnalyticTrace,
    range: ParamRange,
) -> Option<f64> {
    match (left, right) {
        (
            AnalyticTrace::Affine {
                origin: left_origin,
                direction: left_direction,
            },
            AnalyticTrace::Affine {
                origin: right_origin,
                direction: right_direction,
            },
        ) => {
            let origin = left_origin - right_origin;
            let direction = left_direction - right_direction;
            Some(
                (origin + direction * range.lo)
                    .norm()
                    .max((origin + direction * range.hi).norm()),
            )
        }
        (
            AnalyticTrace::Harmonic {
                center: left_center,
                cosine: left_cosine,
                sine: left_sine,
            },
            AnalyticTrace::Harmonic {
                center: right_center,
                cosine: right_cosine,
                sine: right_sine,
            },
        ) => Some(
            (left_center - right_center).norm()
                + (left_cosine - right_cosine).norm()
                + (left_sine - right_sine).norm(),
        ),
        _ => None,
    }
}

fn trace_scale(trace: AnalyticTrace, range: ParamRange) -> f64 {
    match trace {
        AnalyticTrace::Affine { origin, direction } => (origin + direction * range.lo)
            .norm()
            .max((origin + direction * range.hi).norm()),
        AnalyticTrace::Harmonic {
            center,
            cosine,
            sine,
        } => center.norm() + cosine.norm() + sine.norm(),
    }
}

fn curve_coordinate_scale(curve: &CurveGeom, range: ParamRange) -> f64 {
    let bounds = curve.as_curve().bounding_box(range);
    [
        bounds.min.x,
        bounds.min.y,
        bounds.min.z,
        bounds.max.x,
        bounds.max.y,
        bounds.max.z,
    ]
    .into_iter()
    .map(f64::abs)
    .fold(0.0, f64::max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::entity::{
        Edge, FaceDomain, ParamMap1d, PcurveChart, PcurveEndpointKind, PcurveSeam, SeamSide,
        SurfaceParameter,
    };
    use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
    use kgeom::curve::Line;
    use kgeom::curve2d::Line2d;
    use kgeom::frame::Frame;
    use kgeom::surface::{Cylinder, Plane, Sphere};
    use kgeom::vec::{Point2, Point3, Vec2, Vec3};

    #[test]
    fn whole_interval_certificate_requires_a_conservative_residual_bound() {
        let mut store = Store::new();
        let curve = store
            .insert_curve(CurveGeom::Line(
                Line::new(Point3::new(-2.0, 0.5, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap(),
            ))
            .unwrap();
        let edge = store.add(Edge {
            curve: Some(curve),
            vertices: [None, None],
            bounds: Some((-1.0, 3.0)),
            fins: Vec::new(),
            tolerance: None,
        });
        let on_plane = store
            .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        assert_eq!(
            certify_edge_surface_incidence(
                &store,
                edge,
                on_plane,
                kcore::tolerance::LINEAR_RESOLUTION,
            )
            .unwrap(),
            IncidenceCertification::Certified
        );

        let displaced_frame = Frame::new(
            Point3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let displaced = store
            .insert_surface(SurfaceGeom::Plane(Plane::new(displaced_frame)))
            .unwrap();
        assert_eq!(
            certify_edge_surface_incidence(
                &store,
                edge,
                displaced,
                kcore::tolerance::LINEAR_RESOLUTION,
            )
            .unwrap(),
            IncidenceCertification::Indeterminate
        );
    }

    #[test]
    fn singular_endpoint_metadata_matches_surface_degeneracies() {
        let mut store = Store::new();
        let surface = store
            .insert_surface(SurfaceGeom::Sphere(
                Sphere::new(Frame::world(), 1.0).unwrap(),
            ))
            .unwrap();
        let curve = store
            .insert_pcurve(Curve2dGeom::Line(
                Line2d::new(Point2::new(0.0, 0.0), Vec2::new(0.0, 1.0)).unwrap(),
            ))
            .unwrap();
        let range = ParamRange::new(0.0, core::f64::consts::FRAC_PI_2);
        let use_ = FinPcurve::new(curve, range, ParamMap1d::identity())
            .unwrap()
            .with_endpoint_kinds([
                PcurveEndpointKind::Regular,
                PcurveEndpointKind::SurfaceSingularity,
            ]);
        let edge = Edge {
            curve: None,
            vertices: [None, None],
            bounds: Some((range.lo, range.hi)),
            fins: Vec::new(),
            tolerance: None,
        };
        assert_eq!(
            check_pcurve_metadata(&store, &edge, surface, None, use_),
            Ok(())
        );

        let invalid = use_.with_endpoint_kinds([
            PcurveEndpointKind::SurfaceSingularity,
            PcurveEndpointKind::Regular,
        ]);
        assert_eq!(
            check_pcurve_metadata(&store, &edge, surface, None, invalid),
            Err(PcurveIssue::BadSingularity)
        );
    }

    #[test]
    fn seam_metadata_selects_a_full_period_chart_boundary() {
        let mut store = Store::new();
        let surface = store
            .insert_surface(SurfaceGeom::Cylinder(
                Cylinder::new(Frame::world(), 1.0).unwrap(),
            ))
            .unwrap();
        let curve = store
            .insert_pcurve(Curve2dGeom::Line(
                Line2d::new(Point2::new(0.0, 0.0), Vec2::new(0.0, 1.0)).unwrap(),
            ))
            .unwrap();
        let range = ParamRange::new(0.0, 2.0);
        let edge = Edge {
            curve: None,
            vertices: [None, None],
            bounds: Some((range.lo, range.hi)),
            fins: Vec::new(),
            tolerance: None,
        };
        let domain = FaceDomain::from_bounds(0.0, core::f64::consts::TAU, 0.0, 2.0).unwrap();
        let base = FinPcurve::new(curve, range, ParamMap1d::identity()).unwrap();
        let lower = base.with_seam(PcurveSeam::new(SurfaceParameter::U, SeamSide::Lower));
        assert_eq!(
            check_pcurve_metadata(&store, &edge, surface, Some(domain), lower),
            Ok(())
        );

        let wrong_side = base.with_seam(PcurveSeam::new(SurfaceParameter::U, SeamSide::Upper));
        assert_eq!(
            check_pcurve_metadata(&store, &edge, surface, Some(domain), wrong_side),
            Err(PcurveIssue::BadSeam)
        );

        let upper = wrong_side.with_chart(PcurveChart::shifted([1, 0]));
        assert_eq!(
            check_pcurve_metadata(&store, &edge, surface, Some(domain), upper),
            Ok(())
        );
    }
}
