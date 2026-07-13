use super::circle_cone::intersect_bounded_circle_cone;
use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::parameter::{
    PeriodicOverlapPiece, affine_preimage_overlap, fit_scalar_parameter,
    periodic_preimage_overlaps, range_midpoint, validate_period_span,
};
use super::result::{
    ContactKind, SurfaceIntersectionCurve, SurfaceRegionCorrespondence, SurfaceRegionOrientation,
    SurfaceSurfaceCurve, SurfaceSurfaceIntersections, SurfaceSurfacePoint, SurfaceSurfaceRegion,
    SurfaceSurfaceRegionVertex, accept_surface_surface_candidate,
};
use super::support_curve_pair::{SupportCurvePairConfig, emit_support_curve_pair};
use kcore::error::{Error, Result};
use kcore::interval::Interval;
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Line};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cone, Surface};
use kgeom::vec::{Point3, Vec3};

/// Intersect two finite cone parameter windows.
///
/// Supports coaxial cone/cone intersections. Each cone contributes two signed
/// meridian lines; non-coincident line pairs revolve into exact circle branches,
/// while shared apex roots become singular point contacts. Exact coincident
/// charts with equal apexes and half-angles return paired finite regions, circle
/// or ruling edges, singular apex points, or proven-empty evidence. Longitude
/// windows are seam-aware and finite-window regions split at the apex so no
/// positive-area chart crosses the cone singularity. Unsupported coincidence
/// and unsafe remote chart representatives fail closed for the certified
/// general fallback.
pub fn intersect_bounded_cones(
    a: &Cone,
    a_range: [ParamRange; 2],
    b: &Cone,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(a_range, b_range)?;

    let axis = a.frame().z();
    if axis.cross(b.frame().z()).norm() > tolerances.angular() {
        return Err(Error::InvalidGeometry {
            reason: "cone/cone intersection currently supports only coaxial circular cuts",
        });
    }

    let offset = b.frame().origin() - a.frame().origin();
    let radial_offset = offset - axis * offset.dot(axis);
    if radial_offset.norm() > tolerances.linear() {
        return Err(Error::InvalidGeometry {
            reason: "cone/cone intersection currently supports only coaxial circular cuts",
        });
    }

    if let Some(map) = exact_coincident_cone_map(a, b) {
        if compare_cone_windows(a, a_range, b, b_range).is_gt() {
            return intersect_coincident_cone_windows(
                b,
                b_range,
                a,
                a_range,
                exact_coincident_cone_map(b, a)
                    .expect("exact coincident cones remain exact after operand swap"),
                tolerances,
            )
            .map(SurfaceSurfaceIntersections::swapped);
        }
        return intersect_coincident_cone_windows(a, a_range, b, b_range, map, tolerances);
    }

    let roots = meridian_roots(a, b, offset.dot(axis), tolerances)?;
    let mut points = Vec::new();
    let mut curves = Vec::new();
    for root in roots {
        match root {
            MeridianRoot::Circle { radius, z } => add_circle_branch(
                &mut points,
                &mut curves,
                radius,
                z,
                a,
                a_range,
                b,
                b_range,
                tolerances,
            )?,
            MeridianRoot::Apex { z } => add_point(
                &mut points,
                a.frame().origin() + axis * z,
                a,
                a_range,
                b,
                b_range,
                ContactKind::Singular,
                tolerances,
            ),
        }
    }

    SurfaceSurfaceIntersections::canonicalized_complete(points, curves)
}

fn compare_cone_windows(
    a: &Cone,
    a_range: [ParamRange; 2],
    b: &Cone,
    b_range: [ParamRange; 2],
) -> core::cmp::Ordering {
    let a_values = a
        .frame()
        .origin()
        .to_array()
        .into_iter()
        .chain(a.frame().z().to_array())
        .chain(a.frame().x().to_array())
        .chain([
            a.radius(),
            a.half_angle(),
            a_range[0].lo,
            a_range[0].hi,
            a_range[1].lo,
            a_range[1].hi,
        ]);
    let b_values = b
        .frame()
        .origin()
        .to_array()
        .into_iter()
        .chain(b.frame().z().to_array())
        .chain(b.frame().x().to_array())
        .chain([
            b.radius(),
            b.half_angle(),
            b_range[0].lo,
            b_range[0].hi,
            b_range[1].lo,
            b_range[1].hi,
        ]);
    a_values
        .zip(b_values)
        .map(|(a, b)| a.total_cmp(&b))
        .find(|ordering| !ordering.is_eq())
        .unwrap_or(core::cmp::Ordering::Equal)
}

#[derive(Clone, Copy, Debug)]
struct CoincidentConeMap {
    sign: f64,
    u_phase: f64,
    v_phase: f64,
}

fn exact_coincident_cone_map(a: &Cone, b: &Cone) -> Option<CoincidentConeMap> {
    let sign = if b.frame().z() == a.frame().z() {
        1.0
    } else if b.frame().z() == -a.frame().z() {
        -1.0
    } else {
        return None;
    };
    if a.half_angle() != b.half_angle() || a.apex() != b.apex() {
        return None;
    }
    let required_first_x = a.frame().x() * sign;
    let u_phase = math::atan2(
        required_first_x.dot(b.frame().y()),
        required_first_x.dot(b.frame().x()),
    );
    Some(CoincidentConeMap {
        sign,
        u_phase,
        v_phase: b.apex_v() - sign * a.apex_v(),
    })
}

#[derive(Clone, Copy, Debug)]
struct PairedConeSample {
    point: Point3,
    uv_a: [f64; 2],
    uv_b: [f64; 2],
    residual: f64,
    residual_bound: f64,
}

#[allow(clippy::too_many_arguments)]
fn intersect_coincident_cone_windows(
    a: &Cone,
    a_range: [ParamRange; 2],
    b: &Cone,
    b_range: [ParamRange; 2],
    map: CoincidentConeMap,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let tau = core::f64::consts::TAU;
    validate_period_span(
        a_range[0],
        tau,
        0.0,
        "coincident cone longitude windows cannot span more than one turn",
    )?;
    validate_period_span(
        b_range[0],
        tau,
        0.0,
        "coincident cone longitude windows cannot span more than one turn",
    )?;
    validate_coincident_cone_map_corridor(a, a_range, b, b_range, map, tolerances)?;

    let u_tolerance = tolerances.angular();
    let u_overlaps = periodic_preimage_overlaps(
        a_range[0],
        b_range[0],
        map.sign,
        map.u_phase,
        tau,
        u_tolerance,
        "coincident cone periodic chart shift is outside the exact integer corridor",
    )?;
    let Some(v_overlap) = affine_preimage_overlap(
        a_range[1],
        b_range[1],
        map.sign,
        map.v_phase,
        tolerances.linear(),
    ) else {
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    };

    let apex = a.apex_v();
    let apex_in_overlap = fit_scalar_parameter(apex, v_overlap, tolerances.linear());
    if u_overlaps.is_empty() {
        if apex_in_overlap.is_some() {
            return SurfaceSurfaceIntersections::canonicalized_complete(
                vec![coincident_cone_apex_point(
                    a, a_range, b, b_range, tolerances,
                )?],
                Vec::new(),
            );
        }
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    }

    let mut points = Vec::new();
    let mut curves = Vec::new();
    let mut regions = Vec::new();
    if v_overlap.width() <= tolerances.linear() {
        let v = range_midpoint(v_overlap);
        if (v - apex).abs() <= tolerances.linear() {
            points.push(coincident_cone_apex_point(
                a, a_range, b, b_range, tolerances,
            )?);
        } else {
            for u in u_overlaps {
                if u.a.width() > u_tolerance {
                    curves.push(coincident_cone_circle_branch(
                        a, b, u, v, b_range, map, tolerances,
                    )?);
                } else {
                    let sample = paired_cone_sample(
                        a,
                        b,
                        [range_midpoint(u.a), v],
                        u.shift,
                        b_range,
                        map,
                        tolerances,
                    )?;
                    points.push(surface_point_from_cone_sample(sample, ContactKind::Tangent));
                }
            }
        }
    } else {
        let v_pieces = split_cone_range_at_apex(v_overlap, apex, tolerances.linear());
        for u in u_overlaps {
            for v in &v_pieces {
                if u.a.width() > u_tolerance {
                    regions.push(coincident_cone_region(
                        a, b, u, *v, b_range, map, tolerances,
                    )?);
                } else {
                    curves.push(coincident_cone_ruling_branch(
                        a,
                        b,
                        range_midpoint(u.a),
                        u.shift,
                        *v,
                        b_range,
                        map,
                        tolerances,
                    )?);
                }
            }
        }
    }
    SurfaceSurfaceIntersections::canonicalized_complete_with_regions(points, curves, regions)
}

fn validate_coincident_cone_map_corridor(
    a: &Cone,
    a_range: [ParamRange; 2],
    b: &Cone,
    b_range: [ParamRange; 2],
    map: CoincidentConeMap,
    tolerances: Tolerances,
) -> Result<()> {
    if a == b && a_range == b_range {
        return Ok(());
    }
    let periodic_error = coincident_cone_periodic_map_error(a_range, b_range, map)?;
    let scalar_error = coincident_cone_scalar_map_error(a_range, b_range, map)?;
    if periodic_error > tolerances.angular() || scalar_error > tolerances.linear() {
        return Err(Error::InvalidGeometry {
            reason: "coincident cone chart map exceeds the certified parameter-roundoff corridor",
        });
    }
    Ok(())
}

fn coincident_cone_periodic_map_error(
    a_range: [ParamRange; 2],
    b_range: [ParamRange; 2],
    map: CoincidentConeMap,
) -> Result<f64> {
    let scale = [
        a_range[0].lo.abs(),
        a_range[0].hi.abs(),
        b_range[0].lo.abs(),
        b_range[0].hi.abs(),
        map.u_phase.abs(),
        2.0 * core::f64::consts::TAU,
    ]
    .into_iter()
    .fold(1.0_f64, f64::max);
    finite_interval_upper(Interval::point(4.0 * f64::EPSILON) * Interval::point(scale))
}

fn coincident_cone_scalar_map_error(
    a_range: [ParamRange; 2],
    b_range: [ParamRange; 2],
    map: CoincidentConeMap,
) -> Result<f64> {
    if map.v_phase == 0.0 {
        return Ok(0.0);
    }
    let scale = [
        a_range[1].lo.abs(),
        a_range[1].hi.abs(),
        b_range[1].lo.abs(),
        b_range[1].hi.abs(),
        map.v_phase.abs(),
        1.0,
    ]
    .into_iter()
    .fold(1.0_f64, f64::max);
    finite_interval_upper(Interval::point(2.0 * f64::EPSILON) * Interval::point(scale))
}

fn split_cone_range_at_apex(range: ParamRange, apex: f64, tolerance: f64) -> Vec<ParamRange> {
    if apex > range.lo + tolerance && apex < range.hi - tolerance {
        vec![
            ParamRange::new(range.lo, apex),
            ParamRange::new(apex, range.hi),
        ]
    } else {
        vec![range]
    }
}

#[allow(clippy::too_many_arguments)]
fn coincident_cone_region(
    a: &Cone,
    b: &Cone,
    u: PeriodicOverlapPiece,
    v: ParamRange,
    b_range: [ParamRange; 2],
    map: CoincidentConeMap,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceRegion> {
    let mut boundary = Vec::with_capacity(4);
    let mut max_residual = coincident_cone_whole_residual_bound(a, b, u, v, b_range, map)?;
    for uv_a in [
        [u.a.lo, v.lo],
        [u.a.hi, v.lo],
        [u.a.hi, v.hi],
        [u.a.lo, v.hi],
    ] {
        let sample = paired_cone_sample(a, b, uv_a, u.shift, b_range, map, tolerances)?;
        max_residual = max_residual.max(sample.residual_bound);
        boundary.push(SurfaceSurfaceRegionVertex {
            point: sample.point,
            uv_a: sample.uv_a,
            uv_b: sample.uv_b,
            residual: sample.residual,
        });
    }
    Ok(SurfaceSurfaceRegion {
        boundary,
        orientation: SurfaceRegionOrientation::Same,
        correspondence: SurfaceRegionCorrespondence::Polygonal,
        max_residual,
    })
}

#[allow(clippy::too_many_arguments)]
fn coincident_cone_circle_branch(
    a: &Cone,
    b: &Cone,
    u: PeriodicOverlapPiece,
    v: f64,
    b_range: [ParamRange; 2],
    map: CoincidentConeMap,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceCurve> {
    let start = paired_cone_sample(a, b, [u.a.lo, v], u.shift, b_range, map, tolerances)?;
    let end = paired_cone_sample(a, b, [u.a.hi, v], u.shift, b_range, map, tolerances)?;
    let (sin_angle, cos_angle) = math::sincos(a.half_angle());
    let signed_radius = a.radius() + v * sin_angle;
    if signed_radius.abs() <= tolerances.linear() {
        return Err(Error::InvalidGeometry {
            reason: "coincident cone apex latitude must collapse to a singular point",
        });
    }
    let center = a.frame().origin() + a.frame().z() * (v * cos_angle);
    let radial_sign = if signed_radius.is_sign_negative() {
        -1.0
    } else {
        1.0
    };
    let circle = Circle::new(
        Frame::new(center, a.frame().z(), a.frame().x() * radial_sign)?,
        signed_radius.abs(),
    )?;
    Ok(SurfaceSurfaceCurve {
        curve: SurfaceIntersectionCurve::Circle(circle),
        curve_range: u.a,
        uv_a_start: start.uv_a,
        uv_a_end: end.uv_a,
        uv_b_start: start.uv_b,
        uv_b_end: end.uv_b,
        kind: ContactKind::Tangent,
    })
}

#[allow(clippy::too_many_arguments)]
fn coincident_cone_ruling_branch(
    a: &Cone,
    b: &Cone,
    u: f64,
    shift: f64,
    v: ParamRange,
    b_range: [ParamRange; 2],
    map: CoincidentConeMap,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceCurve> {
    let start = paired_cone_sample(a, b, [u, v.lo], shift, b_range, map, tolerances)?;
    let end = paired_cone_sample(a, b, [u, v.hi], shift, b_range, map, tolerances)?;
    let (sin_u, cos_u) = math::sincos(u);
    let (sin_angle, cos_angle) = math::sincos(a.half_angle());
    let radial = a.frame().x() * cos_u + a.frame().y() * sin_u;
    let direction = radial * sin_angle + a.frame().z() * cos_angle;
    let line = Line::new(a.eval([u, 0.0]), direction)?;
    Ok(SurfaceSurfaceCurve {
        curve: SurfaceIntersectionCurve::Line(line),
        curve_range: v,
        uv_a_start: start.uv_a,
        uv_a_end: end.uv_a,
        uv_b_start: start.uv_b,
        uv_b_end: end.uv_b,
        kind: ContactKind::Tangent,
    })
}

#[allow(clippy::too_many_arguments)]
fn paired_cone_sample(
    a: &Cone,
    b: &Cone,
    uv_a: [f64; 2],
    u_shift: f64,
    b_range: [ParamRange; 2],
    map: CoincidentConeMap,
    tolerances: Tolerances,
) -> Result<PairedConeSample> {
    let mapped_u = map.sign * uv_a[0] + map.u_phase + u_shift;
    let mapped_v = map.sign * uv_a[1] + map.v_phase;
    let u_scale = uv_a[0]
        .abs()
        .max(map.u_phase.abs())
        .max(u_shift.abs())
        .max(b_range[0].lo.abs())
        .max(b_range[0].hi.abs())
        .max(1.0);
    let u_roundoff =
        finite_interval_upper(Interval::point(4.0 * f64::EPSILON) * Interval::point(u_scale))?;
    let v_scale = uv_a[1]
        .abs()
        .max(map.v_phase.abs())
        .max(b_range[1].lo.abs())
        .max(b_range[1].hi.abs())
        .max(1.0);
    let v_roundoff = if map.v_phase == 0.0 {
        0.0
    } else {
        finite_interval_upper(Interval::point(2.0 * f64::EPSILON) * Interval::point(v_scale))?
    };
    let u_tolerance = tolerances.angular() + u_roundoff;
    let v_tolerance = tolerances.linear() + v_roundoff;
    let Some(u_b) = fit_scalar_parameter(mapped_u, b_range[0], u_tolerance) else {
        return Err(Error::InvalidGeometry {
            reason: "coincident cone chart overlap did not lift into both source windows",
        });
    };
    let Some(v_b) = fit_scalar_parameter(mapped_v, b_range[1], v_tolerance) else {
        return Err(Error::InvalidGeometry {
            reason: "coincident cone chart overlap did not lift into both source windows",
        });
    };
    paired_cone_sample_at(a, uv_a, b, [u_b, v_b])
}

fn paired_cone_sample_at(
    a: &Cone,
    uv_a: [f64; 2],
    b: &Cone,
    uv_b: [f64; 2],
) -> Result<PairedConeSample> {
    let pa = a.eval(uv_a);
    let pb = b.eval(uv_b);
    let residual = pa.dist(pb);
    let residual_bound =
        conservative_cone_point_distance(pa, pb).ok_or(Error::InvalidGeometry {
            reason: "coincident cone residual arithmetic is non-finite",
        })?;
    Ok(PairedConeSample {
        point: (pa + pb) / 2.0,
        uv_a,
        uv_b,
        residual,
        residual_bound,
    })
}

fn coincident_cone_apex_point(
    a: &Cone,
    a_range: [ParamRange; 2],
    b: &Cone,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfacePoint> {
    let v_a = fit_scalar_parameter(a.apex_v(), a_range[1], tolerances.linear()).ok_or(
        Error::InvalidGeometry {
            reason: "coincident cone apex did not fit inside the first source window",
        },
    )?;
    let v_b = fit_scalar_parameter(b.apex_v(), b_range[1], tolerances.linear()).ok_or(
        Error::InvalidGeometry {
            reason: "coincident cone apex did not fit inside the second source window",
        },
    )?;
    let sample = paired_cone_sample_at(a, [a_range[0].lo, v_a], b, [b_range[0].lo, v_b])?;
    Ok(surface_point_from_cone_sample(
        sample,
        ContactKind::Singular,
    ))
}

fn surface_point_from_cone_sample(
    sample: PairedConeSample,
    kind: ContactKind,
) -> SurfaceSurfacePoint {
    SurfaceSurfacePoint {
        point: sample.point,
        uv_a: sample.uv_a,
        uv_b: sample.uv_b,
        residual: sample.residual,
        kind,
    }
}

fn coincident_cone_whole_residual_bound(
    a: &Cone,
    b: &Cone,
    u: PeriodicOverlapPiece,
    v: ParamRange,
    b_range: [ParamRange; 2],
    map: CoincidentConeMap,
) -> Result<f64> {
    let (sin_phase, cos_phase) = math::sincos(map.u_phase);
    let (sin_angle, cos_angle) = math::sincos(a.half_angle());
    let b_cos = b.frame().x() * cos_phase + b.frame().y() * sin_phase;
    let b_sin = (b.frame().y() * cos_phase - b.frame().x() * sin_phase) * map.sign;
    let mapped_radius = b.radius() + sin_angle * map.v_phase;
    let origin_difference =
        a.frame().origin() - (b.frame().origin() + b.frame().z() * (cos_angle * map.v_phase));
    let cosine_difference = a.frame().x() * a.radius() - b_cos * mapped_radius;
    let sine_difference = a.frame().y() * a.radius() - b_sin * mapped_radius;
    let v_cosine_difference = a.frame().x() * sin_angle - b_cos * (map.sign * sin_angle);
    let v_sine_difference = a.frame().y() * sin_angle - b_sin * (map.sign * sin_angle);
    let v_axis_difference = a.frame().z() * cos_angle - b.frame().z() * (map.sign * cos_angle);
    let max_v = v.lo.abs().max(v.hi.abs());

    let mut bound = Interval::point(conservative_cone_vec_norm(origin_difference)?);
    bound = bound + Interval::point(conservative_cone_vec_norm(cosine_difference)?);
    bound = bound + Interval::point(conservative_cone_vec_norm(sine_difference)?);
    let v_coefficient = Interval::point(conservative_cone_vec_norm(v_cosine_difference)?)
        + Interval::point(conservative_cone_vec_norm(v_sine_difference)?)
        + Interval::point(conservative_cone_vec_norm(v_axis_difference)?);
    bound = bound + v_coefficient * Interval::point(max_v);

    let periodic_error =
        coincident_cone_periodic_map_error([u.a, v], [b_range[0], b_range[1]], map)?;
    let scalar_error = coincident_cone_scalar_map_error([u.a, v], [b_range[0], b_range[1]], map)?;
    let rho = Interval::point(a.radius()) + Interval::new(v.lo, v.hi) * Interval::point(sin_angle);
    let max_rho = rho.lo().abs().max(rho.hi().abs());
    bound = bound
        + Interval::point(max_rho) * Interval::point(periodic_error)
        + Interval::point(scalar_error);

    // Each evaluator forms two deterministic trigonometric outputs, one
    // affine radius, a three-axis radial lift, an axial lift, and their sums.
    // The paired chart map adds two affine operations. Fewer than 512 rounded
    // operations/transcendental ulp errors affect any model coordinate. Using
    // EPSILON rather than unit roundoff makes gamma_512 conservative; the
    // separate phase/scalar terms above retain their domain-scale dependence.
    const ERROR_UNITS: f64 = 512.0;
    let gamma = (ERROR_UNITS * f64::EPSILON) / (1.0 - ERROR_UNITS * f64::EPSILON);
    let origin_scale = a
        .frame()
        .origin()
        .to_array()
        .into_iter()
        .chain(b.frame().origin().to_array())
        .map(f64::abs)
        .fold(0.0_f64, f64::max);
    let model_scale = Interval::point(origin_scale)
        + Interval::point(a.radius().max(b.radius()))
        + Interval::point(4.0 * max_v.max(map.v_phase.abs()));
    let lift_error = Interval::point(2.0 * 3.0_f64.sqrt()) * Interval::point(gamma) * model_scale;
    let underflow_error = Interval::point(ERROR_UNITS * f64::from_bits(1));
    finite_interval_upper(bound + lift_error + underflow_error)
}

fn conservative_cone_vec_norm(value: Vec3) -> Result<f64> {
    let components = value.to_array().map(Interval::point);
    let squared = components
        .into_iter()
        .fold(Interval::point(0.0), |sum, value| sum + value.square());
    squared
        .sqrt()
        .map(Interval::hi)
        .filter(|bound| bound.is_finite())
        .ok_or(Error::InvalidGeometry {
            reason: "coincident cone coefficient residual bound is non-finite",
        })
}

fn conservative_cone_point_distance(a: Point3, b: Point3) -> Option<f64> {
    let difference = [
        Interval::point(a.x) - Interval::point(b.x),
        Interval::point(a.y) - Interval::point(b.y),
        Interval::point(a.z) - Interval::point(b.z),
    ];
    difference
        .into_iter()
        .fold(Interval::point(0.0), |sum, value| sum + value.square())
        .sqrt()
        .map(Interval::hi)
        .filter(|bound| bound.is_finite())
}

fn finite_interval_upper(interval: Interval) -> Result<f64> {
    interval
        .hi()
        .is_finite()
        .then_some(interval.hi())
        .ok_or(Error::InvalidGeometry {
            reason: "coincident cone interval proof is non-finite",
        })
}

#[derive(Clone, Copy)]
enum MeridianRoot {
    Circle { radius: f64, z: f64 },
    Apex { z: f64 },
}

#[derive(Clone, Copy)]
struct MeridianLine {
    slope: f64,
    intercept: f64,
}

fn meridian_roots(
    a: &Cone,
    b: &Cone,
    b_origin_z: f64,
    tolerances: Tolerances,
) -> Result<Vec<MeridianRoot>> {
    let (_, cos_a) = math::sincos(a.half_angle());
    let (_, cos_b) = math::sincos(b.half_angle());
    let tan_a = math::sin(a.half_angle()) / cos_a;
    let tan_b = math::sin(b.half_angle()) / cos_b;
    let axis_sign = if a.frame().z().dot(b.frame().z()) < 0.0 {
        -1.0
    } else {
        1.0
    };

    let mut roots = Vec::new();
    let scale = (a.radius() + b.radius() + b_origin_z.abs()).max(1.0);
    for sign_a in [-1.0, 1.0] {
        let line_a = MeridianLine {
            slope: sign_a * tan_a,
            intercept: sign_a * a.radius(),
        };
        for sign_b in [-1.0, 1.0] {
            let line_b = MeridianLine {
                slope: sign_b * axis_sign * tan_b,
                intercept: sign_b * (b.radius() - axis_sign * b_origin_z * tan_b),
            };
            push_line_intersection(&mut roots, line_a, line_b, scale, tolerances)?;
        }
    }
    roots.sort_by(|a, b| {
        root_z(*a)
            .total_cmp(&root_z(*b))
            .then(root_radius(*a).total_cmp(&root_radius(*b)))
    });
    Ok(roots)
}

fn push_line_intersection(
    roots: &mut Vec<MeridianRoot>,
    a: MeridianLine,
    b: MeridianLine,
    scale: f64,
    tolerances: Tolerances,
) -> Result<()> {
    let denom = a.slope - b.slope;
    let rhs = b.intercept - a.intercept;
    if denom.abs() <= tolerances.angular() {
        if rhs.abs() <= tolerances.linear() * scale {
            return Err(Error::InvalidGeometry {
                reason: "near-coincident or unsupported coincident cones require the general certified fallback",
            });
        }
        return Ok(());
    }

    let z = rhs / denom;
    let radius = a.slope * z + a.intercept;
    if radius < -tolerances.linear() {
        return Ok(());
    }
    if radius <= tolerances.linear() {
        push_apex_root(roots, z, tolerances);
    } else {
        push_circle_root(roots, radius, z, tolerances);
    }
    Ok(())
}

fn push_circle_root(roots: &mut Vec<MeridianRoot>, radius: f64, z: f64, tolerances: Tolerances) {
    if !roots.iter().any(|root| match *root {
        MeridianRoot::Circle {
            radius: other_radius,
            z: other_z,
        } => {
            (radius - other_radius).abs() <= tolerances.linear()
                && (z - other_z).abs() <= tolerances.linear()
        }
        MeridianRoot::Apex { .. } => false,
    }) {
        roots.push(MeridianRoot::Circle { radius, z });
    }
}

fn push_apex_root(roots: &mut Vec<MeridianRoot>, z: f64, tolerances: Tolerances) {
    if !roots.iter().any(|root| match *root {
        MeridianRoot::Apex { z: other_z } => (z - other_z).abs() <= tolerances.linear(),
        MeridianRoot::Circle { radius, z: other_z } => {
            radius <= tolerances.linear() && (z - other_z).abs() <= tolerances.linear()
        }
    }) {
        roots.push(MeridianRoot::Apex { z });
    }
}

fn root_z(root: MeridianRoot) -> f64 {
    match root {
        MeridianRoot::Circle { z, .. } | MeridianRoot::Apex { z } => z,
    }
}

fn root_radius(root: MeridianRoot) -> f64 {
    match root {
        MeridianRoot::Circle { radius, .. } => radius,
        MeridianRoot::Apex { .. } => 0.0,
    }
}

#[allow(clippy::too_many_arguments)]
fn add_circle_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    radius: f64,
    z: f64,
    a: &Cone,
    a_range: [ParamRange; 2],
    b: &Cone,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<()> {
    let center = a.frame().origin() + a.frame().z() * z;
    let circle = Circle::new(Frame::new(center, a.frame().z(), a.frame().x())?, radius)?;
    let a_hit =
        intersect_bounded_circle_cone(&circle, circle.param_range(), a, a_range, tolerances)?;
    let b_hit =
        intersect_bounded_circle_cone(&circle, circle.param_range(), b, b_range, tolerances)?;
    let parameter_tolerance = parameter_tolerance(circle.radius(), tolerances);
    let curve = SurfaceIntersectionCurve::Circle(circle);
    let first_uv = |point| cone_uv_at(point, a, a_range, tolerances);
    let second_uv = |point| cone_uv_at(point, b, b_range, tolerances);
    emit_support_curve_pair(
        SupportCurvePairConfig {
            curve: &curve,
            curve_range: curve.param_range(),
            first_hit: &a_hit,
            second_hit: &b_hit,
            kind: ContactKind::Transverse,
            parameter_tolerance,
            parameter_period: Some(core::f64::consts::TAU),
            branch_tolerance: parameter_tolerance.max(tolerances.linear()),
            first_surface: a,
            second_surface: b,
            first_uv: &first_uv,
            second_uv: &second_uv,
            tolerances,
        },
        points,
        curves,
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn add_point(
    points: &mut Vec<SurfaceSurfacePoint>,
    point: Point3,
    a: &Cone,
    a_range: [ParamRange; 2],
    b: &Cone,
    b_range: [ParamRange; 2],
    kind: ContactKind,
    tolerances: Tolerances,
) {
    let Some(uv_a) = cone_uv_at(point, a, a_range, tolerances) else {
        return;
    };
    let Some(uv_b) = cone_uv_at(point, b, b_range, tolerances) else {
        return;
    };
    let kind = if a.normal(uv_a).is_none() || b.normal(uv_b).is_none() {
        ContactKind::Singular
    } else {
        kind
    };
    if let Some(point) = accept_surface_surface_candidate(a, uv_a, b, uv_b, kind, tolerances) {
        push_point(points, point, tolerances);
    }
}

fn cone_uv_at(
    point: Point3,
    cone: &Cone,
    cone_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let local = cone.frame().to_local(point);
    let (sin_a, cos_a) = math::sincos(cone.half_angle());
    let v = fit_scalar_parameter(local.z / cos_a, cone_range[1], tolerances.linear())?;
    let signed_radius = cone.radius() + v * sin_a;
    let u = if signed_radius.abs() <= tolerances.linear() {
        cone_range[0].lo
    } else {
        let raw_u = math::atan2(local.y / signed_radius, local.x / signed_radius);
        fit_periodic_parameter(
            raw_u,
            cone_range[0],
            parameter_tolerance(signed_radius.abs(), tolerances),
        )?
    };
    Some([u, v])
}

fn push_point(
    points: &mut Vec<SurfaceSurfacePoint>,
    candidate: SurfaceSurfacePoint,
    tolerances: Tolerances,
) {
    if !points
        .iter()
        .any(|point| point.point.dist(candidate.point) <= tolerances.linear())
    {
        points.push(candidate);
    }
}

fn validate_ranges(a_range: [ParamRange; 2], b_range: [ParamRange; 2]) -> Result<()> {
    if a_range
        .iter()
        .any(|range| !range.is_finite() || !range.width().is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "cone/cone intersection requires finite non-reversed first-cone ranges",
        });
    }
    if b_range
        .iter()
        .any(|range| !range.is_finite() || !range.width().is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "cone/cone intersection requires finite non-reversed second-cone ranges",
        });
    }
    Ok(())
}
