use super::circle_torus::intersect_bounded_circle_torus;
use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::parameter::{
    PeriodicOverlapPiece, periodic_preimage_overlaps, range_midpoint, validate_period_span,
};
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfaceOverlap, CurveSurfacePoint,
    SurfaceIntersectionCurve, SurfaceRegionCorrespondence, SurfaceRegionOrientation,
    SurfaceSurfaceCurve, SurfaceSurfaceIntersections, SurfaceSurfacePoint, SurfaceSurfaceRegion,
    SurfaceSurfaceRegionVertex, accept_surface_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::interval::Interval;
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Surface, Torus};
use kgeom::vec::{Point3, Vec3};

/// Intersect two finite torus parameter windows.
///
/// Exact coincident tori return paired finite-window regions, including
/// independent seam splitting in both periodic parameters and exact
/// lower-dimensional circle/point collapses. Noncoincident coaxial tori use a
/// circle/circle meridian reduction; every accepted meridian point revolves
/// into an exact latitude circle. General offset or skew torus/torus
/// intersections remain explicit until SSI result geometry can carry those
/// higher-order branch families.
pub fn intersect_bounded_tori(
    a: &Torus,
    a_range: [ParamRange; 2],
    b: &Torus,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(a_range, b_range)?;

    if let Some(map) = exact_coincident_torus_map(a, b) {
        if compare_torus_windows(a, a_range, b, b_range).is_gt() {
            return intersect_bounded_tori(b, b_range, a, a_range, tolerances)
                .map(SurfaceSurfaceIntersections::swapped);
        }
        return intersect_coincident_torus_windows(a, a_range, b, b_range, map, tolerances);
    }

    let axis = a.frame().z();
    if axis.cross(b.frame().z()).norm() > tolerances.angular() {
        return Err(Error::InvalidGeometry {
            reason: "torus/torus intersection currently supports only coaxial circular cuts",
        });
    }

    let offset = b.frame().origin() - a.frame().origin();
    let radial_offset = offset - axis * offset.dot(axis);
    if radial_offset.norm() > tolerances.linear() {
        return Err(Error::InvalidGeometry {
            reason: "torus/torus intersection currently supports only coaxial circular cuts",
        });
    }

    let roots = meridian_roots(a, b, offset.dot(axis), tolerances)?;
    let mut points = Vec::new();
    let mut curves = Vec::new();
    for root in roots {
        add_circle_branch(
            &mut points,
            &mut curves,
            root.radius,
            root.z,
            root.kind,
            a,
            a_range,
            b,
            b_range,
            tolerances,
        )?;
    }

    SurfaceSurfaceIntersections::canonicalized_complete(points, curves)
}

fn compare_torus_windows(
    a: &Torus,
    a_range: [ParamRange; 2],
    b: &Torus,
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
            a.major_radius(),
            a.minor_radius(),
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
            b.major_radius(),
            b.minor_radius(),
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
struct CoincidentTorusMap {
    sign: f64,
    u_phase: f64,
}

fn exact_coincident_torus_map(a: &Torus, b: &Torus) -> Option<CoincidentTorusMap> {
    if a.frame().origin() != b.frame().origin()
        || a.major_radius() != b.major_radius()
        || a.minor_radius() != b.minor_radius()
    {
        return None;
    }
    let sign = if b.frame().z() == a.frame().z() {
        1.0
    } else if b.frame().z() == -a.frame().z() {
        -1.0
    } else {
        return None;
    };
    let u_phase = math::atan2(
        a.frame().x().dot(b.frame().y()),
        a.frame().x().dot(b.frame().x()),
    );
    Some(CoincidentTorusMap { sign, u_phase })
}

#[derive(Clone, Copy, Debug)]
struct PairedTorusSample {
    point: Point3,
    uv_a: [f64; 2],
    uv_b: [f64; 2],
    residual: f64,
    residual_bound: f64,
}

#[allow(clippy::too_many_arguments)]
fn intersect_coincident_torus_windows(
    a: &Torus,
    a_range: [ParamRange; 2],
    b: &Torus,
    b_range: [ParamRange; 2],
    map: CoincidentTorusMap,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let tau = core::f64::consts::TAU;
    validate_period_span(
        a_range[0],
        tau,
        0.0,
        "coincident torus first longitude window cannot span more than one turn",
    )?;
    validate_period_span(
        a_range[1],
        tau,
        0.0,
        "coincident torus first latitude window cannot span more than one turn",
    )?;
    validate_period_span(
        b_range[0],
        tau,
        0.0,
        "coincident torus second longitude window cannot span more than one turn",
    )?;
    validate_period_span(
        b_range[1],
        tau,
        0.0,
        "coincident torus second latitude window cannot span more than one turn",
    )?;
    validate_coincident_torus_map_corridor(a, a_range, b, b_range, map, tolerances)?;

    let u_tolerance = parameter_tolerance(a.major_radius() - a.minor_radius(), tolerances);
    let v_tolerance = parameter_tolerance(a.minor_radius(), tolerances);
    let u_overlaps = periodic_preimage_overlaps(
        a_range[0],
        b_range[0],
        map.sign,
        map.u_phase,
        tau,
        u_tolerance,
        "coincident torus longitude chart shift is outside the exact integer corridor",
    )?;
    let v_overlaps = periodic_preimage_overlaps(
        a_range[1],
        b_range[1],
        map.sign,
        0.0,
        tau,
        v_tolerance,
        "coincident torus latitude chart shift is outside the exact integer corridor",
    )?;
    if u_overlaps.is_empty() || v_overlaps.is_empty() {
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    }

    let mut points = Vec::new();
    let mut curves = Vec::new();
    let mut regions = Vec::new();
    for u in u_overlaps {
        for v in &v_overlaps {
            let u_positive = u.a.width() > u_tolerance;
            let v_positive = v.a.width() > v_tolerance;
            match (u_positive, v_positive) {
                (true, true) => regions.push(coincident_torus_region(
                    a, b, u, *v, b_range, map, tolerances,
                )?),
                (true, false) => push_curve(
                    &mut curves,
                    coincident_torus_latitude_branch(
                        a,
                        b,
                        u,
                        range_midpoint(v.a),
                        v.shift,
                        b_range,
                        map,
                        tolerances,
                    )?,
                    u_tolerance.max(tolerances.linear()),
                ),
                (false, true) => push_curve(
                    &mut curves,
                    coincident_torus_meridian_branch(
                        a,
                        b,
                        range_midpoint(u.a),
                        u.shift,
                        *v,
                        b_range,
                        map,
                        tolerances,
                    )?,
                    v_tolerance.max(tolerances.linear()),
                ),
                (false, false) => {
                    let sample = paired_torus_sample(
                        a,
                        b,
                        [range_midpoint(u.a), range_midpoint(v.a)],
                        [u.shift, v.shift],
                        b_range,
                        map,
                        tolerances,
                    )?;
                    push_point(
                        &mut points,
                        surface_point_from_torus_sample(sample),
                        tolerances,
                    );
                }
            }
        }
    }
    SurfaceSurfaceIntersections::canonicalized_complete_with_regions(points, curves, regions)
}

fn validate_coincident_torus_map_corridor(
    a: &Torus,
    a_range: [ParamRange; 2],
    b: &Torus,
    b_range: [ParamRange; 2],
    map: CoincidentTorusMap,
    tolerances: Tolerances,
) -> Result<()> {
    if a == b && a_range == b_range {
        return Ok(());
    }
    let u_error = coincident_torus_periodic_map_error(a_range[0], b_range[0], map.u_phase)?;
    let v_error = coincident_torus_periodic_map_error(a_range[1], b_range[1], 0.0)?;
    if u_error > tolerances.angular() || v_error > tolerances.angular() {
        return Err(Error::InvalidGeometry {
            reason: "coincident torus chart map exceeds the certified parameter-roundoff corridor",
        });
    }
    Ok(())
}

fn coincident_torus_periodic_map_error(
    a_range: ParamRange,
    b_range: ParamRange,
    phase: f64,
) -> Result<f64> {
    let scale = [
        a_range.lo.abs(),
        a_range.hi.abs(),
        b_range.lo.abs(),
        b_range.hi.abs(),
        phase.abs(),
        2.0 * core::f64::consts::TAU,
    ]
    .into_iter()
    .fold(1.0_f64, f64::max);
    finite_torus_interval_upper(Interval::point(4.0 * f64::EPSILON) * Interval::point(scale))
}

#[allow(clippy::too_many_arguments)]
fn coincident_torus_region(
    a: &Torus,
    b: &Torus,
    u: PeriodicOverlapPiece,
    v: PeriodicOverlapPiece,
    b_range: [ParamRange; 2],
    map: CoincidentTorusMap,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceRegion> {
    let mut boundary = Vec::with_capacity(4);
    let mut max_residual = coincident_torus_whole_residual_bound(a, b, u, v, b_range, map)?;
    for uv_a in [
        [u.a.lo, v.a.lo],
        [u.a.hi, v.a.lo],
        [u.a.hi, v.a.hi],
        [u.a.lo, v.a.hi],
    ] {
        let sample = paired_torus_sample(a, b, uv_a, [u.shift, v.shift], b_range, map, tolerances)?;
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
fn coincident_torus_latitude_branch(
    a: &Torus,
    b: &Torus,
    u: PeriodicOverlapPiece,
    v: f64,
    v_shift: f64,
    b_range: [ParamRange; 2],
    map: CoincidentTorusMap,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceCurve> {
    let start = paired_torus_sample(
        a,
        b,
        [u.a.lo, v],
        [u.shift, v_shift],
        b_range,
        map,
        tolerances,
    )?;
    let end = paired_torus_sample(
        a,
        b,
        [u.a.hi, v],
        [u.shift, v_shift],
        b_range,
        map,
        tolerances,
    )?;
    let (sin_v, cos_v) = math::sincos(v);
    let center = a.frame().origin() + a.frame().z() * (a.minor_radius() * sin_v);
    let radius = a.major_radius() + a.minor_radius() * cos_v;
    let circle = Circle::new(Frame::new(center, a.frame().z(), a.frame().x())?, radius)?;
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
fn coincident_torus_meridian_branch(
    a: &Torus,
    b: &Torus,
    u: f64,
    u_shift: f64,
    v: PeriodicOverlapPiece,
    b_range: [ParamRange; 2],
    map: CoincidentTorusMap,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceCurve> {
    let start = paired_torus_sample(
        a,
        b,
        [u, v.a.lo],
        [u_shift, v.shift],
        b_range,
        map,
        tolerances,
    )?;
    let end = paired_torus_sample(
        a,
        b,
        [u, v.a.hi],
        [u_shift, v.shift],
        b_range,
        map,
        tolerances,
    )?;
    let (sin_u, cos_u) = math::sincos(u);
    let radial = a.frame().x() * cos_u + a.frame().y() * sin_u;
    let tangent = a.frame().y() * cos_u - a.frame().x() * sin_u;
    let center = a.frame().origin() + radial * a.major_radius();
    let circle = Circle::new(Frame::new(center, -tangent, radial)?, a.minor_radius())?;
    Ok(SurfaceSurfaceCurve {
        curve: SurfaceIntersectionCurve::Circle(circle),
        curve_range: v.a,
        uv_a_start: start.uv_a,
        uv_a_end: end.uv_a,
        uv_b_start: start.uv_b,
        uv_b_end: end.uv_b,
        kind: ContactKind::Tangent,
    })
}

#[allow(clippy::too_many_arguments)]
fn paired_torus_sample(
    a: &Torus,
    b: &Torus,
    uv_a: [f64; 2],
    shifts: [f64; 2],
    b_range: [ParamRange; 2],
    map: CoincidentTorusMap,
    tolerances: Tolerances,
) -> Result<PairedTorusSample> {
    let mapped = [
        map.sign * uv_a[0] + map.u_phase + shifts[0],
        map.sign * uv_a[1] + shifts[1],
    ];
    let u_roundoff =
        coincident_torus_sample_map_error(uv_a[0], b_range[0], map.u_phase, shifts[0])?;
    let v_roundoff = coincident_torus_sample_map_error(uv_a[1], b_range[1], 0.0, shifts[1])?;
    let Some(u_b) = fit_scalar_parameter(
        mapped[0],
        b_range[0],
        parameter_tolerance(b.major_radius() - b.minor_radius(), tolerances) + u_roundoff,
    ) else {
        return Err(Error::InvalidGeometry {
            reason: "coincident torus chart overlap did not lift into the second longitude window",
        });
    };
    let Some(v_b) = fit_scalar_parameter(
        mapped[1],
        b_range[1],
        parameter_tolerance(b.minor_radius(), tolerances) + v_roundoff,
    ) else {
        return Err(Error::InvalidGeometry {
            reason: "coincident torus chart overlap did not lift into the second latitude window",
        });
    };
    let uv_b = [u_b, v_b];
    let pa = a.eval(uv_a);
    let pb = b.eval(uv_b);
    let residual = pa.dist(pb);
    let residual_bound =
        conservative_torus_point_distance(pa, pb).ok_or(Error::InvalidGeometry {
            reason: "coincident torus residual arithmetic is non-finite",
        })?;
    Ok(PairedTorusSample {
        point: (pa + pb) / 2.0,
        uv_a,
        uv_b,
        residual,
        residual_bound,
    })
}

fn coincident_torus_sample_map_error(
    parameter: f64,
    target: ParamRange,
    phase: f64,
    shift: f64,
) -> Result<f64> {
    let scale = parameter
        .abs()
        .max(target.lo.abs())
        .max(target.hi.abs())
        .max(phase.abs())
        .max(shift.abs())
        .max(1.0);
    finite_torus_interval_upper(Interval::point(4.0 * f64::EPSILON) * Interval::point(scale))
}

fn surface_point_from_torus_sample(sample: PairedTorusSample) -> SurfaceSurfacePoint {
    SurfaceSurfacePoint {
        point: sample.point,
        uv_a: sample.uv_a,
        uv_b: sample.uv_b,
        residual: sample.residual,
        kind: ContactKind::Tangent,
    }
}

fn coincident_torus_whole_residual_bound(
    a: &Torus,
    b: &Torus,
    u: PeriodicOverlapPiece,
    v: PeriodicOverlapPiece,
    b_range: [ParamRange; 2],
    map: CoincidentTorusMap,
) -> Result<f64> {
    let (sin_phase, cos_phase) = math::sincos(map.u_phase);
    let b_cos = b.frame().x() * cos_phase + b.frame().y() * sin_phase;
    let b_sin = (b.frame().y() * cos_phase - b.frame().x() * sin_phase) * map.sign;
    let origin_difference = a.frame().origin() - b.frame().origin();
    let major_cosine_difference = a.frame().x() * a.major_radius() - b_cos * b.major_radius();
    let major_sine_difference = a.frame().y() * a.major_radius() - b_sin * b.major_radius();
    let minor_cosine_cosine_difference =
        a.frame().x() * a.minor_radius() - b_cos * b.minor_radius();
    let minor_cosine_sine_difference = a.frame().y() * a.minor_radius() - b_sin * b.minor_radius();
    let minor_sine_difference =
        a.frame().z() * a.minor_radius() - b.frame().z() * (map.sign * b.minor_radius());

    let mut bound = Interval::point(conservative_torus_vec_norm(origin_difference)?);
    for coefficient in [
        major_cosine_difference,
        major_sine_difference,
        minor_cosine_cosine_difference,
        minor_cosine_sine_difference,
        minor_sine_difference,
    ] {
        bound = bound + Interval::point(conservative_torus_vec_norm(coefficient)?);
    }

    let u_error = coincident_torus_periodic_map_error(u.a, b_range[0], map.u_phase)?;
    let v_error = coincident_torus_periodic_map_error(v.a, b_range[1], 0.0)?;
    bound = bound
        + Interval::point(a.major_radius() + a.minor_radius()) * Interval::point(u_error)
        + Interval::point(a.minor_radius()) * Interval::point(v_error);

    let parameter_scale =
        u.a.lo
            .abs()
            .max(u.a.hi.abs())
            .max(v.a.lo.abs())
            .max(v.a.hi.abs())
            .max(b_range[0].lo.abs())
            .max(b_range[0].hi.abs())
            .max(b_range[1].lo.abs())
            .max(b_range[1].hi.abs())
            .max(u.shift.abs())
            .max(v.shift.abs());
    let origin_scale = a
        .frame()
        .origin()
        .to_array()
        .into_iter()
        .chain(b.frame().origin().to_array())
        .map(f64::abs)
        .fold(0.0_f64, f64::max);
    let model_scale = origin_scale
        .max(a.major_radius() + a.minor_radius())
        .max(parameter_scale)
        .max(1.0);
    let rounding = Interval::point(4096.0 * f64::EPSILON) * Interval::point(model_scale);
    let underflow = Interval::point(4096.0 * f64::from_bits(1));
    finite_torus_interval_upper(bound + rounding + underflow)
}

fn conservative_torus_vec_norm(value: Vec3) -> Result<f64> {
    let components = value.to_array().map(Interval::point);
    let squared = components
        .into_iter()
        .fold(Interval::point(0.0), |sum, value| sum + value.square());
    squared
        .sqrt()
        .map(Interval::hi)
        .filter(|bound| bound.is_finite())
        .ok_or(Error::InvalidGeometry {
            reason: "coincident torus coefficient residual bound is non-finite",
        })
}

fn conservative_torus_point_distance(a: Point3, b: Point3) -> Option<f64> {
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

fn finite_torus_interval_upper(interval: Interval) -> Result<f64> {
    interval
        .hi()
        .is_finite()
        .then_some(interval.hi())
        .ok_or(Error::InvalidGeometry {
            reason: "coincident torus interval proof is non-finite",
        })
}

#[derive(Clone, Copy)]
struct MeridianRoot {
    radius: f64,
    z: f64,
    kind: ContactKind,
}

fn meridian_roots(
    a: &Torus,
    b: &Torus,
    b_origin_z: f64,
    tolerances: Tolerances,
) -> Result<Vec<MeridianRoot>> {
    let delta_radius = b.major_radius() - a.major_radius();
    let distance = (delta_radius * delta_radius + b_origin_z * b_origin_z).sqrt();
    let minor_a = a.minor_radius();
    let minor_b = b.minor_radius();

    if distance <= tolerances.linear() {
        if (minor_a - minor_b).abs() <= tolerances.linear() {
            return Err(Error::InvalidGeometry {
                reason: "near-coincident non-identical tori require the general certified fallback",
            });
        }
        return Ok(Vec::new());
    }

    if distance > minor_a + minor_b + tolerances.linear()
        || distance < (minor_a - minor_b).abs() - tolerances.linear()
    {
        return Ok(Vec::new());
    }

    let along = (minor_a * minor_a - minor_b * minor_b + distance * distance) / (2.0 * distance);
    let h_sq = minor_a * minor_a - along * along;
    let sq_tol = squared_tolerance(distance, minor_a, minor_b, tolerances);
    if h_sq < -sq_tol {
        return Ok(Vec::new());
    }

    let e_radius = delta_radius / distance;
    let e_z = b_origin_z / distance;
    let base_radius = a.major_radius() + along * e_radius;
    let base_z = along * e_z;
    let normal_radius = -e_z;
    let normal_z = e_radius;

    let mut roots = Vec::new();
    if h_sq <= sq_tol {
        push_meridian_root(
            &mut roots,
            base_radius,
            base_z,
            ContactKind::Tangent,
            tolerances,
        );
    } else {
        let h = h_sq.sqrt();
        for sign in [-1.0, 1.0] {
            push_meridian_root(
                &mut roots,
                base_radius + sign * h * normal_radius,
                base_z + sign * h * normal_z,
                ContactKind::Transverse,
                tolerances,
            );
        }
    }
    roots.sort_by(|a, b| {
        a.z.total_cmp(&b.z)
            .then(a.radius.total_cmp(&b.radius))
            .then(a.kind.cmp(&b.kind))
    });
    Ok(roots)
}

fn push_meridian_root(
    roots: &mut Vec<MeridianRoot>,
    radius: f64,
    z: f64,
    kind: ContactKind,
    tolerances: Tolerances,
) {
    if radius <= tolerances.linear() {
        return;
    }
    let candidate = MeridianRoot { radius, z, kind };
    if !roots.iter().any(|root| {
        (root.radius - candidate.radius).abs() <= tolerances.linear()
            && (root.z - candidate.z).abs() <= tolerances.linear()
    }) {
        roots.push(candidate);
    }
}

#[allow(clippy::too_many_arguments)]
fn add_circle_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    radius: f64,
    z: f64,
    branch_kind: ContactKind,
    a: &Torus,
    a_range: [ParamRange; 2],
    b: &Torus,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<()> {
    let center = a.frame().origin() + a.frame().z() * z;
    let circle = Circle::new(Frame::new(center, a.frame().z(), a.frame().x())?, radius)?;
    let a_hit =
        intersect_bounded_circle_torus(&circle, circle.param_range(), a, a_range, tolerances)?;
    let b_hit =
        intersect_bounded_circle_torus(&circle, circle.param_range(), b, b_range, tolerances)?;
    add_clipped_branch(
        points,
        curves,
        &circle,
        &a_hit,
        &b_hit,
        branch_kind,
        a,
        a_range,
        b,
        b_range,
        tolerances,
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn add_clipped_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    circle: &Circle,
    a_hit: &CurveSurfaceIntersections,
    b_hit: &CurveSurfaceIntersections,
    branch_kind: ContactKind,
    a: &Torus,
    a_range: [ParamRange; 2],
    b: &Torus,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) {
    let t_tol = parameter_tolerance(circle.radius(), tolerances);
    for a_overlap in &a_hit.overlaps {
        for b_overlap in &b_hit.overlaps {
            let lo = a_overlap.curve.lo.max(b_overlap.curve.lo);
            let hi = a_overlap.curve.hi.min(b_overlap.curve.hi);
            if hi - lo > t_tol {
                let Some(uv_a_start) = torus_uv_at(circle.eval(lo), a, a_range, tolerances) else {
                    continue;
                };
                let Some(uv_a_end) = torus_uv_at(circle.eval(hi), a, a_range, tolerances) else {
                    continue;
                };
                let Some(uv_b_start) = torus_uv_at(circle.eval(lo), b, b_range, tolerances) else {
                    continue;
                };
                let Some(uv_b_end) = torus_uv_at(circle.eval(hi), b, b_range, tolerances) else {
                    continue;
                };
                push_curve(
                    curves,
                    SurfaceSurfaceCurve {
                        curve: SurfaceIntersectionCurve::Circle(*circle),
                        curve_range: ParamRange::new(lo, hi),
                        uv_a_start,
                        uv_a_end,
                        uv_b_start,
                        uv_b_end,
                        kind: branch_kind,
                    },
                    t_tol.max(tolerances.linear()),
                );
            } else if (hi - lo).abs() <= t_tol {
                add_point_from_curve_parameter(
                    points,
                    circle,
                    ((lo + hi) / 2.0).clamp(circle.param_range().lo, circle.param_range().hi),
                    branch_kind,
                    a,
                    a_range,
                    b,
                    b_range,
                    t_tol,
                    tolerances,
                );
            }
        }
    }

    add_isolated_points(
        points,
        circle,
        a_hit,
        b_hit,
        branch_kind,
        a,
        a_range,
        b,
        b_range,
        t_tol,
        tolerances,
    );
}

#[allow(clippy::too_many_arguments)]
fn add_isolated_points(
    points: &mut Vec<SurfaceSurfacePoint>,
    circle: &Circle,
    a_hit: &CurveSurfaceIntersections,
    b_hit: &CurveSurfaceIntersections,
    branch_kind: ContactKind,
    a: &Torus,
    a_range: [ParamRange; 2],
    b: &Torus,
    b_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    for point in &a_hit.points {
        if hit_contains_t(b_hit, point.t_curve, t_tol, tolerances) {
            add_point_from_curve_parameter(
                points,
                circle,
                point.t_curve,
                branch_kind,
                a,
                a_range,
                b,
                b_range,
                t_tol,
                tolerances,
            );
        }
    }
    for point in &b_hit.points {
        if hit_contains_t(a_hit, point.t_curve, t_tol, tolerances) {
            add_point_from_curve_parameter(
                points,
                circle,
                point.t_curve,
                branch_kind,
                a,
                a_range,
                b,
                b_range,
                t_tol,
                tolerances,
            );
        }
    }
    for a_point in &a_hit.points {
        for b_point in &b_hit.points {
            if curve_parameters_match(a_point, b_point, t_tol, tolerances) {
                add_point_from_curve_parameter(
                    points,
                    circle,
                    a_point.t_curve,
                    branch_kind,
                    a,
                    a_range,
                    b,
                    b_range,
                    t_tol,
                    tolerances,
                );
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn add_point_from_curve_parameter(
    points: &mut Vec<SurfaceSurfacePoint>,
    circle: &Circle,
    t: f64,
    kind: ContactKind,
    a: &Torus,
    a_range: [ParamRange; 2],
    b: &Torus,
    b_range: [ParamRange; 2],
    t_tol: f64,
    tolerances: Tolerances,
) {
    let Some(t) = fit_scalar_parameter(t, circle.param_range(), t_tol) else {
        return;
    };
    let point = circle.eval(t);
    let Some(uv_a) = torus_uv_at(point, a, a_range, tolerances) else {
        return;
    };
    let Some(uv_b) = torus_uv_at(point, b, b_range, tolerances) else {
        return;
    };
    if let Some(point) = accept_surface_surface_candidate(a, uv_a, b, uv_b, kind, tolerances) {
        push_point(points, point, tolerances);
    }
}

fn torus_uv_at(
    point: Point3,
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let local = torus.frame().to_local(point);
    let xy = (local.x * local.x + local.y * local.y).sqrt();
    let raw_u = if xy <= tolerances.linear() {
        torus_range[0].lo
    } else {
        math::atan2(local.y, local.x)
    };
    let u_tol = parameter_tolerance(
        xy.max(torus.major_radius() - torus.minor_radius()),
        tolerances,
    );
    let u = fit_periodic_parameter(raw_u, torus_range[0], u_tol)?;
    let raw_v = math::atan2(local.z, xy - torus.major_radius());
    let v = fit_periodic_parameter(
        raw_v,
        torus_range[1],
        parameter_tolerance(torus.minor_radius(), tolerances),
    )?;
    Some([u, v])
}

fn hit_contains_t(
    hit: &CurveSurfaceIntersections,
    t: f64,
    t_tol: f64,
    tolerances: Tolerances,
) -> bool {
    hit.overlaps
        .iter()
        .any(|overlap| overlap_contains_t(overlap, t, t_tol))
        || hit.points.iter().any(|point| {
            curve_parameter_distance(point.t_curve, t) <= t_tol.max(tolerances.angular())
        })
}

fn overlap_contains_t(overlap: &CurveSurfaceOverlap, t: f64, t_tol: f64) -> bool {
    [t, t - core::f64::consts::TAU, t + core::f64::consts::TAU]
        .into_iter()
        .any(|candidate| {
            candidate >= overlap.curve.lo - t_tol && candidate <= overlap.curve.hi + t_tol
        })
}

fn curve_parameters_match(
    a: &CurveSurfacePoint,
    b: &CurveSurfacePoint,
    t_tol: f64,
    tolerances: Tolerances,
) -> bool {
    curve_parameter_distance(a.t_curve, b.t_curve) <= t_tol.max(tolerances.angular())
        || a.point.dist(b.point) <= tolerances.linear()
}

fn curve_parameter_distance(a: f64, b: f64) -> f64 {
    let period = core::f64::consts::TAU;
    let diff = (a - b).abs();
    diff.min((period - diff).abs())
}

fn fit_scalar_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
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

fn push_curve(
    curves: &mut Vec<SurfaceSurfaceCurve>,
    candidate: SurfaceSurfaceCurve,
    tolerance: f64,
) {
    if !curves.iter().any(|curve| {
        (curve.curve_range.lo - candidate.curve_range.lo).abs() <= tolerance
            && (curve.curve_range.hi - candidate.curve_range.hi).abs() <= tolerance
            && curve
                .curve
                .eval(curve.curve_range.lo)
                .dist(candidate.curve.eval(candidate.curve_range.lo))
                <= tolerance
            && curve
                .curve
                .eval(curve.curve_range.hi)
                .dist(candidate.curve.eval(candidate.curve_range.hi))
                <= tolerance
    }) {
        curves.push(candidate);
    }
}

fn squared_tolerance(
    center_distance: f64,
    minor_a: f64,
    minor_b: f64,
    tolerances: Tolerances,
) -> f64 {
    tolerances.linear() * (center_distance + minor_a + minor_b).max(1.0)
}

fn validate_ranges(a_range: [ParamRange; 2], b_range: [ParamRange; 2]) -> Result<()> {
    if a_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "torus/torus intersection requires finite non-reversed first-torus ranges",
        });
    }
    if b_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "torus/torus intersection requires finite non-reversed second-torus ranges",
        });
    }
    Ok(())
}
