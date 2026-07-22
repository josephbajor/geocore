use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::line_cylinder::intersect_bounded_line_cylinder;
use super::parameter::{
    PeriodicOverlapPiece, affine_preimage_overlap, fit_scalar_parameter,
    periodic_preimage_overlaps, range_midpoint, validate_period_span,
};
use super::result::{
    ContactKind, SurfaceIntersectionCurve, SurfaceRegionOrientation, SurfaceSurfaceCurve,
    SurfaceSurfaceIntersections, SurfaceSurfacePoint, SurfaceSurfaceRegion,
    SurfaceSurfaceRegionVertex,
};
use super::support_curve_pair::{SupportCurvePairConfig, emit_support_curve_pair};
use kcore::error::{Error, Result};
use kcore::interval::Interval;
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Line};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Surface};
use kgeom::vec::{Point3, Vec3};

/// Intersect two finite cylinder parameter windows.
///
/// Supports parallel-axis cylinder/cylinder intersections. Those reduce to a
/// circle/circle solve in the plane normal to the axes and produce one tangent
/// ruling or two transverse rulings. Skew and oblique cylinder/cylinder
/// intersections are quartic space curves and remain explicit until SSI result
/// geometry can carry that branch family.
pub fn intersect_bounded_cylinders(
    a: &Cylinder,
    a_range: [ParamRange; 2],
    b: &Cylinder,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    validate_ranges(a_range, b_range)?;

    let axis = a.frame().z();
    if axis.cross(b.frame().z()).norm() > tolerances.angular() {
        return Err(Error::InvalidGeometry {
            reason: "cylinder/cylinder intersection currently supports only parallel-axis ruling cuts",
        });
    }

    let offset = b.frame().origin() - a.frame().origin();
    let radial_offset = offset - axis * offset.dot(axis);
    let distance = radial_offset.norm();
    let radius_a = a.radius();
    let radius_b = b.radius();

    if distance <= tolerances.linear() {
        if (radius_a - radius_b).abs() <= tolerances.linear() {
            if distance == 0.0 && radius_a == radius_b && axis.cross(b.frame().z()).norm() == 0.0 {
                if compare_cylinder_windows(a, a_range, b, b_range).is_gt() {
                    return intersect_coincident_cylinder_windows(
                        b, b_range, a, a_range, tolerances,
                    )
                    .map(SurfaceSurfaceIntersections::swapped);
                }
                return intersect_coincident_cylinder_windows(a, a_range, b, b_range, tolerances);
            }
            return Err(Error::InvalidGeometry {
                reason: "near-coincident non-identical cylinders require the general certified fallback",
            });
        }
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    }

    let x = (radius_a * radius_a - radius_b * radius_b + distance * distance) / (2.0 * distance);
    let h_sq = radius_a * radius_a - x * x;
    let sq_tol = squared_tolerance(radius_a, radius_b, distance, tolerances);
    if h_sq < -sq_tol {
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    }

    let radial_x = radial_offset / distance;
    let radial_y = axis.cross(radial_x);
    let mut points = Vec::new();
    let mut curves = Vec::new();
    if h_sq <= sq_tol {
        let point = a.frame().origin() + radial_x * x.clamp(-radius_a, radius_a);
        add_line_branch(
            &mut points,
            &mut curves,
            point,
            ContactKind::Tangent,
            a,
            a_range,
            b,
            b_range,
            tolerances,
        )?;
    } else {
        let h = h_sq.sqrt();
        for point in [
            a.frame().origin() + radial_x * x - radial_y * h,
            a.frame().origin() + radial_x * x + radial_y * h,
        ] {
            add_line_branch(
                &mut points,
                &mut curves,
                point,
                ContactKind::Transverse,
                a,
                a_range,
                b,
                b_range,
                tolerances,
            )?;
        }
    }

    SurfaceSurfaceIntersections::canonicalized_complete(points, curves)
}

pub(super) fn compare_cylinder_windows(
    a: &Cylinder,
    a_range: [ParamRange; 2],
    b: &Cylinder,
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
struct CoincidentCylinderMap {
    sign: f64,
    u_phase: f64,
    v_phase: f64,
}

#[derive(Clone, Copy, Debug)]
struct PairedCylinderSample {
    point: Point3,
    uv_a: [f64; 2],
    uv_b: [f64; 2],
    residual: f64,
    residual_bound: f64,
}

fn intersect_coincident_cylinder_windows(
    a: &Cylinder,
    a_range: [ParamRange; 2],
    b: &Cylinder,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceIntersections> {
    let tau = core::f64::consts::TAU;
    validate_period_span(
        a_range[0],
        tau,
        0.0,
        "coincident cylinder longitude windows cannot span more than one turn",
    )?;
    validate_period_span(
        b_range[0],
        tau,
        0.0,
        "coincident cylinder longitude windows cannot span more than one turn",
    )?;

    let sign = if a.frame().z().dot(b.frame().z()).is_sign_negative() {
        -1.0
    } else {
        1.0
    };
    let u_phase = math::atan2(
        a.frame().x().dot(b.frame().y()),
        a.frame().x().dot(b.frame().x()),
    );
    let map = CoincidentCylinderMap {
        sign,
        u_phase,
        v_phase: (a.frame().origin() - b.frame().origin()).dot(b.frame().z()),
    };
    let u_tolerance = parameter_tolerance(a.radius(), tolerances);
    let u_overlaps = periodic_preimage_overlaps(
        a_range[0],
        b_range[0],
        map.sign,
        map.u_phase,
        tau,
        u_tolerance,
        "coincident cylinder periodic chart shift is outside the exact integer corridor",
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
    if u_overlaps.is_empty() {
        return Ok(SurfaceSurfaceIntersections::complete_empty());
    }

    let mut points = Vec::new();
    let mut curves = Vec::new();
    let mut regions = Vec::new();
    for overlap in u_overlaps {
        let u_positive = overlap.a.width() > u_tolerance;
        let v_positive = v_overlap.width() > tolerances.linear();
        match (u_positive, v_positive) {
            (true, true) => regions.push(coincident_cylinder_region(
                a, b, overlap, v_overlap, b_range, map, tolerances,
            )?),
            (true, false) => curves.push(coincident_cylinder_circle_branch(
                a,
                b,
                overlap,
                range_midpoint(v_overlap),
                b_range,
                map,
                tolerances,
            )?),
            (false, true) => curves.push(coincident_cylinder_ruling_branch(
                a,
                b,
                range_midpoint(overlap.a),
                overlap.shift,
                v_overlap,
                b_range,
                map,
                tolerances,
            )?),
            (false, false) => {
                let sample = paired_cylinder_sample(
                    a,
                    b,
                    [range_midpoint(overlap.a), range_midpoint(v_overlap)],
                    overlap.shift,
                    b_range,
                    map,
                    tolerances,
                )?;
                points.push(SurfaceSurfacePoint {
                    point: sample.point,
                    uv_a: sample.uv_a,
                    uv_b: sample.uv_b,
                    residual: sample.residual,
                    kind: ContactKind::Tangent,
                });
            }
        }
    }
    SurfaceSurfaceIntersections::canonicalized_complete_with_regions(points, curves, regions)
}

#[allow(clippy::too_many_arguments)]
fn coincident_cylinder_region(
    a: &Cylinder,
    b: &Cylinder,
    u: PeriodicOverlapPiece,
    v: ParamRange,
    b_range: [ParamRange; 2],
    map: CoincidentCylinderMap,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceRegion> {
    let mut boundary = Vec::with_capacity(4);
    let mut max_residual = coincident_cylinder_whole_residual_bound(a, b, u, v, b_range, map)?;
    for uv_a in [
        [u.a.lo, v.lo],
        [u.a.hi, v.lo],
        [u.a.hi, v.hi],
        [u.a.lo, v.hi],
    ] {
        let sample = paired_cylinder_sample(a, b, uv_a, u.shift, b_range, map, tolerances)?;
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
        correspondence: super::result::SurfaceRegionCorrespondence::Polygonal,
        max_residual,
    })
}

#[allow(clippy::too_many_arguments)]
fn coincident_cylinder_circle_branch(
    a: &Cylinder,
    b: &Cylinder,
    u: PeriodicOverlapPiece,
    v: f64,
    b_range: [ParamRange; 2],
    map: CoincidentCylinderMap,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceCurve> {
    let start = paired_cylinder_sample(a, b, [u.a.lo, v], u.shift, b_range, map, tolerances)?;
    let end = paired_cylinder_sample(a, b, [u.a.hi, v], u.shift, b_range, map, tolerances)?;
    let frame = Frame::new(
        a.frame().origin() + a.frame().z() * v,
        a.frame().z(),
        a.frame().x(),
    )?;
    Ok(SurfaceSurfaceCurve {
        curve: SurfaceIntersectionCurve::Circle(Circle::new(frame, a.radius())?),
        curve_range: u.a,
        uv_a_start: start.uv_a,
        uv_a_end: end.uv_a,
        uv_b_start: start.uv_b,
        uv_b_end: end.uv_b,
        kind: ContactKind::Tangent,
    })
}

#[allow(clippy::too_many_arguments)]
fn coincident_cylinder_ruling_branch(
    a: &Cylinder,
    b: &Cylinder,
    u: f64,
    shift: f64,
    v: ParamRange,
    b_range: [ParamRange; 2],
    map: CoincidentCylinderMap,
    tolerances: Tolerances,
) -> Result<SurfaceSurfaceCurve> {
    let start = paired_cylinder_sample(a, b, [u, v.lo], shift, b_range, map, tolerances)?;
    let end = paired_cylinder_sample(a, b, [u, v.hi], shift, b_range, map, tolerances)?;
    let origin = a.eval([u, 0.0]);
    Ok(SurfaceSurfaceCurve {
        curve: SurfaceIntersectionCurve::Line(Line::new(origin, a.frame().z())?),
        curve_range: v,
        uv_a_start: start.uv_a,
        uv_a_end: end.uv_a,
        uv_b_start: start.uv_b,
        uv_b_end: end.uv_b,
        kind: ContactKind::Tangent,
    })
}

#[allow(clippy::too_many_arguments)]
fn paired_cylinder_sample(
    a: &Cylinder,
    b: &Cylinder,
    uv_a: [f64; 2],
    u_shift: f64,
    b_range: [ParamRange; 2],
    map: CoincidentCylinderMap,
    tolerances: Tolerances,
) -> Result<PairedCylinderSample> {
    let uv_b = [
        fit_scalar_parameter(
            map.sign * uv_a[0] + map.u_phase + u_shift,
            b_range[0],
            parameter_tolerance(b.radius(), tolerances),
        ),
        fit_scalar_parameter(
            map.sign * uv_a[1] + map.v_phase,
            b_range[1],
            tolerances.linear(),
        ),
    ];
    let [Some(u_b), Some(v_b)] = uv_b else {
        return Err(Error::InvalidGeometry {
            reason: "coincident cylinder chart overlap did not lift into both source windows",
        });
    };
    let uv_b = [u_b, v_b];
    let pa = a.eval(uv_a);
    let pb = b.eval(uv_b);
    let residual = pa.dist(pb);
    let residual_bound = conservative_point_distance(pa, pb).ok_or(Error::InvalidGeometry {
        reason: "coincident cylinder residual arithmetic is non-finite",
    })?;
    Ok(PairedCylinderSample {
        point: (pa + pb) / 2.0,
        uv_a,
        uv_b,
        residual,
        residual_bound,
    })
}

fn coincident_cylinder_whole_residual_bound(
    a: &Cylinder,
    b: &Cylinder,
    u: PeriodicOverlapPiece,
    v: ParamRange,
    b_range: [ParamRange; 2],
    map: CoincidentCylinderMap,
) -> Result<f64> {
    let (sin_phase, cos_phase) = math::sincos(map.u_phase);
    let b_cos = b.frame().x() * cos_phase + b.frame().y() * sin_phase;
    let b_sin = (b.frame().y() * cos_phase - b.frame().x() * sin_phase) * map.sign;
    let origin_difference = a.frame().origin() - (b.frame().origin() + b.frame().z() * map.v_phase);
    let cosine_difference = a.frame().x() * a.radius() - b_cos * b.radius();
    let sine_difference = a.frame().y() * a.radius() - b_sin * b.radius();
    let axial_difference = a.frame().z() - b.frame().z() * map.sign;
    let max_v = v.lo.abs().max(v.hi.abs());

    let mut bound = Interval::point(conservative_vec_norm(origin_difference)?);
    bound = bound + Interval::point(conservative_vec_norm(cosine_difference)?);
    bound = bound + Interval::point(conservative_vec_norm(sine_difference)?);
    bound =
        bound + Interval::point(conservative_vec_norm(axial_difference)?) * Interval::point(max_v);

    // The coefficient proof above is over the ideal paired trigonometric map.
    // Retain a scale-aware outward allowance for argument reduction and the
    // finite frame operations used by the public evaluators.
    let parameter_scale =
        u.a.lo
            .abs()
            .max(u.a.hi.abs())
            .max(b_range[0].lo.abs())
            .max(b_range[0].hi.abs())
            .max(v.lo.abs())
            .max(v.hi.abs())
            .max(b_range[1].lo.abs())
            .max(b_range[1].hi.abs());
    let model_scale = a
        .frame()
        .origin()
        .norm()
        .max(b.frame().origin().norm())
        .max(a.radius())
        .max(parameter_scale)
        .max(1.0);
    let rounding = Interval::point(4096.0 * f64::EPSILON) * Interval::point(model_scale);
    let result = bound + rounding;
    result
        .hi()
        .is_finite()
        .then_some(result.hi())
        .ok_or(Error::InvalidGeometry {
            reason: "coincident cylinder whole-region residual bound is non-finite",
        })
}

fn conservative_vec_norm(value: Vec3) -> Result<f64> {
    let components = value.to_array().map(Interval::point);
    let squared = components
        .into_iter()
        .fold(Interval::point(0.0), |sum, value| sum + value.square());
    squared
        .sqrt()
        .map(Interval::hi)
        .filter(|bound| bound.is_finite())
        .ok_or(Error::InvalidGeometry {
            reason: "coincident cylinder coefficient residual bound is non-finite",
        })
}

fn conservative_point_distance(a: Point3, b: Point3) -> Option<f64> {
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

#[allow(clippy::too_many_arguments)]
fn add_line_branch(
    points: &mut Vec<SurfaceSurfacePoint>,
    curves: &mut Vec<SurfaceSurfaceCurve>,
    origin: Point3,
    branch_kind: ContactKind,
    a: &Cylinder,
    a_range: [ParamRange; 2],
    b: &Cylinder,
    b_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<()> {
    let line = Line::new(origin, a.frame().z())?;
    let line_range = a_range[1];
    let a_hit = intersect_bounded_line_cylinder(&line, line_range, a, a_range, tolerances)?;
    let b_hit = intersect_bounded_line_cylinder(&line, line_range, b, b_range, tolerances)?;
    let curve = SurfaceIntersectionCurve::Line(line);
    let first_uv = |point| cylinder_uv_at(point, a, a_range, tolerances);
    let second_uv = |point| cylinder_uv_at(point, b, b_range, tolerances);
    emit_support_curve_pair(
        SupportCurvePairConfig {
            curve: &curve,
            curve_range: line_range,
            first_hit: &a_hit,
            second_hit: &b_hit,
            kind: branch_kind,
            parameter_tolerance: tolerances.linear(),
            parameter_period: None,
            branch_tolerance: tolerances.linear(),
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

fn cylinder_uv_at(
    point: Point3,
    cylinder: &Cylinder,
    cylinder_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
    let local = cylinder.frame().to_local(point);
    let raw_u = math::atan2(local.y, local.x);
    let u = fit_periodic_parameter(
        raw_u,
        cylinder_range[0],
        parameter_tolerance(cylinder.radius(), tolerances),
    )?;
    let v = fit_scalar_parameter(local.z, cylinder_range[1], tolerances.linear())?;
    Some([u, v])
}

fn squared_tolerance(radius_a: f64, radius_b: f64, distance: f64, tolerances: Tolerances) -> f64 {
    tolerances.linear() * (radius_a + radius_b + distance).max(1.0)
}

fn validate_ranges(a_range: [ParamRange; 2], b_range: [ParamRange; 2]) -> Result<()> {
    if a_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "cylinder/cylinder intersection requires finite non-reversed first cylinder ranges",
        });
    }
    if b_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "cylinder/cylinder intersection requires finite non-reversed second cylinder ranges",
        });
    }
    Ok(())
}
