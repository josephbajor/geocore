use super::conic::{fit_periodic_parameter, parameter_tolerance};
use super::line_cylinder::intersect_bounded_line_cylinder;
use super::parameter::fit_scalar_parameter;
use super::result::{
    ContactKind, SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
    SurfaceSurfacePoint,
};
use super::support_curve_pair::{SupportCurvePairConfig, emit_support_curve_pair};
use kcore::error::{Error, Result};
use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::Line;
use kgeom::param::ParamRange;
use kgeom::surface::Cylinder;
use kgeom::vec::Point3;

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
            return Err(Error::InvalidGeometry {
                reason: "coincident cylinder/cylinder intersection is a surface overlap",
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
