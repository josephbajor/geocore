use super::circle_circle::intersect_bounded_circles;
use super::conic::{
    ConicPairConfig, ConicPlaneRelation, canonical_angle, ellipse_parameter, polynomial_derivative,
    real_polynomial_roots,
};
use super::error::{IntersectionError, IntersectionResult};
use super::line_ellipse::intersect_bounded_line_ellipse;
use super::result::{ContactKind, CurveCurveIntersections, CurveCurvePoint};
use kcore::error::{Error, Result};
use kcore::operation::{OperationContext, OperationOutcome, OperationScope, SessionPolicy};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::param::ParamRange;
use kgeom::project::{ProjectionBudgetProfile, ProjectionError, project_to_curve_in_scope};

/// Intersect two ellipses restricted to finite parameter ranges.
///
/// Handles skew-plane contacts, coplanar secants/tangencies, periodic arc
/// filtering, tolerance-aware near tangencies, and coincident arc overlaps.
pub fn intersect_bounded_ellipses(
    a: &Ellipse,
    range_a: ParamRange,
    b: &Ellipse,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, tolerances)
        .expect("validated Tolerances always satisfy v1 session precision")
        .with_family_budget_defaults(ProjectionBudgetProfile::curve_aggregate_compatibility());
    match intersect_bounded_ellipses_with_context(a, range_a, b, range_b, &context).into_result() {
        Ok(result) => Ok(result),
        Err(IntersectionError::Kernel(error)) => Err(error),
        Err(
            IntersectionError::UnsupportedCurvePair { .. }
            | IntersectionError::UnsupportedCurveSurfacePair { .. }
            | IntersectionError::UnsupportedSurfacePair { .. },
        ) => {
            unreachable!("the concrete ellipse solver has no unsupported dispatch")
        }
    }
}

/// Context-aware ellipse/ellipse intersection with shared projection accounting.
pub fn intersect_bounded_ellipses_with_context(
    a: &Ellipse,
    range_a: ParamRange,
    b: &Ellipse,
    range_b: ParamRange,
    context: &OperationContext<'_>,
) -> OperationOutcome<CurveCurveIntersections, IntersectionError> {
    let context = context
        .clone()
        .with_family_budget_defaults(ProjectionBudgetProfile::curve_aggregate_compatibility());
    let mut scope = OperationScope::new(&context);
    let result = intersect_bounded_ellipses_in_scope(a, range_a, b, range_b, &mut scope);
    scope.finish_typed(result)
}

pub(crate) fn intersect_bounded_ellipses_in_scope(
    a: &Ellipse,
    range_a: ParamRange,
    b: &Ellipse,
    range_b: ParamRange,
    scope: &mut OperationScope<'_, '_>,
) -> IntersectionResult<CurveCurveIntersections> {
    let tolerances = scope.context().tolerances();
    let pair = ConicPairConfig::ellipses(a, range_a, b, range_b, tolerances)?;
    match pair.plane_relation()? {
        ConicPlaneRelation::Parallel => {
            intersect_parallel_plane(a, range_a, b, range_b, tolerances, pair, scope)
        }
        ConicPlaneRelation::Crossing(line) => {
            intersect_plane_crossing(a, range_a, tolerances, line, pair, scope)
        }
    }
}

fn intersect_parallel_plane(
    a: &Ellipse,
    range_a: ParamRange,
    b: &Ellipse,
    range_b: ParamRange,
    tolerances: Tolerances,
    pair: ConicPairConfig<'_>,
    scope: &mut OperationScope<'_, '_>,
) -> IntersectionResult<CurveCurveIntersections> {
    let center_delta = b.frame().origin() - a.frame().origin();
    if center_delta.dot(a.frame().z()).abs() > tolerances.linear() {
        return Ok(CurveCurveIntersections::complete_empty());
    }

    if ellipse_is_circle(a, tolerances) && ellipse_is_circle(b, tolerances) {
        let ca = Circle::new(*a.frame(), a.major_radius())?;
        let cb = Circle::new(*b.frame(), b.major_radius())?;
        return Ok(intersect_bounded_circles(
            &ca, range_a, &cb, range_b, tolerances,
        )?);
    }

    if ellipses_are_coincident(a, b, tolerances) {
        let (sign, offset) = coincident_parameter_map(a, b);
        return Ok(pair.coincident(sign, offset)?);
    }

    intersect_coplanar_distinct(a, b, tolerances, pair, scope)
}

fn intersect_plane_crossing(
    a: &Ellipse,
    range_a: ParamRange,
    tolerances: Tolerances,
    line: Line,
    pair: ConicPairConfig<'_>,
    _scope: &mut OperationScope<'_, '_>,
) -> IntersectionResult<CurveCurveIntersections> {
    let center_parameter = line.dir().dot(a.frame().origin() - line.origin());
    let line_range = ParamRange::new(
        center_parameter - a.major_radius() - tolerances.linear(),
        center_parameter + a.major_radius() + tolerances.linear(),
    );
    let line_hits = intersect_bounded_line_ellipse(&line, line_range, a, range_a, tolerances)?;

    let mut points = Vec::with_capacity(line_hits.points.len());
    for line_hit in line_hits.points {
        let point = a.eval(line_hit.t_b);
        pair.push_point(point, None, &mut points);
    }
    Ok(CurveCurveIntersections::canonicalized_complete(
        points,
        Vec::new(),
    )?)
}

fn intersect_coplanar_distinct(
    a: &Ellipse,
    b: &Ellipse,
    tolerances: Tolerances,
    pair: ConicPairConfig<'_>,
    scope: &mut OperationScope<'_, '_>,
) -> IntersectionResult<CurveCurveIntersections> {
    let mut points = Vec::new();
    for (t_b, tangent_hint) in coplanar_candidate_parameters(a, b, tolerances, scope)? {
        push_projected_from_b(pair, a, b, t_b, tangent_hint, &mut points, scope)?;
    }
    for (t_a, tangent_hint) in coplanar_candidate_parameters(b, a, tolerances, scope)? {
        push_projected_from_a(pair, a, b, t_a, tangent_hint, &mut points, scope)?;
    }
    Ok(CurveCurveIntersections::canonicalized_complete(
        points,
        Vec::new(),
    )?)
}

fn coplanar_candidate_parameters(
    target: &Ellipse,
    source: &Ellipse,
    tolerances: Tolerances,
    scope: &mut OperationScope<'_, '_>,
) -> IntersectionResult<Vec<(f64, bool)>> {
    let poly = ellipse_into_ellipse_quartic(target, source);
    let mut roots = Vec::new();
    for z in real_polynomial_roots(&poly) {
        push_parameter_candidate(&mut roots, 2.0 * kcore::math::atan(z), false);
    }
    for z in real_polynomial_roots(&polynomial_derivative(&poly)) {
        let t = canonical_angle(2.0 * kcore::math::atan(z));
        let point = source.eval(t);
        if project_to_curve_in_scope(target, point, target.param_range(), scope)
            .map_err(projection_error)?
            .dist
            <= tolerances.linear()
        {
            push_parameter_candidate(&mut roots, t, true);
        }
    }
    let point = source.eval(core::f64::consts::PI);
    let projection = project_to_curve_in_scope(target, point, target.param_range(), scope)
        .map_err(projection_error)?;
    if projection.dist <= tolerances.linear() {
        push_parameter_candidate(
            &mut roots,
            core::f64::consts::PI,
            projection.dist > kcore::tolerance::LINEAR_RESOLUTION,
        );
    }
    Ok(roots)
}

fn push_parameter_candidate(candidates: &mut Vec<(f64, bool)>, t: f64, tangent_hint: bool) {
    let t = canonical_angle(t);
    if let Some(existing) = candidates
        .iter_mut()
        .find(|(existing, _)| angular_distance(*existing, t) <= 1e-10)
    {
        existing.1 |= tangent_hint;
    } else {
        candidates.push((t, tangent_hint));
    }
}

fn angular_distance(a: f64, b: f64) -> f64 {
    let period = core::f64::consts::TAU;
    let d = (a - b).abs();
    d.min(period - d)
}

fn ellipse_into_ellipse_quartic(target: &Ellipse, source: &Ellipse) -> Vec<f64> {
    let center = target.frame().to_local(source.frame().origin());
    let ux = source.frame().x().dot(target.frame().x()) * source.major_radius();
    let vx = source.frame().y().dot(target.frame().x()) * source.minor_radius();
    let uy = source.frame().x().dot(target.frame().y()) * source.major_radius();
    let vy = source.frame().y().dot(target.frame().y()) * source.minor_radius();
    let qx = [center.x + ux, 2.0 * vx, center.x - ux];
    let qy = [center.y + uy, 2.0 * vy, center.y - uy];
    let mut coeffs = [0.0; 5];
    add_scaled_square(
        &mut coeffs,
        qx,
        1.0 / (target.major_radius() * target.major_radius()),
    );
    add_scaled_square(
        &mut coeffs,
        qy,
        1.0 / (target.minor_radius() * target.minor_radius()),
    );
    coeffs[0] -= 1.0;
    coeffs[2] -= 2.0;
    coeffs[4] -= 1.0;
    coeffs.to_vec()
}

fn add_scaled_square(coeffs: &mut [f64; 5], q: [f64; 3], scale: f64) {
    coeffs[0] += scale * q[0] * q[0];
    coeffs[1] += scale * 2.0 * q[0] * q[1];
    coeffs[2] += scale * (2.0 * q[0] * q[2] + q[1] * q[1]);
    coeffs[3] += scale * 2.0 * q[1] * q[2];
    coeffs[4] += scale * q[2] * q[2];
}

fn coincident_parameter_map(a: &Ellipse, b: &Ellipse) -> (f64, f64) {
    let b0 = ellipse_parameter(b.frame().to_local(a.eval(0.0)), b);
    let b1 = ellipse_parameter(b.frame().to_local(a.eval(core::f64::consts::FRAC_PI_2)), b);
    let delta = signed_periodic_delta(b1 - b0);
    let sign = if delta >= 0.0 { 1.0 } else { -1.0 };
    (sign, b0)
}

fn signed_periodic_delta(delta: f64) -> f64 {
    let period = core::f64::consts::TAU;
    let mut d = delta % period;
    if d <= -core::f64::consts::PI {
        d += period;
    }
    if d > core::f64::consts::PI {
        d -= period;
    }
    d
}

fn push_projected_from_b(
    pair: ConicPairConfig<'_>,
    a: &Ellipse,
    b: &Ellipse,
    t_b: f64,
    tangent_hint: bool,
    points: &mut Vec<CurveCurvePoint>,
    scope: &mut OperationScope<'_, '_>,
) -> IntersectionResult<()> {
    let Some(t_b) = pair.fit_parameter_b(t_b) else {
        return Ok(());
    };
    let point_b = b.eval(t_b);
    let projection =
        project_to_curve_in_scope(a, point_b, a.param_range(), scope).map_err(projection_error)?;
    if projection.dist > pair.tolerances().linear() {
        return Ok(());
    }
    let Some(t_a) = pair.fit_parameter_a(projection.t) else {
        return Ok(());
    };
    pair.push_parameters(
        t_a,
        t_b,
        tangent_hint.then_some(ContactKind::Tangent),
        points,
    );
    Ok(())
}

fn push_projected_from_a(
    pair: ConicPairConfig<'_>,
    a: &Ellipse,
    b: &Ellipse,
    t_a: f64,
    tangent_hint: bool,
    points: &mut Vec<CurveCurvePoint>,
    scope: &mut OperationScope<'_, '_>,
) -> IntersectionResult<()> {
    let Some(t_a) = pair.fit_parameter_a(t_a) else {
        return Ok(());
    };
    let point_a = a.eval(t_a);
    let projection =
        project_to_curve_in_scope(b, point_a, b.param_range(), scope).map_err(projection_error)?;
    if projection.dist > pair.tolerances().linear() {
        return Ok(());
    }
    let Some(t_b) = pair.fit_parameter_b(projection.t) else {
        return Ok(());
    };
    pair.push_parameters(
        t_a,
        t_b,
        tangent_hint.then_some(ContactKind::Tangent),
        points,
    );
    Ok(())
}

fn projection_error(error: ProjectionError) -> IntersectionError {
    match error {
        ProjectionError::Policy(error) => IntersectionError::Kernel(error.into()),
        _ => IntersectionError::Kernel(Error::InvalidGeometry {
            reason: "ellipse intersection projection failed",
        }),
    }
}

fn ellipses_are_coincident(a: &Ellipse, b: &Ellipse, tolerances: Tolerances) -> bool {
    a.frame().origin().dist(b.frame().origin()) <= tolerances.linear()
        && (a.major_radius() - b.major_radius()).abs() <= tolerances.linear()
        && (a.minor_radius() - b.minor_radius()).abs() <= tolerances.linear()
        && a.frame().z().cross(b.frame().z()).norm() <= tolerances.angular()
        && (ellipse_is_circle(a, tolerances)
            || a.frame().x().cross(b.frame().x()).norm() <= tolerances.angular())
}

fn ellipse_is_circle(ellipse: &Ellipse, tolerances: Tolerances) -> bool {
    (ellipse.major_radius() - ellipse.minor_radius()).abs() <= tolerances.linear()
}
