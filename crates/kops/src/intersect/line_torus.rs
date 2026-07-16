use super::bounded_polynomial::{
    ExactPolynomial, ExactScalar, RootBracket, RootIsolation, RootIsolationFailure,
};
use super::conic::{fit_periodic_parameter, polynomial_derivative, real_polynomial_roots};
use super::result::{
    ContactKind, CurveSurfaceIntersections, CurveSurfacePoint, accept_curve_surface_candidate,
};
use kcore::error::{Error, Result};
use kcore::interval::Interval;
use kcore::math;
use kcore::tolerance::{ANGULAR_RESOLUTION, Tolerances};
use kgeom::curve::Line;
use kgeom::param::ParamRange;
use kgeom::surface::{Surface, Torus};
use kgeom::vec::Vec3;

const ROOT_TOPOLOGY_INDETERMINATE: &str = "line/torus quartic root topology could not be certified";

struct ExactLineTorusPolynomials {
    surface: ExactPolynomial,
    auxiliary: ExactLineTorusAuxiliary,
}

enum ExactLineTorusAuxiliary {
    General {
        distance_stationary: ExactPolynomial,
        radial_axis: ExactPolynomial,
    },
    Axis {
        center_plane: ExactPolynomial,
    },
}

/// Intersect a line restricted to a finite range with a finite torus
/// parameter window.
///
/// The torus implicit equation reduces to a quartic in the line parameter. A
/// bounded exact pseudo-Sturm classifier owns its distinct-root topology over
/// the tolerance-expanded line range. A second exact quartic covers
/// differentiable torus-distance extrema; its unsquared factor, the radial-axis
/// quadratic, and both domain endpoints complete the tolerance-candidate proof.
/// The legacy rounded roots remain discovery-only and can only downgrade
/// completion.
pub fn intersect_bounded_line_torus(
    line: &Line,
    line_range: ParamRange,
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Result<CurveSurfaceIntersections> {
    validate_ranges(line_range, torus_range)?;

    let local_origin = torus.frame().to_local(line.origin());
    let direction = line.dir();
    let local_direction = Vec3::new(
        direction.dot(torus.frame().x()),
        direction.dot(torus.frame().y()),
        direction.dot(torus.frame().z()),
    );
    let context = TorusLineContext {
        line,
        line_range,
        torus,
        torus_range,
        local_origin,
        local_direction,
        tolerances,
    };
    let mut points = Vec::new();
    let mut complete = torus_ranges_are_full_period(torus_range);

    let Some(isolation_range) = tolerance_expanded_line_range(line_range, tolerances.linear())
    else {
        return CurveSurfaceIntersections::canonicalized_indeterminate(
            points,
            Vec::new(),
            ROOT_TOPOLOGY_INDETERMINATE,
        );
    };
    match exact_line_torus_polynomials(line, torus) {
        Ok(polynomials) => {
            if matches!(
                polynomials
                    .surface
                    .isolate_repeated_roots(isolation_range.lo, isolation_range.hi),
                RootIsolation::Ambiguous(_)
            ) {
                complete = false;
            }
            match polynomials
                .surface
                .isolate(isolation_range.lo, isolation_range.hi)
            {
                RootIsolation::Complete(roots) => {
                    for &root in &roots {
                        let is_repeated_root = match polynomials.surface.root_is_repeated(root) {
                            Ok(is_repeated) => is_repeated,
                            Err(_) => {
                                complete = false;
                                false
                            }
                        };
                        if context
                            .add_root_bracket(&mut points, root, is_repeated_root)
                            .is_none()
                        {
                            complete = false;
                        }
                    }
                }
                RootIsolation::Ambiguous(_) => complete = false,
            }

            match &polynomials.auxiliary {
                ExactLineTorusAuxiliary::General {
                    distance_stationary,
                    radial_axis,
                } => {
                    for (critical_polynomial, requires_unsquared_stationarity) in
                        [(distance_stationary, true), (radial_axis, false)]
                    {
                        if !context.admit_tolerance_critical_points(
                            &mut points,
                            critical_polynomial,
                            isolation_range,
                            requires_unsquared_stationarity,
                        ) {
                            complete = false;
                        }
                    }
                }
                ExactLineTorusAuxiliary::Axis { center_plane } => {
                    // If the squared radial distance q(t) is identically zero,
                    // the line lies on the torus axis and the generic squared
                    // stationarity polynomial is also identically zero. Along
                    // the axis, distance to an open torus has its sole interior
                    // minimum at the center-plane crossing.
                    if !context.admit_tolerance_critical_points(
                        &mut points,
                        center_plane,
                        isolation_range,
                        false,
                    ) {
                        complete = false;
                    }
                }
            }
            if !context.admit_tolerance_endpoints(&mut points) {
                complete = false;
            }
        }
        Err(_) => complete = false,
    }
    let rounded_coefficients =
        rounded_implicit_line_coefficients(local_origin, local_direction, torus);
    if rounded_coefficients
        .iter()
        .all(|coefficient| coefficient.is_finite())
    {
        for parameter in real_polynomial_roots(&rounded_coefficients) {
            if context
                .add_candidate(&mut points, parameter, false)
                .is_some_and(|added| added)
            {
                complete = false;
            }
        }
        for parameter in real_polynomial_roots(&polynomial_derivative(&rounded_coefficients)) {
            if context
                .add_candidate(&mut points, parameter, true)
                .is_some_and(|added| added)
            {
                complete = false;
            }
        }
    }

    if complete {
        CurveSurfaceIntersections::canonicalized_complete(points, Vec::new())
    } else {
        CurveSurfaceIntersections::canonicalized_indeterminate(
            points,
            Vec::new(),
            ROOT_TOPOLOGY_INDETERMINATE,
        )
    }
}

struct TorusLineContext<'a> {
    line: &'a Line,
    line_range: ParamRange,
    torus: &'a Torus,
    torus_range: [ParamRange; 2],
    local_origin: Vec3,
    local_direction: Vec3,
    tolerances: Tolerances,
}

impl TorusLineContext<'_> {
    /// `Some(true)` means a new physical point was emitted, `Some(false)`
    /// means this algebraic candidate was represented by an existing
    /// resolution-coincident point, and `None` means geometric admission
    /// rejected every representative.
    fn add_root_bracket(
        &self,
        points: &mut Vec<CurveSurfacePoint>,
        root: RootBracket,
        force_tangent: bool,
    ) -> Option<bool> {
        let representatives = [root.representative(), root.lo, root.hi];
        let mut last = None;
        for representative in representatives {
            if last == Some(representative.to_bits()) {
                continue;
            }
            last = Some(representative.to_bits());
            if let Some(added) = self.add_candidate(points, representative, force_tangent) {
                return Some(added);
            }
        }
        None
    }

    fn add_candidate(
        &self,
        points: &mut Vec<CurveSurfacePoint>,
        t_line: f64,
        force_tangent: bool,
    ) -> Option<bool> {
        let t_line = fit_line_parameter(t_line, self.line_range, self.tolerances.linear())?;
        let local = self.local_origin + self.local_direction * t_line;
        let uv = torus_uv(local, self.torus, self.torus_range, self.tolerances)?;
        let normal = self.torus.normal(uv)?;
        let kind =
            if force_tangent || normal.dot(self.line.dir()).abs() <= self.tolerances.angular() {
                ContactKind::Tangent
            } else {
                ContactKind::Transverse
            };
        if let Some(point) =
            accept_curve_surface_candidate(self.line, t_line, self.torus, uv, kind, self.tolerances)
        {
            return Some(push_distinct(points, point, self.tolerances));
        }
        None
    }

    fn admit_tolerance_critical_points(
        &self,
        points: &mut Vec<CurveSurfacePoint>,
        polynomial: &ExactPolynomial,
        isolation_range: ParamRange,
        requires_unsquared_stationarity: bool,
    ) -> bool {
        let RootIsolation::Complete(critical_points) =
            polynomial.isolate(isolation_range.lo, isolation_range.hi)
        else {
            return false;
        };
        let mut complete = true;
        for critical_point in critical_points {
            if requires_unsquared_stationarity {
                match self.stationary_factor_contains_zero(critical_point) {
                    Some(false) => continue,
                    None => complete = false,
                    Some(true) => {}
                }
            }
            match self.add_root_bracket(points, critical_point, true) {
                Some(true) => complete = false,
                None if !self.bracket_is_outside_tolerance(critical_point) => complete = false,
                Some(false) | None => {}
            }
        }
        complete
    }

    /// The polynomial stationarity condition is squared:
    /// `G = (s'√q - Rq')(s'√q + Rq')`. This interval check retains only roots
    /// that may satisfy the unsquared distance-stationarity factor.
    fn stationary_factor_contains_zero(&self, bracket: RootBracket) -> Option<bool> {
        let parameter = Interval::new(bracket.lo, bracket.hi);
        let line_origin = self.line.origin().to_array();
        let torus_origin = self.torus.frame().origin().to_array();
        let direction = self.line.dir().to_array();
        let axis = self.torus.frame().z().to_array();

        let mut squared_radius = Interval::point(0.0);
        let mut squared_radius_derivative = Interval::point(0.0);
        let mut axial = Interval::point(0.0);
        let mut axial_derivative = Interval::point(0.0);
        for coordinate in 0..3 {
            let direction_component = Interval::point(direction[coordinate]);
            let axis_component = Interval::point(axis[coordinate]);
            let delta = Interval::point(line_origin[coordinate])
                - Interval::point(torus_origin[coordinate])
                + parameter * direction_component;
            squared_radius = squared_radius + delta.square();
            squared_radius_derivative =
                squared_radius_derivative + delta * direction_component * Interval::point(2.0);
            axial = axial + delta * axis_component;
            axial_derivative = axial_derivative + direction_component * axis_component;
        }
        if !interval_is_finite(squared_radius)
            || !interval_is_finite(squared_radius_derivative)
            || !interval_is_finite(axial)
            || !interval_is_finite(axial_derivative)
        {
            return None;
        }
        let radial_squared = squared_radius - axial.square();
        let radial = radial_squared.sqrt()?;
        let radial_squared_derivative =
            squared_radius_derivative - axial * axial_derivative * Interval::point(2.0);
        let factor = squared_radius_derivative * radial
            - Interval::point(self.torus.major_radius()) * radial_squared_derivative;
        interval_is_finite(factor).then_some(factor.contains_zero())
    }

    fn admit_tolerance_endpoints(&self, points: &mut Vec<CurveSurfacePoint>) -> bool {
        let mut complete = true;
        let mut previous = None;
        for parameter in [self.line_range.lo, self.line_range.hi] {
            if previous == Some(parameter.to_bits()) {
                continue;
            }
            previous = Some(parameter.to_bits());
            let point_bracket = RootBracket {
                lo: parameter,
                hi: parameter,
            };
            match self.add_candidate(points, parameter, false) {
                Some(true) => complete = false,
                None if !self.bracket_is_outside_tolerance(point_bracket) => complete = false,
                Some(false) | None => {}
            }
        }
        complete
    }

    fn bracket_is_outside_tolerance(&self, bracket: RootBracket) -> bool {
        let parameter = Interval::new(bracket.lo, bracket.hi);
        let line_origin = self.line.origin().to_array();
        let torus_origin = self.torus.frame().origin().to_array();
        let direction = self.line.dir().to_array();
        let axis = self.torus.frame().z().to_array();

        let mut squared_radius = Interval::point(0.0);
        let mut axial = Interval::point(0.0);
        for coordinate in 0..3 {
            let delta = Interval::point(line_origin[coordinate])
                - Interval::point(torus_origin[coordinate])
                + parameter * Interval::point(direction[coordinate]);
            squared_radius = squared_radius + delta.square();
            axial = axial + delta * Interval::point(axis[coordinate]);
        }
        if !interval_is_finite(squared_radius) || !interval_is_finite(axial) {
            return false;
        }

        let radial_squared = squared_radius - axial.square();
        let Some(radial) = radial_squared.sqrt() else {
            return false;
        };
        let radial_offset = radial - Interval::point(self.torus.major_radius());
        let cross_section_squared = radial_offset.square() + axial.square();
        let Some(cross_section_distance) = cross_section_squared.sqrt() else {
            return false;
        };
        let signed_distance = cross_section_distance - Interval::point(self.torus.minor_radius());
        if !interval_is_finite(signed_distance) {
            return false;
        }
        let distance_lower_bound = if signed_distance.contains_zero() {
            0.0
        } else {
            signed_distance.lo().abs().min(signed_distance.hi().abs())
        };
        distance_lower_bound > self.tolerances.linear()
    }
}

fn rounded_implicit_line_coefficients(
    local_origin: Vec3,
    local_direction: Vec3,
    torus: &Torus,
) -> [f64; 5] {
    let s2 = local_direction.dot(local_direction);
    let s1 = 2.0 * local_origin.dot(local_direction);
    let s0 = local_origin.dot(local_origin);
    let q2 = local_direction.x * local_direction.x + local_direction.y * local_direction.y;
    let q1 = 2.0 * (local_origin.x * local_direction.x + local_origin.y * local_direction.y);
    let q0 = local_origin.x * local_origin.x + local_origin.y * local_origin.y;
    let major_sq = torus.major_radius() * torus.major_radius();
    let h0 = s0 + major_sq - torus.minor_radius() * torus.minor_radius();
    let h1 = s1;
    let h2 = s2;

    [
        h0 * h0 - 4.0 * major_sq * q0,
        2.0 * h0 * h1 - 4.0 * major_sq * q1,
        h1 * h1 + 2.0 * h0 * h2 - 4.0 * major_sq * q2,
        2.0 * h1 * h2,
        h2 * h2,
    ]
}

fn exact_line_torus_polynomials(
    line: &Line,
    torus: &Torus,
) -> core::result::Result<ExactLineTorusPolynomials, RootIsolationFailure> {
    let direction_norm = line.dir().norm();
    if !direction_norm.is_finite()
        || (direction_norm - 1.0).abs() > 16.0 * ANGULAR_RESOLUTION
        || !torus.frame().is_orthonormal()
    {
        return Err(RootIsolationFailure::UnsafeArithmeticEnvelope);
    }
    let origin = line.origin().to_array();
    let center = torus.frame().origin().to_array();
    let direction = line.dir().to_array();
    let axis = torus.frame().z().to_array();
    let s0 = exact_squared_distance(origin, center)?;
    let linear = exact_dot_difference(origin, center, direction)?.scale(2.0)?;
    let axial_origin = exact_dot_difference(origin, center, axis)?;
    let axial_direction = exact_dot(direction, axis)?;
    let q0 = s0.sub(&axial_origin.mul(&axial_origin)?)?;
    let q1 = linear.sub(&axial_origin.mul(&axial_direction)?.scale(2.0)?)?;
    let q2 = ExactScalar::from_f64(1.0)?.sub(&axial_direction.mul(&axial_direction)?)?;
    let major = ExactScalar::from_f64(torus.major_radius())?;
    let minor = ExactScalar::from_f64(torus.minor_radius())?;
    let major_sq = major.mul(&major)?;
    let minor_sq = minor.mul(&minor)?;
    let h0 = s0.add(&major_sq)?.sub(&minor_sq)?;

    let c0 = h0.mul(&h0)?.sub(&major_sq.mul(&q0)?.scale(4.0)?)?;
    let c1 = h0
        .mul(&linear)?
        .scale(2.0)?
        .sub(&major_sq.mul(&q1)?.scale(4.0)?)?;
    let c2 = linear
        .mul(&linear)?
        .add(&h0.scale(2.0)?)?
        .sub(&major_sq.mul(&q2)?.scale(4.0)?)?;
    let c3 = linear.scale(2.0)?;
    let c4 = ExactScalar::from_f64(1.0)?;
    let surface = ExactPolynomial::new(vec![c0, c1, c2, c3, c4])?;

    // For rho = sqrt(q), the squared distance to the torus spine circle is
    // g = s + R^2 - 2 R rho. Every differentiable stationary point satisfies
    // (s')^2 q - R^2 (q')^2 = 0. The squared equation may add roots, but never
    // removes a true distance extremum; the unsquared interval factor filters
    // certified opposite-sign roots and fails closed when it is unresolved.
    let slope_square_0 = linear.mul(&linear)?;
    let slope_square_1 = linear.scale(4.0)?;
    let slope_square_2 = ExactScalar::from_f64(4.0)?;
    let stationary_0 = slope_square_0
        .mul(&q0)?
        .sub(&major_sq.mul(&q1.mul(&q1)?)?)?;
    let stationary_1 = slope_square_0
        .mul(&q1)?
        .add(&slope_square_1.mul(&q0)?)?
        .sub(&major_sq.mul(&q1.mul(&q2)?.scale(4.0)?)?)?;
    let stationary_2 = slope_square_0
        .mul(&q2)?
        .add(&slope_square_1.mul(&q1)?)?
        .add(&slope_square_2.mul(&q0)?)?
        .sub(&major_sq.mul(&q2.mul(&q2)?.scale(4.0)?)?)?;
    let stationary_3 = slope_square_1.mul(&q2)?.add(&slope_square_2.mul(&q1)?)?;
    let stationary_4 = slope_square_2.mul(&q2)?;
    let stationary_coefficients = vec![
        stationary_0,
        stationary_1,
        stationary_2,
        stationary_3,
        stationary_4,
    ];
    let radial_coefficients = vec![q0, q1, q2];
    let stationary_is_identity = stationary_coefficients.iter().all(ExactScalar::is_zero);
    let radial_is_identity = radial_coefficients.iter().all(ExactScalar::is_zero);
    let auxiliary = match (stationary_is_identity, radial_is_identity) {
        (false, false) => ExactLineTorusAuxiliary::General {
            distance_stationary: ExactPolynomial::new(stationary_coefficients)?,
            radial_axis: ExactPolynomial::new(radial_coefficients)?,
        },
        (true, true) => ExactLineTorusAuxiliary::Axis {
            center_plane: ExactPolynomial::new(vec![axial_origin, axial_direction])?,
        },
        _ => return Err(RootIsolationFailure::ZeroPolynomial),
    };

    Ok(ExactLineTorusPolynomials { surface, auxiliary })
}

fn exact_dot_difference(
    point: [f64; 3],
    origin: [f64; 3],
    direction: [f64; 3],
) -> core::result::Result<ExactScalar, RootIsolationFailure> {
    let mut sum = ExactScalar::zero();
    for axis in 0..3 {
        let direction = ExactScalar::from_f64(direction[axis])?;
        let point = ExactScalar::from_f64(point[axis])?;
        let origin = ExactScalar::from_f64(origin[axis])?;
        sum = sum.add(&direction.mul(&point)?)?;
        sum = sum.sub(&direction.mul(&origin)?)?;
    }
    Ok(sum)
}

fn exact_dot(
    lhs: [f64; 3],
    rhs: [f64; 3],
) -> core::result::Result<ExactScalar, RootIsolationFailure> {
    let mut sum = ExactScalar::zero();
    for axis in 0..3 {
        let lhs = ExactScalar::from_f64(lhs[axis])?;
        let rhs = ExactScalar::from_f64(rhs[axis])?;
        sum = sum.add(&lhs.mul(&rhs)?)?;
    }
    Ok(sum)
}

fn exact_squared_distance(
    point: [f64; 3],
    origin: [f64; 3],
) -> core::result::Result<ExactScalar, RootIsolationFailure> {
    let mut sum = ExactScalar::zero();
    for axis in 0..3 {
        let point = ExactScalar::from_f64(point[axis])?;
        let origin = ExactScalar::from_f64(origin[axis])?;
        sum = sum.add(&point.mul(&point)?)?;
        sum = sum.sub(&point.mul(&origin)?.scale(2.0)?)?;
        sum = sum.add(&origin.mul(&origin)?)?;
    }
    Ok(sum)
}

fn torus_uv(
    local: Vec3,
    torus: &Torus,
    torus_range: [ParamRange; 2],
    tolerances: Tolerances,
) -> Option<[f64; 2]> {
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

fn fit_line_parameter(candidate: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if candidate < range.lo - tolerance || candidate > range.hi + tolerance {
        None
    } else {
        Some(candidate.clamp(range.lo, range.hi))
    }
}

fn parameter_tolerance(radius: f64, tolerances: Tolerances) -> f64 {
    (tolerances.linear() / radius).max(tolerances.angular())
}

fn push_distinct(
    points: &mut Vec<CurveSurfacePoint>,
    candidate: CurveSurfacePoint,
    tolerances: Tolerances,
) -> bool {
    if points
        .iter()
        .any(|point| point.point.dist(candidate.point) <= tolerances.linear())
    {
        return false;
    }
    points.push(candidate);
    true
}

fn tolerance_expanded_line_range(range: ParamRange, tolerance: f64) -> Option<ParamRange> {
    let lo = range.lo - tolerance;
    let hi = range.hi + tolerance;
    (lo.is_finite() && hi.is_finite() && lo <= hi).then_some(ParamRange::new(lo, hi))
}

fn torus_ranges_are_full_period(ranges: [ParamRange; 2]) -> bool {
    ranges.iter().all(|range| {
        let exact_span = ExactScalar::from_f64(range.hi)
            .and_then(|hi| hi.sub(&ExactScalar::from_f64(range.lo)?))
            .and_then(|span| span.sub(&ExactScalar::from_f64(core::f64::consts::TAU)?));
        exact_span.is_ok_and(|difference| difference.sign() >= 0)
    })
}

fn interval_is_finite(interval: Interval) -> bool {
    interval.lo().is_finite() && interval.hi().is_finite()
}

fn validate_ranges(line_range: ParamRange, torus_range: [ParamRange; 2]) -> Result<()> {
    if !line_range.is_finite() || line_range.width() < 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "line/torus intersection requires a finite non-reversed line range",
        });
    }
    if torus_range
        .iter()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(Error::InvalidGeometry {
            reason: "line/torus intersection requires finite non-reversed surface ranges",
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use kgeom::frame::Frame;
    use kgeom::vec::Point3;

    #[test]
    fn squared_stationarity_opposite_factor_is_certified_extraneous() {
        let torus = Torus::new(Frame::world(), 2.0, 0.5).unwrap();
        let line = Line::new(
            Point3::new(
                1.662_094_538_393_015_2,
                -3.583_380_094_172_665_2,
                -1.790_017_403_363_168_4,
            ),
            Vec3::new(
                0.699_371_238_909_804_2,
                -0.609_288_274_968_886_8,
                -0.373_694_618_868_406_7,
            ),
        )
        .unwrap();
        let parameter = -3.949_651_944_209_787;
        let context = TorusLineContext {
            line: &line,
            line_range: ParamRange::new(-5.0, -3.0),
            torus: &torus,
            torus_range: [
                ParamRange::new(0.0, core::f64::consts::TAU),
                ParamRange::new(0.0, core::f64::consts::TAU),
            ],
            local_origin: torus.frame().to_local(line.origin()),
            local_direction: line.dir(),
            tolerances: Tolerances::with_linear(1.0e-4).unwrap(),
        };
        assert_eq!(
            context.stationary_factor_contains_zero(RootBracket {
                lo: parameter,
                hi: parameter,
            }),
            Some(false)
        );
    }
}
