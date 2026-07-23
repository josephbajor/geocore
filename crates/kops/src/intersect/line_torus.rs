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
use kgraph::exact::bounded_polynomial::{
    ExactPolynomial, ExactScalar, RootBracket, RootIsolation, RootIsolationFailure,
};

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

struct AuthoredAxis {
    /// Exact center-plane polynomial in line parameter space: `w ± t`.
    center_plane: ExactPolynomial,
    /// The uniquely replayed local frame offset `w`.
    offset: ExactScalar,
    direction_sign: f64,
    /// Exact squared displacement between the stored line origin and the
    /// ideal affine point `frame.origin + w * frame.z`.
    replay_error_squared: ExactScalar,
}

/// Intersect a line restricted to a finite range with a finite torus
/// parameter window.
///
/// The general torus implicit equation reduces to a quartic in the line
/// parameter. A bounded exact pseudo-Sturm classifier owns its distinct-root
/// topology over the tolerance-expanded line range. A second exact quartic
/// covers differentiable torus-distance extrema; its unsquared factor, the
/// radial-axis quadratic, and both domain endpoints complete the
/// tolerance-candidate proof. Lines authored by uniquely replaying
/// `Frame::point_at(0, 0, w)` with direction `±frame.z` use a separate exact
/// bounded-clearance certificate that includes the stored replay displacement.
/// As with the general analytic reduction, this certificate consumes the
/// kernel's semantic-orthonormal [`kgeom::frame::Frame`] contract.
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
    match authored_axis_from_unique_replay(line, torus) {
        Ok(Some(axis)) => {
            if !authored_axis_segment_is_outside_tolerance(
                &axis,
                line_range,
                torus,
                tolerances.linear(),
            )
            .unwrap_or(false)
            {
                complete = false;
                let _ = context.admit_tolerance_critical_points(
                    &mut points,
                    &axis.center_plane,
                    isolation_range,
                    false,
                );
                let _ = context.admit_tolerance_endpoints(&mut points);
            }
        }
        Ok(None) => match exact_line_torus_polynomials(line, torus) {
            Ok(polynomials) => {
                if !admit_exact_line_torus_polynomials(
                    &context,
                    &mut points,
                    isolation_range,
                    &polynomials,
                ) {
                    complete = false;
                }
            }
            Err(_) => complete = false,
        },
        Err(_) => match exact_line_torus_polynomials(line, torus) {
            Ok(polynomials)
                if matches!(&polynomials.auxiliary, ExactLineTorusAuxiliary::Axis { .. }) =>
            {
                if !admit_exact_line_torus_polynomials(
                    &context,
                    &mut points,
                    isolation_range,
                    &polynomials,
                ) {
                    complete = false;
                }
            }
            Ok(_) | Err(_) => {
                complete = false;
                let _ = context.admit_tolerance_endpoints(&mut points);
            }
        },
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

fn admit_exact_line_torus_polynomials(
    context: &TorusLineContext<'_>,
    points: &mut Vec<CurveSurfacePoint>,
    isolation_range: ParamRange,
    polynomials: &ExactLineTorusPolynomials,
) -> bool {
    let mut complete = true;
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
                    .add_root_bracket(points, root, is_repeated_root)
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
                    points,
                    critical_polynomial,
                    isolation_range,
                    requires_unsquared_stationarity,
                ) {
                    complete = false;
                }
            }
        }
        ExactLineTorusAuxiliary::Axis { center_plane } => {
            // If the squared radial distance q(t) is identically zero, the
            // generic squared stationarity polynomial is also identically
            // zero. The center-plane crossing is its sole interior
            // distance-minimum candidate.
            if !context.admit_tolerance_critical_points(
                points,
                center_plane,
                isolation_range,
                false,
            ) {
                complete = false;
            }
        }
    }
    if !context.admit_tolerance_endpoints(points) {
        complete = false;
    }
    complete
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

/// Recover a narrow Frame-semantic authorship identity.
///
/// This is deliberately not a tolerance-parallel or source-world affine test.
/// Direction components must equal `±frame.z`, the stored origin must replay
/// through `Frame::point_at`, every replaying coordinate quotient must agree,
/// and adjacent floating-point offsets must not replay the same stored point.
fn authored_axis_from_unique_replay(
    line: &Line,
    torus: &Torus,
) -> core::result::Result<Option<AuthoredAxis>, RootIsolationFailure> {
    if !line_torus_contract_is_valid(line, torus) {
        return Err(RootIsolationFailure::UnsafeArithmeticEnvelope);
    }
    let frame = torus.frame();
    let direction_sign = if line.dir() == frame.z() {
        1.0
    } else if line.dir() == -frame.z() {
        -1.0
    } else {
        return Ok(None);
    };

    let origin = line.origin().to_array();
    let center = frame.origin().to_array();
    let axis = frame.z().to_array();
    let mut recovered_offset: Option<f64> = None;
    for coordinate in 0..3 {
        if axis[coordinate] == 0.0 {
            continue;
        }
        let candidate = (origin[coordinate] - center[coordinate]) / axis[coordinate];
        if !candidate.is_finite() || frame.point_at(0.0, 0.0, candidate) != line.origin() {
            continue;
        }
        if recovered_offset.is_some_and(|offset| offset != candidate) {
            return Err(RootIsolationFailure::ParameterResolution);
        }
        recovered_offset = Some(candidate);
    }
    let Some(recovered_offset) = recovered_offset else {
        return Ok(None);
    };
    if frame.point_at(0.0, 0.0, recovered_offset.next_down()) == line.origin()
        || frame.point_at(0.0, 0.0, recovered_offset.next_up()) == line.origin()
    {
        return Err(RootIsolationFailure::ParameterResolution);
    }

    let offset = ExactScalar::from_f64(recovered_offset)?;
    let mut replay_error_squared = ExactScalar::zero();
    for coordinate in 0..3 {
        let error = ExactScalar::from_f64(origin[coordinate])?
            .sub(&ExactScalar::from_f64(center[coordinate])?)?
            .sub(&ExactScalar::from_f64(axis[coordinate])?.mul(&offset)?)?;
        replay_error_squared = replay_error_squared.add(&error.mul(&error)?)?;
    }
    let center_plane =
        ExactPolynomial::new(vec![offset.clone(), ExactScalar::from_f64(direction_sign)?])?;
    Ok(Some(AuthoredAxis {
        center_plane,
        offset,
        direction_sign,
        replay_error_squared,
    }))
}

fn authored_axis_segment_is_outside_tolerance(
    axis: &AuthoredAxis,
    line_range: ParamRange,
    torus: &Torus,
    tolerance: f64,
) -> core::result::Result<bool, RootIsolationFailure> {
    let endpoint_value = |parameter: f64| {
        axis.offset
            .add(&ExactScalar::from_f64(parameter)?.scale(axis.direction_sign)?)
    };
    let lo = endpoint_value(line_range.lo)?;
    let hi = endpoint_value(line_range.hi)?;
    let minimum_axial_squared = if lo.is_zero() || hi.is_zero() || lo.sign() != hi.sign() {
        ExactScalar::zero()
    } else {
        let lo_squared = lo.mul(&lo)?;
        let hi_squared = hi.mul(&hi)?;
        if lo_squared.sub(&hi_squared)?.sign() <= 0 {
            lo_squared
        } else {
            hi_squared
        }
    };

    let major = ExactScalar::from_f64(torus.major_radius())?;
    let clearance =
        ExactScalar::from_f64(torus.minor_radius())?.add(&ExactScalar::from_f64(tolerance)?)?;
    let clearance_squared = clearance.mul(&clearance)?;
    let ideal_squared = minimum_axial_squared.add(&major.mul(&major)?)?;
    // For A = R² + min(h²), B = r + tolerance, and E = |e|², prove
    // sqrt(A) > B + sqrt(E). The two exact strict inequalities below are the
    // radical-free form of that bound. By distance Lipschitz continuity this
    // certifies the stored replayed line segment outside the tolerance tube.
    let margin = ideal_squared
        .sub(&clearance_squared)?
        .sub(&axis.replay_error_squared)?;
    if margin.sign() <= 0 {
        return Ok(false);
    }
    let squared_margin = margin.mul(&margin)?;
    let replay_cross_term = clearance_squared
        .mul(&axis.replay_error_squared)?
        .scale(4.0)?;
    Ok(squared_margin.sub(&replay_cross_term)?.sign() > 0)
}

fn exact_line_torus_polynomials(
    line: &Line,
    torus: &Torus,
) -> core::result::Result<ExactLineTorusPolynomials, RootIsolationFailure> {
    if !line_torus_contract_is_valid(line, torus) {
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

fn line_torus_contract_is_valid(line: &Line, torus: &Torus) -> bool {
    let direction_norm = line.dir().norm();
    direction_norm.is_finite()
        && (direction_norm - 1.0).abs() <= 16.0 * ANGULAR_RESOLUTION
        && torus.frame().is_orthonormal()
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

    fn tilted_authored_axis_fixture() -> (Frame, Torus, Line) {
        let frame = Frame::new(
            Point3::new(1.0 / 1024.0, -1.0 / 2048.0, 1.0 / 4096.0),
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let torus = Torus::new(frame, 1.0 + 2.0_f64.powi(-20), 1.0).unwrap();
        let point = frame.point_at(0.0, 0.0, -1.0 / 128.0);
        let line = Line::new(point, frame.z()).unwrap();
        (frame, torus, line)
    }

    #[test]
    fn tilted_frame_axis_requires_unique_component_exact_replay() {
        let (frame, torus, line) = tilted_authored_axis_fixture();
        let authored = authored_axis_from_unique_replay(&line, &torus).unwrap();
        assert!(authored.is_some());
        assert!(matches!(
            exact_line_torus_polynomials(&line, &torus)
                .unwrap()
                .auxiliary,
            ExactLineTorusAuxiliary::General { .. }
        ));

        let z = frame.z();
        let direction_control =
            Line::new(line.origin(), Vec3::new(z.x.next_up(), z.y, z.z)).unwrap();
        assert!(
            authored_axis_from_unique_replay(&direction_control, &torus)
                .unwrap()
                .is_none()
        );

        let point = line.origin();
        let origin_control =
            Line::new(Point3::new(point.x.next_up(), point.y, point.z), frame.z()).unwrap();
        assert!(
            authored_axis_from_unique_replay(&origin_control, &torus)
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn tilted_frame_axis_rejects_replay_plateaus() {
        let frame = Frame::new(
            Point3::new(1.0, -2.0, 3.0),
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let torus = Torus::new(frame, 2.0, 0.5).unwrap();
        let line = Line::new(frame.origin(), frame.z()).unwrap();

        assert!(matches!(
            authored_axis_from_unique_replay(&line, &torus),
            Err(RootIsolationFailure::ParameterResolution)
        ));
        assert!(matches!(
            exact_line_torus_polynomials(&line, &torus)
                .unwrap()
                .auxiliary,
            ExactLineTorusAuxiliary::General { .. }
        ));
    }

    #[test]
    fn replay_error_correction_can_reject_an_ideal_axis_clearance() {
        let torus = Torus::new(Frame::world(), 2.0, 1.0).unwrap();
        let offset = ExactScalar::zero();
        let axis = AuthoredAxis {
            center_plane: ExactPolynomial::new(vec![
                offset.clone(),
                ExactScalar::from_f64(1.0).unwrap(),
            ])
            .unwrap(),
            offset,
            direction_sign: 1.0,
            // sqrt(E) = 1/4, so the ideal center clearance 2 - 1 is
            // positive, but exactly equals tolerance + sqrt(E).
            replay_error_squared: ExactScalar::from_f64(1.0 / 16.0).unwrap(),
        };

        assert!(
            !authored_axis_segment_is_outside_tolerance(
                &axis,
                ParamRange::new(0.0, 0.0),
                &torus,
                3.0 / 4.0,
            )
            .unwrap()
        );
    }

    #[test]
    fn exact_coefficient_axis_survives_an_authored_replay_plateau() {
        let frame = Frame::new(
            Point3::new(1.0 / 8.0, -1.0 / 4.0, 1.0 / 2.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let torus = Torus::new(frame, 1.0 + 2.0_f64.powi(-20), 1.0).unwrap();
        let line = Line::new(frame.point_at(0.0, 0.0, -1.0 / 128.0), frame.z()).unwrap();

        assert!(matches!(
            authored_axis_from_unique_replay(&line, &torus),
            Err(RootIsolationFailure::ParameterResolution)
        ));
        assert!(matches!(
            exact_line_torus_polynomials(&line, &torus)
                .unwrap()
                .auxiliary,
            ExactLineTorusAuxiliary::Axis { .. }
        ));
    }

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
