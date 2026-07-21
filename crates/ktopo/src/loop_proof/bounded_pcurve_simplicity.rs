//! Structural simplicity proof for bounded analytic pcurve loops.
//!
//! Every decision is made from exact expansion arithmetic, robust predicates,
//! or outward intervals over the authored Line2d/Circle2d carriers. The
//! algorithm enumerates span pairs; it never recognizes named loop layouts.

use super::bounded_pcurve_integral::BoundedPcurveSpan;
use kcore::expansion;
use kcore::interval::Interval;
use kcore::math;
use kcore::predicates::Orientation;
use kgeom::vec::Point2;

const MAX_SAFE_SCALAR: f64 = f64::from_bits(((1023 + 450) as u64) << 52);
const MIN_SAFE_SCALAR: f64 = f64::from_bits(((1023 - 450) as u64) << 52);
const MAX_PERIOD_INDEX: i64 = 1_i64 << 40;

/// One span plus exact topology endpoint identities.
#[derive(Debug, Clone, Copy)]
pub(crate) struct BoundedLoopSpan<'a, K> {
    geometry: BoundedPcurveSpan<'a>,
    tail: K,
    head: K,
    head_join: Option<CertifiedBoundedLoopJoin>,
}

/// Proof token minted only after topology-owned identity, whole-fin
/// incidence, and surface-lifted endpoint distance certify one chart join.
/// The parameter neighborhood is a conservative local radius in the
/// normalized Line2d parameter metric; it is never sufficient by itself to
/// discharge an intersection.
#[derive(Debug, Clone, Copy)]
pub(super) struct CertifiedBoundedLoopJoin {
    parameter_neighborhood: f64,
}

impl CertifiedBoundedLoopJoin {
    pub(super) fn new(parameter_neighborhood: f64) -> Option<Self> {
        (parameter_neighborhood.is_finite() && parameter_neighborhood >= 0.0).then_some(Self {
            parameter_neighborhood,
        })
    }
}

impl<'a, K: Copy> BoundedLoopSpan<'a, K> {
    pub(crate) const fn new(geometry: BoundedPcurveSpan<'a>, tail: K, head: K) -> Self {
        Self {
            geometry,
            tail,
            head,
            head_join: None,
        }
    }

    pub(super) const fn with_head_join(mut self, evidence: CertifiedBoundedLoopJoin) -> Self {
        self.head_join = Some(evidence);
        self
    }

    pub(crate) const fn geometry(self) -> BoundedPcurveSpan<'a> {
        self.geometry
    }

    pub(super) const fn tail(self) -> K {
        self.tail
    }

    pub(super) const fn head(self) -> K {
        self.head
    }
}

/// Typed reason that a complete simplicity proof was unavailable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum BoundedLoopSimplicityGap {
    TooFewSpans,
    NonFiniteInput { span_index: usize },
    DegenerateSpan { span_index: usize },
    UnsupportedCurve { span_index: usize },
    TopologyDiscontinuity { span_index: usize },
    ChartDiscontinuity { span_index: usize },
    ArithmeticGuard { left: usize, right: usize },
    AmbiguousRoot { left: usize, right: usize },
    CoincidentCarrier { left: usize, right: usize },
    PairWorkOverflow,
}

/// Complete result of the bounded analytic loop simplicity proof.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum BoundedLoopSimplicity {
    Certified,
    SelfIntersecting,
    Indeterminate(BoundedLoopSimplicityGap),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PairRelation {
    Disjoint,
    ForbiddenIntersection,
    Indeterminate,
    Coincident,
}

#[derive(Debug, Clone, Copy)]
struct RootUse {
    parameter: Interval,
    fully_inside: bool,
}

#[derive(Debug, Clone, Copy)]
enum RootMembership {
    Outside,
    Candidate(RootUse),
    Ambiguous,
}

#[derive(Debug, Clone, Copy)]
struct CertifiedSharedJoin {
    left_parameter: f64,
    right_parameter: f64,
    evidence: CertifiedBoundedLoopJoin,
}

/// Certify a finite Line2d/Circle2d loop simple.
pub(crate) fn certify_bounded_loop_simplicity<K: Copy + Eq>(
    spans: &[BoundedLoopSpan<'_, K>],
) -> BoundedLoopSimplicity {
    if spans.len() < 2 {
        return BoundedLoopSimplicity::Indeterminate(BoundedLoopSimplicityGap::TooFewSpans);
    }
    if spans
        .len()
        .checked_mul(spans.len().saturating_sub(1))
        .and_then(|work| work.checked_div(2))
        .is_none()
    {
        return BoundedLoopSimplicity::Indeterminate(BoundedLoopSimplicityGap::PairWorkOverflow);
    }
    if let Err(gap) = validate_loop(spans) {
        return BoundedLoopSimplicity::Indeterminate(gap);
    }

    let mut first_gap = None;
    for left in 0..spans.len() {
        for right in left + 1..spans.len() {
            let relation = match pair_relation(spans, left, right) {
                Ok(relation) => relation,
                Err(()) => {
                    first_gap
                        .get_or_insert(BoundedLoopSimplicityGap::ArithmeticGuard { left, right });
                    continue;
                }
            };
            match relation {
                PairRelation::Disjoint => {}
                PairRelation::ForbiddenIntersection => {
                    return BoundedLoopSimplicity::SelfIntersecting;
                }
                PairRelation::Coincident => {
                    first_gap
                        .get_or_insert(BoundedLoopSimplicityGap::CoincidentCarrier { left, right });
                }
                PairRelation::Indeterminate => {
                    first_gap
                        .get_or_insert(BoundedLoopSimplicityGap::AmbiguousRoot { left, right });
                }
            }
        }
    }
    first_gap.map_or(
        BoundedLoopSimplicity::Certified,
        BoundedLoopSimplicity::Indeterminate,
    )
}

fn validate_loop<K: Copy + Eq>(
    spans: &[BoundedLoopSpan<'_, K>],
) -> core::result::Result<(), BoundedLoopSimplicityGap> {
    let mut endpoints = Vec::with_capacity(spans.len());
    for (index, span) in spans.iter().copied().enumerate() {
        let geometry = span.geometry;
        if !geometry.start().is_finite()
            || !geometry.end().is_finite()
            || !finite_point(geometry.chart_offset())
        {
            return Err(BoundedLoopSimplicityGap::NonFiniteInput { span_index: index });
        }
        if geometry.start() == geometry.end() {
            return Err(BoundedLoopSimplicityGap::DegenerateSpan { span_index: index });
        }
        if !matches!(
            geometry.curve(),
            crate::geom::Curve2dGeom::Line(_) | crate::geom::Curve2dGeom::Circle(_)
        ) {
            return Err(BoundedLoopSimplicityGap::UnsupportedCurve { span_index: index });
        }
        let curve = geometry.curve().as_curve();
        let start = translated(curve.eval(geometry.start()), geometry.chart_offset())
            .ok_or(BoundedLoopSimplicityGap::NonFiniteInput { span_index: index })?;
        let end = translated(curve.eval(geometry.end()), geometry.chart_offset())
            .ok_or(BoundedLoopSimplicityGap::NonFiniteInput { span_index: index })?;
        endpoints.push((start, end));
    }
    for index in 0..spans.len() {
        let next = (index + 1) % spans.len();
        if spans[index].head != spans[next].tail {
            return Err(BoundedLoopSimplicityGap::TopologyDiscontinuity { span_index: index });
        }
        if !points_bit_equal(endpoints[index].1, endpoints[next].0)
            && spans[index].head_join.is_none()
        {
            return Err(BoundedLoopSimplicityGap::ChartDiscontinuity { span_index: index });
        }
    }
    Ok(())
}

fn pair_relation<K: Copy + Eq>(
    spans: &[BoundedLoopSpan<'_, K>],
    left_index: usize,
    right_index: usize,
) -> core::result::Result<PairRelation, ()> {
    let left = spans[left_index];
    let right = spans[right_index];
    let shared = shared_endpoint_parameters(left, right);
    match (left.geometry.curve(), right.geometry.curve()) {
        (crate::geom::Curve2dGeom::Line(_), crate::geom::Curve2dGeom::Line(_)) => {
            let certified_joins = certified_shared_joins(left, right);
            line_line_relation(left.geometry, right.geometry, &shared, &certified_joins)
        }
        (crate::geom::Curve2dGeom::Line(_), crate::geom::Curve2dGeom::Circle(_)) => {
            line_circle_relation(left.geometry, right.geometry, &shared)
        }
        (crate::geom::Curve2dGeom::Circle(_), crate::geom::Curve2dGeom::Line(_)) => {
            let reversed = shared
                .iter()
                .map(|&(left, right)| (right, left))
                .collect::<Vec<_>>();
            line_circle_relation(right.geometry, left.geometry, &reversed)
        }
        (crate::geom::Curve2dGeom::Circle(_), crate::geom::Curve2dGeom::Circle(_)) => {
            circle_circle_relation(left.geometry, right.geometry, &shared)
        }
        _ => Err(()),
    }
}

fn certified_shared_joins<K: Copy + Eq>(
    left: BoundedLoopSpan<'_, K>,
    right: BoundedLoopSpan<'_, K>,
) -> Vec<CertifiedSharedJoin> {
    let mut joins = Vec::with_capacity(2);
    if left.head == right.tail
        && let Some(evidence) = left.head_join
    {
        joins.push(CertifiedSharedJoin {
            left_parameter: left.geometry.end(),
            right_parameter: right.geometry.start(),
            evidence,
        });
    }
    if right.head == left.tail
        && let Some(evidence) = right.head_join
    {
        joins.push(CertifiedSharedJoin {
            left_parameter: left.geometry.start(),
            right_parameter: right.geometry.end(),
            evidence,
        });
    }
    joins
}

fn shared_endpoint_parameters<K: Copy + Eq>(
    left: BoundedLoopSpan<'_, K>,
    right: BoundedLoopSpan<'_, K>,
) -> Vec<(f64, f64)> {
    let mut shared = Vec::with_capacity(2);
    for (left_key, left_parameter) in [
        (left.tail, left.geometry.start()),
        (left.head, left.geometry.end()),
    ] {
        for (right_key, right_parameter) in [
            (right.tail, right.geometry.start()),
            (right.head, right.geometry.end()),
        ] {
            if left_key == right_key {
                shared.push((left_parameter, right_parameter));
            }
        }
    }
    shared
}

fn line_line_relation(
    left: BoundedPcurveSpan<'_>,
    right: BoundedPcurveSpan<'_>,
    shared: &[(f64, f64)],
    certified_joins: &[CertifiedSharedJoin],
) -> core::result::Result<PairRelation, ()> {
    let left_line = left.curve().as_line().ok_or(())?;
    let right_line = right.curve().as_line().ok_or(())?;
    let left_origin = exact_translated(left_line.origin(), left.chart_offset())?;
    let right_origin = exact_translated(right_line.origin(), right.chart_offset())?;
    let left_direction = exact_point(left_line.dir())?;
    let right_direction = exact_point(right_line.dir())?;
    let offset = sub_vec(&right_origin, &left_origin)?;
    let determinant = cross_exact(&left_direction, &right_direction)?;
    if determinant.sign() != Orientation::Zero {
        // Distinct nonparallel lines have exactly one carrier intersection.
        // An exact topology-owned common endpoint that evaluates to identical
        // chart bits therefore exhausts that intersection even when interval
        // division cannot enclose the rounded authored endpoint parameter.
        if shared.iter().any(|&(left_parameter, right_parameter)| {
            endpoint_point(left, left_parameter).is_some_and(|left_point| {
                endpoint_point(right, right_parameter)
                    .is_some_and(|right_point| points_bit_equal(left_point, right_point))
            })
        }) {
            return Ok(PairRelation::Disjoint);
        }
        let left_root = ratio_interval(&cross_exact(&offset, &right_direction)?, &determinant)?;
        let right_root = ratio_interval(&cross_exact(&offset, &left_direction)?, &determinant)?;
        let authorized = certified_joins
            .iter()
            .copied()
            .any(|join| certified_join_confines_roots(left_root, right_root, join));
        if authorized {
            return Ok(PairRelation::Disjoint);
        }
        // A topology identity alone does not authorize a rounded near join.
        // Exact evaluated endpoints were handled above, and certified near
        // joins must pass the complete root-neighborhood proof. Classify every
        // remaining root as an ordinary intersection.
        return Ok(classify_root_pair(left, left_root, right, right_root, &[]));
    }
    if cross_exact(&offset, &left_direction)?.sign() != Orientation::Zero {
        return Ok(PairRelation::Disjoint);
    }
    coincident_line_relation(left, right, shared, &left_origin, &left_direction)
}

fn certified_join_confines_roots(
    left_root: Interval,
    right_root: Interval,
    join: CertifiedSharedJoin,
) -> bool {
    let radius = join.evidence.parameter_neighborhood;
    if !radius.is_finite() || radius < 0.0 {
        return false;
    }
    let neighborhood = Interval::new(-radius, radius);
    let left_neighborhood = Interval::point(join.left_parameter) + neighborhood;
    let right_neighborhood = Interval::point(join.right_parameter) + neighborhood;
    left_root.lo() >= left_neighborhood.lo()
        && left_root.hi() <= left_neighborhood.hi()
        && right_root.lo() >= right_neighborhood.lo()
        && right_root.hi() <= right_neighborhood.hi()
}

fn coincident_line_relation(
    left: BoundedPcurveSpan<'_>,
    right: BoundedPcurveSpan<'_>,
    shared: &[(f64, f64)],
    left_origin: &[Exact; 2],
    left_direction: &[Exact; 2],
) -> core::result::Result<PairRelation, ()> {
    if left.curve() == right.curve() && points_bit_equal(left.chart_offset(), right.chart_offset())
    {
        return exact_interval_overlap_relation(
            active_interval(left)?,
            active_interval(right)?,
            shared,
        );
    }
    let denominator = dot_exact(left_direction, left_direction)?;
    let right_line = right.curve().as_line().ok_or(())?;
    let right_origin = exact_translated(right_line.origin(), right.chart_offset())?;
    let right_direction = exact_point(right_line.dir())?;
    let projected_origin = ratio_interval(
        &dot_exact(&sub_vec(&right_origin, left_origin)?, left_direction)?,
        &denominator,
    )?;
    let projected_rate =
        ratio_interval(&dot_exact(&right_direction, left_direction)?, &denominator)?;
    let right_start = projected_origin + projected_rate * Interval::point(right.start());
    let right_end = projected_origin + projected_rate * Interval::point(right.end());
    if !finite_interval(right_start) || !finite_interval(right_end) {
        return Err(());
    }
    let first = active_interval(left)?;
    let second = Interval::new(
        right_start.lo().min(right_end.lo()),
        right_start.hi().max(right_end.hi()),
    );
    if first.hi() < second.lo() || second.hi() < first.lo() {
        return Ok(PairRelation::Disjoint);
    }
    if first.hi().min(second.hi()) > first.lo().max(second.lo()) {
        return Ok(PairRelation::ForbiddenIntersection);
    }
    if shared.iter().any(|&(left_parameter, right_parameter)| {
        first.contains(left_parameter)
            && second.contains(left_parameter)
            && active_interval(right).is_ok_and(|range| range.contains(right_parameter))
    }) {
        Ok(PairRelation::Disjoint)
    } else {
        Ok(PairRelation::Indeterminate)
    }
}

fn line_circle_relation(
    line_span: BoundedPcurveSpan<'_>,
    circle_span: BoundedPcurveSpan<'_>,
    shared: &[(f64, f64)],
) -> core::result::Result<PairRelation, ()> {
    if !circle_span_is_injective(circle_span)? {
        return Ok(PairRelation::ForbiddenIntersection);
    }
    // A line and a nondegenerate circle have at most two intersections. Two
    // distinct topology-owned common endpoints therefore exhaust the carrier
    // intersection set without reconstructing either endpoint numerically.
    if shared.len() == 2
        && shared[0].0.to_bits() != shared[1].0.to_bits()
        && shared[0].1.to_bits() != shared[1].1.to_bits()
    {
        return Ok(PairRelation::Disjoint);
    }
    let line = line_span.curve().as_line().ok_or(())?;
    let circle = circle_span.curve().as_circle().ok_or(())?;
    let origin = exact_translated(line.origin(), line_span.chart_offset())?;
    let direction = exact_point(line.dir())?;
    let center = exact_translated(circle.center(), circle_span.chart_offset())?;
    let cosine = scale_vec(&exact_point(circle.x_dir())?, circle.radius())?;
    let sine = [cosine[1].neg()?, cosine[0].clone()];
    let normal = [direction[1].neg()?, direction[0].clone()];
    let constant = dot_exact(&normal, &sub_vec(&center, &origin)?)?;
    let roots = solve_harmonic(
        &dot_exact(&normal, &cosine)?,
        &dot_exact(&normal, &sine)?,
        &constant,
    )?;
    if roots.identity {
        return Ok(PairRelation::Coincident);
    }
    let denominator = dot_exact(&direction, &direction)?.interval()?;
    let mut uncertain = false;
    for root in roots.roots {
        let circle_membership = root_membership(root, circle_span)?;
        let RootMembership::Candidate(circle_use) = circle_membership else {
            uncertain |= matches!(circle_membership, RootMembership::Ambiguous);
            continue;
        };
        let point = circle_point_interval(&center, &cosine, &sine, circle_use.parameter)?;
        let numerator = interval_dot(
            interval_sub(point, exact_vec_interval(&origin)?),
            exact_vec_interval(&direction)?,
        );
        let line_root = numerator.checked_div(denominator).ok_or(())?;
        match classify_root_pair(
            line_span,
            line_root,
            circle_span,
            circle_use.parameter,
            shared,
        ) {
            PairRelation::ForbiddenIntersection => {
                return Ok(PairRelation::ForbiddenIntersection);
            }
            PairRelation::Indeterminate => uncertain = true,
            PairRelation::Disjoint => {}
            PairRelation::Coincident => return Ok(PairRelation::Coincident),
        }
    }
    Ok(if uncertain {
        PairRelation::Indeterminate
    } else {
        PairRelation::Disjoint
    })
}

fn circle_circle_relation(
    left: BoundedPcurveSpan<'_>,
    right: BoundedPcurveSpan<'_>,
    shared: &[(f64, f64)],
) -> core::result::Result<PairRelation, ()> {
    if !circle_span_is_injective(left)? || !circle_span_is_injective(right)? {
        return Ok(PairRelation::ForbiddenIntersection);
    }
    let left_circle = left.curve().as_circle().ok_or(())?;
    let right_circle = right.curve().as_circle().ok_or(())?;
    let left_center = exact_translated(left_circle.center(), left.chart_offset())?;
    let right_center = exact_translated(right_circle.center(), right.chart_offset())?;
    let left_cosine = scale_vec(&exact_point(left_circle.x_dir())?, left_circle.radius())?;
    let left_sine = [left_cosine[1].neg()?, left_cosine[0].clone()];
    let right_cosine = scale_vec(&exact_point(right_circle.x_dir())?, right_circle.radius())?;
    let right_sine = [right_cosine[1].neg()?, right_cosine[0].clone()];
    let offset = sub_vec(&left_center, &right_center)?;
    let left_radius_sq = dot_exact(&left_cosine, &left_cosine)?;
    let right_radius_sq = dot_exact(&right_cosine, &right_cosine)?;
    if offset
        .iter()
        .all(|coordinate| coordinate.sign() == Orientation::Zero)
    {
        let radius_difference = left_radius_sq.sub(&right_radius_sq)?;
        if radius_difference.sign() != Orientation::Zero {
            return Ok(PairRelation::Disjoint);
        }
        return coincident_circle_relation(left, right, shared);
    }
    let constant = dot_exact(&offset, &offset)?
        .add(&left_radius_sq)?
        .sub(&right_radius_sq)?;
    let left_roots = solve_harmonic(
        &dot_exact(&offset, &left_cosine)?.scale(2.0)?,
        &dot_exact(&offset, &left_sine)?.scale(2.0)?,
        &constant,
    )?;
    let reverse_offset = offset
        .map(|coordinate| coordinate.neg())
        .transpose_array()?;
    let reverse_constant = dot_exact(&reverse_offset, &reverse_offset)?
        .add(&right_radius_sq)?
        .sub(&left_radius_sq)?;
    let right_roots = solve_harmonic(
        &dot_exact(&reverse_offset, &right_cosine)?.scale(2.0)?,
        &dot_exact(&reverse_offset, &right_sine)?.scale(2.0)?,
        &reverse_constant,
    )?;
    if left_roots.identity || right_roots.identity {
        return Ok(PairRelation::Coincident);
    }
    match_circle_roots(
        left,
        right,
        shared,
        &left_center,
        &left_cosine,
        &left_sine,
        left_roots.roots,
        &right_center,
        &right_cosine,
        &right_sine,
        right_roots.roots,
    )
}

trait TransposeArray<T> {
    fn transpose_array(self) -> core::result::Result<[T; 2], ()>;
}

impl<T> TransposeArray<T> for [core::result::Result<T, ()>; 2] {
    fn transpose_array(self) -> core::result::Result<[T; 2], ()> {
        let [first, second] = self;
        Ok([first?, second?])
    }
}

#[allow(clippy::too_many_arguments)]
fn match_circle_roots(
    left: BoundedPcurveSpan<'_>,
    right: BoundedPcurveSpan<'_>,
    shared: &[(f64, f64)],
    left_center: &[Exact; 2],
    left_cosine: &[Exact; 2],
    left_sine: &[Exact; 2],
    left_roots: Vec<Interval>,
    right_center: &[Exact; 2],
    right_cosine: &[Exact; 2],
    right_sine: &[Exact; 2],
    right_roots: Vec<Interval>,
) -> core::result::Result<PairRelation, ()> {
    if left_roots.len() == 1 && right_roots.len() == 1 {
        return Ok(classify_root_pair(
            left,
            left_roots[0],
            right,
            right_roots[0],
            shared,
        ));
    }
    let left_points = left_roots
        .iter()
        .map(|&root| circle_point_interval(left_center, left_cosine, left_sine, root))
        .collect::<core::result::Result<Vec<_>, _>>()?;
    let right_points = right_roots
        .iter()
        .map(|&root| circle_point_interval(right_center, right_cosine, right_sine, root))
        .collect::<core::result::Result<Vec<_>, _>>()?;
    let mut uncertain = false;
    for (left_root_index, &left_root) in left_roots.iter().enumerate() {
        let matches = right_points
            .iter()
            .enumerate()
            .filter(|(_, point)| boxes_intersect(left_points[left_root_index], **point))
            .map(|(index, _)| index)
            .collect::<Vec<_>>();
        let [right_root_index] = matches.as_slice() else {
            return Ok(PairRelation::Indeterminate);
        };
        let right_root = right_roots[*right_root_index];
        match classify_root_pair(left, left_root, right, right_root, shared) {
            PairRelation::ForbiddenIntersection => {
                return Ok(PairRelation::ForbiddenIntersection);
            }
            PairRelation::Indeterminate => uncertain = true,
            PairRelation::Disjoint => {}
            PairRelation::Coincident => return Ok(PairRelation::Coincident),
        }
    }
    Ok(if uncertain {
        PairRelation::Indeterminate
    } else {
        PairRelation::Disjoint
    })
}

fn coincident_circle_relation(
    left: BoundedPcurveSpan<'_>,
    right: BoundedPcurveSpan<'_>,
    shared: &[(f64, f64)],
) -> core::result::Result<PairRelation, ()> {
    if left.curve() != right.curve() || !points_bit_equal(left.chart_offset(), right.chart_offset())
    {
        return Ok(PairRelation::Coincident);
    }
    let first = active_interval(left)?;
    let second = active_interval(right)?;
    let midpoint = 0.5 * (first.lo() + first.hi()) - 0.5 * (second.lo() + second.hi());
    if !midpoint.is_finite() {
        return Err(());
    }
    let period_index = (midpoint / core::f64::consts::TAU).round();
    if !period_index.is_finite() || period_index.abs() > MAX_PERIOD_INDEX as f64 {
        return Err(());
    }
    let mut ambiguous = false;
    for offset in -1..=1 {
        let index = period_index as i64 + offset;
        let shift = period_shift(index);
        let shifted = shifted_interval(second, index);
        if shifted.hi() < first.lo() || first.hi() < shifted.lo() {
            continue;
        }
        if first.hi().min(shifted.hi()) > first.lo().max(shifted.lo()) {
            return Ok(PairRelation::ForbiddenIntersection);
        }
        let shared_boundary = shared.iter().any(|&(left_parameter, right_parameter)| {
            first.contains(left_parameter)
                && shifted.intersects(Interval::point(right_parameter) + shift)
        });
        if !shared_boundary {
            ambiguous = true;
        }
    }
    Ok(if ambiguous {
        PairRelation::Indeterminate
    } else {
        PairRelation::Disjoint
    })
}

fn classify_root_pair(
    left: BoundedPcurveSpan<'_>,
    left_root: Interval,
    right: BoundedPcurveSpan<'_>,
    right_root: Interval,
    shared: &[(f64, f64)],
) -> PairRelation {
    let left_membership = root_membership(left_root, left);
    let right_membership = root_membership(right_root, right);
    let (Ok(left_membership), Ok(right_membership)) = (left_membership, right_membership) else {
        return PairRelation::Indeterminate;
    };
    if matches!(left_membership, RootMembership::Outside)
        || matches!(right_membership, RootMembership::Outside)
    {
        return PairRelation::Disjoint;
    }
    let (RootMembership::Candidate(left_use), RootMembership::Candidate(right_use)) =
        (left_membership, right_membership)
    else {
        return PairRelation::Indeterminate;
    };
    if shared.iter().any(|&(left_parameter, right_parameter)| {
        left_use.parameter.contains(left_parameter) && right_use.parameter.contains(right_parameter)
    }) {
        return PairRelation::Disjoint;
    }
    let endpoint_touch =
        exact_matching_endpoint_touch(left, left_use.parameter, right, right_use.parameter);
    if endpoint_touch || left_use.fully_inside && right_use.fully_inside {
        PairRelation::ForbiddenIntersection
    } else {
        PairRelation::Indeterminate
    }
}

fn exact_matching_endpoint_touch(
    left: BoundedPcurveSpan<'_>,
    left_root: Interval,
    right: BoundedPcurveSpan<'_>,
    right_root: Interval,
) -> bool {
    [left.start(), left.end()].iter().any(|&left_parameter| {
        left_root.contains(left_parameter)
            && [right.start(), right.end()].iter().any(|&right_parameter| {
                right_root.contains(right_parameter)
                    && endpoint_point(left, left_parameter).is_some_and(|left_point| {
                        endpoint_point(right, right_parameter)
                            .is_some_and(|right_point| points_bit_equal(left_point, right_point))
                    })
            })
    })
}

fn endpoint_point(span: BoundedPcurveSpan<'_>, parameter: f64) -> Option<Point2> {
    translated(span.curve().as_curve().eval(parameter), span.chart_offset())
}

fn root_membership(
    root: Interval,
    span: BoundedPcurveSpan<'_>,
) -> core::result::Result<RootMembership, ()> {
    if matches!(span.curve(), crate::geom::Curve2dGeom::Line(_)) {
        return Ok(interval_membership(root, active_interval(span)?));
    }
    let active = active_interval(span)?;
    let midpoint = 0.5 * (active.lo() + active.hi()) - 0.5 * (root.lo() + root.hi());
    if !midpoint.is_finite() {
        return Err(());
    }
    let period_index = (midpoint / core::f64::consts::TAU).round();
    if !period_index.is_finite() || period_index.abs() > MAX_PERIOD_INDEX as f64 {
        return Err(());
    }
    let mut candidate = None;
    let mut ambiguous = false;
    for offset in -2..=2 {
        let index = period_index as i64 + offset;
        match interval_membership(shifted_interval(root, index), active) {
            RootMembership::Outside => {}
            RootMembership::Candidate(value) if candidate.replace(value).is_some() => {
                ambiguous = true;
            }
            RootMembership::Candidate(_) => {}
            RootMembership::Ambiguous => ambiguous = true,
        }
    }
    Ok(if ambiguous {
        RootMembership::Ambiguous
    } else {
        candidate.map_or(RootMembership::Outside, RootMembership::Candidate)
    })
}

fn period_shift(index: i64) -> Interval {
    if index == 0 {
        Interval::point(0.0)
    } else {
        Interval::point(index as f64) * Interval::point(core::f64::consts::TAU)
    }
}

fn shifted_interval(value: Interval, period_index: i64) -> Interval {
    if period_index == 0 {
        value
    } else {
        value + period_shift(period_index)
    }
}

fn exact_interval_overlap_relation(
    first: Interval,
    second: Interval,
    shared: &[(f64, f64)],
) -> core::result::Result<PairRelation, ()> {
    if first.hi() < second.lo() || second.hi() < first.lo() {
        return Ok(PairRelation::Disjoint);
    }
    if first.hi().min(second.hi()) > first.lo().max(second.lo()) {
        return Ok(PairRelation::ForbiddenIntersection);
    }
    if shared
        .iter()
        .any(|&(left, right)| first.contains(left) && second.contains(right))
    {
        Ok(PairRelation::Disjoint)
    } else {
        Ok(PairRelation::ForbiddenIntersection)
    }
}

fn interval_membership(root: Interval, active: Interval) -> RootMembership {
    if root.hi() < active.lo() || root.lo() > active.hi() {
        RootMembership::Outside
    } else {
        RootMembership::Candidate(RootUse {
            parameter: root,
            fully_inside: root.lo() >= active.lo() && root.hi() <= active.hi(),
        })
    }
}

#[derive(Debug)]
struct HarmonicRoots {
    roots: Vec<Interval>,
    identity: bool,
}

fn solve_harmonic(
    cosine: &Exact,
    sine: &Exact,
    constant: &Exact,
) -> core::result::Result<HarmonicRoots, ()> {
    if cosine.sign() == Orientation::Zero
        && sine.sign() == Orientation::Zero
        && constant.sign() == Orientation::Zero
    {
        return Ok(HarmonicRoots {
            roots: Vec::new(),
            identity: true,
        });
    }
    let a = constant.sub(cosine)?;
    let b = sine.scale(2.0)?;
    let c = constant.add(cosine)?;
    let mut half_angles = Vec::with_capacity(2);
    let mut infinity = false;
    if a.sign() == Orientation::Zero {
        infinity = true;
        if b.sign() != Orientation::Zero {
            half_angles.push(ratio_interval(&c.neg()?, &b)?);
        }
    } else {
        let discriminant = b.mul(&b)?.sub(&a.mul(&c)?.scale(4.0)?)?;
        match discriminant.sign() {
            Orientation::Negative => {}
            Orientation::Zero => {
                half_angles.push(ratio_interval(&b.neg()?, &a.scale(2.0)?)?);
            }
            Orientation::Positive => {
                let root = discriminant.interval()?.sqrt().ok_or(())?;
                if root.lo() <= 0.0 || !finite_interval(root) {
                    return Err(());
                }
                let denominator = a.scale(2.0)?.interval()?;
                let negative_b = b.neg()?.interval()?;
                half_angles.push((negative_b - root).checked_div(denominator).ok_or(())?);
                half_angles.push((negative_b + root).checked_div(denominator).ok_or(())?);
            }
        }
    }
    let mut roots = half_angles
        .into_iter()
        .map(twice_atan_interval)
        .collect::<core::result::Result<Vec<_>, _>>()?;
    if infinity {
        roots.push(Interval::point(core::f64::consts::PI));
    }
    roots.sort_by(|left, right| {
        left.lo()
            .total_cmp(&right.lo())
            .then(left.hi().total_cmp(&right.hi()))
    });
    if roots.windows(2).any(|pair| pair[0].hi() >= pair[1].lo()) {
        return Err(());
    }
    Ok(HarmonicRoots {
        roots,
        identity: false,
    })
}

fn twice_atan_interval(value: Interval) -> core::result::Result<Interval, ()> {
    if !finite_interval(value) {
        return Err(());
    }
    let mut lo = 2.0 * math::atan(value.lo());
    let mut hi = 2.0 * math::atan(value.hi());
    if !lo.is_finite() || !hi.is_finite() || lo > hi {
        return Err(());
    }
    for _ in 0..4 {
        lo = lo.next_down();
        hi = hi.next_up();
    }
    Ok(Interval::new(lo, hi))
}

fn circle_span_is_injective(span: BoundedPcurveSpan<'_>) -> core::result::Result<bool, ()> {
    let width = Exact::scalar(span.end())?.sub(&Exact::scalar(span.start())?)?;
    let width = width.interval()?;
    let tau = core::f64::consts::TAU;
    if width.lo() > -tau && width.hi() < tau {
        Ok(true)
    } else if width.lo() >= tau || width.hi() <= -tau {
        Ok(false)
    } else {
        Err(())
    }
}

fn active_interval(span: BoundedPcurveSpan<'_>) -> core::result::Result<Interval, ()> {
    let start = span.start();
    let end = span.end();
    if !start.is_finite() || !end.is_finite() || start == end {
        return Err(());
    }
    Ok(Interval::new(start.min(end), start.max(end)))
}

fn circle_point_interval(
    center: &[Exact; 2],
    cosine: &[Exact; 2],
    sine: &[Exact; 2],
    parameter: Interval,
) -> core::result::Result<[Interval; 2], ()> {
    let (sin, cos) = trig_interval(parameter)?;
    let center = exact_vec_interval(center)?;
    let cosine = exact_vec_interval(cosine)?;
    let sine = exact_vec_interval(sine)?;
    Ok([
        center[0] + cosine[0] * cos + sine[0] * sin,
        center[1] + cosine[1] * cos + sine[1] * sin,
    ])
}

fn trig_interval(parameter: Interval) -> core::result::Result<(Interval, Interval), ()> {
    if !finite_interval(parameter) || parameter.width() > 0.25 {
        return Err(());
    }
    let midpoint = 0.5 * parameter.lo() + 0.5 * parameter.hi();
    let delta = parameter.width().next_up();
    let (sin, cos) = math::sincos(midpoint);
    if !midpoint.is_finite() || !delta.is_finite() || !sin.is_finite() || !cos.is_finite() {
        return Err(());
    }
    Ok((
        Interval::new(
            (-1.0_f64).max((sin.next_down() - delta).next_down()),
            1.0_f64.min((sin.next_up() + delta).next_up()),
        ),
        Interval::new(
            (-1.0_f64).max((cos.next_down() - delta).next_down()),
            1.0_f64.min((cos.next_up() + delta).next_up()),
        ),
    ))
}

#[derive(Debug, Clone)]
struct Exact(Vec<f64>);

impl Exact {
    fn scalar(value: f64) -> core::result::Result<Self, ()> {
        safe_scalar(value).then_some(Self(vec![value])).ok_or(())
    }

    fn add(&self, other: &Self) -> core::result::Result<Self, ()> {
        Self::validated(expansion::sum(&self.0, &other.0))
    }

    fn sub(&self, other: &Self) -> core::result::Result<Self, ()> {
        Self::validated(expansion::sum(&self.0, &expansion::negate(&other.0)))
    }

    fn neg(&self) -> core::result::Result<Self, ()> {
        Self::validated(expansion::negate(&self.0))
    }

    fn scale(&self, factor: f64) -> core::result::Result<Self, ()> {
        if !safe_scalar(factor) {
            return Err(());
        }
        Self::validated(expansion::scale(&self.0, factor))
    }

    fn mul(&self, other: &Self) -> core::result::Result<Self, ()> {
        Self::validated(expansion::mul(&self.0, &other.0))
    }

    fn sign(&self) -> Orientation {
        match expansion::sign(&self.0) {
            value if value < 0 => Orientation::Negative,
            0 => Orientation::Zero,
            _ => Orientation::Positive,
        }
    }

    fn interval(&self) -> core::result::Result<Interval, ()> {
        let mut value = Interval::point(0.0);
        for &component in &self.0 {
            value = value + Interval::point(component);
            if !finite_interval(value) {
                return Err(());
            }
        }
        Ok(value)
    }

    fn validated(components: Vec<f64>) -> core::result::Result<Self, ()> {
        (!components.is_empty() && components.iter().copied().all(safe_scalar))
            .then_some(Self(components))
            .ok_or(())
    }
}

fn safe_scalar(value: f64) -> bool {
    value.is_finite()
        && (value == 0.0 || value.abs() >= MIN_SAFE_SCALAR && value.abs() <= MAX_SAFE_SCALAR)
}

fn exact_point(point: Point2) -> core::result::Result<[Exact; 2], ()> {
    Ok([Exact::scalar(point.x)?, Exact::scalar(point.y)?])
}

fn exact_translated(point: Point2, offset: Point2) -> core::result::Result<[Exact; 2], ()> {
    Ok([
        Exact::scalar(point.x)?.add(&Exact::scalar(offset.x)?)?,
        Exact::scalar(point.y)?.add(&Exact::scalar(offset.y)?)?,
    ])
}

fn sub_vec(left: &[Exact; 2], right: &[Exact; 2]) -> core::result::Result<[Exact; 2], ()> {
    Ok([left[0].sub(&right[0])?, left[1].sub(&right[1])?])
}

fn scale_vec(vector: &[Exact; 2], factor: f64) -> core::result::Result<[Exact; 2], ()> {
    Ok([vector[0].scale(factor)?, vector[1].scale(factor)?])
}

fn dot_exact(left: &[Exact; 2], right: &[Exact; 2]) -> core::result::Result<Exact, ()> {
    left[0].mul(&right[0])?.add(&left[1].mul(&right[1])?)
}

fn cross_exact(left: &[Exact; 2], right: &[Exact; 2]) -> core::result::Result<Exact, ()> {
    left[0].mul(&right[1])?.sub(&left[1].mul(&right[0])?)
}

fn ratio_interval(numerator: &Exact, denominator: &Exact) -> core::result::Result<Interval, ()> {
    numerator
        .interval()?
        .checked_div(denominator.interval()?)
        .ok_or(())
}

fn exact_vec_interval(vector: &[Exact; 2]) -> core::result::Result<[Interval; 2], ()> {
    Ok([vector[0].interval()?, vector[1].interval()?])
}

fn interval_sub(left: [Interval; 2], right: [Interval; 2]) -> [Interval; 2] {
    [left[0] - right[0], left[1] - right[1]]
}

fn interval_dot(left: [Interval; 2], right: [Interval; 2]) -> Interval {
    left[0] * right[0] + left[1] * right[1]
}

fn boxes_intersect(left: [Interval; 2], right: [Interval; 2]) -> bool {
    left[0].intersects(right[0]) && left[1].intersects(right[1])
}

fn translated(point: Point2, offset: Point2) -> Option<Point2> {
    let translated = point + offset;
    finite_point(translated).then_some(translated)
}

fn finite_interval(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite() && value.lo() <= value.hi()
}

fn finite_point(point: Point2) -> bool {
    point.x.is_finite() && point.y.is_finite()
}

fn points_bit_equal(first: Point2, second: Point2) -> bool {
    first.x.to_bits() == second.x.to_bits() && first.y.to_bits() == second.y.to_bits()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Curve2dGeom;
    use kgeom::curve2d::{Circle2d, Curve2d, Line2d};
    use kgeom::vec::Vec2;

    fn line(origin: [f64; 2], direction: [f64; 2]) -> Curve2dGeom {
        Curve2dGeom::Line(
            Line2d::new(
                Point2::new(origin[0], origin[1]),
                Vec2::new(direction[0], direction[1]),
            )
            .unwrap(),
        )
    }

    fn circle(center: [f64; 2], radius: f64) -> Curve2dGeom {
        Curve2dGeom::Circle(
            Circle2d::new(
                Point2::new(center[0], center[1]),
                radius,
                Vec2::new(1.0, 0.0),
            )
            .unwrap(),
        )
    }

    fn span<'a>(
        curve: &'a Curve2dGeom,
        start: f64,
        end: f64,
        offset: [f64; 2],
        tail: u8,
        head: u8,
    ) -> BoundedLoopSpan<'a, u8> {
        BoundedLoopSpan::new(
            BoundedPcurveSpan::new(curve, start, end, Point2::new(offset[0], offset[1])),
            tail,
            head,
        )
    }

    #[test]
    fn simple_mixed_semidisk_is_invariant_under_translation_and_reversal() {
        for offset in [[0.0, 0.0], [17.0, -9.0], [-1024.0, 2048.0]] {
            let arc = circle([0.0, 0.0], 1.0);
            let circle = arc.as_circle().unwrap();
            let arc_start = circle.eval(0.0);
            let arc_end = circle.eval(core::f64::consts::PI);
            let diameter_vector = arc_start - arc_end;
            let diameter_length = diameter_vector.norm();
            let diameter = line(
                [arc_end.x, arc_end.y],
                [diameter_vector.x, diameter_vector.y],
            );
            let forward = [
                span(&diameter, 0.0, diameter_length, offset, 0, 1),
                span(&arc, 0.0, core::f64::consts::PI, offset, 1, 0),
            ];
            assert_eq!(
                certify_bounded_loop_simplicity(&forward),
                BoundedLoopSimplicity::Certified
            );
            let reverse = [
                span(&arc, core::f64::consts::PI, 0.0, offset, 0, 1),
                span(&diameter, diameter_length, 0.0, offset, 1, 0),
            ];
            assert_eq!(
                certify_bounded_loop_simplicity(&reverse),
                BoundedLoopSimplicity::Certified
            );
        }
    }

    #[test]
    fn certified_near_join_does_not_hide_nonparallel_interior_crossing() {
        let epsilon = 2.0_f64.powi(-20);
        let horizontal = line([0.0, 0.0], [1.0, 0.0]);
        let returning = line([1024.0, epsilon], [-1.0, -epsilon / 512.0]);
        let closing = line([0.0, -epsilon], [0.0, 1.0]);
        let loop_spans = [
            span(&horizontal, 0.0, 1024.0, [0.0, 0.0], 0, 1)
                .with_head_join(CertifiedBoundedLoopJoin::new(2.0e-12).unwrap()),
            span(&returning, 0.0, 1024.0, [0.0, 0.0], 1, 2),
            span(&closing, 0.0, epsilon, [0.0, 0.0], 2, 0),
        ];

        // The first two spans are only near at their topology-owned join, but
        // their nonparallel carriers cross at parameter 512 in both interiors.
        assert_eq!(
            certify_bounded_loop_simplicity(&loop_spans),
            BoundedLoopSimplicity::SelfIntersecting
        );
    }

    #[test]
    fn pair_mutations_detect_crossing_tangent_touch_and_adjacent_overlap() {
        let horizontal = line([-2.0, 0.0], [1.0, 0.0]);
        let vertical = line([0.0, -2.0], [0.0, 1.0]);
        assert_eq!(
            line_line_relation(
                BoundedPcurveSpan::new(&horizontal, 0.0, 4.0, Point2::default()),
                BoundedPcurveSpan::new(&vertical, 0.0, 4.0, Point2::default()),
                &[],
                &[],
            )
            .unwrap(),
            PairRelation::ForbiddenIntersection
        );

        let unit_circle = circle([0.0, 0.0], 1.0);
        let tangent = line([-2.0, 1.0], [1.0, 0.0]);
        assert_eq!(
            line_circle_relation(
                BoundedPcurveSpan::new(&tangent, 0.0, 4.0, Point2::default()),
                BoundedPcurveSpan::new(
                    &unit_circle,
                    0.0,
                    core::f64::consts::PI,
                    Point2::default(),
                ),
                &[],
            )
            .unwrap(),
            PairRelation::ForbiddenIntersection
        );

        let endpoint_tangent = line([1.0, 0.0], [0.0, 1.0]);
        let line_use = BoundedPcurveSpan::new(&endpoint_tangent, 0.0, 1.0, Point2::default());
        let circle_use = BoundedPcurveSpan::new(&unit_circle, 0.0, 1.0, Point2::default());
        assert_eq!(
            line_circle_relation(line_use, circle_use, &[(0.0, 0.0)]).unwrap(),
            PairRelation::Disjoint
        );
        assert_eq!(
            line_circle_relation(line_use, circle_use, &[]).unwrap(),
            PairRelation::ForbiddenIntersection
        );

        let shared = line([0.0, 0.0], [1.0, 0.0]);
        let overlap = [
            span(&shared, 0.0, 2.0, [0.0, 0.0], 0, 1),
            span(&shared, 2.0, 1.0, [0.0, 0.0], 1, 2),
            span(&shared, 1.0, 0.0, [0.0, 0.0], 2, 0),
        ];
        assert_eq!(
            certify_bounded_loop_simplicity(&overlap),
            BoundedLoopSimplicity::SelfIntersecting
        );

        assert_eq!(
            line_line_relation(
                BoundedPcurveSpan::new(&shared, 0.0, 1.0, Point2::default()),
                BoundedPcurveSpan::new(&shared, 1.0, 2.0, Point2::default()),
                &[(1.0, 1.0)],
                &[],
            )
            .unwrap(),
            PairRelation::Disjoint
        );

        let other_circle = circle([2.0, 0.0], 1.0);
        assert_eq!(
            circle_circle_relation(
                BoundedPcurveSpan::new(&unit_circle, -1.0, 1.0, Point2::default()),
                BoundedPcurveSpan::new(
                    &other_circle,
                    core::f64::consts::PI - 1.0,
                    core::f64::consts::PI + 1.0,
                    Point2::default(),
                ),
                &[],
            )
            .unwrap(),
            PairRelation::ForbiddenIntersection
        );

        assert_eq!(
            circle_circle_relation(
                BoundedPcurveSpan::new(&unit_circle, 0.0, 1.0, Point2::default()),
                BoundedPcurveSpan::new(&unit_circle, 1.0, 2.0, Point2::default()),
                &[(1.0, 1.0)],
            )
            .unwrap(),
            PairRelation::Disjoint
        );
        assert_eq!(
            circle_circle_relation(
                BoundedPcurveSpan::new(&unit_circle, 0.0, 2.0, Point2::default()),
                BoundedPcurveSpan::new(&unit_circle, 1.0, 3.0, Point2::default()),
                &[],
            )
            .unwrap(),
            PairRelation::ForbiddenIntersection
        );
        let rephased = Curve2dGeom::Circle(
            Circle2d::new(Point2::default(), 1.0, Vec2::new(0.0, 1.0)).unwrap(),
        );
        assert_eq!(
            circle_circle_relation(
                BoundedPcurveSpan::new(&unit_circle, 0.0, 1.0, Point2::default()),
                BoundedPcurveSpan::new(&rephased, 0.0, 1.0, Point2::default()),
                &[],
            )
            .unwrap(),
            PairRelation::Coincident
        );
    }

    #[test]
    fn exact_chart_alignment_is_required_instead_of_numeric_unwrapping() {
        let bottom = line([0.0, 0.0], [1.0, 0.0]);
        let top = line([1.0, 1.0], [-1.0, 0.0]);
        let right = line([1.0, 0.0], [0.0, 1.0]);
        let left = line([0.0, 1.0], [0.0, -1.0]);
        let shifted = [
            span(&bottom, 0.0, 1.0, [8.0, 0.0], 0, 1),
            span(&right, 0.0, 1.0, [8.0, 0.0], 1, 2),
            span(&top, 0.0, 1.0, [8.0, 0.0], 2, 3),
            span(&left, 0.0, 1.0, [8.0, 0.0], 3, 0),
        ];
        assert_eq!(
            certify_bounded_loop_simplicity(&shifted),
            BoundedLoopSimplicity::Certified
        );
        let misaligned = [
            shifted[0],
            shifted[1],
            shifted[2],
            span(&left, 0.0, 1.0, [0.0, 0.0], 3, 0),
        ];
        assert!(matches!(
            certify_bounded_loop_simplicity(&misaligned),
            BoundedLoopSimplicity::Indeterminate(
                BoundedLoopSimplicityGap::ChartDiscontinuity { .. }
            )
        ));
    }

    #[test]
    fn near_join_requires_explicit_topology_and_incidence_evidence() {
        let epsilon = 5.0e-13;
        let bottom = line([0.0, 0.0], [1.0, 0.0]);
        let right = line([1.0 + epsilon, 0.0], [0.0, 1.0]);
        let top = line([1.0, 1.0], [-1.0, 0.0]);
        let left = line([0.0, 1.0], [0.0, -1.0]);
        let missing = [
            span(&bottom, 0.0, 1.0, [0.0, 0.0], 0, 1),
            span(&right, 0.0, 1.0, [0.0, 0.0], 1, 2),
            span(&top, 0.0, 1.0, [0.0, 0.0], 2, 3),
            span(&left, 0.0, 1.0, [0.0, 0.0], 3, 0),
        ];
        assert!(matches!(
            certify_bounded_loop_simplicity(&missing),
            BoundedLoopSimplicity::Indeterminate(
                BoundedLoopSimplicityGap::ChartDiscontinuity { .. }
            )
        ));
        let mut certified = missing;
        certified[0] = certified[0].with_head_join(CertifiedBoundedLoopJoin::new(2.0e-12).unwrap());
        certified[1] = certified[1].with_head_join(CertifiedBoundedLoopJoin::new(2.0e-12).unwrap());
        assert_eq!(
            certify_bounded_loop_simplicity(&certified),
            BoundedLoopSimplicity::Certified
        );
    }
}
