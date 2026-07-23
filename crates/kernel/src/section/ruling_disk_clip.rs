//! Certified clipping of an affine planar ruling by a circular disk trim.
//!
//! The admitted disk is topology-owned: exactly one loop, one fin, and one
//! vertexless whole-circle edge with a complete Plane/Circle/Circle2d
//! incidence certificate. The pcurve may reverse the source-edge parameter,
//! but it may not carry a seam or a non-identity chart.
//!
//! A transverse line/circle solve uses outward interval arithmetic. Both
//! physical crossings are projected back into the authored circular pcurve,
//! lifted strictly inside its full-period range, and mapped to intrinsic
//! source-edge parameters. Tangency, a source-parameter seam crossing,
//! overlapping root enclosures, or any unsafe arithmetic fails closed.

use kcore::interval::Interval;
use kcore::math;
use kcore::operation::OperationScope;
use kgeom::curve2d::Circle2d;
use kgeom::vec::Point2;
use ktopo::entity::{
    EdgeId as RawEdgeId, FaceId as RawFaceId, FinId as RawFinId, FinPcurve, LoopId as RawLoopId,
    Sense,
};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};
use ktopo::store::Store;

use super::ruling_clip::{RulingClipGap, RulingClipOutcome, RulingClipSpan, RulingTrimSite};
use super::{SECTION_WORK, SectionUvLine};
use crate::error::{Error, Result};

const PERIOD: f64 = core::f64::consts::TAU;

/// Fixed work charged after the one-circle disk representation is observed:
/// topology admission, the quadratic proof, and two source-root projections.
const DISK_RULING_WORK: u64 = 4;

#[derive(Debug, Clone, Copy)]
struct IntervalPoint2 {
    x: Interval,
    y: Interval,
}

impl IntervalPoint2 {
    fn point(point: Point2) -> Self {
        Self {
            x: Interval::point(point.x),
            y: Interval::point(point.y),
        }
    }

    fn finite(self) -> bool {
        finite(self.x) && finite(self.y)
    }
}

#[derive(Debug, Clone, Copy)]
struct DiskBoundary {
    face: RawFaceId,
    loop_id: RawLoopId,
    fin: RawFinId,
    edge: RawEdgeId,
    circle: Circle2d,
    use_: FinPcurve,
}

/// Attempt the circular-disk trim class before polygonal Plane clipping.
///
/// `None` means this face does not use the one-loop/one-fin Circle2d
/// representation. No work is charged in that case, preserving the exact
/// polygon path accounting. Once that representation is observed, the fixed
/// disk ceiling is charged before any semantic admission decision.
pub(super) fn try_clip_line_to_disk_trim(
    store: &Store,
    face: RawFaceId,
    trace: SectionUvLine,
    scope: &mut OperationScope<'_, '_>,
) -> Result<Option<RulingClipOutcome>> {
    let Some(candidate) = recognize_disk_boundary(store, face)? else {
        return Ok(None);
    };
    charge(scope, DISK_RULING_WORK)?;
    let boundary = match admit_disk_boundary(store, candidate, scope)? {
        Ok(boundary) => boundary,
        Err(gap) => return Ok(Some(RulingClipOutcome::Indeterminate(gap))),
    };
    Ok(Some(clip_secant(trace, boundary)))
}

fn recognize_disk_boundary(store: &Store, face: RawFaceId) -> Result<Option<DiskBoundary>> {
    let face_data = read(store.get(face))?;
    let [loop_id] = face_data.loops() else {
        return Ok(None);
    };
    let loop_ = read(store.get(*loop_id))?;
    let [fin_id] = loop_.fins() else {
        return Ok(None);
    };
    let fin = read(store.get(*fin_id))?;
    let Some(use_) = fin.pcurve() else {
        return Ok(None);
    };
    let Curve2dGeom::Circle(circle) = read(store.pcurve(use_.curve()))? else {
        return Ok(None);
    };
    Ok(Some(DiskBoundary {
        face,
        loop_id: *loop_id,
        fin: *fin_id,
        edge: fin.edge(),
        circle: *circle,
        use_,
    }))
}

fn admit_disk_boundary(
    store: &Store,
    candidate: DiskBoundary,
    scope: &OperationScope<'_, '_>,
) -> Result<core::result::Result<DiskBoundary, RulingClipGap>> {
    let face_data = read(store.get(candidate.face))?;
    let loop_ = read(store.get(candidate.loop_id))?;
    let fin = read(store.get(candidate.fin))?;
    let edge = read(store.get(candidate.edge))?;
    let (Some(curve_id), Some(use_)) = (edge.curve(), fin.pcurve()) else {
        return Ok(Err(RulingClipGap::UnsupportedTrim));
    };
    let source_is_circle = matches!(read(store.curve(curve_id))?, CurveGeom::Circle(_));
    let active = use_.range();
    let map = use_.edge_to_pcurve();
    let edge_parameters = [map.inverse(active.lo), map.inverse(active.hi)];
    let circle = candidate.circle;
    let values = [
        circle.center().x,
        circle.center().y,
        circle.x_dir().x,
        circle.x_dir().y,
        circle.radius(),
        active.lo,
        active.hi,
        map.scale(),
        map.offset(),
        edge_parameters[0],
        edge_parameters[1],
    ];
    let ownership_is_exact = face_data.loops() == [candidate.loop_id]
        && loop_.face() == candidate.face
        && loop_.fins() == [candidate.fin]
        && fin.parent() == candidate.loop_id
        && fin.edge() == candidate.edge
        && edge.fins().contains(&candidate.fin);
    if !ownership_is_exact
        || certify_whole_fin_incidence(
            store,
            candidate.face,
            candidate.loop_id,
            candidate.fin,
            scope.context().tolerances().linear(),
        ) != WholeFinIncidence::Certified
    {
        return Ok(Err(RulingClipGap::MalformedTrim));
    }
    if !matches!(
        read(store.surface(face_data.surface()))?,
        SurfaceGeom::Plane(_)
    ) || edge.vertices() != [None, None]
        || edge.bounds().is_some()
        || edge.tolerance().is_some()
        || !source_is_circle
        || !is_outer_disk_orientation(face_data.sense(), fin.sense(), use_.sense())
        || use_.closure_winding() != Some([0, 0])
        || use_.seam().is_some()
        || !use_.chart().is_identity()
        || !matches!(read(store.pcurve(use_.curve()))?, Curve2dGeom::Circle(_))
        || values.into_iter().any(|value| !value.is_finite())
        || active.width() != PERIOD
        || map.scale().abs() != 1.0
        || edge_parameters[0].min(edge_parameters[1]) != 0.0
        || edge_parameters[0].max(edge_parameters[1]) != PERIOD
    {
        return Ok(Err(RulingClipGap::UnsupportedTrim));
    }
    Ok(Ok(DiskBoundary { use_, ..candidate }))
}

fn is_outer_disk_orientation(face: Sense, fin: Sense, pcurve: Sense) -> bool {
    fin.times(pcurve) == face
}

fn clip_secant(trace: SectionUvLine, boundary: DiskBoundary) -> RulingClipOutcome {
    let origin = IntervalPoint2::point(trace.origin());
    let direction = IntervalPoint2::point(Point2::new(trace.direction().x, trace.direction().y));
    let center = IntervalPoint2::point(boundary.circle.center());
    let relative = sub(origin, center);
    let x_direction = boundary.circle.x_dir();
    let x = IntervalPoint2::point(Point2::new(x_direction.x, x_direction.y));
    let gram = dot(x, x);
    let a = dot(direction, direction);
    let b = Interval::point(2.0) * dot(direction, relative);
    let c = dot(relative, relative) - Interval::point(boundary.circle.radius()).square() * gram;
    if !finite(a) || !finite(b) || !finite(c) || !finite(gram) || a.lo() <= 0.0 || gram.lo() <= 0.0
    {
        return RulingClipOutcome::Indeterminate(RulingClipGap::ArithmeticGuard);
    }
    let discriminant = b.square() - Interval::point(4.0) * a * c;
    if !finite(discriminant) {
        return RulingClipOutcome::Indeterminate(RulingClipGap::ArithmeticGuard);
    }
    if discriminant.hi() < 0.0 {
        return RulingClipOutcome::Spans(Vec::new());
    }
    if discriminant.lo() <= 0.0 {
        return RulingClipOutcome::Indeterminate(RulingClipGap::TangentialContact);
    }
    let Some(root) = discriminant.sqrt() else {
        return RulingClipOutcome::Indeterminate(RulingClipGap::ArithmeticGuard);
    };
    let denominator = Interval::point(2.0) * a;
    let (Some(first), Some(second)) = (
        (-b - root).checked_div(denominator),
        (-b + root).checked_div(denominator),
    ) else {
        return RulingClipOutcome::Indeterminate(RulingClipGap::ArithmeticGuard);
    };
    let mut roots = [first, second];
    roots.sort_by(interval_order);
    if roots.into_iter().any(|root| !finite(root)) || roots[0].hi() >= roots[1].lo() {
        return RulingClipOutcome::Indeterminate(RulingClipGap::UnorderedCrossings);
    }
    let (Some(start), Some(end)) = (
        trim_site(trace, roots[0], boundary),
        trim_site(trace, roots[1], boundary),
    ) else {
        return RulingClipOutcome::Indeterminate(RulingClipGap::ArithmeticGuard);
    };
    RulingClipOutcome::Spans(vec![RulingClipSpan { start, end }])
}

fn trim_site(
    trace: SectionUvLine,
    carrier_parameter: Interval,
    boundary: DiskBoundary,
) -> Option<RulingTrimSite> {
    let point = interval_line_point(trace, carrier_parameter)?;
    let edge_parameter = boundary_edge_parameter(boundary, point)?;
    Some(RulingTrimSite {
        face: boundary.face,
        loop_id: boundary.loop_id,
        fin: boundary.fin,
        edge: boundary.edge,
        carrier_parameter,
        edge_parameter,
    })
}

fn boundary_edge_parameter(boundary: DiskBoundary, point: IntervalPoint2) -> Option<Interval> {
    let center = IntervalPoint2::point(boundary.circle.center());
    let x_direction = boundary.circle.x_dir();
    let x = IntervalPoint2::point(Point2::new(x_direction.x, x_direction.y));
    let y_direction = x_direction.perp();
    let y = IntervalPoint2::point(Point2::new(y_direction.x, y_direction.y));
    let gram = dot(x, x);
    let radial_projection_scale = Interval::point(boundary.circle.radius()) * gram;
    if !finite(radial_projection_scale) || radial_projection_scale.lo() <= 0.0 {
        return None;
    }
    let relative = sub(point, center);
    let cosine = dot(relative, x).checked_div(radial_projection_scale)?;
    let sine = dot(relative, y).checked_div(radial_projection_scale)?;
    let half_angle = sine.checked_div(Interval::point(1.0) + cosine)?;
    let principal = twice_atan_interval(half_angle)?;
    let pcurve_parameter = lift_principal_to_active(principal, boundary.use_.range())?;
    let map = boundary.use_.edge_to_pcurve();
    let edge_parameter = (pcurve_parameter - Interval::point(map.offset()))
        .checked_div(Interval::point(map.scale()))?;
    if !finite(edge_parameter) || edge_parameter.lo() <= 0.0 || edge_parameter.hi() >= PERIOD {
        return None;
    }
    Some(edge_parameter)
}

fn interval_line_point(trace: SectionUvLine, parameter: Interval) -> Option<IntervalPoint2> {
    let origin = IntervalPoint2::point(trace.origin());
    let direction = IntervalPoint2::point(Point2::new(trace.direction().x, trace.direction().y));
    let point = IntervalPoint2 {
        x: origin.x + direction.x * parameter,
        y: origin.y + direction.y * parameter,
    };
    point.finite().then_some(point)
}

fn lift_principal_to_active(
    principal: Interval,
    active: kgeom::param::ParamRange,
) -> Option<Interval> {
    if !finite(principal) || !active.is_finite() || active.width() != PERIOD {
        return None;
    }
    let midpoint = 0.5 * principal.lo() + 0.5 * principal.hi();
    let base = ((active.lo - midpoint) / PERIOD).round();
    if !base.is_finite() {
        return None;
    }
    let mut accepted = None;
    for offset in [-1.0, 0.0, 1.0] {
        let shift = Interval::point(base + offset) * Interval::point(PERIOD);
        if !finite(shift) {
            return None;
        }
        let candidate = principal + shift;
        if candidate.lo() > active.lo
            && candidate.hi() < active.hi
            && accepted.replace(candidate).is_some()
        {
            return None;
        }
    }
    accepted
}

fn twice_atan_interval(value: Interval) -> Option<Interval> {
    if !finite(value) {
        return None;
    }
    let mut lo = 2.0 * math::atan(value.lo());
    let mut hi = 2.0 * math::atan(value.hi());
    if !lo.is_finite() || !hi.is_finite() || lo > hi {
        return None;
    }
    for _ in 0..4 {
        lo = lo.next_down();
        hi = hi.next_up();
    }
    Some(Interval::new(lo, hi))
}

fn interval_order(a: &Interval, b: &Interval) -> core::cmp::Ordering {
    a.lo().total_cmp(&b.lo()).then(a.hi().total_cmp(&b.hi()))
}

fn sub(a: IntervalPoint2, b: IntervalPoint2) -> IntervalPoint2 {
    IntervalPoint2 {
        x: a.x - b.x,
        y: a.y - b.y,
    }
}

fn dot(a: IntervalPoint2, b: IntervalPoint2) -> Interval {
    a.x * b.x + a.y * b.y
}

fn finite(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite()
}

fn charge(scope: &mut OperationScope<'_, '_>, amount: u64) -> Result<()> {
    scope
        .ledger_mut()
        .charge(SECTION_WORK, amount)
        .map_err(Error::from)
}

fn read<T>(result: kcore::error::Result<T>) -> Result<T> {
    result.map_err(|source| Error::InconsistentTopology { source })
}

#[cfg(test)]
mod tests {
    use kcore::operation::{
        AccountingMode, BudgetPlan, LimitSpec, OperationContext, OperationScope, ResourceKind,
        SessionPolicy,
    };
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;
    use kgeom::param::ParamRange;
    use kgeom::vec::{Point3, Vec2};
    use ktopo::entity::BodyId as RawBodyId;

    use super::*;
    use crate::section::BodySectionBudgetProfile;

    struct DiskFixture {
        store: Store,
        face: RawFaceId,
        loop_id: RawLoopId,
        fin: RawFinId,
        edge: RawEdgeId,
    }

    fn fixture() -> DiskFixture {
        let mut store = Store::new();
        let body = ktopo::make::cylinder(&mut store, &Frame::world(), 1.0, 2.0).unwrap();
        let face = disk_face(&store, body);
        let [loop_id] = *store.get(face).unwrap().loops() else {
            panic!("world cylinder base cap must have one loop")
        };
        let [fin] = *store.get(loop_id).unwrap().fins() else {
            panic!("world cylinder base cap must have one fin")
        };
        DiskFixture {
            edge: store.get(fin).unwrap().edge(),
            store,
            face,
            loop_id,
            fin,
        }
    }

    fn disk_face(store: &Store, body: RawBodyId) -> RawFaceId {
        store
            .faces_of_body(body)
            .unwrap()
            .into_iter()
            .find(|face| {
                matches!(
                    store.surface(store.get(*face).unwrap().surface()).unwrap(),
                    SurfaceGeom::Plane(plane)
                        if plane.frame().origin() == Point3::new(0.0, 0.0, 0.0)
                )
            })
            .expect("finite cylinder has an origin cap")
    }

    fn trace(y: f64) -> SectionUvLine {
        SectionUvLine {
            origin: Point2::new(-2.0, y),
            direction: Vec2::new(1.0, 0.0),
        }
    }

    fn with_scope<T>(
        allowed: Option<u64>,
        run: impl FnOnce(&mut OperationScope<'_, '_>) -> T,
    ) -> T {
        let policy = SessionPolicy::v1();
        let mut context = OperationContext::new(&policy, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(BodySectionBudgetProfile::v1_defaults());
        if let Some(allowed) = allowed {
            context = context.with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SECTION_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        }
        let mut scope = OperationScope::new(&context);
        run(&mut scope)
    }

    fn run(fixture: &DiskFixture, y: f64) -> RulingClipOutcome {
        with_scope(None, |scope| {
            super::super::ruling_clip::clip_ruling_to_face(
                &fixture.store,
                fixture.face,
                trace(y),
                ParamRange::new(-4.0, 4.0),
                scope,
            )
            .unwrap()
        })
    }

    #[test]
    fn world_disk_secant_retains_reversed_map_source_parameters() {
        let fixture = fixture();
        let use_ = fixture.store.get(fixture.fin).unwrap().pcurve().unwrap();
        assert_eq!(use_.edge_to_pcurve().scale(), -1.0);
        let RulingClipOutcome::Spans(spans) = run(&fixture, 0.5) else {
            panic!("world disk secant must certify one span")
        };
        let [span] = spans.as_slice() else {
            panic!("world disk secant must certify exactly one span: {spans:?}")
        };
        let radial = 0.75_f64.sqrt();
        assert!(span.start.carrier_parameter.contains(2.0 - radial));
        assert!(span.end.carrier_parameter.contains(2.0 + radial));
        assert!(
            span.start
                .edge_parameter
                .contains(7.0 * core::f64::consts::PI / 6.0)
        );
        assert!(
            span.end
                .edge_parameter
                .contains(11.0 * core::f64::consts::PI / 6.0)
        );
        for site in [span.start, span.end] {
            assert_eq!(site.face, fixture.face);
            assert_eq!(site.loop_id, fixture.loop_id);
            assert_eq!(site.fin, fixture.fin);
            assert_eq!(site.edge, fixture.edge);
        }
    }

    #[test]
    fn source_seam_and_tangent_contacts_fail_closed() {
        let fixture = fixture();
        assert_eq!(
            run(&fixture, 0.0),
            RulingClipOutcome::Indeterminate(RulingClipGap::ArithmeticGuard)
        );
        assert_eq!(
            run(&fixture, 1.0),
            RulingClipOutcome::Indeterminate(RulingClipGap::TangentialContact)
        );
    }

    #[test]
    fn disk_clip_has_exact_fixed_n_and_n_minus_one_work() {
        let fixture = fixture();
        let run = |allowed| {
            with_scope(Some(allowed), |scope| {
                super::super::ruling_clip::clip_ruling_to_face(
                    &fixture.store,
                    fixture.face,
                    trace(0.5),
                    ParamRange::new(-4.0, 4.0),
                    scope,
                )
            })
        };
        let exact = 1 + DISK_RULING_WORK;
        assert!(matches!(run(exact).unwrap(), RulingClipOutcome::Spans(_)));
        let error = run(exact - 1).unwrap_err();
        let crossing = error.limit().expect("N-1 must retain limit evidence");
        assert_eq!(crossing.stage, SECTION_WORK);
        assert_eq!(crossing.resource, ResourceKind::Work);
        assert_eq!(crossing.consumed, exact);
        assert_eq!(crossing.allowed, exact - 1);
    }
}
