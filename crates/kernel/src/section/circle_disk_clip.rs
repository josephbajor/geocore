//! Certified clipping of a closed planar circle pcurve by a circular disk.
//!
//! The admitted trim is topology-owned: one loop, one fin, and one exact
//! vertexless whole-circle edge whose complete 3D/pcurve incidence is
//! certified by `ktopo`.  The branch circle restricted to the boundary disk
//! is the harmonic
//!
//! `constant + cosine * cos(q) + sine * sin(q)`.
//!
//! Outward interval arithmetic certifies a positive discriminant and isolates
//! both roots in the branch's projective half-angle chart.  Each root point is
//! then projected into the authored boundary-circle chart and mapped back to
//! the source edge's intrinsic `[0, TAU]` parameter.  The edge intervals own
//! source-root order; rounded carrier angles are publication evidence only.
//! Tangency, coincidence, a non-secant pair, either periodic seam, or
//! overlapping enclosures fail closed.
//!
//! `Circle2d` stores one radial axis and defines the other as its perpendicular.
//! For the exact real values represented by those stored floats this gives the
//! structural Gram identity `x·y = 0` and `x·x = y·y = g`, even when rounded
//! normalization leaves `g != 1`. The proof therefore carries an outward
//! enclosure of `g` for both circles; neither the harmonic constant nor the
//! boundary-angle projection assumes an exactly unit stored axis.

use kcore::interval::Interval;
use kcore::math;
use kcore::operation::OperationScope;
use kgeom::curve2d::Circle2d;
use kgeom::param::ParamRange;
use kgeom::vec::Point2;
use ktopo::entity::{
    EdgeId as RawEdgeId, FaceId as RawFaceId, FinId as RawFinId, FinPcurve, LoopId as RawLoopId,
    Sense,
};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};
use ktopo::store::Store;

use crate::error::{Error, Result};

use super::curved_clip::{
    ClosedConicClipGap, ClosedConicClipOutcome, ClosedConicFragment, ClosedConicTrimSite,
};
use super::{SECTION_WORK, SectionUvCircle};

const PERIOD: f64 = core::f64::consts::TAU;

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
struct BranchCircle {
    center: IntervalPoint2,
    x: IntervalPoint2,
    y: IntervalPoint2,
    radius: Interval,
    parameter_scale: f64,
    parameter_offset: f64,
    carrier_range: ParamRange,
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

enum DiskAdmission {
    NotDisk,
    Certified(DiskBoundary),
    Indeterminate(ClosedConicClipGap),
}

#[derive(Debug, Clone, Copy)]
struct Crossing {
    site: ClosedConicTrimSite,
}

/// Attempt the circular-disk trim class before the polygonal plane path.
///
/// `None` means the face does not have the one-circle-loop representation and
/// lets the caller continue with another exact trim class. Once that shape is
/// observed, every failed topology or arithmetic obligation is returned as an
/// explicit indeterminate result rather than falling through.
pub(super) fn try_clip_circle_to_disk_trim(
    store: &Store,
    face: RawFaceId,
    pcurve: SectionUvCircle,
    carrier_range: ParamRange,
    scope: &mut OperationScope<'_, '_>,
) -> Result<Option<ClosedConicClipOutcome>> {
    let boundary = match admit_disk_boundary(store, face, scope)? {
        DiskAdmission::NotDisk => return Ok(None),
        DiskAdmission::Indeterminate(gap) => {
            return Ok(Some(ClosedConicClipOutcome::Indeterminate(gap)));
        }
        DiskAdmission::Certified(boundary) => boundary,
    };
    charge(scope, 1)?;
    let branch = match branch_circle(pcurve, carrier_range) {
        Some(branch) => branch,
        None => {
            return Ok(Some(ClosedConicClipOutcome::Indeterminate(
                ClosedConicClipGap::UnsupportedTrim,
            )));
        }
    };
    Ok(Some(match clip_secant(branch, boundary, scope)? {
        Ok(fragment) => ClosedConicClipOutcome::Fragments(vec![fragment]),
        Err(gap) => ClosedConicClipOutcome::Indeterminate(gap),
    }))
}

fn admit_disk_boundary(
    store: &Store,
    face: RawFaceId,
    scope: &OperationScope<'_, '_>,
) -> Result<DiskAdmission> {
    let face_data = read(store.get(face))?;
    let [loop_id] = face_data.loops() else {
        return Ok(DiskAdmission::NotDisk);
    };
    let loop_ = read(store.get(*loop_id))?;
    let [fin_id] = loop_.fins() else {
        return Ok(DiskAdmission::NotDisk);
    };
    let fin = read(store.get(*fin_id))?;
    let Some(use_) = fin.pcurve() else {
        return Ok(DiskAdmission::NotDisk);
    };
    let Curve2dGeom::Circle(circle) = read(store.pcurve(use_.curve()))? else {
        return Ok(DiskAdmission::NotDisk);
    };

    let edge = read(store.get(fin.edge()))?;
    let Some(curve_id) = edge.curve() else {
        return Ok(DiskAdmission::Indeterminate(
            ClosedConicClipGap::MalformedTrim,
        ));
    };
    let source_is_circle = matches!(read(store.curve(curve_id))?, CurveGeom::Circle(_));
    let active = use_.range();
    let map = use_.edge_to_pcurve();
    let edge_parameters = [map.inverse(active.lo), map.inverse(active.hi)];
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
    if !matches!(
        read(store.surface(face_data.surface()))?,
        SurfaceGeom::Plane(_)
    ) || loop_.face() != face
        || fin.parent() != *loop_id
        || edge.vertices() != [None, None]
        || edge.bounds().is_some()
        || edge.tolerance().is_some()
        || !edge.fins().contains(fin_id)
        || !source_is_circle
        || !is_outer_disk_orientation(face_data.sense(), fin.sense(), use_.sense())
        || use_.closure_winding() != Some([0, 0])
        || use_.seam().is_some()
        || !use_.chart().is_identity()
        || values.into_iter().any(|value| !value.is_finite())
        || active.width() != PERIOD
        || map.scale().abs() != 1.0
        || edge_parameters[0].min(edge_parameters[1]) != 0.0
        || edge_parameters[0].max(edge_parameters[1]) != PERIOD
        || certify_whole_fin_incidence(
            store,
            face,
            *loop_id,
            *fin_id,
            scope.context().tolerances().linear(),
        ) != WholeFinIncidence::Certified
    {
        return Ok(DiskAdmission::Indeterminate(
            ClosedConicClipGap::MalformedTrim,
        ));
    }
    Ok(DiskAdmission::Certified(DiskBoundary {
        face,
        loop_id: *loop_id,
        fin: *fin_id,
        edge: fin.edge(),
        circle: *circle,
        use_,
    }))
}

/// Increasing `Circle2d` parameter is counterclockwise in the surface UV
/// chart. Loop traversal composes the fin's edge sense with the pcurve map
/// sense. Since the admitted face has exactly one loop, that loop is its
/// outer boundary and must be counterclockwise with the face normal up.
fn is_outer_disk_orientation(face: Sense, fin: Sense, pcurve: Sense) -> bool {
    fin.times(pcurve) == face
}

fn branch_circle(circle: SectionUvCircle, carrier_range: ParamRange) -> Option<BranchCircle> {
    let center = circle.center();
    let x = circle.x_direction();
    let y = x.perp();
    let values = [
        center.x,
        center.y,
        x.x,
        x.y,
        circle.radius(),
        circle.parameter_scale(),
        circle.parameter_offset(),
        carrier_range.lo,
        carrier_range.hi,
    ];
    if values.into_iter().any(|value| !value.is_finite())
        || circle.radius() <= 0.0
        || circle.parameter_scale().abs() != 1.0
        || carrier_range.width() != PERIOD
    {
        return None;
    }
    Some(BranchCircle {
        center: IntervalPoint2::point(center),
        x: IntervalPoint2::point(Point2::new(x.x, x.y)),
        y: IntervalPoint2::point(Point2::new(y.x, y.y)),
        radius: Interval::point(circle.radius()),
        parameter_scale: circle.parameter_scale(),
        parameter_offset: circle.parameter_offset(),
        carrier_range,
    })
}

fn clip_secant(
    branch: BranchCircle,
    boundary: DiskBoundary,
    scope: &mut OperationScope<'_, '_>,
) -> Result<core::result::Result<ClosedConicFragment, ClosedConicClipGap>> {
    let boundary_center = IntervalPoint2::point(boundary.circle.center());
    let boundary_radius = Interval::point(boundary.circle.radius());
    let boundary_x = boundary.circle.x_dir();
    let boundary_x = IntervalPoint2::point(Point2::new(boundary_x.x, boundary_x.y));
    let branch_gram = dot(branch.x, branch.x);
    let boundary_gram = dot(boundary_x, boundary_x);
    if !finite(branch_gram)
        || !finite(boundary_gram)
        || branch_gram.lo() <= 0.0
        || boundary_gram.lo() <= 0.0
    {
        return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
    }
    let relative = sub(branch.center, boundary_center);

    if branch.center.x == boundary_center.x
        && branch.center.y == boundary_center.y
        && branch.radius == boundary_radius
    {
        return Ok(Err(ClosedConicClipGap::CoincidentBoundary));
    }

    let two_radius = Interval::point(2.0) * branch.radius;
    let cosine = two_radius * dot(relative, branch.x);
    let sine = two_radius * dot(relative, branch.y);
    let branch_radius_squared = branch.radius.square() * branch_gram;
    let boundary_radius_squared = boundary_radius.square() * boundary_gram;
    let constant = dot(relative, relative) + branch_radius_squared - boundary_radius_squared;
    if !finite(cosine) || !finite(sine) || !finite(constant) {
        return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
    }
    let discriminant = cosine.square() + sine.square() - constant.square();
    if !finite(discriminant) {
        return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
    }
    if discriminant.hi() < 0.0 {
        return Ok(Err(ClosedConicClipGap::NonSecantBoundary));
    }
    if discriminant.lo() <= 0.0 {
        return Ok(Err(ClosedConicClipGap::TangentialContact));
    }

    let quadratic = [
        constant - cosine,
        Interval::point(2.0) * sine,
        constant + cosine,
    ];
    if quadratic[0].contains_zero() {
        return Ok(Err(ClosedConicClipGap::ParameterSeamContact));
    }
    let root_discriminant = Interval::point(4.0) * discriminant;
    let Some(root) = root_discriminant.sqrt() else {
        return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
    };
    let denominator = Interval::point(2.0) * quadratic[0];
    let (Some(first), Some(second)) = (
        (-quadratic[1] - root).checked_div(denominator),
        (-quadratic[1] + root).checked_div(denominator),
    ) else {
        return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
    };
    let mut half_angles = [first, second];
    half_angles.sort_by(interval_order);
    if half_angles.into_iter().any(|root| !finite(root))
        || half_angles[0].hi() >= half_angles[1].lo()
    {
        return Ok(Err(ClosedConicClipGap::UnorderedCrossings));
    }

    let seam_inside = match seam_inside_disk(branch, boundary_center, boundary_radius_squared) {
        Some(inside) => inside,
        None => return Ok(Err(ClosedConicClipGap::ParameterSeamContact)),
    };
    let mut crossings = Vec::with_capacity(2);
    for half_angle in half_angles {
        charge(scope, 1)?;
        let carrier_parameter_enclosure = match super::curved_clip::carrier_parameter_enclosure(
            half_angle,
            branch.parameter_scale,
            branch.parameter_offset,
            branch.carrier_range,
        ) {
            Ok(parameter) => parameter,
            Err(gap) => return Ok(Err(gap)),
        };
        let Some(point) = circle_point(branch, half_angle) else {
            return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
        };
        let edge_parameter = match boundary_edge_parameter(boundary, point) {
            Ok(parameter) => parameter,
            Err(gap) => return Ok(Err(gap)),
        };
        crossings.push(Crossing {
            site: ClosedConicTrimSite {
                face: boundary.face,
                loop_id: boundary.loop_id,
                fin: boundary.fin,
                edge: boundary.edge,
                root_ordinal: 0,
                pcurve_half_angle: half_angle,
                carrier_parameter: carrier_parameter(branch, half_angle),
                carrier_parameter_enclosure,
                edge_parameter,
            },
        });
    }

    let mut edge_order = [0_usize, 1_usize];
    edge_order.sort_by(|&a, &b| {
        interval_order(
            &crossings[a].site.edge_parameter,
            &crossings[b].site.edge_parameter,
        )
    });
    if crossings[edge_order[0]].site.edge_parameter.hi()
        >= crossings[edge_order[1]].site.edge_parameter.lo()
    {
        return Ok(Err(ClosedConicClipGap::UnorderedCrossings));
    }
    for (ordinal, index) in edge_order.into_iter().enumerate() {
        crossings[index].site.root_ordinal = ordinal;
    }

    let mut fragment = if seam_inside {
        ClosedConicFragment {
            start: Some(crossings[1].site),
            end: Some(crossings[0].site),
            wraps_pcurve_seam: true,
        }
    } else {
        ClosedConicFragment {
            start: Some(crossings[0].site),
            end: Some(crossings[1].site),
            wraps_pcurve_seam: false,
        }
    };
    if branch.parameter_scale < 0.0 {
        core::mem::swap(&mut fragment.start, &mut fragment.end);
    }
    Ok(Ok(fragment))
}

fn boundary_edge_parameter(
    boundary: DiskBoundary,
    point: IntervalPoint2,
) -> core::result::Result<Interval, ClosedConicClipGap> {
    let center = IntervalPoint2::point(boundary.circle.center());
    let x = boundary.circle.x_dir();
    let x = IntervalPoint2::point(Point2::new(x.x, x.y));
    let y_direction = boundary.circle.x_dir().perp();
    let y = IntervalPoint2::point(Point2::new(y_direction.x, y_direction.y));
    let radius = Interval::point(boundary.circle.radius());
    let gram = dot(x, x);
    let radial_projection_scale = radius * gram;
    if !finite(radial_projection_scale) || radial_projection_scale.lo() <= 0.0 {
        return Err(ClosedConicClipGap::ArithmeticGuard);
    }
    let relative = sub(point, center);
    let cosine = dot(relative, x)
        .checked_div(radial_projection_scale)
        .ok_or(ClosedConicClipGap::ArithmeticGuard)?;
    let sine = dot(relative, y)
        .checked_div(radial_projection_scale)
        .ok_or(ClosedConicClipGap::ArithmeticGuard)?;
    let denominator = Interval::point(1.0) + cosine;
    if denominator.contains_zero() {
        return Err(ClosedConicClipGap::ParameterSeamContact);
    }
    let half_angle = sine
        .checked_div(denominator)
        .filter(|value| finite(*value))
        .ok_or(ClosedConicClipGap::ArithmeticGuard)?;
    let principal = twice_atan_interval(half_angle)?;
    let active = boundary.use_.range();
    let pcurve_parameter = lift_principal_to_active(principal, active)
        .ok_or(ClosedConicClipGap::ParameterSeamContact)?;
    let map = boundary.use_.edge_to_pcurve();
    let edge_parameter = (pcurve_parameter - Interval::point(map.offset()))
        .checked_div(Interval::point(map.scale()))
        .filter(|value| finite(*value))
        .ok_or(ClosedConicClipGap::ArithmeticGuard)?;
    if edge_parameter.lo() <= 0.0 || edge_parameter.hi() >= PERIOD {
        return Err(ClosedConicClipGap::ParameterSeamContact);
    }
    Ok(edge_parameter)
}

fn lift_principal_to_active(principal: Interval, active: ParamRange) -> Option<Interval> {
    if !finite(principal) || !active.is_finite() || active.width() != PERIOD {
        return None;
    }
    let midpoint = 0.5 * principal.lo() + 0.5 * principal.hi();
    let base = ((active.lo - midpoint) / PERIOD).round();
    if !base.is_finite() {
        return None;
    }
    // `base` is only a finite search seed. It never certifies the winding:
    // acceptance below requires exactly one outward root enclosure to lie
    // strictly inside the authored full-period use. A rounded seed that misses
    // that enclosure can only make this function return `None` (fail closed).
    let mut accepted = None;
    for offset in [-1.0, 0.0, 1.0] {
        // `base + offset` is an integral winding, but multiplying that stored
        // integer by the stored period can round. Preserve the real product
        // with the same outward arithmetic used by every later range test.
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

fn seam_inside_disk(
    branch: BranchCircle,
    boundary_center: IntervalPoint2,
    boundary_radius_squared: Interval,
) -> Option<bool> {
    let seam = sub(branch.center, scale(branch.x, branch.radius));
    let relative = sub(seam, boundary_center);
    let implicit = dot(relative, relative) - boundary_radius_squared;
    if !finite(implicit) {
        None
    } else if implicit.hi() < 0.0 {
        Some(true)
    } else if implicit.lo() > 0.0 {
        Some(false)
    } else {
        None
    }
}

fn circle_point(branch: BranchCircle, half_angle: Interval) -> Option<IntervalPoint2> {
    let square = half_angle.square();
    let denominator = Interval::point(1.0) + square;
    let cosine = (Interval::point(1.0) - square).checked_div(denominator)?;
    let sine = (Interval::point(2.0) * half_angle).checked_div(denominator)?;
    let radial = add(scale(branch.x, cosine), scale(branch.y, sine));
    let point = add(branch.center, scale(radial, branch.radius));
    point.finite().then_some(point)
}

fn carrier_parameter(branch: BranchCircle, half_angle: Interval) -> f64 {
    let midpoint = 0.5 * half_angle.lo() + 0.5 * half_angle.hi();
    let natural = 2.0 * math::atan2(midpoint, 1.0);
    let parameter = (natural - branch.parameter_offset) / branch.parameter_scale;
    if !parameter.is_finite() {
        return branch.carrier_range.lo;
    }
    (parameter - branch.carrier_range.lo).rem_euclid(PERIOD) + branch.carrier_range.lo
}

fn twice_atan_interval(value: Interval) -> core::result::Result<Interval, ClosedConicClipGap> {
    if !finite(value) {
        return Err(ClosedConicClipGap::ArithmeticGuard);
    }
    let mut lo = 2.0 * math::atan(value.lo());
    let mut hi = 2.0 * math::atan(value.hi());
    if !lo.is_finite() || !hi.is_finite() || lo > hi {
        return Err(ClosedConicClipGap::ArithmeticGuard);
    }
    for _ in 0..4 {
        lo = lo.next_down();
        hi = hi.next_up();
    }
    Ok(Interval::new(lo, hi))
}

fn interval_order(a: &Interval, b: &Interval) -> core::cmp::Ordering {
    a.lo().total_cmp(&b.lo()).then(a.hi().total_cmp(&b.hi()))
}

fn add(a: IntervalPoint2, b: IntervalPoint2) -> IntervalPoint2 {
    IntervalPoint2 {
        x: a.x + b.x,
        y: a.y + b.y,
    }
}

fn sub(a: IntervalPoint2, b: IntervalPoint2) -> IntervalPoint2 {
    IntervalPoint2 {
        x: a.x - b.x,
        y: a.y - b.y,
    }
}

fn scale(a: IntervalPoint2, value: Interval) -> IntervalPoint2 {
    IntervalPoint2 {
        x: a.x * value,
        y: a.y * value,
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
    use kcore::operation::{OperationContext, OperationScope, SessionPolicy};
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;
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

    fn with_scope<T>(run: impl FnOnce(&mut OperationScope<'_, '_>) -> T) -> T {
        let policy = SessionPolicy::v1();
        let context = OperationContext::new(&policy, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(BodySectionBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        run(&mut scope)
    }

    fn disk_fixture() -> DiskFixture {
        let mut store = Store::new();
        let body = ktopo::make::cylinder(&mut store, &Frame::world(), 1.0, 2.0).unwrap();
        let face = disk_face(&store, body);
        let [loop_id] = *store.get(face).unwrap().loops() else {
            panic!("cylinder cap must have one loop")
        };
        let [fin] = *store.get(loop_id).unwrap().fins() else {
            panic!("cylinder cap must have one fin")
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

    fn branch(center: Point2, radius: f64, x_direction: Vec2) -> SectionUvCircle {
        SectionUvCircle {
            center,
            radius,
            x_direction,
            parameter_scale: 1.0,
            parameter_offset: 0.0,
        }
    }

    fn run(fixture: &DiskFixture, circle: SectionUvCircle) -> ClosedConicClipOutcome {
        with_scope(|scope| {
            super::super::curved_clip::clip_closed_conic_to_face(
                &fixture.store,
                fixture.face,
                super::super::SectionUvCurve::Circle(circle),
                ParamRange::new(0.0, PERIOD),
                scope,
            )
            .unwrap()
        })
    }

    fn one_arc(outcome: ClosedConicClipOutcome) -> ClosedConicFragment {
        let ClosedConicClipOutcome::Fragments(fragments) = outcome else {
            panic!("expected one certified disk arc, got {outcome:?}")
        };
        let [fragment] = fragments.as_slice() else {
            panic!("expected exactly one disk arc, got {fragments:?}")
        };
        *fragment
    }

    #[test]
    fn strict_secant_retains_one_arc_with_source_edge_root_order() {
        let fixture = disk_fixture();
        let face = fixture.store.get(fixture.face).unwrap();
        let fin = fixture.store.get(fixture.fin).unwrap();
        let use_ = fin.pcurve().unwrap();
        // The origin cap deliberately exercises the doubly-reversed case:
        // reversed fin traversal and reversed edge-to-pcurve map compose to
        // the forward outer-loop orientation and still recover intrinsic
        // source-edge root order below.
        assert_eq!(face.sense(), Sense::Forward);
        assert_eq!(fin.sense(), Sense::Reversed);
        assert_eq!(use_.sense(), Sense::Reversed);
        assert!(is_outer_disk_orientation(
            face.sense(),
            fin.sense(),
            use_.sense()
        ));
        let fragment = one_arc(run(
            &fixture,
            branch(Point2::new(0.5, 0.0), 1.0, Vec2::new(1.0, 0.0)),
        ));
        assert!(fragment.wraps_pcurve_seam);
        let mut sites = [fragment.start.unwrap(), fragment.end.unwrap()];
        assert!(sites[0].pcurve_half_angle.lo() > sites[1].pcurve_half_angle.hi());
        sites.sort_by_key(|site| site.root_ordinal);
        assert_eq!([sites[0].root_ordinal, sites[1].root_ordinal], [0, 1]);
        assert!(sites[0].edge_parameter.hi() < sites[1].edge_parameter.lo());
        for site in sites {
            assert_eq!(site.face, fixture.face);
            assert_eq!(site.loop_id, fixture.loop_id);
            assert_eq!(site.fin, fixture.fin);
            assert_eq!(site.edge, fixture.edge);
            assert!(site.edge_parameter.lo() > 0.0);
            assert!(site.edge_parameter.hi() < PERIOD);
            assert!(site.edge_parameter.lo() < site.edge_parameter.hi());
        }
    }

    #[test]
    fn flipped_face_or_fin_breaks_outer_disk_orientation() {
        let fixture = disk_fixture();
        let face = fixture.store.get(fixture.face).unwrap();
        let fin = fixture.store.get(fixture.fin).unwrap();
        let pcurve = fin.pcurve().unwrap().sense();
        assert!(is_outer_disk_orientation(face.sense(), fin.sense(), pcurve));
        assert!(!is_outer_disk_orientation(
            face.sense(),
            fin.sense().flipped(),
            pcurve
        ));
        assert!(!is_outer_disk_orientation(
            face.sense().flipped(),
            fin.sense(),
            pcurve
        ));
        assert!(is_outer_disk_orientation(
            face.sense().flipped(),
            fin.sense().flipped(),
            pcurve
        ));
    }

    #[test]
    fn shifted_period_lift_is_outward_and_rejects_the_shifted_seam() {
        let winding = 3.0;
        let active_lo = winding * PERIOD;
        let active = ParamRange::new(active_lo, active_lo + PERIOD);
        assert_eq!(active.width(), PERIOD);

        let principal = Interval::new(0.25_f64.next_down(), 0.25_f64.next_up());
        let lifted = lift_principal_to_active(principal, active).unwrap();
        let exact_shift = Interval::point(winding) * Interval::point(PERIOD);
        let oracle = principal + exact_shift;
        assert!(lifted.lo() <= oracle.lo());
        assert!(lifted.hi() >= oracle.hi());
        assert!(lifted.lo() > active.lo && lifted.hi() < active.hi);

        // The same authored periodic branch must not turn its lower endpoint
        // into an interior source parameter through a rounded winding shift.
        assert!(lift_principal_to_active(Interval::point(0.0), active).is_none());
    }

    #[test]
    fn seam_parity_selects_the_nonwrapping_complement_arc() {
        let fixture = disk_fixture();
        let fragment = one_arc(run(
            &fixture,
            branch(Point2::new(0.5, 0.0), 1.0, Vec2::new(-1.0, 0.0)),
        ));
        assert!(!fragment.wraps_pcurve_seam);
        let start = fragment.start.unwrap();
        let end = fragment.end.unwrap();
        assert!(start.pcurve_half_angle.hi() < end.pcurve_half_angle.lo());
        assert_ne!(start.root_ordinal, end.root_ordinal);
    }

    #[test]
    fn tangent_coincident_and_miss_are_never_promoted() {
        let fixture = disk_fixture();
        let cases = [
            (
                branch(Point2::new(2.0, 0.0), 1.0, Vec2::new(1.0, 0.0)),
                ClosedConicClipGap::TangentialContact,
            ),
            (
                branch(Point2::new(0.0, 0.0), 1.0, Vec2::new(1.0, 0.0)),
                ClosedConicClipGap::CoincidentBoundary,
            ),
            (
                branch(Point2::new(3.0, 0.0), 1.0, Vec2::new(1.0, 0.0)),
                ClosedConicClipGap::NonSecantBoundary,
            ),
        ];
        for (circle, expected) in cases {
            assert_eq!(
                run(&fixture, circle),
                ClosedConicClipOutcome::Indeterminate(expected)
            );
        }
    }

    #[test]
    fn branch_and_source_parameter_seams_fail_closed() {
        let fixture = disk_fixture();
        // The branch seam `center - x` is exactly `(0, 1)`, on the disk
        // boundary, even though the two circles are otherwise transverse.
        let branch_seam = branch(Point2::new(0.6, 0.2), 1.0, Vec2::new(0.6, -0.8));
        assert_eq!(
            run(&fixture, branch_seam),
            ClosedConicClipOutcome::Indeterminate(ClosedConicClipGap::ParameterSeamContact)
        );

        let use_ = fixture.store.get(fixture.fin).unwrap().pcurve().unwrap();
        let Curve2dGeom::Circle(boundary) = fixture.store.pcurve(use_.curve()).unwrap() else {
            unreachable!()
        };
        let source_seam = boundary.center() + boundary.x_dir() * boundary.radius();
        let tangent = boundary.x_dir().perp();
        // This branch passes transversely through the boundary pcurve's zero
        // parameter, which maps to the intrinsic source-edge seam.
        let source_parameter_seam = branch(source_seam + tangent * 0.5, 0.5, -tangent);
        assert_eq!(
            run(&fixture, source_parameter_seam),
            ClosedConicClipOutcome::Indeterminate(ClosedConicClipGap::ParameterSeamContact)
        );
    }
}
