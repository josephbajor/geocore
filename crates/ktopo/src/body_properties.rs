//! Certified volume and centroid interrogation for analytic solid B-reps.
//!
//! This module integrates the committed, oriented boundary representation; it
//! never promotes a tessellated approximation into geometric authority.  The
//! admitted representation slice is deliberately finite: Full-valid solid
//! bodies whose faces are planes or right circular cylinders and whose trims
//! are exact `Line2d`/`Circle2d` pcurves (circles are currently planar only).
//! Every returned bound is an outward [`Interval`](kcore::interval::Interval)
//! enclosure. Unsupported valid representations fail closed as typed
//! refusals.

use crate::check::{
    CheckBudgetProfile, CheckLevel, CheckOutcome, CheckReport, check_body_report_in_scope,
};
use crate::entity::{BodyId, BodyKind, FaceId, LoopId, Sense};
use crate::geom::{Curve2dGeom, SurfaceGeom};
use crate::loop_proof::bounded_pcurve_integral::BoundedPcurveSpan;
use crate::loop_proof::prepare_bounded_analytic_loop;
use crate::store::Store;
use kcore::error::{Error, Result};
use kcore::interval::Interval;
use kcore::math;
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSpec, OperationScope, ResourceKind, StageId,
};
use kgeom::vec::{Point2, Point3, Vec3};

const fn known_stage(value: &'static str) -> StageId {
    match StageId::new(value) {
        Ok(stage) => stage,
        Err(_) => panic!("invalid built-in body-properties stage"),
    }
}

/// Cumulative, fixed-degree analytic boundary-integration work.
pub const BODY_PROPERTIES_ANALYTIC_WORK: StageId =
    known_stage("ktopo.interrogate.body-properties-analytic-work");

const DEFAULT_ANALYTIC_WORK: u64 = 1_048_576;
const PLANE_LINE_WORK: u64 = 64;
const PLANE_CIRCLE_WORK: u64 = 192;
const CYLINDER_LINE_WORK: u64 = 512;
const CYLINDER_CIRCLE_WORK: u64 = 512;

/// Aggregate v1 policy for Full validation followed by analytic properties.
#[derive(Debug, Clone, Copy, Default)]
pub struct BodyPropertiesBudgetProfile;

impl BodyPropertiesBudgetProfile {
    /// Canonical family defaults for one certified body-properties query.
    pub fn v1_defaults() -> BudgetPlan {
        CheckBudgetProfile::v1_defaults(CheckLevel::Full).overlaid(
            &BudgetPlan::new([LimitSpec::new(
                BODY_PROPERTIES_ANALYTIC_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                DEFAULT_ANALYTIC_WORK,
            )])
            .expect("valid body-properties budget"),
        )
    }
}

/// A finite certified scalar enclosure.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScalarEnclosure {
    lower: f64,
    upper: f64,
}

impl ScalarEnclosure {
    fn from_interval(value: Interval) -> Option<Self> {
        (finite_interval(value) && value.lo() <= value.hi()).then_some(Self {
            lower: value.lo(),
            upper: value.hi(),
        })
    }

    /// Certified inclusive lower bound.
    pub const fn lower(self) -> f64 {
        self.lower
    }

    /// Certified inclusive upper bound.
    pub const fn upper(self) -> f64 {
        self.upper
    }

    /// Deterministic representative at the enclosure midpoint.
    pub fn midpoint(self) -> f64 {
        0.5 * self.lower + 0.5 * self.upper
    }

    /// Radius around [`Self::midpoint`] containing the certified interval.
    pub fn error_bound(self) -> f64 {
        let midpoint = self.midpoint();
        (midpoint - self.lower).max(self.upper - midpoint).next_up()
    }
}

/// Per-coordinate certified model-space point enclosure.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3Enclosure {
    coordinates: [ScalarEnclosure; 3],
}

impl Point3Enclosure {
    fn from_intervals(values: [Interval; 3]) -> Option<Self> {
        Some(Self {
            coordinates: [
                ScalarEnclosure::from_interval(values[0])?,
                ScalarEnclosure::from_interval(values[1])?,
                ScalarEnclosure::from_interval(values[2])?,
            ],
        })
    }

    /// Certified inclusive coordinate bounds in `(x, y, z)` order.
    pub const fn coordinates(self) -> [ScalarEnclosure; 3] {
        self.coordinates
    }

    /// Deterministic midpoint representative.
    pub fn midpoint(self) -> Point3 {
        Point3::new(
            self.coordinates[0].midpoint(),
            self.coordinates[1].midpoint(),
            self.coordinates[2].midpoint(),
        )
    }

    /// Rotationally invariant radius containing the coordinate box.
    pub fn error_bound(self) -> f64 {
        let mut squared_radius = 0.0_f64;
        for value in self.coordinates {
            let radius = value.error_bound();
            squared_radius = (squared_radius + radius * radius).next_up();
        }
        squared_radius.sqrt().next_up()
    }
}

/// Certified volume and centroid of one solid body.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CertifiedBodyProperties {
    volume: ScalarEnclosure,
    centroid: Point3Enclosure,
}

impl CertifiedBodyProperties {
    /// Certified positive volume enclosure.
    pub const fn volume(self) -> ScalarEnclosure {
        self.volume
    }

    /// Certified model-space centroid enclosure.
    pub const fn centroid(self) -> Point3Enclosure {
        self.centroid
    }
}

/// Why a valid request did not produce certified analytic properties.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyPropertiesRefusal {
    /// The body is not a three-dimensional solid.
    NonSolidBody,
    /// Full validation found faults or unresolved proof obligations.
    BodyNotFullValid,
    /// Exact integration does not consume tolerant topology.
    TolerantTopology,
    /// A face uses a supporting surface outside the Plane/Cylinder slice.
    UnsupportedSurface {
        /// Face whose supporting surface is outside the proof slice.
        face: FaceId,
    },
    /// A face boundary uses a pcurve outside the admitted analytic slice.
    UnsupportedPcurve {
        /// Face whose boundary representation is outside the proof slice.
        face: FaceId,
    },
    /// Topology-owned loop preparation could not reissue its analytic proof.
    UncertifiedAnalyticBoundary {
        /// Face at which topology-owned analytic preparation failed closed.
        face: FaceId,
    },
    /// Outward arithmetic did not prove a finite strictly positive volume.
    NonPositiveVolumeEnclosure,
}

/// Full-check evidence paired with either certified properties or a refusal.
#[derive(Debug, Clone, PartialEq)]
pub enum BodyPropertiesOutcome {
    /// The boundary integral and Full checker both certified the result.
    Certified {
        /// Certified analytic values.
        properties: CertifiedBodyProperties,
        /// Full checker evidence consumed by the theorem.
        full_check: CheckReport,
    },
    /// The request was valid but outside the current proof boundary.
    Refused {
        /// Typed refusal reason.
        reason: BodyPropertiesRefusal,
        /// Full checker evidence, including non-valid reports.
        full_check: CheckReport,
    },
}

impl BodyPropertiesOutcome {
    /// Full checker report retained by either outcome.
    pub const fn full_check(&self) -> &CheckReport {
        match self {
            Self::Certified { full_check, .. } | Self::Refused { full_check, .. } => full_check,
        }
    }
}

/// Exact structural charge for one body-properties query.
///
/// This count is independent of numeric conditioning and is computed before
/// any integration scratch is allocated.
pub fn body_properties_analytic_work(store: &Store, body: BodyId) -> Result<u64> {
    let faces = store.faces_of_body(body)?;
    let mut loops = 0_u64;
    let mut span_work = 0_u64;
    for face_id in &faces {
        let face = store.get(*face_id)?;
        loops = loops
            .checked_add(
                u64::try_from(face.loops.len()).map_err(|_| Error::InvalidGeometry {
                    reason: "body-properties loop count overflow",
                })?,
            )
            .ok_or(Error::InvalidGeometry {
                reason: "body-properties loop count overflow",
            })?;
        let surface = store.get(face.surface)?;
        for &loop_id in &face.loops {
            for &fin_id in &store.get(loop_id)?.fins {
                let fin = store.get(fin_id)?;
                let cost = match fin.pcurve.and_then(|use_| store.get(use_.curve()).ok()) {
                    Some(Curve2dGeom::Line(_)) => match surface {
                        SurfaceGeom::Plane(_) => PLANE_LINE_WORK,
                        SurfaceGeom::Cylinder(_) => CYLINDER_LINE_WORK,
                        _ => 0,
                    },
                    Some(Curve2dGeom::Circle(_)) => match surface {
                        SurfaceGeom::Plane(_) => PLANE_CIRCLE_WORK,
                        SurfaceGeom::Cylinder(_) => CYLINDER_CIRCLE_WORK,
                        _ => 0,
                    },
                    _ => 0,
                };
                span_work = span_work.checked_add(cost).ok_or(Error::InvalidGeometry {
                    reason: "body-properties analytic work overflow",
                })?;
            }
        }
    }
    let face_count = u64::try_from(faces.len()).map_err(|_| Error::InvalidGeometry {
        reason: "body-properties face count overflow",
    })?;
    1_u64
        .checked_add(face_count.checked_mul(16).ok_or(Error::InvalidGeometry {
            reason: "body-properties analytic work overflow",
        })?)
        .and_then(|work| work.checked_add(loops.checked_mul(8)?))
        .and_then(|work| work.checked_add(span_work))
        .ok_or(Error::InvalidGeometry {
            reason: "body-properties analytic work overflow",
        })
}

/// Full-validate and certify volume/centroid in one caller-owned scope.
pub fn certify_body_properties_in_scope(
    store: &Store,
    body: BodyId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<BodyPropertiesOutcome> {
    let full_check = check_body_report_in_scope(store, body, CheckLevel::Full, scope)?;
    if full_check.outcome() != CheckOutcome::Valid {
        return Ok(BodyPropertiesOutcome::Refused {
            reason: BodyPropertiesRefusal::BodyNotFullValid,
            full_check,
        });
    }
    if store.get(body)?.kind != BodyKind::Solid {
        return Ok(BodyPropertiesOutcome::Refused {
            reason: BodyPropertiesRefusal::NonSolidBody,
            full_check,
        });
    }

    let work = body_properties_analytic_work(store, body)?;
    scope
        .ledger_mut()
        .charge(BODY_PROPERTIES_ANALYTIC_WORK, work)
        .map_err(Error::from)?;

    let decision = integrate_body(store, body);
    Ok(match decision {
        Ok(properties) => BodyPropertiesOutcome::Certified {
            properties,
            full_check,
        },
        Err(reason) => BodyPropertiesOutcome::Refused { reason, full_check },
    })
}

#[derive(Debug, Clone, Copy)]
struct Flux {
    volume: Interval,
    moment: [Interval; 3],
}

impl Flux {
    fn zero() -> Self {
        Self {
            volume: Interval::point(0.0),
            moment: [Interval::point(0.0); 3],
        }
    }

    fn add(self, other: Self) -> Self {
        Self {
            volume: self.volume + other.volume,
            moment: [
                self.moment[0] + other.moment[0],
                self.moment[1] + other.moment[1],
                self.moment[2] + other.moment[2],
            ],
        }
    }

    fn finite(self) -> bool {
        finite_interval(self.volume) && self.moment.into_iter().all(finite_interval)
    }
}

fn integrate_body(
    store: &Store,
    body: BodyId,
) -> core::result::Result<CertifiedBodyProperties, BodyPropertiesRefusal> {
    let faces = store
        .faces_of_body(body)
        .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?;
    if faces.is_empty() {
        return Err(BodyPropertiesRefusal::NonPositiveVolumeEnclosure);
    }
    for &face_id in &faces {
        let face = store
            .get(face_id)
            .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?;
        if face.tolerance.is_some() {
            return Err(BodyPropertiesRefusal::TolerantTopology);
        }
    }
    for edge in store
        .edges_of_body(body)
        .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?
    {
        if store
            .get(edge)
            .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?
            .tolerance
            .is_some()
        {
            return Err(BodyPropertiesRefusal::TolerantTopology);
        }
    }
    for vertex in store
        .vertices_of_body(body)
        .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?
    {
        if store
            .get(vertex)
            .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?
            .tolerance
            .is_some()
        {
            return Err(BodyPropertiesRefusal::TolerantTopology);
        }
    }

    let anchor = canonical_anchor(store, &faces)?;
    let mut total = Flux::zero();
    for face_id in faces {
        let face_flux = integrate_face(store, face_id, anchor)?;
        total = total.add(face_flux);
        if !total.finite() {
            return Err(BodyPropertiesRefusal::UncertifiedAnalyticBoundary { face: face_id });
        }
    }
    if !finite_interval(total.volume) || total.volume.lo() <= 0.0 {
        return Err(BodyPropertiesRefusal::NonPositiveVolumeEnclosure);
    }
    let centroid = [
        total.moment[0]
            .checked_div(total.volume)
            .map(|value| value + Interval::point(anchor.x)),
        total.moment[1]
            .checked_div(total.volume)
            .map(|value| value + Interval::point(anchor.y)),
        total.moment[2]
            .checked_div(total.volume)
            .map(|value| value + Interval::point(anchor.z)),
    ];
    let [Some(cx), Some(cy), Some(cz)] = centroid else {
        return Err(BodyPropertiesRefusal::NonPositiveVolumeEnclosure);
    };
    let Some(volume) = ScalarEnclosure::from_interval(total.volume) else {
        return Err(BodyPropertiesRefusal::NonPositiveVolumeEnclosure);
    };
    let Some(centroid) = Point3Enclosure::from_intervals([cx, cy, cz]) else {
        return Err(BodyPropertiesRefusal::NonPositiveVolumeEnclosure);
    };
    Ok(CertifiedBodyProperties { volume, centroid })
}

fn canonical_anchor(
    store: &Store,
    faces: &[FaceId],
) -> core::result::Result<Point3, BodyPropertiesRefusal> {
    let mut anchor = None;
    for &face_id in faces {
        let face = store
            .get(face_id)
            .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?;
        let origin = match store
            .get(face.surface)
            .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?
        {
            SurfaceGeom::Plane(plane) => plane.frame().origin(),
            SurfaceGeom::Cylinder(cylinder) => cylinder.frame().origin(),
            _ => return Err(BodyPropertiesRefusal::UnsupportedSurface { face: face_id }),
        };
        if !finite_point(origin) {
            return Err(BodyPropertiesRefusal::UncertifiedAnalyticBoundary { face: face_id });
        }
        if anchor.is_none_or(|current| point_total_less(origin, current)) {
            anchor = Some(origin);
        }
    }
    anchor.ok_or(BodyPropertiesRefusal::NonPositiveVolumeEnclosure)
}

fn point_total_less(left: Point3, right: Point3) -> bool {
    left.x
        .total_cmp(&right.x)
        .then(left.y.total_cmp(&right.y))
        .then(left.z.total_cmp(&right.z))
        .is_lt()
}

fn integrate_face(
    store: &Store,
    face_id: FaceId,
    anchor: Point3,
) -> core::result::Result<Flux, BodyPropertiesRefusal> {
    let face = store
        .get(face_id)
        .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?;
    let surface = store
        .get(face.surface)
        .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?;
    let mut total = Flux::zero();
    for &loop_id in &face.loops {
        let spans = prepare_property_loop(store, face_id, loop_id, surface)?;
        for span in spans {
            let next = match surface {
                SurfaceGeom::Plane(plane) => integrate_plane_span(*plane, anchor, span),
                SurfaceGeom::Cylinder(cylinder) => integrate_cylinder_span(*cylinder, anchor, span),
                _ => return Err(BodyPropertiesRefusal::UnsupportedSurface { face: face_id }),
            }
            .ok_or(BodyPropertiesRefusal::UncertifiedAnalyticBoundary { face: face_id })?;
            total = total.add(next);
            if !total.finite() {
                return Err(BodyPropertiesRefusal::UncertifiedAnalyticBoundary { face: face_id });
            }
        }
    }
    Ok(total)
}

fn prepare_property_loop<'a>(
    store: &'a Store,
    face_id: FaceId,
    loop_id: LoopId,
    surface: &SurfaceGeom,
) -> core::result::Result<Vec<BoundedPcurveSpan<'a>>, BodyPropertiesRefusal> {
    let loop_ = store
        .get(loop_id)
        .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?;
    if loop_.fins.len() >= 2 {
        let prepared = prepare_bounded_analytic_loop(store, face_id, loop_id)
            .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?
            .ok_or(BodyPropertiesRefusal::UncertifiedAnalyticBoundary { face: face_id })?;
        return Ok(prepared.into_iter().map(|span| span.geometry()).collect());
    }

    let [fin_id] = loop_.fins.as_slice() else {
        return Err(BodyPropertiesRefusal::UncertifiedAnalyticBoundary { face: face_id });
    };
    let fin = store
        .get(*fin_id)
        .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?;
    let edge = store
        .get(fin.edge)
        .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?;
    let use_ = fin
        .pcurve
        .ok_or(BodyPropertiesRefusal::UnsupportedPcurve { face: face_id })?;
    let curve = store
        .get(use_.curve())
        .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?;
    if edge.bounds.is_some()
        || edge.vertices != [None, None]
        || edge.tolerance.is_some()
        || use_.seam().is_some()
    {
        return Err(BodyPropertiesRefusal::UncertifiedAnalyticBoundary { face: face_id });
    }
    match (surface, curve, use_.closure_winding()) {
        (SurfaceGeom::Plane(_), Curve2dGeom::Circle(_), Some([0, 0])) => {}
        (SurfaceGeom::Cylinder(_), Curve2dGeom::Line(line), Some([1 | -1, 0]))
            if line.dir().x != 0.0 && line.dir().y == 0.0 => {}
        _ => return Err(BodyPropertiesRefusal::UnsupportedPcurve { face: face_id }),
    }
    let edge_curve = edge
        .curve
        .and_then(|curve| store.get(curve).ok())
        .ok_or(BodyPropertiesRefusal::UncertifiedAnalyticBoundary { face: face_id })?;
    let range = edge_curve.as_curve().param_range();
    if !range.is_finite() || range.lo >= range.hi {
        return Err(BodyPropertiesRefusal::UncertifiedAnalyticBoundary { face: face_id });
    }
    let (edge_start, edge_end) = traversal_bounds(fin.sense, range.lo, range.hi);
    let start = use_.parameter_at_edge(edge_start);
    let end = use_.parameter_at_edge(edge_end);
    if !start.is_finite() || !end.is_finite() || start == end {
        return Err(BodyPropertiesRefusal::UncertifiedAnalyticBoundary { face: face_id });
    }
    let periods = match surface {
        SurfaceGeom::Plane(_) => [None, None],
        SurfaceGeom::Cylinder(_) => [Some(core::f64::consts::TAU), None],
        _ => return Err(BodyPropertiesRefusal::UnsupportedSurface { face: face_id }),
    };
    let offset = use_
        .chart()
        .apply(Point2::default(), periods)
        .map_err(|_| BodyPropertiesRefusal::UncertifiedAnalyticBoundary { face: face_id })?;
    Ok(vec![BoundedPcurveSpan::new(curve, start, end, offset)])
}

fn traversal_bounds(sense: Sense, lo: f64, hi: f64) -> (f64, f64) {
    if sense.is_forward() {
        (lo, hi)
    } else {
        (hi, lo)
    }
}

fn integrate_plane_span(
    plane: kgeom::surface::Plane,
    anchor: Point3,
    span: BoundedPcurveSpan<'_>,
) -> Option<Flux> {
    let frame = plane.frame();
    let x = frame.x();
    let y = frame.y();
    let relative_coordinates = relative_coordinates(frame.origin(), anchor);
    let x_coordinates = point_vec(x);
    let y_coordinates = point_vec(y);
    let normal = interval_cross(point_vec(x), point_vec(y));
    let h = interval_dot(relative_coordinates, normal);
    match span.curve() {
        Curve2dGeom::Line(line) => {
            let u = Poly::affine_interval(
                Interval::point(line.origin().x) + Interval::point(span.chart_offset().x),
                Interval::point(line.dir().x),
            );
            let v = Poly::affine_interval(
                Interval::point(line.origin().y) + Interval::point(span.chart_offset().y),
                Interval::point(line.dir().y),
            );
            let du = Interval::point(line.dir().x);
            let volume_poly = v.scale(-(h * interval_ratio(1.0, 3.0)) * du);
            let mut moment = [Interval::point(0.0); 3];
            for coordinate in 0..3 {
                let primitive = v
                    .scale(relative_coordinates[coordinate])
                    .add(u.mul(v).scale(x_coordinates[coordinate]))
                    .add(
                        v.mul(v)
                            .scale(y_coordinates[coordinate] * interval_ratio(1.0, 2.0)),
                    )
                    .scale(-(h * interval_ratio(1.0, 4.0)) * du);
                moment[coordinate] = primitive.integrate(span.start(), span.end())?;
            }
            let flux = Flux {
                volume: volume_poly.integrate(span.start(), span.end())?,
                moment,
            };
            flux.finite().then_some(flux)
        }
        Curve2dGeom::Circle(circle) => {
            let circle_x = circle.x_dir();
            let circle_y = circle_x.perp();
            let radius = Interval::point(circle.radius());
            let u = Laurent::coordinate_intervals(
                Interval::point(circle.center().x) + Interval::point(span.chart_offset().x),
                Interval::point(circle_x.x) * radius,
                Interval::point(circle_y.x) * radius,
            );
            let v = Laurent::coordinate_intervals(
                Interval::point(circle.center().y) + Interval::point(span.chart_offset().y),
                Interval::point(circle_x.y) * radius,
                Interval::point(circle_y.y) * radius,
            );
            let du = u.derivative();
            let volume_form = v.scale_interval(-(h * interval_ratio(1.0, 3.0))).mul(du);
            let mut moment = [Interval::point(0.0); 3];
            for coordinate in 0..3 {
                let primitive = v
                    .scale_interval(relative_coordinates[coordinate])
                    .add(u.mul(v).scale_interval(x_coordinates[coordinate]))
                    .add(
                        v.mul(v)
                            .scale_interval(y_coordinates[coordinate] * interval_ratio(1.0, 2.0)),
                    )
                    .scale_interval(-(h * interval_ratio(1.0, 4.0)))
                    .mul(du);
                moment[coordinate] = primitive.integrate(span.start(), span.end())?;
            }
            let flux = Flux {
                volume: volume_form.integrate(span.start(), span.end())?,
                moment,
            };
            flux.finite().then_some(flux)
        }
        _ => None,
    }
}

fn integrate_cylinder_span(
    cylinder: kgeom::surface::Cylinder,
    anchor: Point3,
    span: BoundedPcurveSpan<'_>,
) -> Option<Flux> {
    let Curve2dGeom::Line(line) = span.curve() else {
        return None;
    };
    let frame = cylinder.frame();
    let radius = cylinder.radius();
    let relative_coordinates = relative_coordinates(frame.origin(), anchor);
    let x_coordinates = point_vec(frame.x());
    let y_coordinates = point_vec(frame.y());
    let z_coordinates = point_vec(frame.z());
    let cos = Laurent::cosine();
    let sin = Laurent::sine();
    let mut radial = [Laurent::zero(); 3];
    for coordinate in 0..3 {
        radial[coordinate] = cos
            .scale_interval(Interval::point(radius) * x_coordinates[coordinate])
            .add(sin.scale_interval(Interval::point(radius) * y_coordinates[coordinate]));
    }
    let mut h = Laurent::constant(interval_product(radius, radius));
    for coordinate in 0..3 {
        h = h.add(radial[coordinate].scale_interval(relative_coordinates[coordinate]));
    }
    let mut h_position = [Laurent::zero(); 3];
    for coordinate in 0..3 {
        h_position[coordinate] =
            h.mul(Laurent::constant(relative_coordinates[coordinate]).add(radial[coordinate]));
    }

    let u_origin = Interval::point(line.origin().x) + Interval::point(span.chart_offset().x);
    let u_direction = line.dir().x;
    if u_direction == 0.0 {
        return Some(Flux::zero());
    }
    let v = Poly::affine_interval(
        Interval::point(line.origin().y) + Interval::point(span.chart_offset().y),
        Interval::point(line.dir().y),
    );
    let v2 = v.mul(v);
    let mut volume = ComplexInterval::zero();
    let mut moment = [ComplexInterval::zero(); 3];
    for power in -LAURENT_ORDER..=LAURENT_ORDER {
        let h_coefficient = h.coefficient(power);
        let volume_poly =
            ComplexPoly::from_real(v.scale(interval_ratio(-u_direction, 3.0))).scale(h_coefficient);
        volume = volume.add(volume_poly.integrate_exponential(
            power,
            u_origin,
            u_direction,
            span.start(),
            span.end(),
        )?);
        for coordinate in 0..3 {
            let first = ComplexPoly::from_real(v).scale(h_position[coordinate].coefficient(power));
            let second = ComplexPoly::from_real(
                v2.scale(z_coordinates[coordinate] * interval_ratio(1.0, 2.0)),
            )
            .scale(h_coefficient);
            let form = first
                .add(second)
                .scale_real(interval_ratio(-u_direction, 4.0));
            moment[coordinate] = moment[coordinate].add(form.integrate_exponential(
                power,
                u_origin,
                u_direction,
                span.start(),
                span.end(),
            )?);
        }
    }
    if !volume.im.contains_zero() || moment.iter().any(|value| !value.im.contains_zero()) {
        return None;
    }
    let flux = Flux {
        volume: volume.re,
        moment: [moment[0].re, moment[1].re, moment[2].re],
    };
    flux.finite().then_some(flux)
}

const POLY_DEGREE: usize = 3;

#[derive(Debug, Clone, Copy)]
struct Poly {
    coefficients: [Interval; POLY_DEGREE + 1],
}

impl Poly {
    fn zero() -> Self {
        Self {
            coefficients: [Interval::point(0.0); POLY_DEGREE + 1],
        }
    }

    fn affine_interval(constant: Interval, linear: Interval) -> Self {
        let mut value = Self::zero();
        value.coefficients[0] = constant;
        value.coefficients[1] = linear;
        value
    }

    fn add(mut self, other: Self) -> Self {
        for index in 0..=POLY_DEGREE {
            self.coefficients[index] = self.coefficients[index] + other.coefficients[index];
        }
        self
    }

    fn scale(mut self, factor: Interval) -> Self {
        for coefficient in &mut self.coefficients {
            *coefficient = *coefficient * factor;
        }
        self
    }

    fn mul(self, other: Self) -> Self {
        let mut result = Self::zero();
        for left in 0..=POLY_DEGREE {
            for right in 0..=POLY_DEGREE - left {
                result.coefficients[left + right] = result.coefficients[left + right]
                    + self.coefficients[left] * other.coefficients[right];
            }
        }
        result
    }

    fn integrate(self, start: f64, end: f64) -> Option<Interval> {
        if !start.is_finite() || !end.is_finite() {
            return None;
        }
        let mut result = Interval::point(0.0);
        for (power, coefficient) in self.coefficients.into_iter().enumerate() {
            let integral = monomial_integral(start, end, power)?;
            result = result + coefficient * integral;
        }
        finite_interval(result).then_some(result)
    }
}

const LAURENT_ORDER: i32 = 4;
const LAURENT_COUNT: usize = (LAURENT_ORDER as usize) * 2 + 1;

#[derive(Debug, Clone, Copy)]
struct ComplexInterval {
    re: Interval,
    im: Interval,
}

impl ComplexInterval {
    fn zero() -> Self {
        Self {
            re: Interval::point(0.0),
            im: Interval::point(0.0),
        }
    }

    fn real(value: Interval) -> Self {
        Self {
            re: value,
            im: Interval::point(0.0),
        }
    }

    fn add(self, other: Self) -> Self {
        Self {
            re: self.re + other.re,
            im: self.im + other.im,
        }
    }

    fn sub(self, other: Self) -> Self {
        Self {
            re: self.re - other.re,
            im: self.im - other.im,
        }
    }

    fn mul(self, other: Self) -> Self {
        Self {
            re: self.re * other.re - self.im * other.im,
            im: self.re * other.im + self.im * other.re,
        }
    }

    fn scale(self, factor: Interval) -> Self {
        Self {
            re: self.re * factor,
            im: self.im * factor,
        }
    }

    fn divide_i(self, divisor: Interval) -> Option<Self> {
        Some(Self {
            re: self.im.checked_div(divisor)?,
            im: (-self.re).checked_div(divisor)?,
        })
    }

    fn finite(self) -> bool {
        finite_interval(self.re) && finite_interval(self.im)
    }
}

#[derive(Debug, Clone, Copy)]
struct Laurent {
    coefficients: [ComplexInterval; LAURENT_COUNT],
}

impl Laurent {
    fn zero() -> Self {
        Self {
            coefficients: [ComplexInterval::zero(); LAURENT_COUNT],
        }
    }

    fn constant(value: Interval) -> Self {
        let mut result = Self::zero();
        result.set(0, ComplexInterval::real(value));
        result
    }

    fn cosine() -> Self {
        let mut result = Self::zero();
        let half = Interval::point(0.5);
        result.set(-1, ComplexInterval::real(half));
        result.set(1, ComplexInterval::real(half));
        result
    }

    fn sine() -> Self {
        let mut result = Self::zero();
        let half = Interval::point(0.5);
        result.set(
            -1,
            ComplexInterval {
                re: Interval::point(0.0),
                im: half,
            },
        );
        result.set(
            1,
            ComplexInterval {
                re: Interval::point(0.0),
                im: -half,
            },
        );
        result
    }

    fn coordinate_intervals(constant: Interval, cosine: Interval, sine: Interval) -> Self {
        Self::constant(constant)
            .add(Self::cosine().scale_interval(cosine))
            .add(Self::sine().scale_interval(sine))
    }

    fn index(power: i32) -> usize {
        usize::try_from(power + LAURENT_ORDER).expect("bounded Laurent power")
    }

    fn coefficient(self, power: i32) -> ComplexInterval {
        self.coefficients[Self::index(power)]
    }

    fn set(&mut self, power: i32, value: ComplexInterval) {
        self.coefficients[Self::index(power)] = value;
    }

    fn add(mut self, other: Self) -> Self {
        for index in 0..LAURENT_COUNT {
            self.coefficients[index] = self.coefficients[index].add(other.coefficients[index]);
        }
        self
    }

    fn scale_interval(mut self, factor: Interval) -> Self {
        for coefficient in &mut self.coefficients {
            *coefficient = coefficient.scale(factor);
        }
        self
    }

    fn mul(self, other: Self) -> Self {
        let mut result = Self::zero();
        for left in -LAURENT_ORDER..=LAURENT_ORDER {
            for right in -LAURENT_ORDER..=LAURENT_ORDER {
                let power = left + right;
                if !(-LAURENT_ORDER..=LAURENT_ORDER).contains(&power) {
                    continue;
                }
                let value = result
                    .coefficient(power)
                    .add(self.coefficient(left).mul(other.coefficient(right)));
                result.set(power, value);
            }
        }
        result
    }

    fn derivative(self) -> Self {
        let mut result = Self::zero();
        for power in -LAURENT_ORDER..=LAURENT_ORDER {
            let value = self.coefficient(power);
            let factor = Interval::point(f64::from(power));
            result.set(
                power,
                ComplexInterval {
                    re: -value.im * factor,
                    im: value.re * factor,
                },
            );
        }
        result
    }

    fn integrate(self, start: f64, end: f64) -> Option<Interval> {
        let mut result = ComplexInterval::zero();
        for power in -LAURENT_ORDER..=LAURENT_ORDER {
            let basis = exponential_integral(power, start, end)?;
            result = result.add(self.coefficient(power).mul(basis));
        }
        (result.finite() && result.im.contains_zero()).then_some(result.re)
    }
}

#[derive(Debug, Clone, Copy)]
struct ComplexPoly {
    coefficients: [ComplexInterval; POLY_DEGREE + 1],
}

impl ComplexPoly {
    fn zero() -> Self {
        Self {
            coefficients: [ComplexInterval::zero(); POLY_DEGREE + 1],
        }
    }

    fn from_real(value: Poly) -> Self {
        let mut result = Self::zero();
        for index in 0..=POLY_DEGREE {
            result.coefficients[index] = ComplexInterval::real(value.coefficients[index]);
        }
        result
    }

    fn add(mut self, other: Self) -> Self {
        for index in 0..=POLY_DEGREE {
            self.coefficients[index] = self.coefficients[index].add(other.coefficients[index]);
        }
        self
    }

    fn scale(mut self, factor: ComplexInterval) -> Self {
        for coefficient in &mut self.coefficients {
            *coefficient = coefficient.mul(factor);
        }
        self
    }

    fn scale_real(mut self, factor: Interval) -> Self {
        for coefficient in &mut self.coefficients {
            *coefficient = coefficient.scale(factor);
        }
        self
    }

    fn integrate_exponential(
        self,
        harmonic: i32,
        u_origin: Interval,
        u_direction: f64,
        start: f64,
        end: f64,
    ) -> Option<ComplexInterval> {
        let basis = exponential_monomial_integrals(harmonic, u_origin, u_direction, start, end)?;
        let mut result = ComplexInterval::zero();
        for (coefficient, integral) in self.coefficients.into_iter().zip(basis) {
            result = result.add(coefficient.mul(integral));
        }
        result.finite().then_some(result)
    }
}

fn exponential_integral(power: i32, start: f64, end: f64) -> Option<ComplexInterval> {
    if power == 0 {
        return Some(ComplexInterval::real(
            Interval::point(end) - Interval::point(start),
        ));
    }
    let start_angle = Interval::point(f64::from(power)) * Interval::point(start);
    let end_angle = Interval::point(f64::from(power)) * Interval::point(end);
    unit_complex(end_angle)
        .sub(unit_complex(start_angle))
        .divide_i(Interval::point(f64::from(power)))
}

fn exponential_monomial_integrals(
    harmonic: i32,
    u_origin: Interval,
    u_direction: f64,
    start: f64,
    end: f64,
) -> Option<[ComplexInterval; POLY_DEGREE + 1]> {
    let mut result = [ComplexInterval::zero(); POLY_DEGREE + 1];
    if harmonic == 0 {
        for (power, value) in result.iter_mut().enumerate() {
            *value = ComplexInterval::real(monomial_integral(start, end, power)?);
        }
        return Some(result);
    }
    let lambda = Interval::point(f64::from(harmonic)) * Interval::point(u_direction);
    if !finite_interval(lambda) || lambda.contains_zero() {
        return None;
    }
    let angle = |parameter: f64| {
        Interval::point(f64::from(harmonic))
            * (u_origin + Interval::point(u_direction) * Interval::point(parameter))
    };
    let start_exp = unit_complex(angle(start));
    let end_exp = unit_complex(angle(end));
    result[0] = end_exp.sub(start_exp).divide_i(lambda)?;
    for power in 1..=POLY_DEGREE {
        let boundary = end_exp
            .scale(point_power(end, power)?)
            .sub(start_exp.scale(point_power(start, power)?));
        result[power] = boundary
            .sub(result[power - 1].scale(Interval::point(power as f64)))
            .divide_i(lambda)?;
    }
    result.iter().all(|value| value.finite()).then_some(result)
}

fn monomial_integral(start: f64, end: f64, power: usize) -> Option<Interval> {
    let exponent = power.checked_add(1)?;
    let numerator = point_power(end, exponent)? - point_power(start, exponent)?;
    let divisor_value = 1.0 / exponent as f64;
    let divisor = Interval::new(divisor_value.next_down(), divisor_value.next_up());
    let result = numerator * divisor;
    finite_interval(result).then_some(result)
}

fn point_power(value: f64, power: usize) -> Option<Interval> {
    if !value.is_finite() {
        return None;
    }
    let mut result = Interval::point(1.0);
    let factor = Interval::point(value);
    for _ in 0..power {
        result = result * factor;
    }
    finite_interval(result).then_some(result)
}

fn unit_complex(angle: Interval) -> ComplexInterval {
    if !finite_interval(angle) {
        return ComplexInterval {
            re: Interval::point(f64::INFINITY),
            im: Interval::point(f64::INFINITY),
        };
    }
    let midpoint = 0.5 * angle.lo() + 0.5 * angle.hi();
    let radius = (midpoint - angle.lo())
        .abs()
        .max((angle.hi() - midpoint).abs())
        .next_up();
    let (sine, cosine) = math::sincos(midpoint);
    let cover = |value: f64| {
        Interval::new(
            (value - radius).next_down().max(-1.0),
            (value + radius).next_up().min(1.0),
        )
    };
    ComplexInterval {
        re: cover(cosine),
        im: cover(sine),
    }
}

fn interval_ratio(numerator: f64, denominator: f64) -> Interval {
    let value = numerator / denominator;
    Interval::new(value.next_down(), value.next_up())
}

fn interval_product(left: f64, right: f64) -> Interval {
    Interval::point(left) * Interval::point(right)
}

fn point_vec(value: Vec3) -> [Interval; 3] {
    [
        Interval::point(value.x),
        Interval::point(value.y),
        Interval::point(value.z),
    ]
}

fn relative_coordinates(origin: Point3, anchor: Point3) -> [Interval; 3] {
    [
        Interval::point(origin.x) - Interval::point(anchor.x),
        Interval::point(origin.y) - Interval::point(anchor.y),
        Interval::point(origin.z) - Interval::point(anchor.z),
    ]
}

fn interval_dot(left: [Interval; 3], right: [Interval; 3]) -> Interval {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

fn interval_cross(left: [Interval; 3], right: [Interval; 3]) -> [Interval; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

fn finite_interval(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite()
}

fn finite_point(value: Point3) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
}

#[cfg(test)]
mod tests {
    use super::*;
    use kcore::operation::{OperationContext, OperationScope, SessionPolicy};
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;
    use kgeom::vec::Point2;

    fn query(store: &Store, body: BodyId) -> BodyPropertiesOutcome {
        let session = SessionPolicy::v1();
        let context = OperationContext::new(&session, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(BodyPropertiesBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        certify_body_properties_in_scope(store, body, &mut scope).unwrap()
    }

    #[test]
    fn valid_sphere_refuses_the_unsupported_surface_without_tessellating() {
        let mut store = Store::new();
        let body = crate::make::sphere(&mut store, &Frame::world(), 2.0).unwrap();
        let BodyPropertiesOutcome::Refused { reason, full_check } = query(&store, body) else {
            panic!("unsupported sphere unexpectedly produced analytic properties")
        };
        assert_eq!(full_check.outcome(), CheckOutcome::Valid);
        assert!(matches!(
            reason,
            BodyPropertiesRefusal::UnsupportedSurface { .. }
        ));
    }

    #[test]
    fn valid_sheet_refuses_non_solid_interrogation() {
        let mut store = Store::new();
        let body = crate::make::planar_sheet(
            &mut store,
            &Frame::world(),
            &[
                Point2::new(-1.0, -1.0),
                Point2::new(1.0, -1.0),
                Point2::new(1.0, 1.0),
                Point2::new(-1.0, 1.0),
            ],
        )
        .unwrap();
        let BodyPropertiesOutcome::Refused { reason, full_check } = query(&store, body) else {
            panic!("sheet unexpectedly produced solid properties")
        };
        assert_eq!(full_check.outcome(), CheckOutcome::Valid);
        assert_eq!(reason, BodyPropertiesRefusal::NonSolidBody);
    }
}
