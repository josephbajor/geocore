//! Operation-local authority for source-edge intersection-root identity.
//!
//! A trim endpoint on an edge is not identified by its rounded model-space
//! point.  It is identified by the source edge and the ordinal of the root in
//! the edge's intrinsic parameter direction.  The ordinal is certified once
//! for an `(edge, opposing face)` query and shared by every incident fin,
//! section branch, and operand ordering that observes that query.
//!
//! This first exact-family slice admits bounded line edges against cylindrical
//! faces and vertexless whole-period circle edges against planar faces.
//! Bounded or vertex-backed circle edges remain unsupported until periodic
//! copy enumeration has its own certified integer-range proof. Root
//! coefficients and enclosures use outward interval arithmetic over authored
//! source values. Exact affine predicates decide harmonic degeneracies.
//! Tangency, coincidence, parameter-seam contact, unordered enclosures, and
//! incomplete observed evidence fail closed with stable typed gaps.

use std::collections::HashMap;

use kcore::interval::Interval;
use kcore::math;
use kcore::operation::OperationScope;
use kcore::predicates::{Orientation, affine_dot3};
use kgeom::curve::{Circle, Line};
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::Vec3;
use ktopo::entity::{EdgeId as RawEdgeId, FaceId as RawFaceId};
use ktopo::geom::{CurveGeom, SurfaceGeom};
use ktopo::store::Store;

use crate::error::{Error, Result};

use super::SECTION_WORK;

/// One operation-shared source-root identity.
///
/// The opposing face remains part of the authority's query key.  At a stitched
/// endpoint it is already retained by the topology site, so the compact root
/// key needs only the source edge and its intrinsic root ordinal.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct SourceRootKey {
    edge: RawEdgeId,
    ordinal: usize,
}

impl SourceRootKey {
    pub(crate) const fn edge(self) -> RawEdgeId {
        self.edge
    }

    pub(crate) const fn ordinal(self) -> usize {
        self.ordinal
    }
}

/// The complete source-root query whose result owns ordinal assignment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct SourceRootQuery {
    edge: RawEdgeId,
    opposing_face: RawFaceId,
}

impl SourceRootQuery {
    pub(crate) const fn new(edge: RawEdgeId, opposing_face: RawFaceId) -> Self {
        Self {
            edge,
            opposing_face,
        }
    }
}

/// Stable fail-closed classes for source-root certification and resolution.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RootIdentityGap {
    /// The edge/surface pair is outside this analytic authority slice.
    UnsupportedGeometry,
    /// The source edge lacks a usable intrinsic finite domain or curve.
    MalformedSourceEdge,
    /// Outward arithmetic could not retain finite, decisive evidence.
    ArithmeticGuard,
    /// Root multiplicity could not be separated from tangency.
    TangentialOrUnresolvedMultiplicity,
    /// The source edge and opposing surface have a non-discrete intersection.
    CoincidentGeometry,
    /// A periodic root lies on the source curve's canonical parameter seam.
    ParameterSeamContact,
    /// A root cannot be separated from a bounded edge endpoint.
    EdgeBoundaryContact,
    /// Distinct complete root enclosures lack a strict intrinsic order.
    UnorderedRoots,
    /// The observed parameter evidence is non-finite.
    InvalidObservation,
    /// The observed parameter enclosure intersects no certified source root.
    NoMatchingRoot,
    /// The observed parameter enclosure intersects more than one source root.
    AmbiguousObservation,
}

impl RootIdentityGap {
    /// Stable diagnostic suitable for a section gap.
    pub(crate) const fn reason(self) -> &'static str {
        match self {
            Self::UnsupportedGeometry => {
                "source-root identity does not support this edge/surface pair"
            }
            Self::MalformedSourceEdge => {
                "source-root identity requires an exact edge with a usable intrinsic domain"
            }
            Self::ArithmeticGuard => {
                "source-root identity could not certify a finite arithmetic enclosure"
            }
            Self::TangentialOrUnresolvedMultiplicity => {
                "source-root identity could not separate a transverse root from tangency"
            }
            Self::CoincidentGeometry => {
                "a source edge has a non-discrete intersection with the opposing face surface"
            }
            Self::ParameterSeamContact => {
                "a source root lies on an unresolved periodic parameter seam"
            }
            Self::EdgeBoundaryContact => {
                "a source root cannot be separated from a bounded edge endpoint"
            }
            Self::UnorderedRoots => {
                "source-edge roots could not be put in strict intrinsic parameter order"
            }
            Self::InvalidObservation => "an observed source-edge parameter enclosure is not finite",
            Self::NoMatchingRoot => {
                "observed source-edge evidence matches no certified opposing-face root"
            }
            Self::AmbiguousObservation => {
                "observed source-edge evidence matches multiple opposing-face roots"
            }
        }
    }
}

/// A complete, strictly ordered root set for one source query.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct CertifiedSourceRootOrder {
    query: SourceRootQuery,
    roots: Vec<Interval>,
}

impl CertifiedSourceRootOrder {
    /// Roots in strictly increasing intrinsic source-edge parameter order.
    pub(crate) fn roots(&self) -> &[Interval] {
        &self.roots
    }

    fn resolve(&self, observed: Interval) -> RootResolution {
        if !finite(observed) {
            return RootResolution::Indeterminate(RootIdentityGap::InvalidObservation);
        }
        // The observation comes from a topology pcurve whose lifted trace may
        // be tolerance-close, rather than coefficient-identical, to the 3D
        // edge. A unique overlap assigns the analytic root identity only;
        // consumers must retain the hull of both enclosures, never their
        // intersection, as metric evidence.
        resolve_order(self.query.edge, &self.roots, observed)
    }
}

/// Result of certifying the complete root order for one query.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum RootOrderOutcome {
    Certified(CertifiedSourceRootOrder),
    Indeterminate(RootIdentityGap),
}

/// Result of resolving one observed edge-parameter enclosure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RootResolution {
    Resolved(SourceRootKey),
    Indeterminate(RootIdentityGap),
}

/// Operation-local cache and sole ordinal authority for source roots.
#[derive(Debug, Default)]
pub(crate) struct RootIdentityAuthority {
    orders: HashMap<SourceRootQuery, RootOrderOutcome>,
}

impl RootIdentityAuthority {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    /// Certify or retrieve the complete root order for `query`.
    ///
    /// Semantic outcomes are cached, while resource-limit failures propagate
    /// without poisoning the operation-local authority.
    pub(crate) fn certify_order(
        &mut self,
        store: &Store,
        query: SourceRootQuery,
        scope: &mut OperationScope<'_, '_>,
    ) -> Result<RootOrderOutcome> {
        charge(scope, 1)?;
        if let Some(outcome) = self.orders.get(&query) {
            return Ok(outcome.clone());
        }
        let outcome = certify_query(store, query, scope)?;
        self.orders.insert(query, outcome.clone());
        Ok(outcome)
    }

    /// Resolve observed intrinsic edge-parameter evidence to exactly one root.
    pub(crate) fn resolve(
        &mut self,
        store: &Store,
        query: SourceRootQuery,
        observed: Interval,
        scope: &mut OperationScope<'_, '_>,
    ) -> Result<RootResolution> {
        if !finite(observed) {
            return Ok(RootResolution::Indeterminate(
                RootIdentityGap::InvalidObservation,
            ));
        }
        Ok(match self.certify_order(store, query, scope)? {
            RootOrderOutcome::Certified(order) => order.resolve(observed),
            RootOrderOutcome::Indeterminate(gap) => RootResolution::Indeterminate(gap),
        })
    }
}

fn read<T>(result: kcore::error::Result<T>) -> Result<T> {
    result.map_err(|source| Error::InconsistentTopology { source })
}

fn charge(scope: &mut OperationScope<'_, '_>, amount: u64) -> Result<()> {
    scope
        .ledger_mut()
        .charge(SECTION_WORK, amount)
        .map_err(Error::from)
}

fn certify_query(
    store: &Store,
    query: SourceRootQuery,
    scope: &mut OperationScope<'_, '_>,
) -> Result<RootOrderOutcome> {
    charge(scope, 1)?;
    let edge = read(store.get(query.edge))?;
    let face = read(store.get(query.opposing_face))?;
    let Some(curve_id) = edge.curve() else {
        return Ok(RootOrderOutcome::Indeterminate(
            RootIdentityGap::MalformedSourceEdge,
        ));
    };
    let curve = read(store.curve(curve_id))?;
    let surface = read(store.surface(face.surface()))?;
    // One fixed analytic certificate unit. Limit failures remain facade
    // errors and are never cached as semantic root gaps.
    charge(scope, 4)?;
    let roots = match (curve, surface) {
        (CurveGeom::Line(line), SurfaceGeom::Cylinder(cylinder)) => {
            let Some((lo, hi)) = edge.bounds() else {
                return Ok(RootOrderOutcome::Indeterminate(
                    RootIdentityGap::MalformedSourceEdge,
                ));
            };
            let Some(active) = active_interval(lo, hi) else {
                return Ok(RootOrderOutcome::Indeterminate(
                    RootIdentityGap::MalformedSourceEdge,
                ));
            };
            certify_line_cylinder(*line, *cylinder, active)
        }
        (CurveGeom::Circle(circle), SurfaceGeom::Plane(plane)) => {
            if edge.bounds().is_some() || edge.vertices() != [None, None] {
                return Ok(RootOrderOutcome::Indeterminate(
                    RootIdentityGap::UnsupportedGeometry,
                ));
            }
            certify_circle_plane(*circle, *plane)
        }
        _ => {
            return Ok(RootOrderOutcome::Indeterminate(
                RootIdentityGap::UnsupportedGeometry,
            ));
        }
    };
    Ok(match roots {
        Ok(roots) => RootOrderOutcome::Certified(CertifiedSourceRootOrder { query, roots }),
        Err(gap) => RootOrderOutcome::Indeterminate(gap),
    })
}

fn certify_line_cylinder(
    line: Line,
    cylinder: Cylinder,
    active: Interval,
) -> core::result::Result<Vec<Interval>, RootIdentityGap> {
    if !valid_active(active) {
        return Err(RootIdentityGap::MalformedSourceEdge);
    }
    let frame = cylinder.frame();
    let origin = line.origin();
    let direction = line.dir();
    let relative = interval_sub(interval_point(origin), interval_point(frame.origin()));
    let x = interval_point(frame.x());
    let y = interval_point(frame.y());
    let direction_i = interval_point(direction);
    let qx = interval_dot(x, relative);
    let qy = interval_dot(y, relative);
    let dx = interval_dot(x, direction_i);
    let dy = interval_dot(y, direction_i);
    let a = dx.square() + dy.square();
    let b = Interval::point(2.0) * (qx * dx + qy * dy);
    let c = qx.square() + qy.square() - Interval::point(cylinder.radius()).square();
    if !finite(a) || !finite(b) || !finite(c) {
        return Err(RootIdentityGap::ArithmeticGuard);
    }

    let zero = [0.0; 3];
    let exact_dx = affine_dot3(frame.x().to_array(), direction.to_array(), zero, 0.0)
        .ok_or(RootIdentityGap::ArithmeticGuard)?;
    let exact_dy = affine_dot3(frame.y().to_array(), direction.to_array(), zero, 0.0)
        .ok_or(RootIdentityGap::ArithmeticGuard)?;
    if exact_dx.sign() == Orientation::Zero && exact_dy.sign() == Orientation::Zero {
        return if excludes_zero(c) {
            Ok(Vec::new())
        } else {
            Err(RootIdentityGap::CoincidentGeometry)
        };
    }
    if a.contains_zero() {
        return Err(RootIdentityGap::ArithmeticGuard);
    }
    let roots = transverse_quadratic_roots(a, b, c)?;
    retain_bounded_roots(roots, active)
}

fn certify_circle_plane(
    circle: Circle,
    plane: Plane,
) -> core::result::Result<Vec<Interval>, RootIdentityGap> {
    let tau = core::f64::consts::TAU;
    let active = Interval::new(0.0, tau);
    let normal = plane.frame().z();
    let circle_frame = circle.frame();
    let radius = Interval::point(circle.radius());
    let cosine = radius * affine_dot_interval(normal, circle_frame.x(), Vec3::new(0.0, 0.0, 0.0));
    let sine = radius * affine_dot_interval(normal, circle_frame.y(), Vec3::new(0.0, 0.0, 0.0));
    let constant = affine_dot_interval(normal, circle_frame.origin(), plane.frame().origin());
    if !finite(cosine) || !finite(sine) || !finite(constant) {
        return Err(RootIdentityGap::ArithmeticGuard);
    }

    let zero = [0.0; 3];
    let cosine_exact = affine_dot3(normal.to_array(), circle_frame.x().to_array(), zero, 0.0)
        .ok_or(RootIdentityGap::ArithmeticGuard)?;
    let sine_exact = affine_dot3(normal.to_array(), circle_frame.y().to_array(), zero, 0.0)
        .ok_or(RootIdentityGap::ArithmeticGuard)?;
    let constant_exact = affine_dot3(
        normal.to_array(),
        circle_frame.origin().to_array(),
        plane.frame().origin().to_array(),
        0.0,
    )
    .ok_or(RootIdentityGap::ArithmeticGuard)?;
    if cosine_exact.sign() == Orientation::Zero && sine_exact.sign() == Orientation::Zero {
        return if constant_exact.sign() == Orientation::Zero {
            Err(RootIdentityGap::CoincidentGeometry)
        } else {
            Ok(Vec::new())
        };
    }

    let discriminant = cosine.square() + sine.square() - constant.square();
    if !finite(discriminant) {
        return Err(RootIdentityGap::ArithmeticGuard);
    }
    if discriminant.hi() < 0.0 {
        return Ok(Vec::new());
    }
    if discriminant.lo() <= 0.0 {
        return Err(RootIdentityGap::TangentialOrUnresolvedMultiplicity);
    }
    let quadratic = [
        constant - cosine,
        Interval::point(2.0) * sine,
        constant + cosine,
    ];
    if quadratic[0].contains_zero() {
        return Err(RootIdentityGap::ParameterSeamContact);
    }
    let half_angles = transverse_quadratic_roots(quadratic[0], quadratic[1], quadratic[2])?;
    let mut roots = Vec::with_capacity(half_angles.len());
    for half_angle in half_angles {
        let principal = twice_atan_interval(half_angle)?;
        if principal.lo() <= 0.0 && principal.hi() >= 0.0 {
            return Err(RootIdentityGap::ParameterSeamContact);
        }
        let canonical = if principal.hi() < 0.0 {
            principal + Interval::point(tau)
        } else {
            principal
        };
        roots.push(canonical);
    }
    strict_sort(&mut roots)?;
    retain_bounded_roots(roots, active)
}

fn transverse_quadratic_roots(
    a: Interval,
    b: Interval,
    c: Interval,
) -> core::result::Result<Vec<Interval>, RootIdentityGap> {
    if a.contains_zero() || !finite(a) || !finite(b) || !finite(c) {
        return Err(RootIdentityGap::ArithmeticGuard);
    }
    let discriminant = b.square() - Interval::point(4.0) * a * c;
    if !finite(discriminant) {
        return Err(RootIdentityGap::ArithmeticGuard);
    }
    if discriminant.hi() < 0.0 {
        return Ok(Vec::new());
    }
    if discriminant.lo() <= 0.0 {
        return Err(RootIdentityGap::TangentialOrUnresolvedMultiplicity);
    }
    let root = discriminant
        .sqrt()
        .ok_or(RootIdentityGap::ArithmeticGuard)?;
    let denominator = Interval::point(2.0) * a;
    let first = (-b - root)
        .checked_div(denominator)
        .ok_or(RootIdentityGap::ArithmeticGuard)?;
    let second = (-b + root)
        .checked_div(denominator)
        .ok_or(RootIdentityGap::ArithmeticGuard)?;
    let mut roots = vec![first, second];
    strict_sort(&mut roots)?;
    Ok(roots)
}

fn retain_bounded_roots(
    roots: Vec<Interval>,
    active: Interval,
) -> core::result::Result<Vec<Interval>, RootIdentityGap> {
    let mut retained = Vec::new();
    for root in roots {
        if !finite(root) {
            return Err(RootIdentityGap::ArithmeticGuard);
        }
        if root.hi() < active.lo() || root.lo() > active.hi() {
            continue;
        }
        if root.lo() <= active.lo() || root.hi() >= active.hi() {
            return Err(RootIdentityGap::EdgeBoundaryContact);
        }
        retained.push(root);
    }
    strict_sort(&mut retained)?;
    Ok(retained)
}

fn strict_sort(roots: &mut [Interval]) -> core::result::Result<(), RootIdentityGap> {
    roots.sort_by(|a, b| a.lo().total_cmp(&b.lo()).then(a.hi().total_cmp(&b.hi())));
    if roots.windows(2).any(|pair| pair[0].hi() >= pair[1].lo()) {
        return Err(RootIdentityGap::UnorderedRoots);
    }
    Ok(())
}

fn resolve_order(edge: RawEdgeId, roots: &[Interval], observed: Interval) -> RootResolution {
    let mut matched = roots
        .iter()
        .enumerate()
        .filter(|(_, root)| root.intersects(observed));
    let Some((ordinal, _)) = matched.next() else {
        return RootResolution::Indeterminate(RootIdentityGap::NoMatchingRoot);
    };
    if matched.next().is_some() {
        return RootResolution::Indeterminate(RootIdentityGap::AmbiguousObservation);
    }
    RootResolution::Resolved(SourceRootKey { edge, ordinal })
}

fn twice_atan_interval(value: Interval) -> core::result::Result<Interval, RootIdentityGap> {
    if !finite(value) {
        return Err(RootIdentityGap::ArithmeticGuard);
    }
    let mut lo = 2.0 * math::atan(value.lo());
    let mut hi = 2.0 * math::atan(value.hi());
    if !lo.is_finite() || !hi.is_finite() || lo > hi {
        return Err(RootIdentityGap::ArithmeticGuard);
    }
    // Deterministic atan is documented to be within one ulp.  Two ulps cover
    // its error and two more cover the multiplication and interval endpoint.
    for _ in 0..4 {
        lo = lo.next_down();
        hi = hi.next_up();
    }
    Ok(Interval::new(lo, hi))
}

#[derive(Debug, Clone, Copy)]
struct IntervalVec3 {
    x: Interval,
    y: Interval,
    z: Interval,
}

fn interval_point(value: Vec3) -> IntervalVec3 {
    IntervalVec3 {
        x: Interval::point(value.x),
        y: Interval::point(value.y),
        z: Interval::point(value.z),
    }
}

fn interval_sub(a: IntervalVec3, b: IntervalVec3) -> IntervalVec3 {
    IntervalVec3 {
        x: a.x - b.x,
        y: a.y - b.y,
        z: a.z - b.z,
    }
}

fn interval_dot(a: IntervalVec3, b: IntervalVec3) -> Interval {
    a.x * b.x + a.y * b.y + a.z * b.z
}

fn affine_dot_interval(normal: Vec3, point: Vec3, origin: Vec3) -> Interval {
    interval_dot(
        interval_point(normal),
        interval_sub(interval_point(point), interval_point(origin)),
    )
}

fn excludes_zero(value: Interval) -> bool {
    value.hi() < 0.0 || value.lo() > 0.0
}

fn finite(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite()
}

fn valid_active(active: Interval) -> bool {
    finite(active) && active.lo() < active.hi()
}

fn active_interval(lo: f64, hi: f64) -> Option<Interval> {
    (lo.is_finite() && hi.is_finite() && lo < hi).then(|| Interval::new(lo, hi))
}

#[cfg(test)]
mod tests {
    use kcore::operation::{OperationContext, OperationScope, SessionPolicy};
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;
    use kgeom::vec::{Point3, Vec3};

    use super::*;
    use crate::section::BodySectionBudgetProfile;

    fn with_scope<T>(run: impl FnOnce(&mut OperationScope<'_, '_>) -> T) -> T {
        let policy = SessionPolicy::v1();
        let context = OperationContext::new(&policy, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(BodySectionBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        run(&mut scope)
    }

    #[test]
    fn line_cylinder_roots_are_complete_and_intrinsically_ordered() {
        let line = Line::new(Point3::new(-2.0, 0.0, 0.25), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let cylinder = Cylinder::new(Frame::world(), 1.0).unwrap();
        let roots = certify_line_cylinder(line, cylinder, Interval::new(0.0, 4.0)).unwrap();
        assert_eq!(roots.len(), 2);
        assert!(roots[0].contains(1.0));
        assert!(roots[1].contains(3.0));
        assert!(roots[0].hi() < roots[1].lo());
    }

    #[test]
    fn circle_plane_roots_use_source_circle_parameter_order() {
        let circle = Circle::new(Frame::world(), 1.0).unwrap();
        let plane_frame =
            Frame::from_z(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let plane = Plane::new(plane_frame);
        let roots = certify_circle_plane(circle, plane).unwrap();
        assert_eq!(roots.len(), 2);
        assert!(roots[0].contains(core::f64::consts::FRAC_PI_2));
        assert!(roots[1].contains(3.0 * core::f64::consts::FRAC_PI_2));
        assert!(roots[0].hi() < roots[1].lo());
    }

    #[test]
    fn circle_plane_tangency_fails_closed() {
        let circle = Circle::new(Frame::world(), 1.0).unwrap();
        let plane_frame =
            Frame::from_z(Point3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let result = certify_circle_plane(circle, Plane::new(plane_frame));
        assert_eq!(
            result,
            Err(RootIdentityGap::TangentialOrUnresolvedMultiplicity)
        );
    }

    #[test]
    fn bounded_circle_edges_are_not_admitted_as_periodic_root_authority() {
        let mut store = Store::new();
        let sheet = ktopo::make::cylindrical_sheet(&mut store, &Frame::world(), 1.0, 2.0).unwrap();
        let bounded_circle = store
            .edges_of_body(sheet)
            .unwrap()
            .into_iter()
            .find(|edge_id| {
                let edge = store.get(*edge_id).unwrap();
                edge.bounds().is_some()
                    && edge.curve().is_some_and(|curve| {
                        matches!(store.curve(curve).unwrap(), CurveGeom::Circle(_))
                    })
            })
            .expect("cylindrical sheet must expose a bounded circle edge");
        let block = ktopo::make::block(&mut store, &Frame::world(), [2.0; 3]).unwrap();
        let plane_face = store
            .faces_of_body(block)
            .unwrap()
            .into_iter()
            .find(|face_id| {
                matches!(
                    store
                        .surface(store.get(*face_id).unwrap().surface())
                        .unwrap(),
                    SurfaceGeom::Plane(_)
                )
            })
            .expect("block must expose a planar face");
        let outcome = with_scope(|scope| {
            RootIdentityAuthority::new()
                .certify_order(
                    &store,
                    SourceRootQuery::new(bounded_circle, plane_face),
                    scope,
                )
                .unwrap()
        });
        assert_eq!(
            outcome,
            RootOrderOutcome::Indeterminate(RootIdentityGap::UnsupportedGeometry)
        );
    }

    #[test]
    fn rigid_frame_change_preserves_intrinsic_root_order() {
        let reference = certify_line_cylinder(
            Line::new(Point3::new(-2.0, 0.0, 0.25), Vec3::new(1.0, 0.0, 0.0)).unwrap(),
            Cylinder::new(Frame::world(), 1.0).unwrap(),
            Interval::new(0.0, 4.0),
        )
        .unwrap();
        let transformed_frame = Frame::new(
            Point3::new(4.0, -3.0, 2.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap();
        let transformed = certify_line_cylinder(
            Line::new(
                transformed_frame.origin() - transformed_frame.x() * 2.0
                    + transformed_frame.z() * 0.25,
                transformed_frame.x(),
            )
            .unwrap(),
            Cylinder::new(transformed_frame, 1.0).unwrap(),
            Interval::new(0.0, 4.0),
        )
        .unwrap();
        assert_eq!(reference.len(), transformed.len());
        for (a, b) in reference.iter().zip(&transformed) {
            assert!(a.intersects(*b));
        }
    }

    #[test]
    fn observation_resolution_never_uses_metric_points() {
        let mut roots = vec![Interval::new(0.9, 1.1), Interval::new(2.9, 3.1)];
        strict_sort(&mut roots).unwrap();
        // Handles are intentionally obtained from authored topology; their
        // numeric representation is opaque and irrelevant to this test.
        let mut store = Store::new();
        let body = ktopo::make::block(&mut store, &Frame::world(), [1.0; 3]).unwrap();
        let edge = store.edges_of_body(body).unwrap()[0];
        assert!(matches!(
            resolve_order(edge, &roots, Interval::new(0.95, 1.05)),
            RootResolution::Resolved(SourceRootKey { ordinal: 0, .. })
        ));
        let first = resolve_order(edge, &roots, Interval::new(0.95, 1.05));
        let second = resolve_order(edge, &roots, Interval::new(2.95, 3.05));
        assert_ne!(first, second, "one edge's two roots must not alias");
        assert_eq!(
            resolve_order(edge, &roots, Interval::new(1.0, 3.0)),
            RootResolution::Indeterminate(RootIdentityGap::AmbiguousObservation)
        );
        assert_eq!(
            resolve_order(edge, &roots, Interval::new(5.0, 6.0)),
            RootResolution::Indeterminate(RootIdentityGap::NoMatchingRoot)
        );
    }
}
