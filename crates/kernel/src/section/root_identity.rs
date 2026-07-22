//! Operation-local authority for source-edge intersection-root identity.
//!
//! A trim endpoint on an edge is not identified by its rounded model-space
//! point.  It is identified by the source edge and the ordinal of the root in
//! the edge's intrinsic parameter direction.  The ordinal is certified once
//! for an `(edge, opposing face)` query and shared by every incident fin,
//! section branch, and operand ordering that observes that query.
//!
//! This exact-family slice admits bounded line edges against cylindrical
//! faces and vertexless whole-period circle edges against planar faces or
//! cylinders whose axes are exactly parallel to the circle normal. Bounded or
//! vertex-backed circle edges remain unsupported until periodic copy
//! enumeration has its own certified integer-range proof. Root coefficients
//! and enclosures use outward interval arithmetic over authored source values.
//! Exact affine predicates decide family admission and semantic degeneracies.
//! The complete second-harmonic Circle/Cylinder restriction is certified as a
//! compactified tan-half-angle quartic. Tangency, coincidence, parameter-seam
//! contact, unordered enclosures, and incomplete observed evidence fail closed
//! with stable typed gaps.

use std::collections::HashMap;

use kcore::interval::Interval;
use kcore::math;
use kcore::operation::OperationScope;
use kcore::predicates::{Orientation, affine_dot3, orient3d};
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

/// Canonical scalar witness for one isolated source root.
///
/// The interval is the authority: it was produced by the complete analytic
/// root-order proof and contains exactly one transverse root. `parameter` is
/// a deterministic finite value inside that interval for authoring a
/// tolerance-bounded topology split. Floating-point equality of parameters
/// never owns endpoint identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct CertifiedSourceRootScalar {
    parameter_bits: u64,
    enclosure_bits: [u64; 2],
}

impl CertifiedSourceRootScalar {
    fn from_enclosure(enclosure: Interval) -> Option<Self> {
        if !finite(enclosure) || enclosure.lo() > enclosure.hi() {
            return None;
        }
        // The split form avoids overflow in `lo + hi` and is deterministic
        // over the certified finite enclosure.
        let parameter = 0.5 * enclosure.lo() + 0.5 * enclosure.hi();
        if !parameter.is_finite() || parameter < enclosure.lo() || parameter > enclosure.hi() {
            return None;
        }
        Some(Self {
            parameter_bits: parameter.to_bits(),
            enclosure_bits: [enclosure.lo().to_bits(), enclosure.hi().to_bits()],
        })
    }

    pub(crate) const fn parameter(self) -> f64 {
        f64::from_bits(self.parameter_bits)
    }

    pub(crate) fn enclosure(self) -> Interval {
        Interval::new(
            f64::from_bits(self.enclosure_bits[0]),
            f64::from_bits(self.enclosure_bits[1]),
        )
    }
}

impl SourceRootKey {
    pub(crate) const fn new(edge: RawEdgeId, ordinal: usize) -> Self {
        Self { edge, ordinal }
    }

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

    /// Materialize the canonical scalar witness for an identity issued from
    /// this exact query order.
    pub(crate) fn materialize(&self, key: SourceRootKey) -> Option<CertifiedSourceRootScalar> {
        if key.edge != self.query.edge {
            return None;
        }
        self.roots
            .get(key.ordinal)
            .copied()
            .and_then(CertifiedSourceRootScalar::from_enclosure)
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
    if edge.tolerance().is_some() {
        return Ok(RootOrderOutcome::Indeterminate(
            RootIdentityGap::MalformedSourceEdge,
        ));
    }
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
        (CurveGeom::Circle(circle), SurfaceGeom::Cylinder(cylinder)) => {
            if edge.bounds().is_some() || edge.vertices() != [None, None] {
                return Ok(RootOrderOutcome::Indeterminate(
                    RootIdentityGap::UnsupportedGeometry,
                ));
            }
            charge_circle_cylinder_quartic(scope)?;
            certify_circle_cylinder(*circle, *cylinder)
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

/// Certify the intrinsic roots of a whole source circle against a cylinder
/// whose axis is exactly parallel or antiparallel to the circle normal.
///
/// In that family the cylinder implicit restricted to the circle is a complete
/// second-harmonic trigonometric polynomial. Stored [`Frame`] axes satisfy an
/// orthonormal semantic contract but are rounded binary64 vectors. Cramer's
/// rule first expresses the source basis in the cylinder's actual stored
/// `(X,Y,Z)` basis; every resulting Gram term is retained as a coefficient
/// rather than collapsed into a scalar error guard. The tan-half-angle
/// substitution produces a full quartic. Both real half-angle branches are
/// compactified onto `[0, 1]`, and interval range plus derivative certificates
/// prove a complete cover with either zero roots or exactly two unique
/// transverse roots. Anything near a multiple root, a chart boundary, or a
/// four-root rounded-frame case fails closed. The source circle's own
/// `[0, TAU]` parameter order remains the sole root ordinal authority.
fn certify_circle_cylinder(
    circle: Circle,
    cylinder: Cylinder,
) -> core::result::Result<Vec<Interval>, RootIdentityGap> {
    let circle_frame = circle.frame();
    let cylinder_frame = cylinder.frame();
    let circle_axis = circle_frame.z();
    let cylinder_axis = cylinder_frame.z();
    if !vectors_are_exactly_parallel(circle_axis, cylinder_axis) {
        return Err(RootIdentityGap::UnsupportedGeometry);
    }
    if [
        circle.radius(),
        cylinder.radius(),
        circle_frame.origin().x,
        circle_frame.origin().y,
        circle_frame.origin().z,
        cylinder_frame.origin().x,
        cylinder_frame.origin().y,
        cylinder_frame.origin().z,
    ]
    .into_iter()
    .any(|value| !value.is_finite())
    {
        return Err(RootIdentityGap::ArithmeticGuard);
    }

    let relative = interval_sub(
        interval_point(circle_frame.origin()),
        interval_point(cylinder_frame.origin()),
    );
    if !finite_interval_vec3(relative) {
        return Err(RootIdentityGap::ArithmeticGuard);
    }
    if points_are_exactly_axis_aligned(
        circle_frame.origin(),
        cylinder_frame.origin(),
        cylinder_axis,
    )? && circle_frame.x() == cylinder_frame.x()
        && circle_frame.y() == cylinder_frame.y()
    {
        return if circle.radius() == cylinder.radius() {
            Err(RootIdentityGap::CoincidentGeometry)
        } else {
            Ok(Vec::new())
        };
    }

    let cylinder_x = interval_point(cylinder_frame.x());
    let cylinder_y = interval_point(cylinder_frame.y());
    let cylinder_z = interval_point(cylinder_frame.z());
    let basis_determinant = interval_determinant(cylinder_x, cylinder_y, cylinder_z);
    if basis_determinant.contains_zero() || !finite(basis_determinant) {
        return Err(RootIdentityGap::ArithmeticGuard);
    }
    // Coordinates in the cylinder's actual stored `(X, Y, Z)` basis. Dot
    // products are not inverse coordinates when the semantically orthonormal
    // frame axes retain binary64 Gram residuals. Cramer's rule makes axial
    // translation disappear and describes the parametric cylinder set rather
    // than a surrogate orthogonal projection.
    let radial_coordinates = |vector: IntervalVec3| {
        let x = interval_determinant(vector, cylinder_y, cylinder_z)
            .checked_div(basis_determinant)
            .ok_or(RootIdentityGap::ArithmeticGuard)?;
        let y = interval_determinant(cylinder_x, vector, cylinder_z)
            .checked_div(basis_determinant)
            .ok_or(RootIdentityGap::ArithmeticGuard)?;
        if !finite(x) || !finite(y) {
            return Err(RootIdentityGap::ArithmeticGuard);
        }
        Ok([x, y])
    };
    let center = radial_coordinates(relative)?;
    let circle_x = interval_point(circle_frame.x());
    let circle_y = interval_point(circle_frame.y());
    let radial_x = radial_coordinates(circle_x)?;
    let radial_y = radial_coordinates(circle_y)?;

    let radius = Interval::point(circle.radius());
    let radius_squared = radius.square();
    let constant =
        center[0].square() + center[1].square() - Interval::point(cylinder.radius()).square();
    let first_cosine =
        Interval::point(2.0) * radius * (center[0] * radial_x[0] + center[1] * radial_x[1]);
    let first_sine =
        Interval::point(2.0) * radius * (center[0] * radial_y[0] + center[1] * radial_y[1]);
    let cosine_squared = radius_squared * (radial_x[0].square() + radial_x[1].square());
    let sine_squared = radius_squared * (radial_y[0].square() + radial_y[1].square());
    let cosine_sine = Interval::point(2.0)
        * radius_squared
        * (radial_x[0] * radial_y[0] + radial_x[1] * radial_y[1]);
    let two = Interval::point(2.0);
    let four = Interval::point(4.0);
    // `(1 + h^2)^2 f(2 atan h)`, in increasing powers of `h`.
    let quartic = [
        constant + first_cosine + cosine_squared,
        two * (first_sine + cosine_sine),
        two * constant - two * cosine_squared + four * sine_squared,
        two * (first_sine - cosine_sine),
        constant - first_cosine + cosine_squared,
    ];
    if quartic.iter().any(|coefficient| !finite(*coefficient)) {
        return Err(RootIdentityGap::ArithmeticGuard);
    }
    certify_periodic_quartic_roots(quartic)
}

const QUARTIC_ROOT_MAX_DEPTH: usize = 24;
const QUARTIC_ROOT_MAX_NODES: usize = 16_384;
// Each compact half-angle branch may visit at most `MAX_NODES`. Precharging
// both complete covers makes the semantic result independent of early exits
// while bounding the recursive work below the default Section allowance.
const CIRCLE_CYLINDER_QUARTIC_WORK: u64 = 2 * QUARTIC_ROOT_MAX_NODES as u64;

fn charge_circle_cylinder_quartic(scope: &mut OperationScope<'_, '_>) -> Result<()> {
    charge(scope, CIRCLE_CYLINDER_QUARTIC_WORK)
}

/// Certify all roots of a periodic quartic half-angle numerator.
///
/// Positive and negative half angles are independently compactified by
/// `h = +/-s/(1-s)`, `s in [0, 1]`. The transformed polynomial has the same
/// finite roots because its positive denominator was cleared. A recursive
/// interval cover discards only cells whose polynomial range excludes zero;
/// every retained root cell must additionally have a one-signed derivative
/// and opposite endpoint signs, proving existence and uniqueness. Thus the
/// accepted two cells are a complete two-root proof, not candidate samples.
fn certify_periodic_quartic_roots(
    quartic: [Interval; 5],
) -> core::result::Result<Vec<Interval>, RootIdentityGap> {
    if quartic[0].contains_zero() {
        return Err(RootIdentityGap::ParameterSeamContact);
    }
    // `h = +/-infinity` is the ordinary source parameter `PI`, but is a chart
    // boundary for both compactifications. Do not infer its multiplicity from
    // a degree drop.
    if quartic[4].contains_zero() {
        return Err(RootIdentityGap::TangentialOrUnresolvedMultiplicity);
    }

    let mut compact_roots = Vec::new();
    for half_angle_sign in [1_i8, -1_i8] {
        let compact = compactified_quartic(quartic, half_angle_sign)?;
        for root in isolate_compact_quartic(compact)? {
            compact_roots.push((half_angle_sign, root));
        }
    }
    if compact_roots.is_empty() {
        return Ok(Vec::new());
    }
    if compact_roots.len() != 2 {
        return Err(RootIdentityGap::TangentialOrUnresolvedMultiplicity);
    }

    let tau = core::f64::consts::TAU;
    let mut roots = Vec::with_capacity(2);
    for (half_angle_sign, compact) in compact_roots {
        if compact.lo() <= 0.0 || compact.hi() >= 1.0 {
            return Err(RootIdentityGap::ParameterSeamContact);
        }
        let magnitude = compact
            .checked_div(Interval::point(1.0) - compact)
            .ok_or(RootIdentityGap::ArithmeticGuard)?;
        let principal = if half_angle_sign > 0 {
            twice_atan_interval(magnitude)?
        } else {
            twice_atan_interval(-magnitude)? + Interval::point(tau)
        };
        if principal.lo() <= 0.0 || principal.hi() >= tau {
            return Err(RootIdentityGap::ParameterSeamContact);
        }
        roots.push(principal);
    }
    strict_sort(&mut roots)?;
    Ok(roots)
}

fn vectors_are_exactly_parallel(first: Vec3, second: Vec3) -> bool {
    if first == second || first == -second {
        return true;
    }
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        .into_iter()
        .all(|basis| {
            orient3d(first.to_array(), second.to_array(), basis, [0.0; 3]) == Orientation::Zero
        })
}

/// Whether the exact displacement between two authored points is parallel to
/// `axis`, without first rounding that displacement to a [`Vec3`].
fn points_are_exactly_axis_aligned(
    point: Vec3,
    origin: Vec3,
    axis: Vec3,
) -> core::result::Result<bool, RootIdentityGap> {
    // These normals are the three components of `(point - origin) x axis`.
    // `affine_dot3` evaluates each component as an exact expansion directly
    // over the two stored points, avoiding a lossy componentwise subtraction.
    let cross_normals = [
        Vec3::new(0.0, axis.z, -axis.y),
        Vec3::new(-axis.z, 0.0, axis.x),
        Vec3::new(axis.y, -axis.x, 0.0),
    ];
    for normal in cross_normals {
        let component = affine_dot3(normal.to_array(), point.to_array(), origin.to_array(), 0.0)
            .ok_or(RootIdentityGap::ArithmeticGuard)?;
        if component.sign() != Orientation::Zero {
            return Ok(false);
        }
    }
    Ok(true)
}

fn compactified_quartic(
    quartic: [Interval; 5],
    half_angle_sign: i8,
) -> core::result::Result<[Interval; 5], RootIdentityGap> {
    const BINOMIAL: [[u8; 5]; 5] = [
        [1, 0, 0, 0, 0],
        [1, 1, 0, 0, 0],
        [1, 2, 1, 0, 0],
        [1, 3, 3, 1, 0],
        [1, 4, 6, 4, 1],
    ];
    let mut transformed: [Option<Interval>; 5] = [None; 5];
    for (power, coefficient) in quartic.into_iter().enumerate() {
        let signed = if half_angle_sign < 0 && power % 2 == 1 {
            -coefficient
        } else {
            coefficient
        };
        for (residual_power, binomial) in BINOMIAL[4 - power].iter().enumerate().take(5 - power) {
            let scalar = f64::from(*binomial);
            let mut term = signed * Interval::point(scalar);
            if residual_power % 2 == 1 {
                term = -term;
            }
            let output_power = power + residual_power;
            transformed[output_power] = Some(match transformed[output_power] {
                Some(accumulated) => accumulated + term,
                None => term,
            });
        }
    }
    let transformed = transformed.map(|coefficient| coefficient.unwrap_or(Interval::point(0.0)));
    transformed
        .iter()
        .all(|coefficient| finite(*coefficient))
        .then_some(transformed)
        .ok_or(RootIdentityGap::ArithmeticGuard)
}

#[derive(Default)]
struct CompactRootCover {
    certified: Vec<Interval>,
    unresolved: Vec<Interval>,
    visited: usize,
    exhausted: bool,
}

fn isolate_compact_quartic(
    coefficients: [Interval; 5],
) -> core::result::Result<Vec<Interval>, RootIdentityGap> {
    let derivative = quartic_derivative(coefficients);
    let mut cover = CompactRootCover::default();
    classify_compact_interval(
        coefficients,
        derivative,
        Interval::new(0.0, 1.0),
        0,
        &mut cover,
    );
    if cover.exhausted {
        return Err(RootIdentityGap::TangentialOrUnresolvedMultiplicity);
    }

    let mut unresolved = merge_touching_intervals(cover.unresolved);
    for candidate in unresolved.drain(..) {
        let derivative_range = polynomial_value(derivative, candidate);
        let Some(_) = strict_interval_sign(derivative_range) else {
            return Err(RootIdentityGap::TangentialOrUnresolvedMultiplicity);
        };
        let left = strict_interval_sign(polynomial_value(
            coefficients,
            Interval::point(candidate.lo()),
        ));
        let right = strict_interval_sign(polynomial_value(
            coefficients,
            Interval::point(candidate.hi()),
        ));
        match (left, right) {
            (Some(a), Some(b)) if a != b => {
                cover
                    .certified
                    .push(refine_compact_root(coefficients, candidate, a, b));
            }
            (Some(a), Some(b)) if a == b => {}
            _ => return Err(RootIdentityGap::TangentialOrUnresolvedMultiplicity),
        }
    }
    strict_sort(&mut cover.certified)?;
    Ok(cover.certified)
}

fn classify_compact_interval(
    coefficients: [Interval; 5],
    derivative: [Interval; 4],
    domain: Interval,
    depth: usize,
    cover: &mut CompactRootCover,
) {
    if cover.exhausted {
        return;
    }
    if cover.visited == QUARTIC_ROOT_MAX_NODES {
        cover.exhausted = true;
        return;
    }
    cover.visited += 1;
    if excludes_zero(polynomial_value(coefficients, domain)) {
        return;
    }

    let derivative_sign = strict_interval_sign(polynomial_value(derivative, domain));
    let left_sign =
        strict_interval_sign(polynomial_value(coefficients, Interval::point(domain.lo())));
    let right_sign =
        strict_interval_sign(polynomial_value(coefficients, Interval::point(domain.hi())));
    if derivative_sign.is_some()
        && let (Some(left), Some(right)) = (left_sign, right_sign)
    {
        if left != right {
            cover
                .certified
                .push(refine_compact_root(coefficients, domain, left, right));
        }
        return;
    }
    if depth == QUARTIC_ROOT_MAX_DEPTH {
        cover.unresolved.push(domain);
        return;
    }
    let midpoint = 0.5 * domain.lo() + 0.5 * domain.hi();
    if midpoint <= domain.lo() || midpoint >= domain.hi() {
        cover.unresolved.push(domain);
        return;
    }
    classify_compact_interval(
        coefficients,
        derivative,
        Interval::new(domain.lo(), midpoint),
        depth + 1,
        cover,
    );
    classify_compact_interval(
        coefficients,
        derivative,
        Interval::new(midpoint, domain.hi()),
        depth + 1,
        cover,
    );
}

fn refine_compact_root(
    coefficients: [Interval; 5],
    mut bracket: Interval,
    mut left_sign: i8,
    right_sign: i8,
) -> Interval {
    debug_assert_ne!(left_sign, right_sign);
    for _ in 0..80 {
        let midpoint = 0.5 * bracket.lo() + 0.5 * bracket.hi();
        if midpoint <= bracket.lo() || midpoint >= bracket.hi() {
            break;
        }
        match strict_interval_sign(polynomial_value(coefficients, Interval::point(midpoint))) {
            Some(sign) if sign == left_sign => {
                bracket = Interval::new(midpoint, bracket.hi());
                left_sign = sign;
            }
            Some(sign) if sign == right_sign => {
                bracket = Interval::new(bracket.lo(), midpoint);
            }
            _ => break,
        }
    }
    bracket
}

fn merge_touching_intervals(mut intervals: Vec<Interval>) -> Vec<Interval> {
    intervals.sort_by(|a, b| a.lo().total_cmp(&b.lo()).then(a.hi().total_cmp(&b.hi())));
    let mut merged: Vec<Interval> = Vec::with_capacity(intervals.len());
    for interval in intervals {
        if let Some(last) = merged.last_mut()
            && interval.lo() <= last.hi()
        {
            *last = Interval::new(last.lo(), last.hi().max(interval.hi()));
            continue;
        }
        merged.push(interval);
    }
    merged
}

fn quartic_derivative(coefficients: [Interval; 5]) -> [Interval; 4] {
    core::array::from_fn(|power| coefficients[power + 1] * Interval::point((power + 1) as f64))
}

fn polynomial_value<const N: usize>(coefficients: [Interval; N], argument: Interval) -> Interval {
    let mut value = coefficients[N - 1];
    for coefficient in coefficients[..N - 1].iter().rev() {
        value = value * argument + *coefficient;
    }
    value
}

fn strict_interval_sign(value: Interval) -> Option<i8> {
    if value.lo() > 0.0 {
        Some(1)
    } else if value.hi() < 0.0 {
        Some(-1)
    } else {
        None
    }
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

fn interval_cross(a: IntervalVec3, b: IntervalVec3) -> IntervalVec3 {
    IntervalVec3 {
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x,
    }
}

fn interval_determinant(a: IntervalVec3, b: IntervalVec3, c: IntervalVec3) -> Interval {
    interval_dot(a, interval_cross(b, c))
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

fn finite_interval_vec3(value: IntervalVec3) -> bool {
    finite(value.x) && finite(value.y) && finite(value.z)
}

fn valid_active(active: Interval) -> bool {
    finite(active) && active.lo() < active.hi()
}

fn active_interval(lo: f64, hi: f64) -> Option<Interval> {
    (lo.is_finite() && hi.is_finite() && lo < hi).then(|| Interval::new(lo, hi))
}

#[cfg(test)]
mod tests {
    use kcore::operation::{
        AccountingMode, BudgetPlan, LimitSpec, OperationContext, OperationScope, ResourceKind,
        SessionPolicy,
    };
    use kcore::tolerance::{LINEAR_RESOLUTION, Tolerances};
    use kgeom::frame::Frame;
    use kgeom::vec::{Point3, Vec3};
    use ktopo::tolerance::EntityTolerance;

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
    fn canonical_scalar_materialization_retains_the_independent_exact_roots() {
        // For x(t) = -2 + t on x^2 + y^2 = 1, the intrinsic roots are
        // exactly the dyadic scalars 1 and 3. This oracle is independent of
        // the interval midpoint used by materialization.
        let roots = certify_line_cylinder(
            Line::new(Point3::new(-2.0, 0.0, 0.25), Vec3::new(1.0, 0.0, 0.0)).unwrap(),
            Cylinder::new(Frame::world(), 1.0).unwrap(),
            Interval::new(0.0, 4.0),
        )
        .unwrap();
        let mut store = Store::new();
        let body = ktopo::make::block(&mut store, &Frame::world(), [1.0; 3]).unwrap();
        let edge = store.edges_of_body(body).unwrap()[0];
        let opposing_face = store.faces_of_body(body).unwrap()[0];
        let order = CertifiedSourceRootOrder {
            query: SourceRootQuery::new(edge, opposing_face),
            roots,
        };

        for (ordinal, exact) in [1.0, 3.0].into_iter().enumerate() {
            let key = SourceRootKey::new(edge, ordinal);
            let first = order.materialize(key).unwrap();
            let repeated = order.materialize(key).unwrap();
            assert_eq!(first, repeated);
            assert!(first.enclosure().contains(exact));
            assert!(first.enclosure().contains(first.parameter()));
            assert!(first.parameter().is_finite());
        }
        assert!(
            order
                .materialize(SourceRootKey::new(edge, usize::MAX))
                .is_none()
        );
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
    fn circle_cylinder_roots_are_outward_enclosed_in_source_circle_order() {
        for frame in [
            Frame::world(),
            Frame::new(
                Point3::new(2.5, -1.75, 0.625),
                Vec3::new(0.48, 0.64, 0.6),
                Vec3::new(0.8, -0.6, 0.0),
            )
            .unwrap(),
        ] {
            let cylinder =
                Cylinder::new(frame.with_origin(frame.point_at(-0.5, 0.0, -2.0)), 1.0).unwrap();
            let circle =
                Circle::new(frame.with_origin(frame.point_at(0.5, 0.0, -1.0)), 1.0).unwrap();
            let roots = certify_circle_cylinder(circle, cylinder).unwrap();
            assert_eq!(roots.len(), 2);
            assert!(roots[0].contains(2.0 * core::f64::consts::PI / 3.0));
            assert!(roots[1].contains(4.0 * core::f64::consts::PI / 3.0));
            assert!(roots[0].hi() < roots[1].lo());
        }
    }

    #[test]
    fn circle_cylinder_miss_tangent_coincidence_and_nonparallel_fail_closed() {
        let circle = Circle::new(Frame::world(), 1.0).unwrap();
        let cylinder_at = |x, y, radius| {
            Cylinder::new(Frame::world().with_origin(Point3::new(x, y, 0.0)), radius).unwrap()
        };
        assert!(
            certify_circle_cylinder(circle, cylinder_at(3.0, 0.0, 1.0))
                .unwrap()
                .is_empty()
        );
        assert_eq!(
            certify_circle_cylinder(circle, cylinder_at(0.0, 2.0, 1.0)),
            Err(RootIdentityGap::TangentialOrUnresolvedMultiplicity)
        );
        assert_eq!(
            certify_circle_cylinder(circle, cylinder_at(0.0, 0.0, 1.0)),
            Err(RootIdentityGap::CoincidentGeometry)
        );
        assert!(
            certify_circle_cylinder(circle, cylinder_at(0.0, 0.0, 0.5))
                .unwrap()
                .is_empty()
        );
        let nonparallel = Cylinder::new(
            Frame::from_z(Point3::new(0.5, 0.0, 0.0), Vec3::new(0.0, 1.0, 1.0)).unwrap(),
            1.0,
        )
        .unwrap();
        assert_eq!(
            certify_circle_cylinder(circle, nonparallel),
            Err(RootIdentityGap::UnsupportedGeometry)
        );
    }

    #[test]
    fn circle_cylinder_parallel_admission_uses_exact_predicates() {
        let first = Vec3::new(1.0, -2.0, 3.0);
        let parallel = first * 4.0;
        let near_parallel = Vec3::new(4.0, -8.0, 12.0_f64.next_up());
        assert_ne!(first, parallel);
        assert!(vectors_are_exactly_parallel(first, parallel));
        assert!(vectors_are_exactly_parallel(first, -parallel));
        assert!(!vectors_are_exactly_parallel(first, near_parallel));
    }

    #[test]
    fn circle_cylinder_full_quartic_certifies_nonzero_rounded_gram_harmonics() {
        let raw_axis = Vec3::new(1.0, 2.0, 3.0);
        let cylinder_frame = Frame::new(
            Point3::new(3.25, -1.5, 0.75),
            raw_axis,
            Vec3::new(4.0, -1.0, 0.5),
        )
        .unwrap();
        let circle_origin = cylinder_frame.point_at(1.0, 0.0, 0.25);
        let circle_frame =
            Frame::new(circle_origin, raw_axis, Vec3::new(-0.75, 1.25, -0.5)).unwrap();
        assert_eq!(circle_frame.z(), cylinder_frame.z());

        // Express the independently rounded source basis in the actual stored
        // cylinder basis. The binary64 change of basis has a genuine nonzero
        // second harmonic; the quartic must retain and certify it.
        let determinant = cylinder_frame
            .x()
            .dot(cylinder_frame.y().cross(cylinder_frame.z()));
        let radial = |vector: Vec3| {
            [
                vector.dot(cylinder_frame.y().cross(cylinder_frame.z())) / determinant,
                cylinder_frame.x().dot(vector.cross(cylinder_frame.z())) / determinant,
            ]
        };
        let projected_x = radial(circle_frame.x());
        let projected_y = radial(circle_frame.y());
        let second_cosine = projected_x[0] * projected_x[0] + projected_x[1] * projected_x[1]
            - projected_y[0] * projected_y[0]
            - projected_y[1] * projected_y[1];
        let second_sine = projected_x[0] * projected_y[0] + projected_x[1] * projected_y[1];
        assert!(second_cosine != 0.0 || second_sine != 0.0);

        let cylinder = Cylinder::new(cylinder_frame, 1.0).unwrap();
        let circle = Circle::new(circle_frame, 1.0).unwrap();
        let roots = certify_circle_cylinder(circle, cylinder).unwrap();
        assert_eq!(roots.len(), 2);
        assert!(roots[0].hi() < roots[1].lo());
    }

    #[test]
    fn circle_cylinder_one_ulp_near_tangency_fails_closed_in_rounded_frame() {
        let frame = Frame::new(
            Point3::new(-2.0, 4.0, 1.25),
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(4.0, -1.0, 0.5),
        )
        .unwrap();
        let circle = Circle::new(frame, 1.0).unwrap();
        let almost_tangent = Cylinder::new(
            frame.with_origin(frame.point_at(0.0, 2.0_f64.next_down(), 0.0)),
            1.0,
        )
        .unwrap();
        assert_eq!(
            certify_circle_cylinder(circle, almost_tangent),
            Err(RootIdentityGap::TangentialOrUnresolvedMultiplicity)
        );
    }

    #[test]
    fn circle_cylinder_oblique_axial_translation_is_coincident_or_a_miss() {
        let frame = Frame::new(
            Point3::default(),
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(4.0, -1.0, 0.5),
        )
        .unwrap();
        assert!(frame.x().dot(frame.z()) != 0.0 || frame.y().dot(frame.z()) != 0.0);
        let translated = frame.with_origin(frame.point_at(0.0, 0.0, 2.0));
        let cylinder = Cylinder::new(frame, 1.0).unwrap();
        assert_eq!(
            certify_circle_cylinder(Circle::new(translated, 1.0).unwrap(), cylinder),
            Err(RootIdentityGap::CoincidentGeometry)
        );
        assert!(
            certify_circle_cylinder(Circle::new(translated, 0.75).unwrap(), cylinder)
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn circle_cylinder_relative_origin_overflow_fails_before_exact_predicates() {
        let circle = Circle::new(
            Frame::world().with_origin(Point3::new(f64::MAX, 0.0, 0.0)),
            1.0,
        )
        .unwrap();
        let cylinder = Cylinder::new(
            Frame::world().with_origin(Point3::new(-f64::MAX, 0.0, 0.0)),
            1.0,
        )
        .unwrap();
        assert_eq!(
            certify_circle_cylinder(circle, cylinder),
            Err(RootIdentityGap::ArithmeticGuard)
        );
    }

    #[test]
    fn circle_cylinder_large_origins_do_not_round_into_false_axial_coincidence() {
        let cylinder_origin = Point3::new(1.0, 2.0, 3.0);
        let cylinder_frame = Frame::from_z(cylinder_origin, Vec3::new(1.0, 1.0, 1.0)).unwrap();
        let large = (1_u64 << 56) as f64;
        let circle_origin = Point3::new(large, large, large);
        let circle_frame = cylinder_frame.with_origin(circle_origin);

        // Rounded componentwise subtraction loses 1, 2, and 3 respectively,
        // manufacturing a vector parallel to the shared axis. The exact
        // affine point-origin predicate retains those stored-point bits.
        let rounded = circle_origin - cylinder_origin;
        assert_eq!(rounded, Vec3::new(large, large, large));
        assert!(vectors_are_exactly_parallel(rounded, cylinder_frame.z()));
        assert!(
            !points_are_exactly_axis_aligned(circle_origin, cylinder_origin, cylinder_frame.z(),)
                .unwrap()
        );
        let outward = interval_sub(
            interval_point(circle_origin),
            interval_point(cylinder_origin),
        );
        assert!(outward.x.lo() < rounded.x && outward.x.hi() > rounded.x);
        assert!(outward.y.lo() < rounded.y && outward.y.hi() > rounded.y);
        assert!(outward.z.lo() < rounded.z && outward.z.hi() > rounded.z);

        let outcome = certify_circle_cylinder(
            Circle::new(circle_frame, 1.0).unwrap(),
            Cylinder::new(cylinder_frame, 1.0).unwrap(),
        );
        assert_ne!(outcome, Err(RootIdentityGap::CoincidentGeometry));
    }

    #[test]
    fn quartic_cover_precharge_accepts_n_and_refuses_n_minus_one() {
        let run = |allowed| {
            let overrides = BudgetPlan::new([LimitSpec::new(
                SECTION_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            )])
            .unwrap();
            let policy = SessionPolicy::v1();
            let context = OperationContext::new(&policy, Tolerances::default())
                .unwrap()
                .with_family_budget_defaults(BodySectionBudgetProfile::v1_defaults())
                .with_budget_overrides(overrides);
            let mut scope = OperationScope::new(&context);
            charge_circle_cylinder_quartic(&mut scope)
        };
        assert!(run(CIRCLE_CYLINDER_QUARTIC_WORK).is_ok());
        let error = run(CIRCLE_CYLINDER_QUARTIC_WORK - 1).unwrap_err();
        let crossing = error.limit().expect("N-1 must retain limit evidence");
        assert_eq!(crossing.stage, SECTION_WORK);
        assert_eq!(crossing.resource, ResourceKind::Work);
        assert_eq!(crossing.consumed, CIRCLE_CYLINDER_QUARTIC_WORK);
        assert_eq!(crossing.allowed, CIRCLE_CYLINDER_QUARTIC_WORK - 1);
    }

    #[test]
    fn periodic_quartic_refuses_four_distinct_transverse_roots() {
        // `(h^2 - 1)(h^2 - 4)` has the four simple real roots
        // `-2, -1, 1, 2`. Every root is individually certifiable, but a
        // Circle/Cylinder source-root order may publish only the proven
        // zero-or-two exact family.
        assert_eq!(
            certify_periodic_quartic_roots([
                Interval::point(4.0),
                Interval::point(0.0),
                Interval::point(-5.0),
                Interval::point(0.0),
                Interval::point(1.0),
            ]),
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
    fn tolerant_source_edge_is_rejected_before_analytic_family_dispatch() {
        let mut store = Store::new();
        let body = ktopo::make::block(&mut store, &Frame::world(), [2.0; 3]).unwrap();
        let edge = store.edges_of_body(body).unwrap()[0];
        let opposing_face = store.faces_of_body(body).unwrap()[0];
        let imported = EntityTolerance::imported_xt(2.0 * LINEAR_RESOLUTION).unwrap();
        let mut transaction = store.transaction().unwrap();
        transaction.assembly().get_mut(edge).unwrap().tolerance = Some(imported);
        transaction.commit_checked_body(body).unwrap();

        let outcome = with_scope(|scope| {
            RootIdentityAuthority::new()
                .certify_order(&store, SourceRootQuery::new(edge, opposing_face), scope)
                .unwrap()
        });
        assert_eq!(
            outcome,
            RootOrderOutcome::Indeterminate(RootIdentityGap::MalformedSourceEdge)
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
