use kcore::error::{Error, Result};
use kcore::math;
use kcore::predicates::{Orientation, orient2d, polygon_orientation2d_iter};
use kcore::proof::{Completion, IncompleteEvidence};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::nurbs::{CurvePairProjectionPlane, CurvePairRootCertificate, NurbsCurve};
use kgeom::param::ParamRange;
use kgeom::surface::Surface;
use kgeom::vec::Point3;
use kgraph::{SkewCylinderBranchCarrier, SkewCylinderSheet};

const MISSING_COMPLETION_REASON: &str =
    "intersection algorithm did not provide complete-domain exclusion evidence";

/// Local character of an isolated curve/curve contact.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[non_exhaustive]
pub enum ContactKind {
    /// Curve tangents are independent at the contact.
    Transverse,
    /// Curves touch without crossing, including overlap endpoints.
    Tangent,
    /// At least one curve is singular at the contact.
    Singular,
}

/// One isolated curve/curve intersection with both parameter values.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveCurvePoint {
    /// Symmetric representative point between the two evaluations.
    pub point: Point3,
    /// Parameter on the first curve.
    pub t_a: f64,
    /// Parameter on the second curve.
    pub t_b: f64,
    /// Distance between the two evaluated points.
    pub residual: f64,
    /// Local contact character.
    pub kind: ContactKind,
}

/// Direction correspondence between coincident parameter intervals.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ParamOrientation {
    /// Low parameter corresponds to low parameter.
    Same,
    /// Low parameter corresponds to high parameter.
    Reversed,
}

/// A positive-length coincident interval between two curves.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveCurveOverlap {
    /// Coincident interval on the first curve.
    pub a: ParamRange,
    /// Coincident interval on the second curve.
    pub b: ParamRange,
    /// Parameter direction correspondence.
    pub orientation: ParamOrientation,
}

/// Curve/curve intersection evidence with explicit domain-completion status.
#[derive(Debug, Clone, PartialEq)]
pub struct CurveCurveIntersections {
    /// Isolated contacts in deterministic parameter order.
    pub points: Vec<CurveCurvePoint>,
    /// Coincident intervals in deterministic parameter order. On a complete
    /// result these extents are certified; on an indeterminate result they are
    /// verified provisional discoveries only.
    pub overlaps: Vec<CurveCurveOverlap>,
    completion: Completion,
    root_certificates: Vec<CurvePairRootCertificate>,
    incomplete_evidence: Vec<IncompleteEvidence>,
}

impl Default for CurveCurveIntersections {
    fn default() -> Self {
        Self {
            points: Vec::new(),
            overlaps: Vec::new(),
            completion: Completion::Indeterminate {
                reason: MISSING_COMPLETION_REASON,
            },
            root_certificates: Vec::new(),
            incomplete_evidence: Vec::new(),
        }
    }
}

impl CurveCurveIntersections {
    /// Validate and sort partial intersection evidence. This conservative
    /// constructor is indeterminate until a solver supplies completion proof.
    pub fn canonicalized(
        points: Vec<CurveCurvePoint>,
        overlaps: Vec<CurveCurveOverlap>,
    ) -> Result<Self> {
        Self::canonicalized_with_completion(
            points,
            overlaps,
            Completion::Indeterminate {
                reason: MISSING_COMPLETION_REASON,
            },
        )
    }

    /// Validate and sort partial evidence with a stable algorithm-specific
    /// explanation for missing completion proof.
    pub fn canonicalized_indeterminate(
        points: Vec<CurveCurvePoint>,
        overlaps: Vec<CurveCurveOverlap>,
        reason: &'static str,
    ) -> Result<Self> {
        Self::canonicalized_with_completion(points, overlaps, Completion::Indeterminate { reason })
    }

    /// Validate and sort verified partial evidence while retaining structured
    /// reasons why complete-domain proof remains unavailable.
    pub fn canonicalized_with_incomplete_evidence(
        points: Vec<CurveCurvePoint>,
        overlaps: Vec<CurveCurveOverlap>,
        reason: &'static str,
        incomplete_evidence: Vec<IncompleteEvidence>,
    ) -> Result<Self> {
        Self::canonicalized_with_proof_evidence(
            points,
            overlaps,
            Completion::Indeterminate { reason },
            Vec::new(),
            incomplete_evidence,
        )
    }

    /// Validate and sort contacts together with exact root certificates and
    /// structured unresolved proof obligations.
    pub fn canonicalized_with_proof_evidence(
        mut points: Vec<CurveCurvePoint>,
        mut overlaps: Vec<CurveCurveOverlap>,
        completion: Completion,
        mut root_certificates: Vec<CurvePairRootCertificate>,
        incomplete_evidence: Vec<IncompleteEvidence>,
    ) -> Result<Self> {
        if completion.is_complete() && !incomplete_evidence.is_empty() {
            return Err(Error::InvalidGeometry {
                reason: "complete curve intersection results cannot retain incomplete evidence",
            });
        }
        Self::validate_and_sort(&mut points, &mut overlaps)?;
        root_certificates.sort_by(compare_root_certificates);
        Ok(Self {
            points,
            overlaps,
            completion,
            root_certificates,
            incomplete_evidence,
        })
    }

    /// Validate and sort evidence whose solver covered the complete requested
    /// domain.
    pub fn canonicalized_complete(
        points: Vec<CurveCurvePoint>,
        overlaps: Vec<CurveCurveOverlap>,
    ) -> Result<Self> {
        Self::canonicalized_with_completion(points, overlaps, Completion::Complete)
    }

    /// Proven empty result over the complete requested domain.
    pub fn complete_empty() -> Self {
        Self {
            points: Vec::new(),
            overlaps: Vec::new(),
            completion: Completion::Complete,
            root_certificates: Vec::new(),
            incomplete_evidence: Vec::new(),
        }
    }

    /// Empty partial result with a stable missing-proof diagnostic.
    pub fn indeterminate_empty(reason: &'static str) -> Self {
        Self {
            points: Vec::new(),
            overlaps: Vec::new(),
            completion: Completion::Indeterminate { reason },
            root_certificates: Vec::new(),
            incomplete_evidence: Vec::new(),
        }
    }

    /// Empty verified partial result with structured missing-proof evidence.
    pub fn indeterminate_empty_with_evidence(
        reason: &'static str,
        incomplete_evidence: Vec<IncompleteEvidence>,
    ) -> Self {
        Self {
            points: Vec::new(),
            overlaps: Vec::new(),
            completion: Completion::Indeterminate { reason },
            root_certificates: Vec::new(),
            incomplete_evidence,
        }
    }

    fn canonicalized_with_completion(
        points: Vec<CurveCurvePoint>,
        overlaps: Vec<CurveCurveOverlap>,
        completion: Completion,
    ) -> Result<Self> {
        Self::canonicalized_with_proof_evidence(
            points,
            overlaps,
            completion,
            Vec::new(),
            Vec::new(),
        )
    }

    fn validate_and_sort(
        points: &mut [CurveCurvePoint],
        overlaps: &mut [CurveCurveOverlap],
    ) -> Result<()> {
        if points.iter().any(|p| {
            !p.point.x.is_finite()
                || !p.point.y.is_finite()
                || !p.point.z.is_finite()
                || !p.t_a.is_finite()
                || !p.t_b.is_finite()
                || !p.residual.is_finite()
                || p.residual < 0.0
        }) {
            return Err(Error::InvalidGeometry {
                reason: "non-finite or negative curve intersection point data",
            });
        }
        if overlaps.iter().any(|o| {
            !o.a.is_finite() || !o.b.is_finite() || o.a.width() <= 0.0 || o.b.width() <= 0.0
        }) {
            return Err(Error::InvalidGeometry {
                reason: "curve overlap ranges must be finite and have positive width",
            });
        }
        points.sort_by(|a, b| {
            a.t_a
                .total_cmp(&b.t_a)
                .then(a.t_b.total_cmp(&b.t_b))
                .then(a.kind.cmp(&b.kind))
        });
        overlaps.sort_by(|a, b| {
            a.a.lo
                .total_cmp(&b.a.lo)
                .then(a.a.hi.total_cmp(&b.a.hi))
                .then(a.b.lo.total_cmp(&b.b.lo))
                .then(a.b.hi.total_cmp(&b.b.hi))
                .then(a.orientation.cmp(&b.orientation))
        });
        Ok(())
    }

    /// True when no contacts or overlaps were discovered. Consult
    /// [`CurveCurveIntersections::is_proven_empty`] before treating this as a
    /// miss.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty() && self.overlaps.is_empty()
    }

    /// Completion evidence for the complete requested parameter domains.
    pub fn completion(&self) -> Completion {
        self.completion
    }

    /// True only when the solver covered the complete requested domains.
    pub fn is_complete(&self) -> bool {
        self.completion.is_complete()
    }

    /// Structured reasons why complete-domain proof remains unavailable.
    ///
    /// Evidence remains in deterministic proof-obligation order and survives
    /// canonicalization and operand swapping. Complete constructors always
    /// return an empty slice.
    pub fn incomplete_evidence(&self) -> &[IncompleteEvidence] {
        &self.incomplete_evidence
    }

    /// Exact unique-root certificates in deterministic parameter-region order.
    pub fn root_certificates(&self) -> &[CurvePairRootCertificate] {
        &self.root_certificates
    }

    /// True only for an empty result backed by complete-domain proof.
    pub fn is_proven_empty(&self) -> bool {
        self.is_complete() && self.is_empty()
    }

    /// Swap the first and second curve parameter data while preserving
    /// completion evidence and canonical first-curve ordering.
    pub fn swapped(mut self) -> Self {
        for point in &mut self.points {
            core::mem::swap(&mut point.t_a, &mut point.t_b);
        }
        for overlap in &mut self.overlaps {
            core::mem::swap(&mut overlap.a, &mut overlap.b);
        }
        for certificate in &mut self.root_certificates {
            *certificate = certificate.swapped();
        }
        self.points.sort_by(|a, b| {
            a.t_a
                .total_cmp(&b.t_a)
                .then(a.t_b.total_cmp(&b.t_b))
                .then(a.kind.cmp(&b.kind))
        });
        self.overlaps.sort_by(|a, b| {
            a.a.lo
                .total_cmp(&b.a.lo)
                .then(a.a.hi.total_cmp(&b.a.hi))
                .then(a.b.lo.total_cmp(&b.b.lo))
                .then(a.b.hi.total_cmp(&b.b.hi))
                .then(a.orientation.cmp(&b.orientation))
        });
        self.root_certificates.sort_by(compare_root_certificates);
        self
    }
}

fn compare_root_certificates(
    a: &CurvePairRootCertificate,
    b: &CurvePairRootCertificate,
) -> core::cmp::Ordering {
    a.first_range()
        .lo
        .total_cmp(&b.first_range().lo)
        .then(a.first_range().hi.total_cmp(&b.first_range().hi))
        .then(a.second_range().lo.total_cmp(&b.second_range().lo))
        .then(a.second_range().hi.total_cmp(&b.second_range().hi))
        .then(projection_rank(a.projection_plane()).cmp(&projection_rank(b.projection_plane())))
        .then(
            a.determinant_lower_bound()
                .total_cmp(&b.determinant_lower_bound()),
        )
}

fn projection_rank(plane: CurvePairProjectionPlane) -> u8 {
    match plane {
        CurvePairProjectionPlane::Xy => 0,
        CurvePairProjectionPlane::Xz => 1,
        CurvePairProjectionPlane::Yz => 2,
        _ => 255,
    }
}

/// Evaluate and tolerance-filter one candidate parameter pair.
pub fn accept_curve_curve_candidate(
    a: &dyn Curve,
    t_a: f64,
    b: &dyn Curve,
    t_b: f64,
    kind: ContactKind,
    tolerances: Tolerances,
) -> Option<CurveCurvePoint> {
    if !t_a.is_finite() || !t_b.is_finite() {
        return None;
    }
    let pa = a.eval(t_a);
    let pb = b.eval(t_b);
    let residual = pa.dist(pb);
    if !residual.is_finite() || residual > tolerances.linear() {
        return None;
    }
    Some(CurveCurvePoint {
        point: (pa + pb) / 2.0,
        t_a,
        t_b,
        residual,
        kind,
    })
}

/// One isolated curve/surface intersection with curve and surface parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveSurfacePoint {
    /// Symmetric representative point between the two evaluations.
    pub point: Point3,
    /// Parameter on the curve.
    pub t_curve: f64,
    /// Parameters on the surface.
    pub uv_surface: [f64; 2],
    /// Distance between the two evaluated points.
    pub residual: f64,
    /// Local contact character.
    pub kind: ContactKind,
}

/// A positive-length curve interval contained in a surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveSurfaceOverlap {
    /// Coincident interval on the curve.
    pub curve: ParamRange,
    /// Surface parameters at `curve.lo`.
    pub uv_start: [f64; 2],
    /// Surface parameters at `curve.hi`.
    pub uv_end: [f64; 2],
}

/// Curve/surface intersection evidence with explicit domain-completion status.
#[derive(Debug, Clone, PartialEq)]
pub struct CurveSurfaceIntersections {
    /// Isolated contacts in deterministic curve-parameter order.
    pub points: Vec<CurveSurfacePoint>,
    /// Positive-length intervals where the curve lies on the surface.
    pub overlaps: Vec<CurveSurfaceOverlap>,
    completion: Completion,
}

impl Default for CurveSurfaceIntersections {
    fn default() -> Self {
        Self {
            points: Vec::new(),
            overlaps: Vec::new(),
            completion: Completion::Indeterminate {
                reason: MISSING_COMPLETION_REASON,
            },
        }
    }
}

impl CurveSurfaceIntersections {
    /// Validate and sort partial intersection evidence. This conservative
    /// constructor is indeterminate until a solver supplies completion proof.
    pub fn canonicalized(
        points: Vec<CurveSurfacePoint>,
        overlaps: Vec<CurveSurfaceOverlap>,
    ) -> Result<Self> {
        Self::canonicalized_with_completion(
            points,
            overlaps,
            Completion::Indeterminate {
                reason: MISSING_COMPLETION_REASON,
            },
        )
    }

    /// Validate and sort partial evidence with a stable algorithm-specific
    /// explanation for missing completion proof.
    pub fn canonicalized_indeterminate(
        points: Vec<CurveSurfacePoint>,
        overlaps: Vec<CurveSurfaceOverlap>,
        reason: &'static str,
    ) -> Result<Self> {
        Self::canonicalized_with_completion(points, overlaps, Completion::Indeterminate { reason })
    }

    /// Validate and sort evidence whose solver covered the complete requested
    /// domain.
    pub fn canonicalized_complete(
        points: Vec<CurveSurfacePoint>,
        overlaps: Vec<CurveSurfaceOverlap>,
    ) -> Result<Self> {
        Self::canonicalized_with_completion(points, overlaps, Completion::Complete)
    }

    /// Proven empty result over the complete requested domain.
    pub fn complete_empty() -> Self {
        Self {
            points: Vec::new(),
            overlaps: Vec::new(),
            completion: Completion::Complete,
        }
    }

    /// Empty partial result with a stable missing-proof diagnostic.
    pub fn indeterminate_empty(reason: &'static str) -> Self {
        Self {
            points: Vec::new(),
            overlaps: Vec::new(),
            completion: Completion::Indeterminate { reason },
        }
    }

    fn canonicalized_with_completion(
        mut points: Vec<CurveSurfacePoint>,
        mut overlaps: Vec<CurveSurfaceOverlap>,
        completion: Completion,
    ) -> Result<Self> {
        if points.iter().any(|p| {
            !p.point.x.is_finite()
                || !p.point.y.is_finite()
                || !p.point.z.is_finite()
                || !p.t_curve.is_finite()
                || !p.uv_surface[0].is_finite()
                || !p.uv_surface[1].is_finite()
                || !p.residual.is_finite()
                || p.residual < 0.0
        }) {
            return Err(Error::InvalidGeometry {
                reason: "non-finite or negative curve/surface intersection point data",
            });
        }
        if overlaps.iter().any(|o| {
            !o.curve.is_finite()
                || o.curve.width() <= 0.0
                || o.uv_start.iter().any(|v| !v.is_finite())
                || o.uv_end.iter().any(|v| !v.is_finite())
        }) {
            return Err(Error::InvalidGeometry {
                reason: "curve/surface overlap data must be finite and have positive width",
            });
        }
        points.sort_by(|a, b| {
            a.t_curve
                .total_cmp(&b.t_curve)
                .then(a.uv_surface[0].total_cmp(&b.uv_surface[0]))
                .then(a.uv_surface[1].total_cmp(&b.uv_surface[1]))
                .then(a.kind.cmp(&b.kind))
        });
        overlaps.sort_by(|a, b| {
            a.curve
                .lo
                .total_cmp(&b.curve.lo)
                .then(a.curve.hi.total_cmp(&b.curve.hi))
                .then(a.uv_start[0].total_cmp(&b.uv_start[0]))
                .then(a.uv_start[1].total_cmp(&b.uv_start[1]))
        });
        Ok(Self {
            points,
            overlaps,
            completion,
        })
    }

    /// True when no contacts or overlaps were discovered. This alone is not
    /// proof of a miss.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty() && self.overlaps.is_empty()
    }

    /// Completion evidence for the complete requested domains.
    pub fn completion(&self) -> Completion {
        self.completion
    }

    /// True only when the solver covered the complete requested domains.
    pub fn is_complete(&self) -> bool {
        self.completion.is_complete()
    }

    /// True only for an empty result backed by complete-domain proof.
    pub fn is_proven_empty(&self) -> bool {
        self.is_complete() && self.is_empty()
    }
}

/// Evaluate and tolerance-filter one curve/surface candidate.
pub fn accept_curve_surface_candidate(
    curve: &dyn Curve,
    t_curve: f64,
    surface: &dyn Surface,
    uv_surface: [f64; 2],
    kind: ContactKind,
    tolerances: Tolerances,
) -> Option<CurveSurfacePoint> {
    if !t_curve.is_finite() || !uv_surface[0].is_finite() || !uv_surface[1].is_finite() {
        return None;
    }
    let pc = curve.eval(t_curve);
    let ps = surface.eval(uv_surface);
    let residual = pc.dist(ps);
    if !residual.is_finite() || residual > tolerances.linear() {
        return None;
    }
    Some(CurveSurfacePoint {
        point: (pc + ps) / 2.0,
        t_curve,
        uv_surface,
        residual,
        kind,
    })
}

/// Curve geometry carrying a surface/surface intersection branch.
// The skew composite carrier stays inline so the established value-carrier
// contract survives branch handoff without indirection.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq)]
pub enum SurfaceIntersectionCurve {
    /// Straight intersection branch.
    Line(Line),
    /// Circular intersection branch.
    Circle(Circle),
    /// Elliptic intersection branch.
    Ellipse(Ellipse),
    /// Exact B-spline/NURBS intersection branch.
    Nurbs(NurbsCurve),
    /// Certified procedural full-cycle sheet of a strict-positive skew pair.
    SkewCylinder(SkewCylinderBranchCarrier),
}

impl SurfaceIntersectionCurve {
    /// Evaluate the branch at its native curve parameter.
    pub fn eval(&self, t: f64) -> Point3 {
        match self {
            SurfaceIntersectionCurve::Line(line) => line.eval(t),
            SurfaceIntersectionCurve::Circle(circle) => circle.eval(t),
            SurfaceIntersectionCurve::Ellipse(ellipse) => ellipse.eval(t),
            SurfaceIntersectionCurve::Nurbs(nurbs) => nurbs.eval(t),
            SurfaceIntersectionCurve::SkewCylinder(carrier) => carrier.eval(t),
        }
    }

    /// Natural parameter range for the branch geometry.
    pub fn param_range(&self) -> ParamRange {
        match self {
            SurfaceIntersectionCurve::Line(line) => line.param_range(),
            SurfaceIntersectionCurve::Circle(circle) => circle.param_range(),
            SurfaceIntersectionCurve::Ellipse(ellipse) => ellipse.param_range(),
            SurfaceIntersectionCurve::Nurbs(nurbs) => nurbs.param_range(),
            SurfaceIntersectionCurve::SkewCylinder(carrier) => carrier.param_range(),
        }
    }
}

/// One isolated surface/surface contact with parameters on both surfaces.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceSurfacePoint {
    /// Symmetric representative point between the two evaluations.
    pub point: Point3,
    /// Parameters on the first surface.
    pub uv_a: [f64; 2],
    /// Parameters on the second surface.
    pub uv_b: [f64; 2],
    /// Distance between the two evaluated points.
    pub residual: f64,
    /// Local contact character.
    pub kind: ContactKind,
}

/// A positive-length surface/surface intersection branch.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceSurfaceCurve {
    /// Curve carrying the intersection branch.
    pub curve: SurfaceIntersectionCurve,
    /// Active parameter interval on `curve`.
    pub curve_range: ParamRange,
    /// First-surface parameters at `curve_range.lo`.
    pub uv_a_start: [f64; 2],
    /// First-surface parameters at `curve_range.hi`.
    pub uv_a_end: [f64; 2],
    /// Second-surface parameters at `curve_range.lo`.
    pub uv_b_start: [f64; 2],
    /// Second-surface parameters at `curve_range.hi`.
    pub uv_b_end: [f64; 2],
    /// Local contact character along the branch.
    pub kind: ContactKind,
}

/// Orientation of the second surface chart relative to the first across a
/// coincident surface region.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SurfaceRegionOrientation {
    /// The two chart boundaries have the same winding.
    Same,
    /// The two chart boundaries have opposite winding.
    Reversed,
}

/// Exact nonlinear chart correspondence for one signed-axis coincident-sphere
/// octant.
///
/// Fields are private so only the sphere/sphere solver can mint evidence after
/// proving exact center/radius equality, signed coordinate-permutation frames,
/// and matching physical octants. The source windows define the complete
/// positive-area region; polygon vertices are only exact physical boundary
/// anchors and are never interpolated to approximate this map.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrthogonalSphereOctantMap {
    first_range: [ParamRange; 2],
    second_range: [ParamRange; 2],
    /// For each second-chart local coordinate, the corresponding first-chart
    /// local coordinate and its exact sign.
    second_from_first_axis: [u8; 3],
    second_from_first_sign: [f64; 3],
}

impl OrthogonalSphereOctantMap {
    pub(crate) const fn new(
        first_range: [ParamRange; 2],
        second_range: [ParamRange; 2],
        second_from_first_axis: [u8; 3],
        second_from_first_sign: [f64; 3],
    ) -> Self {
        Self {
            first_range,
            second_range,
            second_from_first_axis,
            second_from_first_sign,
        }
    }

    /// Complete first-chart octant window.
    pub const fn first_range(self) -> [ParamRange; 2] {
        self.first_range
    }

    /// Complete second-chart octant window.
    pub const fn second_range(self) -> [ParamRange; 2] {
        self.second_range
    }

    /// Evaluate the exact analytic first-to-second sphere-chart map.
    pub fn map_first_to_second(self, uv: [f64; 2]) -> Option<[f64; 2]> {
        map_sphere_octant_parameter(
            self.first_range,
            self.second_range,
            self.second_from_first_axis,
            self.second_from_first_sign,
            uv,
        )
    }

    /// Evaluate the exact analytic second-to-first sphere-chart map.
    pub fn map_second_to_first(self, uv: [f64; 2]) -> Option<[f64; 2]> {
        let (axis, sign) = inverse_signed_axis_permutation(
            self.second_from_first_axis,
            self.second_from_first_sign,
        );
        map_sphere_octant_parameter(self.second_range, self.first_range, axis, sign, uv)
    }

    pub(crate) const fn swapped(self) -> Self {
        let (axis, sign) = inverse_signed_axis_permutation(
            self.second_from_first_axis,
            self.second_from_first_sign,
        );
        Self::new(self.second_range, self.first_range, axis, sign)
    }
}

/// Exact nonlinear chart correspondence for the intersection of two
/// arbitrary-frame coincident-sphere octants.
///
/// The two source windows and both frame-to-frame local-coordinate maps are
/// retained explicitly. Mapping is defined only on the mutual window
/// intersection, so callers never interpolate the curved spherical-polygon
/// boundary anchors.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArbitrarySphereOctantMap {
    first_range: [ParamRange; 2],
    second_range: [ParamRange; 2],
    second_from_first: [[f64; 3]; 3],
    first_from_second: [[f64; 3]; 3],
    parameter_allowance: f64,
}

impl ArbitrarySphereOctantMap {
    pub(crate) const fn new(
        first_range: [ParamRange; 2],
        second_range: [ParamRange; 2],
        second_from_first: [[f64; 3]; 3],
        first_from_second: [[f64; 3]; 3],
        parameter_allowance: f64,
    ) -> Self {
        Self {
            first_range,
            second_range,
            second_from_first,
            first_from_second,
            parameter_allowance,
        }
    }

    /// Complete first-chart octant window before mutual-domain clipping.
    pub const fn first_range(self) -> [ParamRange; 2] {
        self.first_range
    }

    /// Complete second-chart octant window before mutual-domain clipping.
    pub const fn second_range(self) -> [ParamRange; 2] {
        self.second_range
    }

    /// Map a first-chart parameter when its physical point lies in both
    /// retained octant windows.
    pub fn map_first_to_second(self, uv: [f64; 2]) -> Option<[f64; 2]> {
        map_arbitrary_sphere_octant_parameter(
            self.first_range,
            self.second_range,
            self.second_from_first,
            self.parameter_allowance,
            uv,
        )
    }

    /// Map a second-chart parameter when its physical point lies in both
    /// retained octant windows.
    pub fn map_second_to_first(self, uv: [f64; 2]) -> Option<[f64; 2]> {
        map_arbitrary_sphere_octant_parameter(
            self.second_range,
            self.first_range,
            self.first_from_second,
            self.parameter_allowance,
            uv,
        )
    }

    const fn swapped(self) -> Self {
        Self::new(
            self.second_range,
            self.first_range,
            self.first_from_second,
            self.second_from_first,
            self.parameter_allowance,
        )
    }

    fn is_finite(self) -> bool {
        self.first_range
            .into_iter()
            .chain(self.second_range)
            .all(ParamRange::is_finite)
            && self
                .second_from_first
                .into_iter()
                .chain(self.first_from_second)
                .flatten()
                .all(f64::is_finite)
            && self.parameter_allowance.is_finite()
            && self.parameter_allowance >= 0.0
    }

    fn anchors_match(self, first: [f64; 2], second: [f64; 2]) -> bool {
        let Some(mapped_second) = self.map_first_to_second(first) else {
            return false;
        };
        let Some(mapped_first) = self.map_second_to_first(second) else {
            return false;
        };
        mapped_second
            .into_iter()
            .zip(second)
            .chain(mapped_first.into_iter().zip(first))
            .all(|(mapped, stored)| (mapped - stored).abs() <= self.parameter_allowance)
    }
}

/// Certified nonlinear chart correspondence for one exact coincident-sphere
/// overlap between two general, non-octant parameter windows.
///
/// The source rectangles remain the domain authority: boundary anchors record
/// the exact physical arrangement certified by the sphere/sphere solver, while
/// mapping is admitted only where a source point also belongs to the other
/// retained window. This avoids treating curved latitude arcs as polygonal
/// interpolation data.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeneralSphereWindowMap {
    first_range: [ParamRange; 2],
    second_range: [ParamRange; 2],
    second_from_first: [[f64; 3]; 3],
    first_from_second: [[f64; 3]; 3],
    parameter_allowance: f64,
}

impl GeneralSphereWindowMap {
    pub(crate) const fn new(
        first_range: [ParamRange; 2],
        second_range: [ParamRange; 2],
        second_from_first: [[f64; 3]; 3],
        first_from_second: [[f64; 3]; 3],
        parameter_allowance: f64,
    ) -> Self {
        Self {
            first_range,
            second_range,
            second_from_first,
            first_from_second,
            parameter_allowance,
        }
    }

    /// Complete first-chart window before mutual-domain clipping.
    pub const fn first_range(self) -> [ParamRange; 2] {
        self.first_range
    }

    /// Complete second-chart window before mutual-domain clipping.
    pub const fn second_range(self) -> [ParamRange; 2] {
        self.second_range
    }

    /// Map a first-chart parameter when its physical point lies in both
    /// retained windows.
    pub fn map_first_to_second(self, uv: [f64; 2]) -> Option<[f64; 2]> {
        map_arbitrary_sphere_octant_parameter(
            self.first_range,
            self.second_range,
            self.second_from_first,
            self.parameter_allowance,
            uv,
        )
    }

    /// Map a second-chart parameter when its physical point lies in both
    /// retained windows.
    pub fn map_second_to_first(self, uv: [f64; 2]) -> Option<[f64; 2]> {
        map_arbitrary_sphere_octant_parameter(
            self.second_range,
            self.first_range,
            self.first_from_second,
            self.parameter_allowance,
            uv,
        )
    }

    const fn swapped(self) -> Self {
        Self::new(
            self.second_range,
            self.first_range,
            self.first_from_second,
            self.second_from_first,
            self.parameter_allowance,
        )
    }

    fn is_finite(self) -> bool {
        self.first_range
            .into_iter()
            .chain(self.second_range)
            .all(ParamRange::is_finite)
            && self
                .second_from_first
                .into_iter()
                .chain(self.first_from_second)
                .flatten()
                .all(f64::is_finite)
            && self.parameter_allowance.is_finite()
            && self.parameter_allowance >= 0.0
    }

    fn anchors_match(self, first: [f64; 2], second: [f64; 2]) -> bool {
        let Some(mapped_second) = self.map_first_to_second(first) else {
            return false;
        };
        let Some(mapped_first) = self.map_second_to_first(second) else {
            return false;
        };
        mapped_second
            .into_iter()
            .zip(second)
            .chain(mapped_first.into_iter().zip(first))
            .all(|(mapped, stored)| (mapped - stored).abs() <= self.parameter_allowance)
    }
}

const fn inverse_signed_axis_permutation(axis: [u8; 3], sign: [f64; 3]) -> ([u8; 3], [f64; 3]) {
    let mut inverse_axis = [0_u8; 3];
    let mut inverse_sign = [0.0; 3];
    let mut target = 0;
    while target < 3 {
        let source = axis[target] as usize;
        inverse_axis[source] = target as u8;
        inverse_sign[source] = sign[target];
        target += 1;
    }
    (inverse_axis, inverse_sign)
}

/// How the complete paired region interior is represented.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SurfaceRegionCorrespondence {
    /// Both chart boundaries and their interior correspondence are polygonal.
    Polygonal,
    /// Exact analytic nonlinear correspondence over a signed-axis sphere
    /// octant.
    OrthogonalSphereOctant(OrthogonalSphereOctantMap),
    /// Exact analytic correspondence over the convex spherical-polygon
    /// intersection of two arbitrary-frame sphere octants.
    ArbitrarySphereOctant(ArbitrarySphereOctantMap),
    /// Certified analytic correspondence over the single-cycle intersection
    /// of two general arbitrary-frame sphere windows.
    GeneralSphereWindow(GeneralSphereWindowMap),
}

impl SurfaceRegionCorrespondence {
    fn swapped(self) -> Self {
        match self {
            Self::Polygonal => Self::Polygonal,
            Self::OrthogonalSphereOctant(map) => Self::OrthogonalSphereOctant(map.swapped()),
            Self::ArbitrarySphereOctant(map) => Self::ArbitrarySphereOctant(map.swapped()),
            Self::GeneralSphereWindow(map) => Self::GeneralSphereWindow(map.swapped()),
        }
    }
}

/// One paired boundary vertex of a coincident surface region.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceSurfaceRegionVertex {
    /// Symmetric representative point between the two surface evaluations.
    pub point: Point3,
    /// Parameters on the first surface.
    pub uv_a: [f64; 2],
    /// Parameters on the second surface.
    pub uv_b: [f64; 2],
    /// Distance between the two surface evaluations at this vertex.
    pub residual: f64,
}

/// A positive-area coincident region with paired boundary data in both
/// surface charts.
///
/// Polygonal boundaries are canonicalized counter-clockwise in the first
/// chart. Nonlinear correspondences define their own exact domain and anchor
/// contract. The owning solver must supply a `max_residual` proof over the
/// complete region.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceSurfaceRegion {
    /// Deterministic boundary vertices or exact nonlinear-domain anchors, as
    /// specified by `correspondence`.
    pub boundary: Vec<SurfaceSurfaceRegionVertex>,
    /// Winding correspondence between the paired chart boundaries.
    pub orientation: SurfaceRegionOrientation,
    /// Whole-interior paired-chart representation.
    pub correspondence: SurfaceRegionCorrespondence,
    /// Conservative residual bound over the complete region.
    pub max_residual: f64,
}

/// Surface/surface intersection evidence with explicit domain-completion
/// status.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceSurfaceIntersections {
    /// Isolated contacts in deterministic surface-parameter order.
    pub points: Vec<SurfaceSurfacePoint>,
    /// Positive-length intersection branches.
    pub curves: Vec<SurfaceSurfaceCurve>,
    /// Positive-area coincident regions.
    pub regions: Vec<SurfaceSurfaceRegion>,
    completion: Completion,
    incomplete_evidence: Vec<IncompleteEvidence>,
}

impl Default for SurfaceSurfaceIntersections {
    fn default() -> Self {
        Self {
            points: Vec::new(),
            curves: Vec::new(),
            regions: Vec::new(),
            completion: Completion::Indeterminate {
                reason: MISSING_COMPLETION_REASON,
            },
            incomplete_evidence: Vec::new(),
        }
    }
}

impl SurfaceSurfaceIntersections {
    /// Validate and sort partial intersection evidence. This conservative
    /// constructor is indeterminate until a solver supplies completion proof.
    pub fn canonicalized(
        points: Vec<SurfaceSurfacePoint>,
        curves: Vec<SurfaceSurfaceCurve>,
    ) -> Result<Self> {
        Self::canonicalized_with_completion(
            points,
            curves,
            Completion::Indeterminate {
                reason: MISSING_COMPLETION_REASON,
            },
        )
    }

    /// Validate and sort partial evidence with a stable algorithm-specific
    /// explanation for missing completion proof.
    pub fn canonicalized_indeterminate(
        points: Vec<SurfaceSurfacePoint>,
        curves: Vec<SurfaceSurfaceCurve>,
        reason: &'static str,
    ) -> Result<Self> {
        Self::canonicalized_with_completion(points, curves, Completion::Indeterminate { reason })
    }

    /// Validate and sort verified partial evidence while retaining structured
    /// reasons why complete-domain proof remains unavailable.
    pub fn canonicalized_with_incomplete_evidence(
        mut points: Vec<SurfaceSurfacePoint>,
        mut curves: Vec<SurfaceSurfaceCurve>,
        reason: &'static str,
        incomplete_evidence: Vec<IncompleteEvidence>,
    ) -> Result<Self> {
        let mut regions = Vec::new();
        Self::validate_and_sort(&mut points, &mut curves, &mut regions)?;
        Ok(Self {
            points,
            curves,
            regions,
            completion: Completion::Indeterminate { reason },
            incomplete_evidence,
        })
    }

    /// Validate and sort evidence whose solver covered the complete requested
    /// domain.
    pub fn canonicalized_complete(
        points: Vec<SurfaceSurfacePoint>,
        curves: Vec<SurfaceSurfaceCurve>,
    ) -> Result<Self> {
        Self::canonicalized_with_completion(points, curves, Completion::Complete)
    }

    /// Validate and sort contacts, branches, and positive-area regions whose
    /// solver covered the complete requested domain.
    pub fn canonicalized_complete_with_regions(
        mut points: Vec<SurfaceSurfacePoint>,
        mut curves: Vec<SurfaceSurfaceCurve>,
        mut regions: Vec<SurfaceSurfaceRegion>,
    ) -> Result<Self> {
        Self::validate_and_sort(&mut points, &mut curves, &mut regions)?;
        Ok(Self {
            points,
            curves,
            regions,
            completion: Completion::Complete,
            incomplete_evidence: Vec::new(),
        })
    }

    /// Proven empty result over the complete requested domain.
    pub fn complete_empty() -> Self {
        Self {
            points: Vec::new(),
            curves: Vec::new(),
            regions: Vec::new(),
            completion: Completion::Complete,
            incomplete_evidence: Vec::new(),
        }
    }

    /// Empty partial result with a stable missing-proof diagnostic.
    pub fn indeterminate_empty(reason: &'static str) -> Self {
        Self {
            points: Vec::new(),
            curves: Vec::new(),
            regions: Vec::new(),
            completion: Completion::Indeterminate { reason },
            incomplete_evidence: Vec::new(),
        }
    }

    /// Empty verified partial result with structured missing-proof evidence.
    pub fn indeterminate_empty_with_evidence(
        reason: &'static str,
        incomplete_evidence: Vec<IncompleteEvidence>,
    ) -> Self {
        Self {
            points: Vec::new(),
            curves: Vec::new(),
            regions: Vec::new(),
            completion: Completion::Indeterminate { reason },
            incomplete_evidence,
        }
    }

    fn canonicalized_with_completion(
        mut points: Vec<SurfaceSurfacePoint>,
        mut curves: Vec<SurfaceSurfaceCurve>,
        completion: Completion,
    ) -> Result<Self> {
        let mut regions = Vec::new();
        Self::validate_and_sort(&mut points, &mut curves, &mut regions)?;
        Ok(Self {
            points,
            curves,
            regions,
            completion,
            incomplete_evidence: Vec::new(),
        })
    }

    fn validate_and_sort(
        points: &mut [SurfaceSurfacePoint],
        curves: &mut [SurfaceSurfaceCurve],
        regions: &mut [SurfaceSurfaceRegion],
    ) -> Result<()> {
        if points.iter().any(|p| {
            !p.point.x.is_finite()
                || !p.point.y.is_finite()
                || !p.point.z.is_finite()
                || p.uv_a.iter().any(|v| !v.is_finite())
                || p.uv_b.iter().any(|v| !v.is_finite())
                || !p.residual.is_finite()
                || p.residual < 0.0
        }) {
            return Err(Error::InvalidGeometry {
                reason: "non-finite or negative surface/surface point data",
            });
        }
        if curves.iter().any(|c| {
            !c.curve_range.is_finite()
                || c.curve_range.width() <= 0.0
                || c.uv_a_start.iter().any(|v| !v.is_finite())
                || c.uv_a_end.iter().any(|v| !v.is_finite())
                || c.uv_b_start.iter().any(|v| !v.is_finite())
                || c.uv_b_end.iter().any(|v| !v.is_finite())
        }) {
            return Err(Error::InvalidGeometry {
                reason: "surface/surface curve data must be finite and have positive width",
            });
        }
        for region in regions.iter_mut() {
            canonicalize_region(region)?;
        }
        Self::sort_evidence(points, curves, regions);
        Ok(())
    }

    fn sort_evidence(
        points: &mut [SurfaceSurfacePoint],
        curves: &mut [SurfaceSurfaceCurve],
        regions: &mut [SurfaceSurfaceRegion],
    ) {
        points.sort_by(|a, b| {
            a.uv_a[0]
                .total_cmp(&b.uv_a[0])
                .then(a.uv_a[1].total_cmp(&b.uv_a[1]))
                .then(a.uv_b[0].total_cmp(&b.uv_b[0]))
                .then(a.uv_b[1].total_cmp(&b.uv_b[1]))
                .then(a.kind.cmp(&b.kind))
        });
        curves.sort_by(|a, b| {
            surface_curve_family_rank(&a.curve)
                .cmp(&surface_curve_family_rank(&b.curve))
                .then(
                    a.curve_range
                        .lo
                        .total_cmp(&b.curve_range.lo)
                        .then(a.curve_range.hi.total_cmp(&b.curve_range.hi))
                        .then(a.uv_a_start[0].total_cmp(&b.uv_a_start[0]))
                        .then(a.uv_a_start[1].total_cmp(&b.uv_a_start[1])),
                )
        });
        regions.sort_by(compare_regions);
    }

    /// True when no contacts or branches were discovered. This alone is not
    /// proof of a miss.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty() && self.curves.is_empty() && self.regions.is_empty()
    }

    /// Completion evidence for the complete requested surface domains.
    pub fn completion(&self) -> Completion {
        self.completion
    }

    /// True only when the solver covered the complete requested domains.
    pub fn is_complete(&self) -> bool {
        self.completion.is_complete()
    }

    /// Structured reasons why complete-domain proof remains unavailable.
    ///
    /// Legacy indeterminate constructors may return an empty slice until
    /// their owning solver is migrated. Evidence remains in the deterministic
    /// proof-obligation order supplied by the owning solver; canonicalization
    /// and operand swapping preserve that order and retain distinct repeated
    /// observations. A complete result always returns an empty slice because
    /// complete constructors have no evidence parameter.
    pub fn incomplete_evidence(&self) -> &[IncompleteEvidence] {
        &self.incomplete_evidence
    }

    /// True only for an empty result backed by complete-domain proof.
    pub fn is_proven_empty(&self) -> bool {
        self.is_complete() && self.is_empty()
    }

    /// Swap the first and second surface parameter data while restoring
    /// canonical first-surface ordering.
    pub fn swapped(mut self) -> Self {
        for point in &mut self.points {
            core::mem::swap(&mut point.uv_a, &mut point.uv_b);
        }
        for curve in &mut self.curves {
            core::mem::swap(&mut curve.uv_a_start, &mut curve.uv_b_start);
            core::mem::swap(&mut curve.uv_a_end, &mut curve.uv_b_end);
        }
        for region in &mut self.regions {
            for vertex in &mut region.boundary {
                core::mem::swap(&mut vertex.uv_a, &mut vertex.uv_b);
            }
            region.correspondence = region.correspondence.swapped();
            canonicalize_region(region)
                .expect("validated surface region remains valid after operand swap");
        }
        Self::sort_evidence(&mut self.points, &mut self.curves, &mut self.regions);
        self
    }
}

const fn surface_curve_family_rank(curve: &SurfaceIntersectionCurve) -> u8 {
    match curve {
        SurfaceIntersectionCurve::SkewCylinder(carrier) => match carrier.sheet() {
            SkewCylinderSheet::Lower => 0,
            SkewCylinderSheet::Upper => 1,
        },
        SurfaceIntersectionCurve::Line(_)
        | SurfaceIntersectionCurve::Circle(_)
        | SurfaceIntersectionCurve::Ellipse(_)
        | SurfaceIntersectionCurve::Nurbs(_) => 2,
    }
}

fn canonicalize_region(region: &mut SurfaceSurfaceRegion) -> Result<()> {
    if region.boundary.len() < 3
        || !region.max_residual.is_finite()
        || region.max_residual < 0.0
        || region.boundary.iter().any(|vertex| {
            !vertex.point.x.is_finite()
                || !vertex.point.y.is_finite()
                || !vertex.point.z.is_finite()
                || vertex.uv_a.iter().any(|value| !value.is_finite())
                || vertex.uv_b.iter().any(|value| !value.is_finite())
                || !vertex.residual.is_finite()
                || vertex.residual < 0.0
                || vertex.residual > region.max_residual
        })
    {
        return Err(Error::InvalidGeometry {
            reason: "surface/surface region data must be finite, nonnegative, and have at least three bounded vertices",
        });
    }

    if matches!(
        region.correspondence,
        SurfaceRegionCorrespondence::OrthogonalSphereOctant(_)
    ) {
        if region.boundary.len() != 3 || region.orientation != SurfaceRegionOrientation::Same {
            return Err(Error::InvalidGeometry {
                reason: "orthogonal sphere octant regions require three exact boundary anchors and same orientation",
            });
        }
        region.boundary.sort_by(compare_region_physical_vertices);
        return Ok(());
    }

    if let SurfaceRegionCorrespondence::ArbitrarySphereOctant(map) = region.correspondence {
        if !map.is_finite()
            || region.orientation != SurfaceRegionOrientation::Same
            || region
                .boundary
                .iter()
                .any(|vertex| !map.anchors_match(vertex.uv_a, vertex.uv_b))
        {
            return Err(Error::InvalidGeometry {
                reason: "arbitrary sphere octant regions require mutually mapped boundary anchors and same orientation",
            });
        }
        let first = region
            .boundary
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| compare_region_physical_vertices(a, b))
            .map(|(index, _)| index)
            .expect("region has at least three vertices");
        region.boundary.rotate_left(first);
        return Ok(());
    }

    if let SurfaceRegionCorrespondence::GeneralSphereWindow(map) = region.correspondence {
        if !map.is_finite()
            || region.orientation != SurfaceRegionOrientation::Same
            || region
                .boundary
                .iter()
                .any(|vertex| !map.anchors_match(vertex.uv_a, vertex.uv_b))
        {
            return Err(Error::InvalidGeometry {
                reason: "general sphere window regions require mutually mapped certified boundary anchors and same orientation",
            });
        }
        let first = region
            .boundary
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| compare_region_physical_vertices(a, b))
            .map(|(index, _)| index)
            .expect("region has at least three vertices");
        region.boundary.rotate_left(first);
        return Ok(());
    }

    let orientation_a =
        polygon_orientation2d_iter(region.boundary.iter().map(|vertex| vertex.uv_a));
    let orientation_b =
        polygon_orientation2d_iter(region.boundary.iter().map(|vertex| vertex.uv_b));
    if orientation_a == Orientation::Zero || orientation_b == Orientation::Zero {
        return Err(Error::InvalidGeometry {
            reason: "surface/surface region boundaries must have positive area in both charts",
        });
    }
    let expected_orientation = if orientation_a == orientation_b {
        SurfaceRegionOrientation::Same
    } else {
        SurfaceRegionOrientation::Reversed
    };
    if region.orientation != expected_orientation {
        return Err(Error::InvalidGeometry {
            reason: "surface/surface region orientation disagrees with paired chart winding",
        });
    }
    if orientation_a == Orientation::Negative {
        region.boundary.reverse();
    }

    if !is_strictly_convex_in_first_chart(&region.boundary) {
        return Err(Error::InvalidGeometry {
            reason: "surface/surface region boundary must be a nondegenerate convex polygon",
        });
    }
    let first = region
        .boundary
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| compare_region_vertices(a, b))
        .map(|(index, _)| index)
        .expect("region has at least three vertices");
    region.boundary.rotate_left(first);
    Ok(())
}

fn is_strictly_convex_in_first_chart(boundary: &[SurfaceSurfaceRegionVertex]) -> bool {
    boundary.len() >= 3
        && boundary
            .iter()
            .all(|vertex| vertex.uv_a.iter().all(|value| value.is_finite()))
        && (0..boundary.len()).all(|index| {
            let a = boundary[index].uv_a;
            let b = boundary[(index + 1) % boundary.len()].uv_a;
            let c = boundary[(index + 2) % boundary.len()].uv_a;
            orient2d(a, b, c) == Orientation::Positive
        })
}

fn compare_region_vertices(
    a: &SurfaceSurfaceRegionVertex,
    b: &SurfaceSurfaceRegionVertex,
) -> core::cmp::Ordering {
    a.uv_a[0]
        .total_cmp(&b.uv_a[0])
        .then(a.uv_a[1].total_cmp(&b.uv_a[1]))
        .then(a.uv_b[0].total_cmp(&b.uv_b[0]))
        .then(a.uv_b[1].total_cmp(&b.uv_b[1]))
}

fn compare_region_physical_vertices(
    a: &SurfaceSurfaceRegionVertex,
    b: &SurfaceSurfaceRegionVertex,
) -> core::cmp::Ordering {
    a.point
        .x
        .total_cmp(&b.point.x)
        .then(a.point.y.total_cmp(&b.point.y))
        .then(a.point.z.total_cmp(&b.point.z))
        .then_with(|| compare_region_vertices(a, b))
}

fn map_sphere_octant_parameter(
    source_range: [ParamRange; 2],
    target_range: [ParamRange; 2],
    target_from_source_axis: [u8; 3],
    target_from_source_sign: [f64; 3],
    uv: [f64; 2],
) -> Option<[f64; 2]> {
    if !source_range[0].contains(uv[0]) || !source_range[1].contains(uv[1]) {
        return None;
    }
    let (sin_u, cos_u) = math::sincos(uv[0]);
    let (sin_v, cos_v) = math::sincos(uv[1]);
    let source_local = [cos_v * cos_u, cos_v * sin_u, sin_v];
    let target_local: [f64; 3] = core::array::from_fn(|target| {
        source_local[target_from_source_axis[target] as usize] * target_from_source_sign[target]
    });
    let radial = (target_local[0] * target_local[0] + target_local[1] * target_local[1]).sqrt();
    let raw_v = math::atan2(target_local[2], radial);
    let scale = uv[0]
        .abs()
        .max(uv[1].abs())
        .max(target_range[0].lo.abs())
        .max(target_range[0].hi.abs())
        .max(1.0);
    let tolerance = 256.0 * f64::EPSILON * scale;
    let v = fit_closed_scalar(raw_v, target_range[1], tolerance)?;
    let u = if radial <= tolerance {
        target_range[0].lo
    } else {
        fit_closed_periodic(
            math::atan2(target_local[1], target_local[0]),
            target_range[0],
            tolerance,
        )?
    };
    Some([u, v])
}

fn map_arbitrary_sphere_octant_parameter(
    source_range: [ParamRange; 2],
    target_range: [ParamRange; 2],
    target_from_source: [[f64; 3]; 3],
    parameter_allowance: f64,
    uv: [f64; 2],
) -> Option<[f64; 2]> {
    if !source_range[0].contains(uv[0]) || !source_range[1].contains(uv[1]) {
        return None;
    }
    let (sin_u, cos_u) = math::sincos(uv[0]);
    let (sin_v, cos_v) = math::sincos(uv[1]);
    let half_pi = core::f64::consts::FRAC_PI_2;
    let source_local = if uv[1].to_bits() == half_pi.to_bits() {
        [0.0, 0.0, 1.0]
    } else if uv[1].to_bits() == (-half_pi).to_bits() {
        [0.0, 0.0, -1.0]
    } else {
        [cos_v * cos_u, cos_v * sin_u, sin_v]
    };
    let target_local = target_from_source
        .map(|row| row[0] * source_local[0] + row[1] * source_local[1] + row[2] * source_local[2]);
    let radial = (target_local[0] * target_local[0] + target_local[1] * target_local[1]).sqrt();
    let v = fit_closed_scalar(
        math::atan2(target_local[2], radial),
        target_range[1],
        parameter_allowance,
    )?;
    let u = if radial <= parameter_allowance {
        target_range[0].lo
    } else {
        fit_closed_periodic(
            math::atan2(target_local[1], target_local[0]),
            target_range[0],
            parameter_allowance,
        )?
    };
    Some([u, v])
}

fn fit_closed_scalar(value: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    if value < range.lo - tolerance || value > range.hi + tolerance {
        return None;
    }
    Some(value.clamp(range.lo, range.hi))
}

fn fit_closed_periodic(value: f64, range: ParamRange, tolerance: f64) -> Option<f64> {
    let tau = core::f64::consts::TAU;
    let midpoint = range.lo + 0.5 * range.width();
    let shift = ((midpoint - value) / tau).round();
    [shift - 1.0, shift, shift + 1.0]
        .into_iter()
        .filter_map(|shift| fit_closed_scalar(value + shift * tau, range, tolerance))
        .min_by(|first, second| {
            (first - midpoint)
                .abs()
                .total_cmp(&(second - midpoint).abs())
        })
}

fn compare_regions(a: &SurfaceSurfaceRegion, b: &SurfaceSurfaceRegion) -> core::cmp::Ordering {
    a.boundary
        .iter()
        .zip(&b.boundary)
        .map(|(a, b)| compare_region_vertices(a, b))
        .find(|ordering| !ordering.is_eq())
        .unwrap_or_else(|| a.boundary.len().cmp(&b.boundary.len()))
        .then(a.orientation.cmp(&b.orientation))
        .then(a.max_residual.total_cmp(&b.max_residual))
}

/// Evaluate and tolerance-filter one surface/surface point candidate.
pub fn accept_surface_surface_candidate(
    a: &dyn Surface,
    uv_a: [f64; 2],
    b: &dyn Surface,
    uv_b: [f64; 2],
    kind: ContactKind,
    tolerances: Tolerances,
) -> Option<SurfaceSurfacePoint> {
    if uv_a.iter().any(|v| !v.is_finite()) || uv_b.iter().any(|v| !v.is_finite()) {
        return None;
    }
    let pa = a.eval(uv_a);
    let pb = b.eval(uv_b);
    let residual = pa.dist(pb);
    if !residual.is_finite() || residual > tolerances.linear() {
        return None;
    }
    Some(SurfaceSurfacePoint {
        point: (pa + pb) / 2.0,
        uv_a,
        uv_b,
        residual,
        kind,
    })
}
