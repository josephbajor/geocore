use kcore::error::{Error, Result};
use kcore::proof::{Completion, IncompleteEvidence};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::surface::Surface;
use kgeom::vec::Point3;

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
    /// Coincident intervals in deterministic parameter order.
    pub overlaps: Vec<CurveCurveOverlap>,
    completion: Completion,
}

impl Default for CurveCurveIntersections {
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
        mut points: Vec<CurveCurvePoint>,
        mut overlaps: Vec<CurveCurveOverlap>,
        completion: Completion,
    ) -> Result<Self> {
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
        Ok(Self {
            points,
            overlaps,
            completion,
        })
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
        self
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
}

impl SurfaceIntersectionCurve {
    /// Evaluate the branch at its native curve parameter.
    pub fn eval(&self, t: f64) -> Point3 {
        match self {
            SurfaceIntersectionCurve::Line(line) => line.eval(t),
            SurfaceIntersectionCurve::Circle(circle) => circle.eval(t),
            SurfaceIntersectionCurve::Ellipse(ellipse) => ellipse.eval(t),
            SurfaceIntersectionCurve::Nurbs(nurbs) => nurbs.eval(t),
        }
    }

    /// Natural parameter range for the branch geometry.
    pub fn param_range(&self) -> ParamRange {
        match self {
            SurfaceIntersectionCurve::Line(line) => line.param_range(),
            SurfaceIntersectionCurve::Circle(circle) => circle.param_range(),
            SurfaceIntersectionCurve::Ellipse(ellipse) => ellipse.param_range(),
            SurfaceIntersectionCurve::Nurbs(nurbs) => nurbs.param_range(),
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

/// Surface/surface intersection evidence with explicit domain-completion
/// status.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceSurfaceIntersections {
    /// Isolated contacts in deterministic surface-parameter order.
    pub points: Vec<SurfaceSurfacePoint>,
    /// Positive-length intersection branches.
    pub curves: Vec<SurfaceSurfaceCurve>,
    completion: Completion,
    incomplete_evidence: Vec<IncompleteEvidence>,
}

impl Default for SurfaceSurfaceIntersections {
    fn default() -> Self {
        Self {
            points: Vec::new(),
            curves: Vec::new(),
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
        Self::validate_and_sort(&mut points, &mut curves)?;
        Ok(Self {
            points,
            curves,
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

    /// Proven empty result over the complete requested domain.
    pub fn complete_empty() -> Self {
        Self {
            points: Vec::new(),
            curves: Vec::new(),
            completion: Completion::Complete,
            incomplete_evidence: Vec::new(),
        }
    }

    /// Empty partial result with a stable missing-proof diagnostic.
    pub fn indeterminate_empty(reason: &'static str) -> Self {
        Self {
            points: Vec::new(),
            curves: Vec::new(),
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
            completion: Completion::Indeterminate { reason },
            incomplete_evidence,
        }
    }

    fn canonicalized_with_completion(
        mut points: Vec<SurfaceSurfacePoint>,
        mut curves: Vec<SurfaceSurfaceCurve>,
        completion: Completion,
    ) -> Result<Self> {
        Self::validate_and_sort(&mut points, &mut curves)?;
        Ok(Self {
            points,
            curves,
            completion,
            incomplete_evidence: Vec::new(),
        })
    }

    fn validate_and_sort(
        points: &mut [SurfaceSurfacePoint],
        curves: &mut [SurfaceSurfaceCurve],
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
        points.sort_by(|a, b| {
            a.uv_a[0]
                .total_cmp(&b.uv_a[0])
                .then(a.uv_a[1].total_cmp(&b.uv_a[1]))
                .then(a.uv_b[0].total_cmp(&b.uv_b[0]))
                .then(a.uv_b[1].total_cmp(&b.uv_b[1]))
                .then(a.kind.cmp(&b.kind))
        });
        curves.sort_by(|a, b| {
            a.curve_range
                .lo
                .total_cmp(&b.curve_range.lo)
                .then(a.curve_range.hi.total_cmp(&b.curve_range.hi))
                .then(a.uv_a_start[0].total_cmp(&b.uv_a_start[0]))
                .then(a.uv_a_start[1].total_cmp(&b.uv_a_start[1]))
        });
        Ok(())
    }

    /// True when no contacts or branches were discovered. This alone is not
    /// proof of a miss.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty() && self.curves.is_empty()
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

    /// Swap the first and second surface parameter data.
    pub fn swapped(mut self) -> Self {
        for point in &mut self.points {
            core::mem::swap(&mut point.uv_a, &mut point.uv_b);
        }
        for curve in &mut self.curves {
            core::mem::swap(&mut curve.uv_a_start, &mut curve.uv_b_start);
            core::mem::swap(&mut curve.uv_a_end, &mut curve.uv_b_end);
        }
        self
    }
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
