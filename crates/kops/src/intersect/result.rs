use kcore::error::{Error, Result};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Ellipse, Line};
use kgeom::nurbs::NurbsCurve;
use kgeom::param::ParamRange;
use kgeom::surface::Surface;
use kgeom::vec::Point3;

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

/// Complete curve/curve intersection result. Empty is a proven miss;
/// failure to establish an answer is represented by `Err`, not by empty.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CurveCurveIntersections {
    /// Isolated contacts in deterministic parameter order.
    pub points: Vec<CurveCurvePoint>,
    /// Coincident intervals in deterministic parameter order.
    pub overlaps: Vec<CurveCurveOverlap>,
}

impl CurveCurveIntersections {
    /// Validate and sort intersection evidence into canonical parameter order.
    pub fn canonicalized(
        mut points: Vec<CurveCurvePoint>,
        mut overlaps: Vec<CurveCurveOverlap>,
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
        Ok(Self { points, overlaps })
    }

    /// True when the curves were proven not to intersect in the search ranges.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty() && self.overlaps.is_empty()
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

/// Complete curve/surface intersection result. Empty is a proven miss;
/// failure to establish an answer is represented by `Err`, not by empty.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CurveSurfaceIntersections {
    /// Isolated contacts in deterministic curve-parameter order.
    pub points: Vec<CurveSurfacePoint>,
    /// Positive-length intervals where the curve lies on the surface.
    pub overlaps: Vec<CurveSurfaceOverlap>,
}

impl CurveSurfaceIntersections {
    /// Validate and sort intersection evidence into canonical parameter order.
    pub fn canonicalized(
        mut points: Vec<CurveSurfacePoint>,
        mut overlaps: Vec<CurveSurfaceOverlap>,
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
        Ok(Self { points, overlaps })
    }

    /// True when the curve and surface were proven not to intersect.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty() && self.overlaps.is_empty()
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

/// Complete surface/surface intersection result. Empty is a proven miss;
/// failure to establish an answer is represented by `Err`, not by empty.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceSurfaceIntersections {
    /// Isolated contacts in deterministic surface-parameter order.
    pub points: Vec<SurfaceSurfacePoint>,
    /// Positive-length intersection branches.
    pub curves: Vec<SurfaceSurfaceCurve>,
}

impl SurfaceSurfaceIntersections {
    /// Validate and sort intersection evidence into canonical order.
    pub fn canonicalized(
        mut points: Vec<SurfaceSurfacePoint>,
        mut curves: Vec<SurfaceSurfaceCurve>,
    ) -> Result<Self> {
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
        Ok(Self { points, curves })
    }

    /// True when the surfaces were proven not to intersect.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty() && self.curves.is_empty()
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
