use kcore::error::{Error, Result};
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::param::ParamRange;
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
