use super::result::{
    ContactKind, CurveCurveIntersections, CurveCurveOverlap, ParamOrientation,
    accept_curve_curve_candidate,
};
use kcore::error::{Error, Result};
use kcore::tolerance::Tolerances;
use kgeom::curve::Line;
use kgeom::param::ParamRange;

/// Intersect two lines restricted to finite parameter ranges.
///
/// The result distinguishes isolated contacts from positive-length collinear
/// overlap and is canonicalized in the first line's parameter order.
pub fn intersect_bounded_lines(
    a: &Line,
    range_a: ParamRange,
    b: &Line,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    validate_range(range_a)?;
    validate_range(range_b)?;
    let ua = a.dir();
    let ub = b.dir();
    let cross = ua.cross(ub);
    if cross.norm() <= tolerances.angular() {
        return parallel_intersection(a, range_a, b, range_b, tolerances);
    }

    let offset = a.origin() - b.origin();
    let dot = ua.dot(ub);
    let denominator = 1.0 - dot * dot;
    let d = ua.dot(offset);
    let e = ub.dot(offset);
    let mut t_a = (dot * e - d) / denominator;
    let mut t_b = (e - dot * d) / denominator;
    let parameter_tol = tolerances.linear(); // line parameters are arc length
    if t_a < range_a.lo - parameter_tol
        || t_a > range_a.hi + parameter_tol
        || t_b < range_b.lo - parameter_tol
        || t_b > range_b.hi + parameter_tol
    {
        return Ok(CurveCurveIntersections::complete_empty());
    }
    t_a = t_a.clamp(range_a.lo, range_a.hi);
    t_b = t_b.clamp(range_b.lo, range_b.hi);
    let points = accept_curve_curve_candidate(a, t_a, b, t_b, ContactKind::Transverse, tolerances)
        .into_iter()
        .collect();
    CurveCurveIntersections::canonicalized_complete(points, Vec::new())
}

fn parallel_intersection(
    a: &Line,
    range_a: ParamRange,
    b: &Line,
    range_b: ParamRange,
    tolerances: Tolerances,
) -> Result<CurveCurveIntersections> {
    let delta = a.origin() - b.origin();
    let line_distance = delta.cross(b.dir()).norm();
    if line_distance > tolerances.linear() {
        return Ok(CurveCurveIntersections::complete_empty());
    }

    // b(s) = a(t), with s = offset + direction*t for collinear unit lines.
    let direction = a.dir().dot(b.dir());
    let offset = delta.dot(b.dir());
    let mapped_0 = (range_b.lo - offset) / direction;
    let mapped_1 = (range_b.hi - offset) / direction;
    let mapped_lo = mapped_0.min(mapped_1);
    let mapped_hi = mapped_0.max(mapped_1);
    let lo = range_a.lo.max(mapped_lo);
    let hi = range_a.hi.min(mapped_hi);
    if hi < lo - tolerances.linear() {
        return Ok(CurveCurveIntersections::complete_empty());
    }
    if hi - lo <= tolerances.linear() {
        let t_a = ((lo + hi) / 2.0).clamp(range_a.lo, range_a.hi);
        let t_b = (offset + direction * t_a).clamp(range_b.lo, range_b.hi);
        let points = accept_curve_curve_candidate(a, t_a, b, t_b, ContactKind::Tangent, tolerances)
            .into_iter()
            .collect();
        return CurveCurveIntersections::canonicalized_complete(points, Vec::new());
    }

    let s0 = offset + direction * lo;
    let s1 = offset + direction * hi;
    let overlap = CurveCurveOverlap {
        a: ParamRange::new(lo, hi),
        b: ParamRange::new(s0.min(s1), s0.max(s1)),
        orientation: if direction >= 0.0 {
            ParamOrientation::Same
        } else {
            ParamOrientation::Reversed
        },
    };
    CurveCurveIntersections::canonicalized_complete(Vec::new(), vec![overlap])
}

fn validate_range(range: ParamRange) -> Result<()> {
    if !range.is_finite() || range.width() < 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "line intersection requires a finite non-reversed range",
        });
    }
    Ok(())
}
