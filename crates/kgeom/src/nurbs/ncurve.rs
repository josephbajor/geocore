//! NURBS curves (polynomial B-spline and rational).

use super::basis::ders_basis_funs;
use super::knots::KnotVector;
use super::ops::{Hpt, insert_knot, refine_knots};
use crate::aabb::Aabb3;
use crate::curve::{Curve, CurveDerivs};
use crate::param::ParamRange;
use crate::vec::{Point3, Vec3};
use kcore::error::{Error, Result};

/// Binomial coefficients up to order 3 (all the rational derivative
/// formula A4.2 needs at curve order ≤ 3).
const BINOMIAL: [[f64; 4]; 4] = [
    [1.0, 0.0, 0.0, 0.0],
    [1.0, 1.0, 0.0, 0.0],
    [1.0, 2.0, 1.0, 0.0],
    [1.0, 3.0, 3.0, 1.0],
];

/// A B-spline curve, polynomial (`weights == None`) or rational.
///
/// Control points are stored as Euclidean positions; rational curves carry
/// positive weights alongside. Knot operations on rational curves act in
/// homogeneous space, so the point set is preserved exactly.
///
/// Periodic NURBS are deferred to M3 (see module docs);
/// [`Curve::periodicity`] reports `None`.
#[derive(Debug, Clone, PartialEq)]
pub struct NurbsCurve {
    knots: KnotVector,
    points: Vec<Point3>,
    weights: Option<Vec<f64>>,
}

impl NurbsCurve {
    /// Validated construction. `knots.len()` must equal
    /// `points.len() + degree + 1`; weights, if present, must match the
    /// control count and be positive and finite.
    pub fn new(
        degree: usize,
        knots: Vec<f64>,
        points: Vec<Point3>,
        weights: Option<Vec<f64>>,
    ) -> Result<NurbsCurve> {
        let knots = KnotVector::new(degree, knots)?;
        if points.len() != knots.control_count() {
            return Err(Error::InvalidGeometry {
                reason: "control point count does not match knot vector",
            });
        }
        if points
            .iter()
            .any(|point| !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite())
        {
            return Err(Error::InvalidGeometry {
                reason: "curve control points must be finite",
            });
        }
        if let Some(w) = &weights {
            if w.len() != points.len() {
                return Err(Error::InvalidGeometry {
                    reason: "weight count does not match control point count",
                });
            }
            if w.iter().any(|&wi| !wi.is_finite() || wi <= 0.0) {
                return Err(Error::InvalidGeometry {
                    reason: "weights must be positive and finite",
                });
            }
        }
        Ok(NurbsCurve {
            knots,
            points,
            weights,
        })
    }

    /// Degree.
    pub fn degree(&self) -> usize {
        self.knots.degree()
    }

    /// Knot vector.
    pub fn knots(&self) -> &KnotVector {
        &self.knots
    }

    /// Control points (Euclidean).
    pub fn points(&self) -> &[Point3] {
        &self.points
    }

    /// Weights, if rational.
    pub fn weights(&self) -> Option<&[f64]> {
        self.weights.as_deref()
    }

    /// True if the curve carries weights.
    pub fn is_rational(&self) -> bool {
        self.weights.is_some()
    }

    /// Rebuild from refined raw parts, preserving rationality.
    fn from_hpts(degree: usize, knots: Vec<f64>, hpts: Vec<Hpt>) -> Result<NurbsCurve> {
        let (points, weights): (Vec<Point3>, Vec<f64>) = hpts.into_iter().map(Hpt::project).unzip();
        NurbsCurve::new(degree, knots, points, Some(weights))
    }

    /// Apply a homogeneous/Euclidean knot operation and rebuild.
    fn with_op(
        &self,
        op: impl Fn(&KnotVector, &[Hpt]) -> Result<(Vec<f64>, Vec<Hpt>)>,
        op_poly: impl Fn(&KnotVector, &[Vec3]) -> Result<(Vec<f64>, Vec<Vec3>)>,
    ) -> Result<NurbsCurve> {
        match &self.weights {
            Some(w) => {
                let hpts: Vec<Hpt> = self
                    .points
                    .iter()
                    .zip(w)
                    .map(|(&p, &wi)| Hpt::lift(p, wi))
                    .collect();
                let (knots, hpts) = op(&self.knots, &hpts)?;
                Self::from_hpts(self.degree(), knots, hpts)
            }
            None => {
                let (knots, pts) = op_poly(&self.knots, &self.points)?;
                NurbsCurve::new(self.degree(), knots, pts, None)
            }
        }
    }

    /// Curve with `u` inserted `times` times (A5.1). The point set is
    /// unchanged; the control polygon is refined.
    pub fn with_knot_inserted(&self, u: f64, times: usize) -> Result<NurbsCurve> {
        self.with_op(
            |kv, pts| insert_knot(kv, pts, u, times),
            |kv, pts| insert_knot(kv, pts, u, times),
        )
    }

    /// Curve with every value of `xs` inserted once per occurrence
    /// (refinement; see [`super::ops`] for the A5.4 note).
    pub fn with_knots_refined(&self, xs: &[f64]) -> Result<NurbsCurve> {
        let degree = self.degree();
        self.with_op(
            |kv, pts| refine_knots(degree, kv, pts, xs),
            |kv, pts| refine_knots(degree, kv, pts, xs),
        )
    }

    /// Split at `t` (strictly inside the domain) into two curves that
    /// together trace the same point set. Requires a clamped knot vector.
    pub fn split_at(&self, t: f64) -> Result<(NurbsCurve, NurbsCurve)> {
        if !self.knots.is_clamped() {
            return Err(Error::InvalidGeometry {
                reason: "splitting requires a clamped knot vector",
            });
        }
        let domain = self.knots.domain();
        if !(domain.lo < t && t < domain.hi) {
            return Err(Error::InvalidGeometry {
                reason: "split parameter must lie strictly inside the domain",
            });
        }
        let p = self.degree();
        if p == 0 {
            return Err(Error::InvalidGeometry {
                reason: "degree-zero NURBS splitting is unsupported",
            });
        }
        let need = p - self.knots.multiplicity(t);
        let full = if need > 0 {
            self.with_knot_inserted(t, need)?
        } else {
            self.clone()
        };

        let knots = full.knots.as_slice();
        let f = knots
            .iter()
            .position(|&k| k == t)
            .expect("t has full multiplicity after insertion");

        let mut left_knots = knots[..f + p].to_vec();
        left_knots.push(t);
        let mut right_knots = vec![t];
        right_knots.extend_from_slice(&knots[f..]);

        let left_pts = full.points[..f].to_vec();
        let right_pts = full.points[f - 1..].to_vec();
        let (left_w, right_w) = match &full.weights {
            Some(w) => (Some(w[..f].to_vec()), Some(w[f - 1..].to_vec())),
            None => (None, None),
        };
        Ok((
            NurbsCurve::new(p, left_knots, left_pts, left_w)?,
            NurbsCurve::new(p, right_knots, right_pts, right_w)?,
        ))
    }

    /// Clamped subcurve over `range`, preserving the original parameter
    /// values and rational representation.
    pub fn restricted_to(&self, range: ParamRange) -> Result<NurbsCurve> {
        if !self.knots.is_clamped() {
            return Err(Error::InvalidGeometry {
                reason: "restricting a NURBS curve requires a clamped knot vector",
            });
        }
        let domain = self.knots.domain();
        if range.lo < domain.lo || range.hi > domain.hi {
            return Err(Error::InvalidGeometry {
                reason: "NURBS curve restriction lies outside its domain",
            });
        }
        let mut restricted = self.clone();
        if range.lo > domain.lo {
            restricted = restricted.split_at(range.lo)?.1;
        }
        if range.hi < domain.hi {
            restricted = restricted.split_at(range.hi)?.0;
        }
        Ok(restricted)
    }

    fn subrange_control_box(&self, range: ParamRange) -> Aabb3 {
        self.restricted_to(range).map_or_else(
            |_| Aabb3::from_points(&self.points),
            |curve| Aabb3::from_points(&curve.points),
        )
    }

    /// Decompose into Bezier segments (knot refinement to full interior
    /// multiplicity, then span slicing). Each returned curve is a clamped
    /// Bezier over one span of this curve's domain; together they trace the
    /// same point set. Requires a clamped knot vector.
    pub fn to_beziers(&self) -> Result<Vec<NurbsCurve>> {
        if !self.knots.is_clamped() {
            return Err(Error::InvalidGeometry {
                reason: "Bezier extraction requires a clamped knot vector",
            });
        }
        let p = self.degree();
        let domain = self.knots.domain();
        // Raise every distinct interior knot to multiplicity p.
        let knots = self.knots.as_slice();
        let mut xs = Vec::new();
        let mut i = 0;
        while i < knots.len() {
            let k = knots[i];
            let mult = knots[i..].iter().take_while(|&&x| x == k).count();
            if domain.lo < k && k < domain.hi {
                xs.extend(core::iter::repeat_n(k, p - mult));
            }
            i += mult;
        }
        let full = if xs.is_empty() {
            self.clone()
        } else {
            self.with_knots_refined(&xs)?
        };

        let fk = full.knots.as_slice();
        let segments = (full.points.len() - 1) / p;
        let mut out = Vec::with_capacity(segments);
        for s in 0..segments {
            let a = fk[s * p + p];
            let b = fk[s * p + p + 1];
            let mut seg_knots = vec![a; p + 1];
            seg_knots.extend(core::iter::repeat_n(b, p + 1));
            let pts = full.points[s * p..=s * p + p].to_vec();
            let w = full.weights.as_ref().map(|w| w[s * p..=s * p + p].to_vec());
            out.push(NurbsCurve::new(p, seg_knots, pts, w)?);
        }
        Ok(out)
    }

    /// Clamp an evaluation parameter into the domain (out-of-domain
    /// parameters are a caller bug per the trait contract; NaN maps to the
    /// domain start so span search never sees a non-finite parameter).
    fn clamp_param(&self, t: f64) -> f64 {
        self.knots.domain().clamp_param(t)
    }
}

impl Curve for NurbsCurve {
    fn as_any(&self) -> &dyn core::any::Any {
        self
    }

    // Index-based loops mirror the book algorithms (A3.2 / A4.2), where
    // derivative order k is the semantic object, not a slice position.
    #[allow(clippy::needless_range_loop)]
    fn eval_derivs(&self, t: f64, order: usize) -> CurveDerivs {
        let order = order.min(3);
        let t = self.clamp_param(t);
        let p = self.degree();
        let span = self.knots.find_span(t);
        let ders = ders_basis_funs(&self.knots, span, t, order);
        let base = span - p;
        let mut out = CurveDerivs::default();
        match &self.weights {
            // Polynomial: A3.2 CurveDerivsAlg1.
            None => {
                for k in 0..=order {
                    let mut v = Vec3::default();
                    for (j, &nk) in ders[k].iter().enumerate() {
                        v += self.points[base + j] * nk;
                    }
                    out.d[k] = v;
                }
            }
            // Rational: derivatives of the homogeneous curve, then A4.2.
            Some(w) => {
                let mut aders = [Vec3::default(); 4];
                let mut wders = [0.0_f64; 4];
                for k in 0..=order {
                    for (j, &nk) in ders[k].iter().enumerate() {
                        let idx = base + j;
                        let wn = w[idx] * nk;
                        aders[k] += self.points[idx] * wn;
                        wders[k] += wn;
                    }
                }
                for k in 0..=order {
                    let mut v = aders[k];
                    for i in 1..=k {
                        v -= out.d[k - i] * (BINOMIAL[k][i] * wders[i]);
                    }
                    out.d[k] = v / wders[0];
                }
            }
        }
        out
    }

    fn param_range(&self) -> ParamRange {
        self.knots.domain()
    }

    fn periodicity(&self) -> Option<f64> {
        None
    }

    /// Convex-hull box of the exact clamped subcurve control points. Positive
    /// rational weights make the projected curve a convex combination, so
    /// this is conservative and tightens under parameter subdivision.
    fn bounding_box(&self, range: ParamRange) -> Aabb3 {
        debug_assert!(range.is_finite());
        self.subrange_control_box(range)
    }
}

// `Comb` for plain Vec3 lives in ops.rs; nothing else here.

#[cfg(test)]
mod tests {
    use super::*;
    use crate::conformance::check_curve;

    /// A general (non-planar) clamped cubic with interior knots chosen off
    /// the conformance harness's sample stencils.
    fn cubic_polynomial() -> NurbsCurve {
        NurbsCurve::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 0.35, 0.65, 1.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 2.0, 0.5),
                Point3::new(2.5, 2.5, -1.0),
                Point3::new(4.0, 0.5, 0.7),
                Point3::new(5.0, -1.5, 0.0),
                Point3::new(6.0, 0.0, 1.2),
            ],
            None,
        )
        .unwrap()
    }

    fn rational_cubic() -> NurbsCurve {
        NurbsCurve::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 0.35, 0.65, 1.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 2.0, 0.5),
                Point3::new(2.5, 2.5, -1.0),
                Point3::new(4.0, 0.5, 0.7),
                Point3::new(5.0, -1.5, 0.0),
                Point3::new(6.0, 0.0, 1.2),
            ],
            Some(vec![1.0, 0.8, 1.4, 2.0, 0.6, 1.0]),
        )
        .unwrap()
    }

    /// Quarter circle as a rational quadratic Bezier: exact.
    fn quarter_circle() -> NurbsCurve {
        NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            Some(vec![1.0, core::f64::consts::FRAC_1_SQRT_2, 1.0]),
        )
        .unwrap()
    }

    /// Full unit circle as a 4-arc rational quadratic B-spline. Interior
    /// break values are deliberately uneven (0.3, 0.55, 0.81): each Bezier
    /// arc is exact regardless of its knot span, and the values keep the
    /// conformance harness's fixed sample points away from the C¹ breaks.
    fn full_circle() -> NurbsCurve {
        let w = core::f64::consts::FRAC_1_SQRT_2;
        NurbsCurve::new(
            2,
            vec![
                0.0, 0.0, 0.0, 0.3, 0.3, 0.55, 0.55, 0.81, 0.81, 1.0, 1.0, 1.0,
            ],
            vec![
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(-1.0, 1.0, 0.0),
                Point3::new(-1.0, 0.0, 0.0),
                Point3::new(-1.0, -1.0, 0.0),
                Point3::new(0.0, -1.0, 0.0),
                Point3::new(1.0, -1.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ],
            Some(vec![1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0]),
        )
        .unwrap()
    }

    fn assert_same_curve(a: &NurbsCurve, b: &NurbsCurve, range: ParamRange) {
        for i in 0..=100 {
            let t = range.lerp(i as f64 / 100.0);
            let (pa, pb) = (a.eval(t), b.eval(t));
            assert!(
                pa.dist(pb) < 1e-12,
                "curves differ at t = {t}: {pa:?} vs {pb:?}"
            );
        }
    }

    #[test]
    fn conformance_polynomial_cubic() {
        check_curve(&cubic_polynomial());
    }

    #[test]
    fn conformance_rational_cubic() {
        check_curve(&rational_cubic());
    }

    #[test]
    fn conformance_quarter_circle() {
        check_curve(&quarter_circle());
    }

    #[test]
    fn conformance_full_circle() {
        check_curve(&full_circle());
    }

    #[test]
    fn quarter_circle_is_exact() {
        let c = quarter_circle();
        for i in 0..=1000 {
            let t = i as f64 / 1000.0;
            let p = c.eval(t);
            assert!(
                (p.norm() - 1.0).abs() < 1e-12,
                "off circle at t = {t}: |p| = {}",
                p.norm()
            );
            assert!(p.x >= -1e-12 && p.y >= -1e-12, "wrong quadrant at t = {t}");
        }
    }

    #[test]
    fn full_circle_is_exact() {
        let c = full_circle();
        for i in 0..=1000 {
            let t = i as f64 / 1000.0;
            let p = c.eval(t);
            assert!(
                (p.norm() - 1.0).abs() < 1e-12,
                "off circle at t = {t}: |p| = {}",
                p.norm()
            );
        }
        // Closed: both ends at (1, 0, 0).
        assert!(c.eval(0.0).dist(c.eval(1.0)) < 1e-15);
    }

    #[test]
    fn knot_insertion_preserves_shape() {
        for base in [cubic_polynomial(), rational_cubic(), full_circle()] {
            let refined = base.with_knot_inserted(0.4, 2).unwrap();
            assert_eq!(refined.points().len(), base.points().len() + 2);
            assert_eq!(refined.knots().multiplicity(0.4), 2);
            assert_same_curve(&base, &refined, base.param_range());
        }
    }

    #[test]
    fn knot_refinement_preserves_shape() {
        let base = cubic_polynomial();
        let refined = base
            .with_knots_refined(&[0.1, 0.35, 0.5, 0.5, 0.9])
            .unwrap();
        assert_eq!(refined.points().len(), base.points().len() + 5);
        assert_eq!(refined.knots().multiplicity(0.5), 2);
        assert_eq!(refined.knots().multiplicity(0.35), 2);
        assert_same_curve(&base, &refined, base.param_range());
    }

    #[test]
    fn split_preserves_shape_on_both_sides() {
        for base in [cubic_polynomial(), rational_cubic(), full_circle()] {
            let (left, right) = base.split_at(0.4).unwrap();
            assert_eq!(left.param_range(), ParamRange::new(0.0, 0.4));
            assert_eq!(right.param_range(), ParamRange::new(0.4, 1.0));
            assert_same_curve(&base, &left, left.param_range());
            assert_same_curve(&base, &right, right.param_range());
        }
    }

    #[test]
    fn bezier_extraction_preserves_shape() {
        for base in [cubic_polynomial(), full_circle()] {
            let segs = base.to_beziers().unwrap();
            let p = base.degree();
            for seg in &segs {
                assert_eq!(seg.points().len(), p + 1);
                assert!(seg.knots().is_clamped());
                assert_same_curve(&base, seg, seg.param_range());
            }
            // Segments tile the domain.
            assert_eq!(segs.first().unwrap().param_range().lo, 0.0);
            assert_eq!(segs.last().unwrap().param_range().hi, 1.0);
            for pair in segs.windows(2) {
                assert_eq!(pair[0].param_range().hi, pair[1].param_range().lo);
            }
        }
    }

    #[test]
    fn bounding_box_contains_curve() {
        let c = rational_cubic();
        // Inflate by session resolution: evaluated points can exceed the
        // exact convex-hull bound by a few ulps of rounding.
        let bb = c
            .bounding_box(c.param_range())
            .inflated(kcore::tolerance::LINEAR_RESOLUTION);
        for i in 0..=100 {
            let t = i as f64 / 100.0;
            assert!(bb.contains(c.eval(t)));
        }
    }

    #[test]
    fn subrange_bounding_box_uses_restricted_control_hull() {
        let curve = cubic_polynomial();
        let range = ParamRange::new(0.0, 0.1);
        let full = Aabb3::from_points(curve.points());
        let subrange = curve
            .bounding_box(range)
            .inflated(kcore::tolerance::LINEAR_RESOLUTION);
        assert!(subrange.max.x < full.max.x);
        let restricted = curve.restricted_to(range).unwrap();
        assert_eq!(restricted.param_range(), range);
        for index in 0..=100 {
            let parameter = range.lo + range.width() * f64::from(index) / 100.0;
            assert!(subrange.contains(curve.eval(parameter)));
            assert!(restricted.eval(parameter).dist(curve.eval(parameter)) < 1.0e-12);
        }
    }

    #[test]
    fn validation_errors() {
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ];
        // Knot count mismatch.
        assert!(NurbsCurve::new(2, vec![0.0, 0.0, 1.0, 1.0], pts.clone(), None).is_err());
        // Weight count mismatch.
        assert!(
            NurbsCurve::new(
                2,
                vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                pts.clone(),
                Some(vec![1.0, 1.0]),
            )
            .is_err()
        );
        // Non-positive weight.
        assert!(
            NurbsCurve::new(
                2,
                vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                pts.clone(),
                Some(vec![1.0, 0.0, 1.0]),
            )
            .is_err()
        );
        // Decreasing knots.
        assert!(NurbsCurve::new(2, vec![0.0, 0.0, 0.5, 0.2, 1.0, 1.0], pts, None).is_err());
        // Non-finite control point.
        let mut non_finite = vec![Point3::new(0.0, 0.0, 0.0); 3];
        non_finite[1].x = f64::NAN;
        assert!(NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], non_finite, None,).is_err());

        // Insertion beyond degree multiplicity.
        let c = cubic_polynomial();
        assert!(c.with_knot_inserted(0.35, 3).is_err());
        // Insertion at the domain boundary.
        assert!(c.with_knot_inserted(0.0, 1).is_err());
        // Split outside the domain.
        assert!(c.split_at(0.0).is_err());
        assert!(c.split_at(1.5).is_err());
    }

    /// Non-finite parameters are caller bugs, but the defensive clamp must
    /// stay deterministic: NaN pins to the domain start and infinities clamp
    /// to the nearest bound, bit-identically to boundary evaluation.
    #[test]
    fn non_finite_parameters_clamp_deterministically() {
        for curve in [cubic_polynomial(), rational_cubic()] {
            let domain = curve.param_range();
            assert_eq!(curve.eval(f64::NAN), curve.eval(domain.lo));
            assert_eq!(curve.eval(f64::NEG_INFINITY), curve.eval(domain.lo));
            assert_eq!(curve.eval(f64::INFINITY), curve.eval(domain.hi));
            let derivs = curve.eval_derivs(f64::NAN, 2);
            assert_eq!(derivs.d, curve.eval_derivs(domain.lo, 2).d);
        }
    }
}
