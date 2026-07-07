//! NURBS tensor-product surfaces (polynomial and rational).

use super::basis::ders_basis_funs;
use super::knots::KnotVector;
use super::ops::{Hpt, insert_knot};
use crate::aabb::Aabb3;
use crate::param::ParamRange;
use crate::surface::{Dir, Surface, SurfaceDerivs};
use crate::vec::{Point3, Vec3};
use kcore::error::{Error, Result};

/// A B-spline tensor-product surface, polynomial (`weights == None`) or
/// rational.
///
/// The control net is stored row-major: entry `(i, j)` — `i` along `u`
/// (`0..nu`), `j` along `v` (`0..nv`) — lives at index `i * nv + j`.
/// Weights, if present, use the same layout and must be positive.
///
/// Degenerate patches (collapsed control-point edges, e.g. a sphere built
/// as a NURBS) evaluate fine but report no [`Surface::degeneracies`] yet;
/// detecting collapsed edges is the topology checker's job in M2.
#[derive(Debug, Clone, PartialEq)]
pub struct NurbsSurface {
    knots_u: KnotVector,
    knots_v: KnotVector,
    points: Vec<Point3>,
    weights: Option<Vec<f64>>,
}

impl NurbsSurface {
    /// Validated construction. Control counts are dictated by the knot
    /// vectors: `points.len()` must equal
    /// `knots_u.control_count() * knots_v.control_count()`.
    pub fn new(
        degree_u: usize,
        degree_v: usize,
        knots_u: Vec<f64>,
        knots_v: Vec<f64>,
        points: Vec<Point3>,
        weights: Option<Vec<f64>>,
    ) -> Result<NurbsSurface> {
        let knots_u = KnotVector::new(degree_u, knots_u)?;
        let knots_v = KnotVector::new(degree_v, knots_v)?;
        let (nu, nv) = (knots_u.control_count(), knots_v.control_count());
        if points.len() != nu * nv {
            return Err(Error::InvalidGeometry {
                reason: "control net size does not match knot vectors",
            });
        }
        if let Some(w) = &weights {
            if w.len() != points.len() {
                return Err(Error::InvalidGeometry {
                    reason: "weight count does not match control net size",
                });
            }
            if w.iter().any(|&wi| !wi.is_finite() || wi <= 0.0) {
                return Err(Error::InvalidGeometry {
                    reason: "weights must be positive and finite",
                });
            }
        }
        Ok(NurbsSurface {
            knots_u,
            knots_v,
            points,
            weights,
        })
    }

    /// Degree in `u`.
    pub fn degree_u(&self) -> usize {
        self.knots_u.degree()
    }

    /// Degree in `v`.
    pub fn degree_v(&self) -> usize {
        self.knots_v.degree()
    }

    /// Knot vector in the given direction.
    pub fn knots(&self, dir: Dir) -> &KnotVector {
        match dir {
            Dir::U => &self.knots_u,
            Dir::V => &self.knots_v,
        }
    }

    /// Control net (row-major, `u` rows by `v` columns).
    pub fn points(&self) -> &[Point3] {
        &self.points
    }

    /// Weights, if rational (same layout as [`NurbsSurface::points`]).
    pub fn weights(&self) -> Option<&[f64]> {
        self.weights.as_deref()
    }

    /// True if the surface carries weights.
    pub fn is_rational(&self) -> bool {
        self.weights.is_some()
    }

    /// Control-net counts `(nu, nv)`.
    pub fn net_size(&self) -> (usize, usize) {
        (self.knots_u.control_count(), self.knots_v.control_count())
    }

    /// Surface with the knot `x` inserted `times` times in direction `dir`
    /// (A5.3, realized by applying the A5.1 alphas along every control
    /// row/column). The point set is unchanged.
    pub fn with_knot_inserted(&self, dir: Dir, x: f64, times: usize) -> Result<NurbsSurface> {
        let (nu, nv) = self.net_size();
        // Work in homogeneous space for rational surfaces.
        let lift = |i: usize| -> Hpt {
            let w = self.weights.as_ref().map_or(1.0, |w| w[i]);
            Hpt::lift(self.points[i], w)
        };
        let (new_knots, columns): (Vec<f64>, Vec<Vec<Hpt>>) = match dir {
            Dir::U => {
                // Each v-column (fixed j) is a curve in u.
                let mut cols = Vec::with_capacity(nv);
                let mut knots_out = None;
                for j in 0..nv {
                    let col: Vec<Hpt> = (0..nu).map(|i| lift(i * nv + j)).collect();
                    let (nk, npts) = insert_knot(&self.knots_u, &col, x, times)?;
                    knots_out.get_or_insert(nk);
                    cols.push(npts);
                }
                (knots_out.expect("nv >= 1"), cols)
            }
            Dir::V => {
                // Each u-row (fixed i) is a curve in v.
                let mut rows = Vec::with_capacity(nu);
                let mut knots_out = None;
                for i in 0..nu {
                    let row: Vec<Hpt> = (0..nv).map(|j| lift(i * nv + j)).collect();
                    let (nk, npts) = insert_knot(&self.knots_v, &row, x, times)?;
                    knots_out.get_or_insert(nk);
                    rows.push(npts);
                }
                (knots_out.expect("nu >= 1"), rows)
            }
        };

        // Reassemble the row-major net.
        let (new_nu, new_nv) = match dir {
            Dir::U => (columns[0].len(), nv),
            Dir::V => (nu, columns[0].len()),
        };
        let mut points = Vec::with_capacity(new_nu * new_nv);
        let mut weights = Vec::with_capacity(new_nu * new_nv);
        for (i, j) in (0..new_nu).flat_map(|i| (0..new_nv).map(move |j| (i, j))) {
            let h = match dir {
                Dir::U => columns[j][i],
                Dir::V => columns[i][j],
            };
            let (p, w) = h.project();
            points.push(p);
            weights.push(w);
        }
        let weights = self.weights.as_ref().map(|_| weights);
        let (ku, kv) = match dir {
            Dir::U => (new_knots, self.knots_v.as_slice().to_vec()),
            Dir::V => (self.knots_u.as_slice().to_vec(), new_knots),
        };
        NurbsSurface::new(self.degree_u(), self.degree_v(), ku, kv, points, weights)
    }

    /// Homogeneous derivative table `(A_kl, w_kl)` for `k + l <= order`,
    /// shared by the polynomial and rational paths (weights default to 1).
    fn homogeneous_derivs(&self, u: f64, v: f64, order: usize) -> ([[Vec3; 3]; 3], [[f64; 3]; 3]) {
        let (p, q) = (self.degree_u(), self.degree_v());
        let su = self.knots_u.find_span(u);
        let sv = self.knots_v.find_span(v);
        let nu = ders_basis_funs(&self.knots_u, su, u, order);
        let nv = ders_basis_funs(&self.knots_v, sv, v, order);
        let (_, nvc) = (self.knots_u.control_count(), self.knots_v.control_count());
        let mut a = [[Vec3::default(); 3]; 3];
        let mut w = [[0.0_f64; 3]; 3];
        for k in 0..=order {
            for l in 0..=(order - k) {
                let mut acc = Vec3::default();
                let mut wacc = 0.0;
                for (sj, &nvl) in nv[l].iter().enumerate() {
                    let mut tmp = Vec3::default();
                    let mut wtmp = 0.0;
                    for (ri, &nuk) in nu[k].iter().enumerate() {
                        let idx = (su - p + ri) * nvc + (sv - q + sj);
                        let wi = self.weights.as_ref().map_or(1.0, |w| w[idx]);
                        tmp += self.points[idx] * (wi * nuk);
                        wtmp += wi * nuk;
                    }
                    acc += tmp * nvl;
                    wacc += wtmp * nvl;
                }
                a[k][l] = acc;
                w[k][l] = wacc;
            }
        }
        (a, w)
    }
}

impl Surface for NurbsSurface {
    fn eval_derivs(&self, uv: [f64; 2], order: usize) -> SurfaceDerivs {
        let order = order.min(2);
        let du = self.knots_u.domain();
        let dv = self.knots_v.domain();
        let u = uv[0].clamp(du.lo, du.hi);
        let v = uv[1].clamp(dv.lo, dv.hi);
        let (a, w) = self.homogeneous_derivs(u, v, order);

        let mut out = SurfaceDerivs::default();
        if self.weights.is_none() {
            // Polynomial (A3.6): weights are all 1, so w[0][0] == 1 and the
            // homogeneous numerators are the derivatives directly.
            out.p = a[0][0];
            if order >= 1 {
                out.du = a[1][0];
                out.dv = a[0][1];
            }
            if order >= 2 {
                out.duu = a[2][0];
                out.duv = a[1][1];
                out.dvv = a[0][2];
            }
            return out;
        }
        // Rational (A4.4), unrolled for order <= 2.
        let w00 = w[0][0];
        let s00 = a[0][0] / w00;
        out.p = s00;
        if order >= 1 {
            out.du = (a[1][0] - s00 * w[1][0]) / w00;
            out.dv = (a[0][1] - s00 * w[0][1]) / w00;
        }
        if order >= 2 {
            out.duu = (a[2][0] - out.du * (2.0 * w[1][0]) - s00 * w[2][0]) / w00;
            out.dvv = (a[0][2] - out.dv * (2.0 * w[0][1]) - s00 * w[0][2]) / w00;
            out.duv = (a[1][1] - out.du * w[0][1] - out.dv * w[1][0] - s00 * w[1][1]) / w00;
        }
        out
    }

    fn param_range(&self) -> [ParamRange; 2] {
        [self.knots_u.domain(), self.knots_v.domain()]
    }

    fn periodicity(&self) -> [Option<f64>; 2] {
        [None, None]
    }

    /// Convex-hull box of the control net (valid for rational surfaces
    /// because all weights are positive); conservative for any sub-range.
    fn bounding_box(&self, range: [ParamRange; 2]) -> Aabb3 {
        debug_assert!(range[0].is_finite() && range[1].is_finite());
        Aabb3::from_points(&self.points)
    }
}

#[cfg(test)]
#[allow(clippy::disallowed_methods)] // tests may cross-check against platform libm
mod tests {
    use super::*;
    use crate::conformance::check_surface;
    use crate::frame::Frame;
    use crate::surface::Cylinder;

    /// A wavy bicubic polynomial patch; interior knots off the conformance
    /// sample stencils.
    fn bicubic() -> NurbsSurface {
        let ku = vec![0.0, 0.0, 0.0, 0.0, 0.35, 0.65, 1.0, 1.0, 1.0, 1.0];
        let kv = vec![0.0, 0.0, 0.0, 0.0, 0.45, 1.0, 1.0, 1.0, 1.0];
        let (nu, nv) = (6, 5);
        let mut pts = Vec::with_capacity(nu * nv);
        for i in 0..nu {
            for j in 0..nv {
                let x = i as f64;
                let y = j as f64;
                let z = ((i * 7 + j * 3) % 5) as f64 * 0.3 - 0.6;
                pts.push(Point3::new(x, y, z));
            }
        }
        NurbsSurface::new(3, 3, ku, kv, pts, None).unwrap()
    }

    /// Exact quarter-cylinder patch (radius 2 about the world z axis):
    /// rational quadratic in `u` (90° arc), quadratic in `v` (three collinear
    /// rows along the axis).
    fn quarter_cylinder() -> NurbsSurface {
        let r = 2.0;
        let w = core::f64::consts::FRAC_1_SQRT_2;
        let arc = [
            (Point3::new(r, 0.0, 0.0), 1.0),
            (Point3::new(r, r, 0.0), w),
            (Point3::new(0.0, r, 0.0), 1.0),
        ];
        let heights = [0.0, 1.5, 3.0];
        let mut pts = Vec::new();
        let mut ws = Vec::new();
        for (p, wi) in arc {
            for h in heights {
                pts.push(Point3::new(p.x, p.y, h));
                ws.push(wi);
            }
        }
        NurbsSurface::new(
            2,
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            pts,
            Some(ws),
        )
        .unwrap()
    }

    fn grid_samples() -> Vec<[f64; 2]> {
        let mut uvs = Vec::new();
        for i in 0..=20 {
            for j in 0..=20 {
                uvs.push([i as f64 / 20.0, j as f64 / 20.0]);
            }
        }
        uvs
    }

    #[test]
    fn conformance_bicubic_polynomial() {
        check_surface(&bicubic());
    }

    #[test]
    fn conformance_rational_quarter_cylinder() {
        check_surface(&quarter_cylinder());
    }

    #[test]
    fn quarter_cylinder_lies_exactly_on_cylinder() {
        let s = quarter_cylinder();
        let cyl = Cylinder::new(Frame::world(), 2.0).unwrap();
        for uv in grid_samples() {
            let p = s.eval(uv);
            let radial = (p.x * p.x + p.y * p.y).sqrt();
            assert!(
                (radial - 2.0).abs() < 1e-12,
                "off cylinder at {uv:?}: r = {radial}"
            );
            // And inside the right z band and quadrant.
            assert!((-1e-12..=3.0 + 1e-12).contains(&p.z));
            assert!(p.x >= -1e-12 && p.y >= -1e-12);
            // Normal agrees with the analytic cylinder's radial direction.
            if let Some(n) = s.normal(uv) {
                let u_angle = f64::atan2(p.y, p.x);
                let n_exact = cyl.normal([u_angle, p.z]).unwrap();
                assert!(
                    (n - n_exact).norm() < 1e-9 || (n + n_exact).norm() < 1e-9,
                    "normal mismatch at {uv:?}"
                );
            }
        }
    }

    #[test]
    fn knot_insertion_preserves_shape_both_directions() {
        for base in [bicubic(), quarter_cylinder()] {
            for dir in [Dir::U, Dir::V] {
                let refined = base.with_knot_inserted(dir, 0.4, 1).unwrap();
                let (bnu, bnv) = base.net_size();
                let (rnu, rnv) = refined.net_size();
                match dir {
                    Dir::U => assert_eq!((rnu, rnv), (bnu + 1, bnv)),
                    Dir::V => assert_eq!((rnu, rnv), (bnu, bnv + 1)),
                }
                for uv in grid_samples() {
                    let (a, b) = (base.eval(uv), refined.eval(uv));
                    assert!(
                        a.dist(b) < 1e-12,
                        "shape changed at {uv:?} ({dir:?}): {a:?} vs {b:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn bounding_box_contains_surface() {
        let s = quarter_cylinder();
        // Inflate by session resolution: evaluated points can exceed the
        // exact convex-hull bound by a few ulps of rounding.
        let bb = s
            .bounding_box(s.param_range())
            .inflated(kcore::tolerance::LINEAR_RESOLUTION);
        for uv in grid_samples() {
            assert!(bb.contains(s.eval(uv)));
        }
    }

    #[test]
    fn validation_errors() {
        let pts = vec![Point3::new(0.0, 0.0, 0.0); 9];
        let k = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        // Net size mismatch.
        assert!(NurbsSurface::new(2, 2, k.clone(), k.clone(), pts[..6].to_vec(), None).is_err());
        // Weight count mismatch.
        assert!(
            NurbsSurface::new(2, 2, k.clone(), k.clone(), pts.clone(), Some(vec![1.0; 8])).is_err()
        );
        // Negative weight.
        let mut w = vec![1.0; 9];
        w[4] = -0.5;
        assert!(NurbsSurface::new(2, 2, k.clone(), k, pts, Some(w)).is_err());
    }
}
