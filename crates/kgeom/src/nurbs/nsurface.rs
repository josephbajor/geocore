//! NURBS tensor-product surfaces (polynomial and rational).

use super::basis::ders_basis_funs;
use super::knots::KnotVector;
use super::ops::{Hpt, insert_knot, refine_knots};
use crate::aabb::Aabb3;
use crate::param::{ParamRange, wrap_periodic};
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
    /// A period is present only after the corresponding clamped seam has
    /// passed the explicit position/C1 closure contract.
    periodicity: [Option<f64>; 2],
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
        if points
            .iter()
            .any(|point| !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite())
        {
            return Err(Error::InvalidGeometry {
                reason: "surface control points must be finite",
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
            periodicity: [None, None],
        })
    }

    /// Certify selected clamped directions as periodic and closed.
    ///
    /// This is deliberately narrower than accepting an arbitrary cyclic
    /// B-spline basis: each selected knot vector must be clamped, and the two
    /// endpoint rows and their clamped first-derivative rows must agree. The
    /// polynomial contract uses `linear_tolerance`; rational seams require
    /// exact homogeneous equality so tiny positive weights cannot amplify an
    /// admitted residual. Once certified, evaluation wraps that parameter and
    /// [`Surface::periodicity`] advertises the knot-domain width.
    pub fn with_certified_periodicity(
        mut self,
        periodic: [bool; 2],
        linear_tolerance: f64,
    ) -> Result<NurbsSurface> {
        if !linear_tolerance.is_finite() || linear_tolerance < 0.0 {
            return Err(Error::InvalidGeometry {
                reason: "periodic NURBS seam tolerance must be finite and nonnegative",
            });
        }
        for (axis, dir) in [Dir::U, Dir::V].into_iter().enumerate() {
            if periodic[axis] {
                self.certify_clamped_c1_seam(dir, linear_tolerance)?;
                self.periodicity[axis] = Some(self.knots(dir).domain().width());
            }
        }
        Ok(self)
    }

    fn certify_clamped_c1_seam(&self, dir: Dir, linear_tolerance: f64) -> Result<()> {
        let knots = self.knots(dir);
        if !knots.is_clamped() {
            return Err(Error::InvalidGeometry {
                reason: "periodic NURBS certification requires a clamped knot vector",
            });
        }
        let degree = knots.degree();
        let raw_knots = knots.as_slice();
        let controls = knots.control_count();
        if degree == 0 || controls < 2 {
            return Err(Error::InvalidGeometry {
                reason: "periodic NURBS C1 certification requires positive degree and two controls",
            });
        }
        let start_denominator = raw_knots[degree + 1] - raw_knots[degree];
        let end_denominator = raw_knots[controls] - raw_knots[controls - 1];
        if !(start_denominator > 0.0 && end_denominator > 0.0) {
            return Err(Error::InvalidGeometry {
                reason: "periodic NURBS seam has no positive endpoint derivative span",
            });
        }

        let (nu, nv) = self.net_size();
        let seam_count = match dir {
            Dir::U => nv,
            Dir::V => nu,
        };
        let indices = |seam: usize| match dir {
            Dir::U => (seam, nv + seam, (nu - 2) * nv + seam, (nu - 1) * nv + seam),
            Dir::V => {
                let row = seam * nv;
                (row, row + 1, row + nv - 2, row + nv - 1)
            }
        };
        let weight = |index: usize| self.weights.as_ref().map_or(1.0, |weights| weights[index]);
        let derivative_scale_start = degree as f64 / start_denominator;
        let derivative_scale_end = degree as f64 / end_denominator;
        let rational = self.weights.is_some();

        for seam in 0..seam_count {
            let (start, next, previous, end) = indices(seam);
            let (w0, w1, wp, we) = (weight(start), weight(next), weight(previous), weight(end));
            let weight_scale = w0.abs().max(we.abs()).max(1.0);
            let weight_roundoff = 128.0 * f64::EPSILON * weight_scale;
            let position_closed = if rational {
                w0 == we && self.points[start] == self.points[end]
            } else {
                (w0 - we).abs() <= weight_roundoff
                    && self.points[start].dist(self.points[end]) <= linear_tolerance
            };
            if !position_closed {
                return Err(Error::InvalidGeometry {
                    reason: "periodic NURBS seam does not close in position",
                });
            }

            // Equality of these homogeneous endpoint-derivative controls,
            // together with the boundary controls above, proves equality of
            // the rational first partial over the entire transverse seam.
            let start_dw = (w1 - w0) * derivative_scale_start;
            let end_dw = (we - wp) * derivative_scale_end;
            let start_dh =
                (self.points[next] * w1 - self.points[start] * w0) * derivative_scale_start;
            let end_dh =
                (self.points[end] * we - self.points[previous] * wp) * derivative_scale_end;
            if rational {
                // Exact homogeneous equality makes the rational quotient and
                // its first partial identical without assuming a lower bound
                // on the positive weight function. An approximate residual
                // here could be amplified without bound by tiny weights.
                if start_dw != end_dw || start_dh != end_dh {
                    return Err(Error::InvalidGeometry {
                        reason: "periodic rational NURBS seam is not exactly C1 in homogeneous space",
                    });
                }
                continue;
            }
            let derivative_weight_scale = start_dw.abs().max(end_dw.abs()).max(weight_scale);
            if (start_dw - end_dw).abs() > 256.0 * f64::EPSILON * derivative_weight_scale {
                return Err(Error::InvalidGeometry {
                    reason: "periodic NURBS seam is not C1 in homogeneous weight",
                });
            }
            let magnitude = start_dh.norm().max(end_dh.norm()).max(weight_scale);
            let derivative_tolerance = linear_tolerance * weight_scale / knots.domain().width()
                + 256.0 * f64::EPSILON * magnitude;
            if (start_dh - end_dh).norm() > derivative_tolerance {
                return Err(Error::InvalidGeometry {
                    reason: "periodic NURBS seam is not C1 in position",
                });
            }
        }
        Ok(())
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

    /// Original-source point/first-partial enclosure for a proof rectangle.
    ///
    /// This proof-support API evaluates the retained knot vectors and
    /// homogeneous controls directly. It does not sample or construct a
    /// rounded restricted control net, and either parameter interval may
    /// collapse for constant-coordinate traces.
    pub fn source_differential_enclosure(
        &self,
        range: [ParamRange; 2],
        center: [f64; 2],
    ) -> Option<super::NurbsSurfaceSourceDifferentialEnclosure> {
        super::surface_range_interval::source_differential_enclosure(self, range, center)
    }

    /// Conservative source-span work for one differential enclosure.
    pub fn source_differential_enclosure_work_units(&self) -> Option<usize> {
        super::surface_range_interval::source_differential_enclosure_work_units(self)
    }

    /// Surface with the knot `x` inserted `times` times in direction `dir`
    /// (A5.3, realized by applying the A5.1 alphas along every control
    /// row/column). The point set is unchanged.
    pub fn with_knot_inserted(&self, dir: Dir, x: f64, times: usize) -> Result<NurbsSurface> {
        self.with_directional_op(dir, |knots, points| insert_knot(knots, points, x, times))
    }

    /// Surface with every value of `xs` inserted once per occurrence in
    /// direction `dir`. Rational control nets are refined in homogeneous
    /// space, preserving the represented surface exactly.
    pub fn with_knots_refined(&self, dir: Dir, xs: &[f64]) -> Result<NurbsSurface> {
        if xs.is_empty() {
            return Ok(self.clone());
        }
        let degree = match dir {
            Dir::U => self.degree_u(),
            Dir::V => self.degree_v(),
        };
        self.with_directional_op(dir, |knots, points| refine_knots(degree, knots, points, xs))
    }

    fn with_directional_op(
        &self,
        dir: Dir,
        op: impl Fn(&KnotVector, &[Hpt]) -> Result<(Vec<f64>, Vec<Hpt>)>,
    ) -> Result<NurbsSurface> {
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
                    let (nk, npts) = op(&self.knots_u, &col)?;
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
                    let (nk, npts) = op(&self.knots_v, &row)?;
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
        let mut result =
            NurbsSurface::new(self.degree_u(), self.degree_v(), ku, kv, points, weights)?;
        // Knot insertion/refinement preserves the represented surface and its
        // already-certified seam contract.
        result.periodicity = self.periodicity;
        Ok(result)
    }

    /// Split at `parameter` in direction `dir` into two surfaces clamped in
    /// that direction whose union is the original surface. The parameter
    /// must lie strictly inside that direction's domain.
    pub fn split_at(&self, dir: Dir, parameter: f64) -> Result<(NurbsSurface, NurbsSurface)> {
        let knots = self.knots(dir);
        if !knots.is_clamped() {
            return Err(Error::InvalidGeometry {
                reason: "splitting a NURBS surface requires clamped knot vectors",
            });
        }
        let domain = knots.domain();
        if !(domain.lo < parameter && parameter < domain.hi) {
            return Err(Error::InvalidGeometry {
                reason: "surface split parameter must lie strictly inside the domain",
            });
        }
        let degree = knots.degree();
        let needed = degree - knots.multiplicity(parameter);
        let full = if needed > 0 {
            self.with_knot_inserted(dir, parameter, needed)?
        } else {
            self.clone()
        };
        let split_knots = full.knots(dir).as_slice();
        let split = split_knots
            .iter()
            .position(|&knot| knot == parameter)
            .expect("split knot has full multiplicity after insertion");

        let mut left_knots = split_knots[..split + degree].to_vec();
        left_knots.push(parameter);
        let mut right_knots = vec![parameter];
        right_knots.extend_from_slice(&split_knots[split..]);

        let (nu, nv) = full.net_size();
        match dir {
            Dir::U => {
                let cut = split * nv;
                let overlap = (split - 1) * nv;
                let left_points = full.points[..cut].to_vec();
                let right_points = full.points[overlap..].to_vec();
                let (left_weights, right_weights) = match &full.weights {
                    Some(weights) => (
                        Some(weights[..cut].to_vec()),
                        Some(weights[overlap..].to_vec()),
                    ),
                    None => (None, None),
                };
                let knots_v = full.knots_v.as_slice().to_vec();
                let mut left = NurbsSurface::new(
                    full.degree_u(),
                    full.degree_v(),
                    left_knots,
                    knots_v.clone(),
                    left_points,
                    left_weights,
                )?;
                let mut right = NurbsSurface::new(
                    full.degree_u(),
                    full.degree_v(),
                    right_knots,
                    knots_v,
                    right_points,
                    right_weights,
                )?;
                left.periodicity[1] = full.periodicity[1];
                right.periodicity[1] = full.periodicity[1];
                Ok((left, right))
            }
            Dir::V => {
                let left_points = slice_columns(&full.points, nu, nv, 0, split);
                let right_points = slice_columns(&full.points, nu, nv, split - 1, nv);
                let (left_weights, right_weights) = match &full.weights {
                    Some(weights) => (
                        Some(slice_columns(weights, nu, nv, 0, split)),
                        Some(slice_columns(weights, nu, nv, split - 1, nv)),
                    ),
                    None => (None, None),
                };
                let knots_u = full.knots_u.as_slice().to_vec();
                let mut left = NurbsSurface::new(
                    full.degree_u(),
                    full.degree_v(),
                    knots_u.clone(),
                    left_knots,
                    left_points,
                    left_weights,
                )?;
                let mut right = NurbsSurface::new(
                    full.degree_u(),
                    full.degree_v(),
                    knots_u,
                    right_knots,
                    right_points,
                    right_weights,
                )?;
                left.periodicity[0] = full.periodicity[0];
                right.periodicity[0] = full.periodicity[0];
                Ok((left, right))
            }
        }
    }

    /// Exact clamped sub-surface over the positive-area parameter rectangle
    /// `range`, preserving the original parameter values and rational form.
    pub fn restricted_to(&self, range: [ParamRange; 2]) -> Result<NurbsSurface> {
        if !self.knots_u.is_clamped() || !self.knots_v.is_clamped() {
            return Err(Error::InvalidGeometry {
                reason: "restricting a NURBS surface requires clamped knot vectors",
            });
        }
        let domain = [self.knots_u.domain(), self.knots_v.domain()];
        for axis in 0..2 {
            if !range[axis].is_finite()
                || range[axis].width() <= 0.0
                || range[axis].lo < domain[axis].lo
                || range[axis].hi > domain[axis].hi
            {
                return Err(Error::InvalidGeometry {
                    reason: "surface restriction must be a positive-area rectangle inside the domain",
                });
            }
        }

        let mut restricted = self.clone();
        for (axis, dir) in [Dir::U, Dir::V].into_iter().enumerate() {
            if range[axis].lo > domain[axis].lo {
                restricted = restricted.split_at(dir, range[axis].lo)?.1;
            }
            if range[axis].hi < domain[axis].hi {
                restricted = restricted.split_at(dir, range[axis].hi)?.0;
            }
        }
        Ok(restricted)
    }

    /// Decompose a clamped surface into tensor-product Bezier patches in
    /// deterministic `u`-major, then `v`-major order. Each patch retains its
    /// source parameter rectangle and the patches cover the surface exactly.
    pub fn to_bezier_patches(&self) -> Result<Vec<NurbsSurface>> {
        if !self.knots_u.is_clamped() || !self.knots_v.is_clamped() {
            return Err(Error::InvalidGeometry {
                reason: "Bezier patch extraction requires clamped knot vectors",
            });
        }
        let refinement_u = refinement_knots(&self.knots_u);
        let refinement_v = refinement_knots(&self.knots_v);
        let refined_u = self.with_knots_refined(Dir::U, &refinement_u)?;
        let full = refined_u.with_knots_refined(Dir::V, &refinement_v)?;

        let (degree_u, degree_v) = (full.degree_u(), full.degree_v());
        let (nu, nv) = full.net_size();
        let count_u = (nu - 1) / degree_u;
        let count_v = (nv - 1) / degree_v;
        debug_assert_eq!((nu - 1) % degree_u, 0);
        debug_assert_eq!((nv - 1) % degree_v, 0);

        let mut patches = Vec::with_capacity(count_u * count_v);
        for patch_u in 0..count_u {
            let u0 = full.knots_u.as_slice()[patch_u * degree_u + degree_u];
            let u1 = full.knots_u.as_slice()[patch_u * degree_u + degree_u + 1];
            let mut knots_u = vec![u0; degree_u + 1];
            knots_u.extend(core::iter::repeat_n(u1, degree_u + 1));
            for patch_v in 0..count_v {
                let v0 = full.knots_v.as_slice()[patch_v * degree_v + degree_v];
                let v1 = full.knots_v.as_slice()[patch_v * degree_v + degree_v + 1];
                let mut knots_v = vec![v0; degree_v + 1];
                knots_v.extend(core::iter::repeat_n(v1, degree_v + 1));

                let mut points = Vec::with_capacity((degree_u + 1) * (degree_v + 1));
                let mut weights = full
                    .weights
                    .as_ref()
                    .map(|_| Vec::with_capacity((degree_u + 1) * (degree_v + 1)));
                for local_u in 0..=degree_u {
                    for local_v in 0..=degree_v {
                        let index =
                            (patch_u * degree_u + local_u) * nv + patch_v * degree_v + local_v;
                        points.push(full.points[index]);
                        if let (Some(source), Some(target)) = (&full.weights, &mut weights) {
                            target.push(source[index]);
                        }
                    }
                }
                patches.push(NurbsSurface::new(
                    degree_u,
                    degree_v,
                    knots_u.clone(),
                    knots_v,
                    points,
                    weights,
                )?);
            }
        }
        Ok(patches)
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
    fn as_any(&self) -> &dyn core::any::Any {
        self
    }

    fn eval_derivs(&self, uv: [f64; 2], order: usize) -> SurfaceDerivs {
        let order = order.min(2);
        let du = self.knots_u.domain();
        let dv = self.knots_v.domain();
        let u = self.periodicity[0].map_or_else(
            || uv[0].clamp(du.lo, du.hi),
            |period| wrap_periodic(uv[0], du.lo, period),
        );
        let v = self.periodicity[1].map_or_else(
            || uv[1].clamp(dv.lo, dv.hi),
            |period| wrap_periodic(uv[1], dv.lo, period),
        );
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
        self.periodicity
    }

    /// Outward interval enclosure evaluated directly from this source surface.
    /// Rounded knot insertion and restricted control nets never participate in
    /// exclusion bounds.
    fn bounding_box(&self, range: [ParamRange; 2]) -> Aabb3 {
        debug_assert!(range[0].is_finite() && range[1].is_finite());
        let domain = self.param_range();
        let u_ranges = periodic_range_parts(range[0], domain[0], self.periodicity[0]);
        let v_ranges = periodic_range_parts(range[1], domain[1], self.periodicity[1]);
        let mut bounds = Aabb3::empty();
        for u in &u_ranges {
            for v in &v_ranges {
                bounds = bounds.union(super::surface_range_interval::position_range_aabb(
                    self,
                    [*u, *v],
                ));
            }
        }
        bounds
    }
}

fn periodic_range_parts(
    requested: ParamRange,
    domain: ParamRange,
    period: Option<f64>,
) -> Vec<ParamRange> {
    let Some(period) = period else {
        return vec![requested];
    };
    if requested.width() >= period {
        return vec![domain];
    }
    let start = wrap_periodic(requested.lo, domain.lo, period);
    let end = start + requested.width();
    if end <= domain.hi {
        vec![ParamRange::new(start, end)]
    } else {
        vec![
            ParamRange::new(start, domain.hi),
            ParamRange::new(domain.lo, domain.lo + (end - domain.hi)),
        ]
    }
}

fn slice_columns<T: Copy>(
    values: &[T],
    rows: usize,
    columns: usize,
    start: usize,
    end: usize,
) -> Vec<T> {
    let mut sliced = Vec::with_capacity(rows * (end - start));
    for row in 0..rows {
        sliced.extend_from_slice(&values[row * columns + start..row * columns + end]);
    }
    sliced
}

fn refinement_knots(knots: &KnotVector) -> Vec<f64> {
    let degree = knots.degree();
    let domain = knots.domain();
    let values = knots.as_slice();
    let mut refinement = Vec::new();
    let mut index = 0;
    while index < values.len() {
        let value = values[index];
        let multiplicity = values[index..]
            .iter()
            .take_while(|&&candidate| candidate == value)
            .count();
        if domain.lo < value && value < domain.hi {
            refinement.extend(core::iter::repeat_n(value, degree - multiplicity));
        }
        index += multiplicity;
    }
    refinement
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

    fn rational_bicubic() -> NurbsSurface {
        let polynomial = bicubic();
        let weights = (0..polynomial.points.len())
            .map(|index| 0.75 + 0.125 * f64::from((index % 7) as u32))
            .collect();
        NurbsSurface::new(
            polynomial.degree_u(),
            polynomial.degree_v(),
            polynomial.knots_u.as_slice().to_vec(),
            polynomial.knots_v.as_slice().to_vec(),
            polynomial.points,
            Some(weights),
        )
        .unwrap()
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

    /// Clamped cubic loop extruded along `v`. The rational form uses unequal
    /// interior weights while matching both homogeneous seam derivative rows.
    fn periodic_loop(rational: bool) -> NurbsSurface {
        let weights_u = [1.0, 0.75, 1.25, 1.0];
        let controls_u = if rational {
            [
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(2.0, 1.0, 0.0),
                Point3::new(0.4, -0.6, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ]
        } else {
            [
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(2.0, 1.0, 0.0),
                Point3::new(0.0, -1.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ]
        };
        let mut points = Vec::new();
        let mut weights = Vec::new();
        for (control, weight) in controls_u.into_iter().zip(weights_u) {
            for z in [0.0, 2.0] {
                points.push(Point3::new(control.x, control.y, z));
                weights.push(weight);
            }
        }
        NurbsSurface::new(
            3,
            1,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            points,
            rational.then_some(weights),
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

    fn assert_patch_boundary_bitwise(a: &NurbsSurface, b: &NurbsSurface, dir: Dir) {
        let (nu_a, nv_a) = a.net_size();
        let (nu_b, nv_b) = b.net_size();
        let index_pairs: Vec<_> = match dir {
            Dir::U => {
                assert_eq!(nv_a, nv_b);
                (0..nv_a).map(|v| ((nu_a - 1) * nv_a + v, v)).collect()
            }
            Dir::V => {
                assert_eq!(nu_a, nu_b);
                (0..nu_a).map(|u| (u * nv_a + nv_a - 1, u * nv_b)).collect()
            }
        };
        for (index_a, index_b) in index_pairs {
            let (point_a, point_b) = (a.points[index_a], b.points[index_b]);
            assert_eq!(point_a.x.to_bits(), point_b.x.to_bits());
            assert_eq!(point_a.y.to_bits(), point_b.y.to_bits());
            assert_eq!(point_a.z.to_bits(), point_b.z.to_bits());
            match (&a.weights, &b.weights) {
                (Some(weights_a), Some(weights_b)) => {
                    assert_eq!(weights_a[index_a].to_bits(), weights_b[index_b].to_bits());
                }
                (None, None) => {}
                _ => panic!("adjacent patches must preserve rational form"),
            }
        }
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
        for base in [bicubic(), rational_bicubic(), quarter_cylinder()] {
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
    fn split_and_restriction_preserve_polynomial_and_rational_surfaces() {
        for base in [bicubic(), rational_bicubic(), quarter_cylinder()] {
            for dir in [Dir::U, Dir::V] {
                let (left, right) = base.split_at(dir, 0.4).unwrap();
                let axis = match dir {
                    Dir::U => 0,
                    Dir::V => 1,
                };
                assert_eq!(left.param_range()[axis], ParamRange::new(0.0, 0.4));
                assert_eq!(right.param_range()[axis], ParamRange::new(0.4, 1.0));
                for uv in grid_samples() {
                    if uv[axis] <= 0.4 {
                        assert!(left.eval(uv).dist(base.eval(uv)) < 1.0e-11);
                    }
                    if uv[axis] >= 0.4 {
                        assert!(right.eval(uv).dist(base.eval(uv)) < 1.0e-11);
                    }
                }
            }

            let range = [ParamRange::new(0.2, 0.8), ParamRange::new(0.1, 0.7)];
            let restricted = base.restricted_to(range).unwrap();
            assert_eq!(restricted.param_range(), range);
            assert_eq!(restricted.is_rational(), base.is_rational());
            for i in 0..=20 {
                for j in 0..=20 {
                    let uv = [
                        range[0].lerp(f64::from(i) / 20.0),
                        range[1].lerp(f64::from(j) / 20.0),
                    ];
                    assert!(restricted.eval(uv).dist(base.eval(uv)) < 1.0e-11);
                }
            }
        }
    }

    #[test]
    fn bezier_patches_cover_each_source_span_in_deterministic_order() {
        let expected = [
            [ParamRange::new(0.0, 0.35), ParamRange::new(0.0, 0.45)],
            [ParamRange::new(0.0, 0.35), ParamRange::new(0.45, 1.0)],
            [ParamRange::new(0.35, 0.65), ParamRange::new(0.0, 0.45)],
            [ParamRange::new(0.35, 0.65), ParamRange::new(0.45, 1.0)],
            [ParamRange::new(0.65, 1.0), ParamRange::new(0.0, 0.45)],
            [ParamRange::new(0.65, 1.0), ParamRange::new(0.45, 1.0)],
        ];
        for base in [bicubic(), rational_bicubic()] {
            let patches = base.to_bezier_patches().unwrap();
            assert_eq!(patches.len(), expected.len());
            for (patch, range) in patches.iter().zip(expected) {
                assert_eq!(patch.param_range(), range);
                assert_eq!(patch.net_size(), (4, 4));
                assert_eq!(patch.is_rational(), base.is_rational());
                for i in 0..=6 {
                    for j in 0..=6 {
                        let uv = [
                            range[0].lerp(f64::from(i) / 6.0),
                            range[1].lerp(f64::from(j) / 6.0),
                        ];
                        assert!(patch.eval(uv).dist(base.eval(uv)) < 1.0e-11);
                    }
                }
            }
            for patch_u in 0..3 {
                assert_patch_boundary_bitwise(
                    &patches[patch_u * 2],
                    &patches[patch_u * 2 + 1],
                    Dir::V,
                );
            }
            for patch_u in 0..2 {
                for patch_v in 0..2 {
                    assert_patch_boundary_bitwise(
                        &patches[patch_u * 2 + patch_v],
                        &patches[(patch_u + 1) * 2 + patch_v],
                        Dir::U,
                    );
                }
            }
        }

        let rational = quarter_cylinder();
        let patches = rational.to_bezier_patches().unwrap();
        assert_eq!(patches.len(), 1);
        assert!(patches[0].is_rational());
        assert_eq!(patches[0], rational);
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
    fn certified_periodic_surface_wraps_polynomial_and_rational_evaluation() {
        for rational in [false, true] {
            let source = periodic_loop(rational);
            let start = source.eval_derivs([0.0, 0.37], 1);
            let end = source.eval_derivs([1.0, 0.37], 1);
            assert!(start.p.dist(end.p) < 1.0e-14);
            assert!((start.du - end.du).norm() < 1.0e-13);

            let periodic = source
                .with_certified_periodicity([true, false], 1.0e-12)
                .unwrap();
            assert_eq!(periodic.periodicity(), [Some(1.0), None]);
            let reference = periodic.eval_derivs([0.23, 0.37], 2);
            for u in [-1.77, 1.23, 8.23] {
                let wrapped = periodic.eval_derivs([u, 0.37], 2);
                assert!(wrapped.p.dist(reference.p) < 2.0e-14);
                assert!((wrapped.du - reference.du).norm() < 2.0e-13);
                assert!((wrapped.dv - reference.dv).norm() < 2.0e-13);
            }
        }
    }

    #[test]
    fn periodic_certification_rejects_false_seams_and_unclamped_basis() {
        assert!(
            NurbsSurface::new(
                0,
                1,
                vec![0.0, 1.0],
                vec![0.0, 0.0, 1.0, 1.0],
                vec![Point3::default(); 2],
                None,
            )
            .is_err()
        );
        let mut open = periodic_loop(false);
        open.points[6].x += 1.0e-4;
        assert!(
            open.with_certified_periodicity([true, false], 1.0e-8)
                .is_err()
        );

        let mut tangent_break = periodic_loop(false);
        tangent_break.points[4].y += 1.0e-4;
        assert!(
            tangent_break
                .with_certified_periodicity([true, false], 1.0e-8)
                .is_err()
        );

        let mut rational_weight_break = periodic_loop(true);
        rational_weight_break.weights.as_mut().unwrap()[4] += 1.0e-4;
        assert!(
            rational_weight_break
                .with_certified_periodicity([true, false], 1.0e-8)
                .is_err()
        );

        // A loose homogeneous tolerance is unsound when a positive rational
        // denominator can be arbitrarily small: quotient differentiation can
        // amplify this modest Euclidean tangent break without bound.
        let mut tiny_weights = periodic_loop(true);
        for weight in tiny_weights.weights.as_mut().unwrap() {
            *weight *= 1.0e-200;
        }
        tiny_weights.points[4].y += 1.0e-4;
        assert!(
            tiny_weights
                .with_certified_periodicity([true, false], 1.0e-8)
                .is_err()
        );

        let source = periodic_loop(false);
        let unclamped = NurbsSurface::new(
            2,
            1,
            vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            vec![0.0, 0.0, 1.0, 1.0],
            source.points,
            None,
        )
        .unwrap();
        assert!(
            unclamped
                .with_certified_periodicity([true, false], 1.0e-8)
                .is_err()
        );
    }

    #[test]
    fn periodic_bounds_cover_shifted_and_seam_crossing_ranges() {
        let surface = periodic_loop(true)
            .with_certified_periodicity([true, false], 1.0e-12)
            .unwrap();
        for range in [
            [ParamRange::new(0.8, 1.2), ParamRange::new(0.2, 0.8)],
            [ParamRange::new(-3.0, 4.0), ParamRange::new(0.2, 0.8)],
        ] {
            let bounds = surface
                .bounding_box(range)
                .inflated(kcore::tolerance::LINEAR_RESOLUTION);
            for i in 0..=80 {
                for j in 0..=20 {
                    let uv = [
                        range[0].lerp(f64::from(i) / 80.0),
                        range[1].lerp(f64::from(j) / 20.0),
                    ];
                    assert!(bounds.contains(surface.eval(uv)));
                }
            }
        }
    }

    #[test]
    fn partition_preserves_only_uncut_periodicity() {
        let surface = periodic_loop(false)
            .with_certified_periodicity([true, false], 1.0e-12)
            .unwrap();
        let (low_v, high_v) = surface.split_at(Dir::V, 0.4).unwrap();
        assert_eq!(low_v.periodicity(), [Some(1.0), None]);
        assert_eq!(high_v.periodicity(), [Some(1.0), None]);
        let (low_u, high_u) = surface.split_at(Dir::U, 0.4).unwrap();
        assert_eq!(low_u.periodicity(), [None, None]);
        assert_eq!(high_u.periodicity(), [None, None]);

        let transverse = surface
            .restricted_to([ParamRange::new(0.0, 1.0), ParamRange::new(0.2, 0.8)])
            .unwrap();
        assert_eq!(transverse.periodicity(), [Some(1.0), None]);
        let longitudinal = surface
            .restricted_to([ParamRange::new(0.2, 0.8), ParamRange::new(0.0, 1.0)])
            .unwrap();
        assert_eq!(longitudinal.periodicity(), [None, None]);
    }

    #[test]
    fn subrange_bounding_box_uses_restricted_control_net() {
        for surface in [bicubic(), rational_bicubic(), quarter_cylinder()] {
            let range = [ParamRange::new(0.0, 0.1), ParamRange::new(0.0, 0.1)];
            let full = Aabb3::from_points(surface.points());
            let bounds = surface
                .bounding_box(range)
                .inflated(kcore::tolerance::LINEAR_RESOLUTION);
            assert!(bounds.max.x < full.max.x || bounds.max.y < full.max.y);
            assert!(bounds.max.z < full.max.z);
            for i in 0..=40 {
                for j in 0..=40 {
                    let uv = [
                        range[0].lerp(f64::from(i) / 40.0),
                        range[1].lerp(f64::from(j) / 40.0),
                    ];
                    assert!(bounds.contains(surface.eval(uv)));
                }
            }
        }
    }

    #[test]
    fn surface_partition_operations_reject_unsupported_ranges() {
        let surface = bicubic();
        assert!(surface.split_at(Dir::U, 0.0).is_err());
        assert!(surface.split_at(Dir::V, 1.0).is_err());
        assert!(
            surface
                .restricted_to([ParamRange::new(0.2, 0.2), ParamRange::new(0.0, 1.0)])
                .is_err()
        );
        assert!(
            surface
                .restricted_to([ParamRange::new(0.0, 1.1), ParamRange::new(0.0, 1.0)])
                .is_err()
        );

        let unclamped = NurbsSurface::new(
            1,
            1,
            vec![0.0, 1.0, 2.0, 3.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
            ],
            None,
        )
        .unwrap();
        assert!(unclamped.split_at(Dir::U, 1.5).is_err());
        assert!(unclamped.restricted_to(unclamped.param_range()).is_err());
        assert!(unclamped.to_bezier_patches().is_err());
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

        let mut non_finite = vec![Point3::new(0.0, 0.0, 0.0); 9];
        non_finite[4].z = f64::NAN;
        let k = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        assert!(NurbsSurface::new(2, 2, k.clone(), k, non_finite, None).is_err());
    }
}
