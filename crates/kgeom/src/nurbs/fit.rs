//! Global curve interpolation (A9.1).

use super::knots::KnotVector;
use super::ncurve::NurbsCurve;
use crate::vec::Point3;
use kcore::error::{Error, Result};

/// Interpolate a clamped polynomial B-spline curve of `degree` through
/// `points` (A9.1 `GlobalCurveInterp`): chord-length parameterization,
/// averaged knots, dense linear solve.
///
/// Requires at least `degree + 1` points and strictly positive chords
/// (consecutive duplicate points would degenerate the parameterization).
pub fn interpolate(points: &[Point3], degree: usize) -> Result<NurbsCurve> {
    let n = points.len();
    if degree < 1 {
        return Err(Error::InvalidGeometry {
            reason: "interpolation degree must be at least 1",
        });
    }
    if n < degree + 1 {
        return Err(Error::InvalidGeometry {
            reason: "interpolation needs at least degree + 1 points",
        });
    }

    // Chord-length parameters (E9.4/E9.5).
    let mut chords = Vec::with_capacity(n - 1);
    let mut total = 0.0;
    for pair in points.windows(2) {
        let c = pair[0].dist(pair[1]);
        if c <= 0.0 {
            return Err(Error::InvalidGeometry {
                reason: "interpolation points must be pairwise distinct in sequence",
            });
        }
        chords.push(c);
        total += c;
    }
    let mut ubar = Vec::with_capacity(n);
    ubar.push(0.0);
    let mut acc = 0.0;
    for &c in &chords[..n - 2] {
        acc += c;
        ubar.push(acc / total);
    }
    ubar.push(1.0);

    // Averaged knot vector (E9.8).
    let mut knots = vec![0.0; n + degree + 1];
    for k in knots.iter_mut().rev().take(degree + 1) {
        *k = 1.0;
    }
    for j in 1..n - degree {
        let avg: f64 = ubar[j..j + degree].iter().sum::<f64>() / degree as f64;
        knots[j + degree] = avg;
    }
    let kv = KnotVector::new(degree, knots.clone())?;

    // Coefficient matrix: row i holds the nonzero basis functions at ubar[i].
    let mut a = vec![vec![0.0; n]; n];
    for (i, &u) in ubar.iter().enumerate() {
        let span = kv.find_span(u);
        let basis = super::basis::basis_funs(&kv, span, u);
        for (j, &b) in basis.iter().enumerate() {
            a[i][span - degree + j] = b;
        }
    }
    let control = solve_dense(a, points.to_vec())?;
    NurbsCurve::new(degree, knots, control, None)
}

/// Gaussian elimination with partial pivoting on a dense system with `Point3`
/// right-hand sides. Deterministic: pivot ties resolve to the first maximal
/// row.
fn solve_dense(mut a: Vec<Vec<f64>>, mut b: Vec<Point3>) -> Result<Vec<Point3>> {
    let n = b.len();
    for col in 0..n {
        // Partial pivot.
        let mut pivot = col;
        let mut best = a[col][col].abs();
        for (row, arow) in a.iter().enumerate().skip(col + 1) {
            let v = arow[col].abs();
            if v > best {
                best = v;
                pivot = row;
            }
        }
        if best < 1e-13 {
            return Err(Error::InvalidGeometry {
                reason: "singular interpolation system",
            });
        }
        a.swap(col, pivot);
        b.swap(col, pivot);

        let diag = a[col][col];
        let pivot_row = a[col].clone();
        let pivot_rhs = b[col];
        for row in col + 1..n {
            let factor = a[row][col] / diag;
            if factor == 0.0 {
                continue;
            }
            for (av, &pv) in a[row].iter_mut().zip(&pivot_row).skip(col) {
                *av -= factor * pv;
            }
            b[row] -= pivot_rhs * factor;
        }
    }
    // Back substitution.
    let mut x = vec![Point3::default(); n];
    for col in (0..n).rev() {
        let mut acc = b[col];
        for (&xv, &av) in x[col + 1..].iter().zip(&a[col][col + 1..]) {
            acc -= xv * av;
        }
        x[col] = acc / a[col][col];
    }
    Ok(x)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::conformance::check_curve;
    use crate::curve::Curve;

    #[test]
    fn cubic_interpolation_passes_through_planar_points() {
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.5, 0.0),
            Point3::new(2.0, 0.5, 0.0),
            Point3::new(3.5, 2.0, 0.0),
            Point3::new(5.0, 1.0, 0.0),
            Point3::new(6.0, -0.5, 0.0),
        ];
        let c = interpolate(&pts, 3).unwrap();
        assert_interpolates(&c, &pts);
    }

    #[test]
    fn cubic_interpolation_passes_through_3d_points() {
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 1.0),
            Point3::new(2.0, 1.0, -1.0),
            Point3::new(3.0, 3.0, 0.5),
            Point3::new(4.5, 2.0, 2.0),
            Point3::new(5.0, 0.0, 1.0),
            Point3::new(6.5, -1.0, 0.2),
        ];
        let c = interpolate(&pts, 3).unwrap();
        assert_interpolates(&c, &pts);
        check_curve(&c);
    }

    fn assert_interpolates(c: &NurbsCurve, pts: &[Point3]) {
        // Recompute the chord-length parameters the fit used.
        let total: f64 = pts.windows(2).map(|w| w[0].dist(w[1])).sum();
        let mut u = 0.0;
        for (k, p) in pts.iter().enumerate() {
            if k > 0 {
                u += pts[k - 1].dist(*p) / total;
            }
            let q = c.eval(if k == pts.len() - 1 { 1.0 } else { u });
            assert!(
                p.dist(q) < 1e-9,
                "interpolant misses point {k}: {p:?} vs {q:?}"
            );
        }
    }

    #[test]
    fn degenerate_inputs_rejected() {
        let p = Point3::new(1.0, 0.0, 0.0);
        // Too few points.
        assert!(interpolate(&[p, p], 3).is_err());
        // Coincident consecutive points.
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 1.0, 0.0),
        ];
        assert!(interpolate(&pts, 3).is_err());
    }
}
