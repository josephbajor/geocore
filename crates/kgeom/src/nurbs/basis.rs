//! B-spline basis functions and their derivatives.

use super::knots::KnotVector;

/// All `p + 1` basis functions that are nonzero on `span` at `u`
/// (A2.2 `BasisFuns`): entry `j` is `N_{span-p+j, p}(u)`.
pub fn basis_funs(kv: &KnotVector, span: usize, u: f64) -> Vec<f64> {
    let p = kv.degree();
    let knots = kv.as_slice();
    let mut n = vec![0.0; p + 1];
    let mut left = vec![0.0; p + 1];
    let mut right = vec![0.0; p + 1];
    n[0] = 1.0;
    for j in 1..=p {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        let mut saved = 0.0;
        for r in 0..j {
            let temp = n[r] / (right[r + 1] + left[j - r]);
            n[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        n[j] = saved;
    }
    n
}

/// Basis functions and derivatives up to `order` at `u` on `span`
/// (A2.3 `DersBasisFuns`): `ders[k][j]` is the `k`-th derivative of
/// `N_{span-p+j, p}` at `u`. Rows for `k > degree` are zero.
pub fn ders_basis_funs(kv: &KnotVector, span: usize, u: f64, order: usize) -> Vec<Vec<f64>> {
    let p = kv.degree();
    let knots = kv.as_slice();
    let mut ndu = vec![vec![0.0; p + 1]; p + 1];
    let mut left = vec![0.0; p + 1];
    let mut right = vec![0.0; p + 1];
    ndu[0][0] = 1.0;
    for j in 1..=p {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        let mut saved = 0.0;
        for r in 0..j {
            // Lower triangle: knot differences; upper: basis functions.
            ndu[j][r] = right[r + 1] + left[j - r];
            let temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }

    let mut ders = vec![vec![0.0; p + 1]; order + 1];
    for j in 0..=p {
        ders[0][j] = ndu[j][p];
    }
    let n_eff = order.min(p);
    let mut a1 = vec![0.0; p + 1];
    let mut a2 = vec![0.0; p + 1];
    for r in 0..=p {
        a1.fill(0.0);
        a2.fill(0.0);
        a1[0] = 1.0;
        for k in 1..=n_eff {
            let mut d = 0.0;
            let rk = r as i64 - k as i64;
            let pk = p - k;
            if r >= k {
                a2[0] = a1[0] / ndu[pk + 1][rk as usize];
                d = a2[0] * ndu[rk as usize][pk];
            }
            let j1 = if rk >= -1 { 1 } else { (-rk) as usize };
            let j2 = if r as i64 - 1 <= pk as i64 {
                k - 1
            } else {
                p - r
            };
            for j in j1..=j2 {
                let idx = (rk + j as i64) as usize;
                a2[j] = (a1[j] - a1[j - 1]) / ndu[pk + 1][idx];
                d += a2[j] * ndu[idx][pk];
            }
            if r <= pk {
                a2[k] = -a1[k - 1] / ndu[pk + 1][r];
                d += a2[k] * ndu[r][pk];
            }
            ders[k][r] = d;
            core::mem::swap(&mut a1, &mut a2);
        }
    }
    // Multiply through by p! / (p - k)!.
    let mut factor = p as f64;
    for (k, row) in ders.iter_mut().enumerate().take(n_eff + 1).skip(1) {
        for value in row.iter_mut() {
            *value *= factor;
        }
        factor *= (p - k) as f64;
    }
    ders
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nurbs::knots::KnotVector;

    fn cubic() -> KnotVector {
        KnotVector::new(3, vec![0.0, 0.0, 0.0, 0.0, 0.35, 0.65, 1.0, 1.0, 1.0, 1.0]).unwrap()
    }

    #[test]
    fn partition_of_unity() {
        let kv = cubic();
        for i in 0..=200 {
            let u = i as f64 / 200.0;
            let span = kv.find_span(u);
            let n = basis_funs(&kv, span, u);
            let sum: f64 = n.iter().sum();
            assert!((sum - 1.0).abs() < 1e-14, "u = {u}, sum = {sum}");
            assert!(n.iter().all(|&v| v >= -1e-15), "negative basis at {u}");
        }
    }

    #[test]
    fn ders_row_zero_matches_basis_funs() {
        let kv = cubic();
        for i in 0..=50 {
            let u = i as f64 / 50.0;
            let span = kv.find_span(u);
            let n = basis_funs(&kv, span, u);
            let d = ders_basis_funs(&kv, span, u, 3);
            for j in 0..n.len() {
                assert!((n[j] - d[0][j]).abs() < 1e-15);
            }
        }
    }

    #[test]
    fn derivatives_match_finite_differences() {
        let kv = cubic();
        let h = 1e-6;
        // Stay away from knots so the FD stencil sits inside one span.
        for &u in &[0.1, 0.2, 0.45, 0.55, 0.8, 0.9] {
            let span = kv.find_span(u);
            let d = ders_basis_funs(&kv, span, u, 2);
            let np = basis_funs(&kv, kv.find_span(u + h), u + h);
            let nm = basis_funs(&kv, kv.find_span(u - h), u - h);
            debug_assert_eq!(kv.find_span(u + h), span);
            debug_assert_eq!(kv.find_span(u - h), span);
            for j in 0..d[0].len() {
                let fd1 = (np[j] - nm[j]) / (2.0 * h);
                assert!(
                    (d[1][j] - fd1).abs() < 1e-6 * (1.0 + fd1.abs()),
                    "u={u} j={j}: {} vs {}",
                    d[1][j],
                    fd1
                );
                let fd2 = (np[j] - 2.0 * d[0][j] + nm[j]) / (h * h);
                assert!(
                    (d[2][j] - fd2).abs() < 1e-3 * (1.0 + fd2.abs()),
                    "u={u} j={j}: {} vs {}",
                    d[2][j],
                    fd2
                );
            }
        }
    }

    #[test]
    fn derivative_rows_above_degree_are_zero() {
        let kv = KnotVector::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap();
        let d = ders_basis_funs(&kv, kv.find_span(0.4), 0.4, 3);
        assert!(d[3].iter().all(|&v| v == 0.0));
    }

    #[test]
    fn derivative_sum_is_zero() {
        // Derivative rows of a partition of unity must sum to zero.
        let kv = cubic();
        for &u in &[0.1, 0.5, 0.9] {
            let span = kv.find_span(u);
            let d = ders_basis_funs(&kv, span, u, 3);
            for row in d.iter().skip(1) {
                let sum: f64 = row.iter().sum();
                assert!(sum.abs() < 1e-9, "u={u}: derivative row sums to {sum}");
            }
        }
    }
}
