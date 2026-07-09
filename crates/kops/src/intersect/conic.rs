use kcore::math;
use kcore::tolerance::Tolerances;
use kgeom::curve::Ellipse;
use kgeom::param::ParamRange;
use kgeom::vec::Vec3;

pub(super) fn ellipse_parameter(local: Vec3, ellipse: &Ellipse) -> f64 {
    math::atan2(
        local.y / ellipse.minor_radius(),
        local.x / ellipse.major_radius(),
    )
}

pub(super) fn fit_periodic_parameter(
    candidate: f64,
    range: ParamRange,
    tolerance: f64,
) -> Option<f64> {
    let period = core::f64::consts::TAU;
    let k_min = ((range.lo - tolerance - candidate) / period).ceil() as i64;
    let k_max = ((range.hi + tolerance - candidate) / period).floor() as i64;
    if k_min > k_max {
        return None;
    }
    Some((candidate + k_min as f64 * period).clamp(range.lo, range.hi))
}

pub(super) fn parameter_tolerance(radius: f64, tolerances: Tolerances) -> f64 {
    (tolerances.linear() / radius).max(tolerances.angular())
}

pub(super) fn push_angle_root(roots: &mut Vec<f64>, t: f64) {
    let t = canonical_angle(t);
    if !roots
        .iter()
        .any(|existing| angular_distance(*existing, t) <= 1e-10)
    {
        roots.push(t);
    }
}

pub(super) fn canonical_angle(t: f64) -> f64 {
    let period = core::f64::consts::TAU;
    let mut s = t % period;
    if s < 0.0 {
        s += period;
    }
    if period - s <= 1e-14 { 0.0 } else { s }
}

fn angular_distance(a: f64, b: f64) -> f64 {
    let period = core::f64::consts::TAU;
    let d = (a - b).abs();
    d.min(period - d)
}

pub(super) fn real_polynomial_roots(coeffs: &[f64]) -> Vec<f64> {
    let poly = trim_polynomial(coeffs);
    let degree = poly.len().saturating_sub(1);
    if degree == 0 {
        return Vec::new();
    }
    if degree == 1 {
        return vec![-poly[0] / poly[1]];
    }

    let bound = polynomial_root_bound(&poly);
    let mut critical = real_polynomial_roots(&polynomial_derivative(&poly));
    critical.retain(|x| x.is_finite() && *x > -bound && *x < bound);
    critical.sort_by(f64::total_cmp);
    dedup_sorted_scalars(&mut critical, 1e-10);

    let value_tol = polynomial_value_tolerance(&poly);
    let mut roots = Vec::new();
    let mut cuts = Vec::with_capacity(critical.len() + 2);
    cuts.push(-bound);
    cuts.extend(critical.iter().copied());
    cuts.push(bound);

    for &x in &critical {
        if eval_polynomial(&poly, x).abs() <= value_tol {
            roots.push(x);
        }
    }

    for interval in cuts.windows(2) {
        let lo = interval[0];
        let hi = interval[1];
        let f_lo = eval_polynomial(&poly, lo);
        let f_hi = eval_polynomial(&poly, hi);
        if f_lo.abs() <= value_tol {
            roots.push(lo);
            continue;
        }
        if f_hi.abs() <= value_tol {
            roots.push(hi);
            continue;
        }
        if f_lo.signum() == f_hi.signum() {
            continue;
        }
        roots.push(bisect_polynomial_root(&poly, lo, hi));
    }

    roots.retain(|x| x.is_finite());
    roots.sort_by(f64::total_cmp);
    dedup_sorted_scalars(&mut roots, 1e-9);
    roots
}

fn trim_polynomial(coeffs: &[f64]) -> Vec<f64> {
    let mut hi = coeffs.len();
    while hi > 1 && coeffs[hi - 1].abs() <= 1e-14 {
        hi -= 1;
    }
    coeffs[..hi].to_vec()
}

pub(super) fn polynomial_derivative(poly: &[f64]) -> Vec<f64> {
    poly.iter()
        .enumerate()
        .skip(1)
        .map(|(i, c)| *c * i as f64)
        .collect()
}

fn polynomial_root_bound(poly: &[f64]) -> f64 {
    let leading = poly[poly.len() - 1].abs();
    let mut max_ratio: f64 = 0.0;
    for coeff in &poly[..poly.len() - 1] {
        max_ratio = max_ratio.max(coeff.abs() / leading);
    }
    1.0 + max_ratio
}

fn polynomial_value_tolerance(poly: &[f64]) -> f64 {
    let scale = poly.iter().fold(0.0_f64, |acc, coeff| acc.max(coeff.abs()));
    (scale * 1e-12).max(1e-12)
}

fn eval_polynomial(poly: &[f64], x: f64) -> f64 {
    let mut y = 0.0;
    for &coeff in poly.iter().rev() {
        y = y * x + coeff;
    }
    y
}

fn bisect_polynomial_root(poly: &[f64], mut lo: f64, mut hi: f64) -> f64 {
    let mut f_lo = eval_polynomial(poly, lo);
    for _ in 0..100 {
        let mid = (lo + hi) / 2.0;
        let f_mid = eval_polynomial(poly, mid);
        if f_mid == 0.0 || (hi - lo).abs() <= 1e-13 * (1.0 + mid.abs()) {
            return mid;
        }
        if f_lo.signum() == f_mid.signum() {
            lo = mid;
            f_lo = f_mid;
        } else {
            hi = mid;
        }
    }
    (lo + hi) / 2.0
}

fn dedup_sorted_scalars(values: &mut Vec<f64>, tolerance: f64) {
    let mut out = Vec::with_capacity(values.len());
    for value in values.drain(..) {
        if !out
            .iter()
            .any(|existing: &f64| (*existing - value).abs() <= tolerance * (1.0 + value.abs()))
        {
            out.push(value);
        }
    }
    *values = out;
}
