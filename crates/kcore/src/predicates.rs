//! Robust geometric predicates.
//!
//! Each predicate runs a fast floating-point evaluation guarded by a forward
//! error bound (Shewchuk's stage-A filter). When the filter cannot certify
//! the sign, it falls back to exact expansion arithmetic, so the returned
//! [`Orientation`] is always the true sign of the underlying determinant.
//!
//! Performance note: the fallback here is the *fully exact* computation
//! rather than Shewchuk's staged B/C/D adaptivity. That trades speed on
//! near-degenerate inputs for a much smaller trusted core. Staged adaptivity
//! is a planned optimization (roadmap M8) once profiling justifies it.

use crate::expansion;
use crate::interval::Interval;

/// Sign of a predicate's underlying determinant.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Orientation {
    /// Determinant is negative.
    Negative,
    /// Determinant is exactly zero (degenerate configuration).
    Zero,
    /// Determinant is positive.
    Positive,
}

impl Orientation {
    #[inline]
    fn from_scalar(det: f64) -> Self {
        if det > 0.0 {
            Orientation::Positive
        } else if det < 0.0 {
            Orientation::Negative
        } else {
            Orientation::Zero
        }
    }

    #[inline]
    fn from_sign(s: i8) -> Self {
        match s.cmp(&0) {
            core::cmp::Ordering::Greater => Orientation::Positive,
            core::cmp::Ordering::Less => Orientation::Negative,
            core::cmp::Ordering::Equal => Orientation::Zero,
        }
    }

    /// Compact integer sign: -1, 0, or 1.
    #[inline]
    pub fn as_i8(self) -> i8 {
        match self {
            Orientation::Negative => -1,
            Orientation::Zero => 0,
            Orientation::Positive => 1,
        }
    }
}

/// Exact sign and a finite floating approximation of `b² - 4ac`.
///
/// The approximation is evidence for subsequent numeric work such as a square
/// root; it must never replace [`Self::sign`] for classification.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuadraticDiscriminant {
    sign: Orientation,
    approximation: f64,
    used_exact_fallback: bool,
}

impl QuadraticDiscriminant {
    /// Exact sign of `b² - 4ac`.
    pub const fn sign(self) -> Orientation {
        self.sign
    }

    /// Finite, sign-consistent approximation of the exact discriminant.
    pub const fn approximation(self) -> f64 {
        self.approximation
    }

    /// Whether outward interval arithmetic was inconclusive and exact
    /// expansion arithmetic supplied the result.
    pub const fn used_exact_fallback(self) -> bool {
        self.used_exact_fallback
    }
}

/// Real roots of the homogeneous half-angle quadratic for
/// `cosine*cos(t) + sine*sin(t) + constant == 0`.
///
/// Finite entries are roots in `y = tan(t/2)`. [`Self::has_infinity_root`]
/// represents the projective root `y = infinity`, i.e. `t = pi`. An
/// identically-zero harmonic has no discrete roots and is reported through
/// [`Self::is_identity`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HarmonicHalfAngleRoots {
    finite_roots: [f64; 2],
    finite_root_count: u8,
    has_infinity_root: bool,
    discriminant: Orientation,
    used_exact_fallback: bool,
    identity: bool,
}

impl HarmonicHalfAngleRoots {
    /// Finite half-angle roots in the deterministic quadratic-formula order.
    pub fn finite_roots(&self) -> &[f64] {
        &self.finite_roots[..usize::from(self.finite_root_count)]
    }

    /// Whether the homogeneous quadratic has a root at `y = infinity`.
    pub const fn has_infinity_root(self) -> bool {
        self.has_infinity_root
    }

    /// Exact sign of the half-angle quadratic discriminant.
    pub const fn discriminant(self) -> Orientation {
        self.discriminant
    }

    /// Whether exact expansion arithmetic classified the discriminant.
    pub const fn used_exact_fallback(self) -> bool {
        self.used_exact_fallback
    }

    /// Whether every parameter satisfies the harmonic equation.
    pub const fn is_identity(self) -> bool {
        self.identity
    }
}

const HARMONIC_SAFE_MIN: f64 = f64::from_bits(((1023 - 170) as u64) << 52);
const HARMONIC_SAFE_MAX: f64 = f64::from_bits(((1023 + 170) as u64) << 52);

fn floor_binary_exponent(value: f64) -> i32 {
    debug_assert!(value.is_finite() && value != 0.0);
    let bits = value.abs().to_bits();
    let stored = ((bits >> 52) & 0x7ff) as i32;
    if stored != 0 {
        stored - 1023
    } else {
        let fraction = bits & ((1_u64 << 52) - 1);
        let highest = 63 - fraction.leading_zeros() as i32;
        highest - 1074
    }
}

fn normal_power_of_two(exponent: i32) -> f64 {
    debug_assert!((-1022..=1023).contains(&exponent));
    f64::from_bits(((1023 + exponent) as u64) << 52)
}

fn scale_power_of_two(mut value: f64, mut exponent: i32) -> Option<f64> {
    while exponent > 1023 {
        value *= normal_power_of_two(1023);
        exponent -= 1023;
    }
    while exponent < -1022 {
        value *= normal_power_of_two(-1022);
        exponent += 1022;
    }
    value *= normal_power_of_two(exponent);
    value.is_finite().then_some(value)
}

fn normalize_harmonic_coefficients(coefficients: [f64; 3]) -> Option<[f64; 3]> {
    if coefficients
        .iter()
        .any(|coefficient| !coefficient.is_finite())
    {
        return None;
    }
    let max = coefficients
        .iter()
        .map(|coefficient| coefficient.abs())
        .fold(0.0, f64::max);
    if max == 0.0 || (HARMONIC_SAFE_MIN..=HARMONIC_SAFE_MAX).contains(&max) {
        return Some(coefficients);
    }

    let exponent = -floor_binary_exponent(max);
    let mut normalized = [0.0; 3];
    for (index, coefficient) in coefficients.into_iter().enumerate() {
        let scaled = scale_power_of_two(coefficient, exponent)?;
        if coefficient != 0.0
            && (scaled == 0.0
                || scale_power_of_two(scaled, -exponent)?.to_bits() != coefficient.to_bits())
        {
            return None;
        }
        normalized[index] = scaled;
    }
    Some(normalized)
}

// `split` multiplies each operand by less than 2^28, so component exponents
// in [-500, 500] keep every split intermediate finite and normal. A nonzero
// residual of two 53-bit significands is no smaller than roughly 2^-105 times
// their product. Restricting product exponents to [-400, 400] therefore keeps
// residues above 2^-506; multiplying the `ac` expansion by four then stays in
// the normal, finite exponent range [-504, 402].
const EXACT_COMPONENT_MIN: f64 = f64::from_bits(((1023 - 500) as u64) << 52);
const EXACT_COMPONENT_MAX: f64 = f64::from_bits(((1023 + 500) as u64) << 52);
const EXACT_PRODUCT_MIN: f64 = f64::from_bits(((1023 - 400) as u64) << 52);
const EXACT_PRODUCT_MAX: f64 = f64::from_bits(((1023 + 400) as u64) << 52);

fn exact_components_are_normal(expansion: &[f64]) -> bool {
    expansion
        .iter()
        .all(|component| *component == 0.0 || component.is_normal())
}

fn exact_product_for_discriminant(a: f64, b: f64) -> Option<Vec<f64>> {
    if a == 0.0 || b == 0.0 {
        return Some(vec![0.0]);
    }
    if !(EXACT_COMPONENT_MIN..=EXACT_COMPONENT_MAX).contains(&a.abs())
        || !(EXACT_COMPONENT_MIN..=EXACT_COMPONENT_MAX).contains(&b.abs())
    {
        return None;
    }
    let product = a.abs() * b.abs();
    if !(EXACT_PRODUCT_MIN..=EXACT_PRODUCT_MAX).contains(&product) {
        return None;
    }
    let (rounded, residue) = expansion::two_product(a, b);
    let expansion = expansion::from_two(rounded, residue);
    exact_components_are_normal(&expansion).then_some(expansion)
}

#[derive(Debug, Clone, Copy)]
struct ExactSignedApproximation {
    sign: Orientation,
    approximation: f64,
    used_exact_fallback: bool,
}

fn harmonic_norm_difference(
    cosine: f64,
    sine: f64,
    constant: f64,
) -> Option<ExactSignedApproximation> {
    let rounded = cosine * cosine + sine * sine - constant * constant;
    let interval = Interval::point(cosine).square() + Interval::point(sine).square()
        - Interval::point(constant).square();
    if interval.lo().is_finite()
        && interval.hi().is_finite()
        && rounded.is_finite()
        && let Some(sign) = interval.sign()
    {
        let sign = Orientation::from_sign(sign);
        if sign == Orientation::from_scalar(rounded) {
            return Some(ExactSignedApproximation {
                sign,
                approximation: if sign == Orientation::Zero {
                    0.0
                } else {
                    rounded
                },
                used_exact_fallback: false,
            });
        }
    }

    let cosine_square = exact_product_for_discriminant(cosine, cosine)?;
    let sine_square = exact_product_for_discriminant(sine, sine)?;
    let constant_square = exact_product_for_discriminant(constant, constant)?;
    let sum = expansion::sum(&cosine_square, &sine_square);
    let difference = expansion::sum(&sum, &expansion::negate(&constant_square));
    if !exact_components_are_normal(&difference) {
        return None;
    }
    let sign = Orientation::from_sign(expansion::sign(&difference));
    let approximation = if sign == Orientation::Zero {
        0.0
    } else {
        let approximation = expansion::approx(&difference);
        if Orientation::from_scalar(approximation) == sign {
            approximation
        } else {
            *difference.last()?
        }
    };
    if !approximation.is_finite() || Orientation::from_scalar(approximation) != sign {
        return None;
    }
    Some(ExactSignedApproximation {
        sign,
        approximation,
        used_exact_fallback: true,
    })
}

/// Classify the quadratic discriminant `b² - 4ac` without tolerance.
///
/// Every returned [`QuadraticDiscriminant`] is an exact classification of the
/// supplied finite `f64` coefficients as dyadic rationals. Outward interval
/// arithmetic certifies ordinary cases; cancellation routes to exact expansion
/// arithmetic.
///
/// Non-finite inputs or a fallback path outside the conservative
/// exact-expansion exponent envelope return `None` rather than an uncertain
/// classification; exact algebraic zero returns [`Orientation::Zero`] with
/// approximation `0.0`. In particular, fallback refuses nonzero product,
/// residue, and scale-by-four paths that could enter the subnormal range or
/// overflow.
///
/// This helper does not decide whether `a` is numerically usable as a quadratic
/// coefficient; callers retain their metric coefficient policy.
pub fn quadratic_discriminant(a: f64, b: f64, c: f64) -> Option<QuadraticDiscriminant> {
    if !a.is_finite() || !b.is_finite() || !c.is_finite() {
        return None;
    }

    let rounded = b * b - 4.0 * a * c;
    let interval = Interval::point(b).square()
        - (Interval::point(a) * Interval::point(c)) * Interval::point(4.0);
    if interval.lo().is_finite()
        && interval.hi().is_finite()
        && rounded.is_finite()
        && let Some(sign) = interval.sign()
    {
        let sign = Orientation::from_sign(sign);
        let rounded_sign = Orientation::from_scalar(rounded);
        if sign == rounded_sign {
            return Some(QuadraticDiscriminant {
                sign,
                approximation: if sign == Orientation::Zero {
                    0.0
                } else {
                    rounded
                },
                used_exact_fallback: false,
            });
        }
    }

    let square = exact_product_for_discriminant(b, b)?;
    let product = exact_product_for_discriminant(a, c)?;
    let four_ac = expansion::scale(&product, 4.0);
    if !exact_components_are_normal(&four_ac) {
        return None;
    }
    let discriminant = expansion::sum(&square, &expansion::negate(&four_ac));
    if !exact_components_are_normal(&discriminant) {
        return None;
    }

    let sign = Orientation::from_sign(expansion::sign(&discriminant));
    let approximation = if sign == Orientation::Zero {
        0.0
    } else {
        let approximation = expansion::approx(&discriminant);
        let approximation_sign = Orientation::from_scalar(approximation);
        if approximation_sign == sign {
            approximation
        } else {
            *discriminant.last()?
        }
    };
    if !approximation.is_finite() || Orientation::from_scalar(approximation) != sign {
        return None;
    }
    Some(QuadraticDiscriminant {
        sign,
        approximation,
        used_exact_fallback: true,
    })
}

/// Solve `cosine*cos(t) + sine*sin(t) + constant == 0` through the
/// homogeneous half-angle chart.
///
/// Power-of-two normalization preserves the original harmonic exactly while
/// keeping ordinary products inside the exact-expansion envelope. Root count
/// is classified from the exact sign of
/// `cosine² + sine² - constant²`. The rounded half-angle coefficients
/// `(constant - cosine)y² + 2*sine*y + (cosine + constant)` are used only for
/// numeric root construction and the ordinary bit-compatible path. Polynomial
/// degree remains algebraic: only an exactly zero rounded leading coefficient
/// creates the projective `t = pi` root. No caller tolerance is used for these
/// decisions.
///
/// Returns `None` for non-finite input, an exact normalization that cannot be
/// represented without losing a coefficient, an unclassifiable discriminant,
/// or a finite half-angle root outside `f64` range. Callers must treat `None`
/// as incomplete numeric evidence, never as a certified miss.
pub fn harmonic_half_angle_roots(
    cosine: f64,
    sine: f64,
    constant: f64,
) -> Option<HarmonicHalfAngleRoots> {
    let [cosine, sine, constant] = normalize_harmonic_coefficients([cosine, sine, constant])?;
    if cosine == 0.0 && sine == 0.0 && constant == 0.0 {
        return Some(HarmonicHalfAngleRoots {
            finite_roots: [0.0; 2],
            finite_root_count: 0,
            has_infinity_root: false,
            discriminant: Orientation::Zero,
            used_exact_fallback: false,
            identity: true,
        });
    }

    let a = constant - cosine;
    let b = 2.0 * sine;
    let c = cosine + constant;
    if !a.is_finite() || !b.is_finite() || !c.is_finite() {
        return None;
    }
    if a == 0.0 {
        if b == 0.0 {
            return Some(HarmonicHalfAngleRoots {
                finite_roots: [0.0; 2],
                finite_root_count: 0,
                has_infinity_root: true,
                discriminant: Orientation::Zero,
                used_exact_fallback: false,
                identity: false,
            });
        }
        let root = -c / b;
        if !root.is_finite() {
            return None;
        }
        return Some(HarmonicHalfAngleRoots {
            finite_roots: [root, 0.0],
            finite_root_count: 1,
            has_infinity_root: true,
            discriminant: Orientation::Positive,
            used_exact_fallback: false,
            identity: false,
        });
    }

    let discriminant = harmonic_norm_difference(cosine, sine, constant)?;
    let sign = discriminant.sign;
    let rounded_half_angle_discriminant = b * b - 4.0 * a * c;
    let mut finite_roots = [0.0; 2];
    let finite_root_count = match sign {
        Orientation::Negative => 0,
        Orientation::Zero => {
            finite_roots[0] = -b / (2.0 * a);
            1
        }
        Orientation::Positive
            if !discriminant.used_exact_fallback
                && Orientation::from_scalar(rounded_half_angle_discriminant)
                    == Orientation::Positive =>
        {
            let root = rounded_half_angle_discriminant.sqrt();
            finite_roots = [(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)];
            2
        }
        Orientation::Positive => {
            let root = (4.0 * discriminant.approximation).sqrt();
            let q = -0.5 * (b + root.copysign(b));
            if q == 0.0 || !q.is_finite() {
                return None;
            }
            finite_roots = [q / a, c / q];
            if (a > 0.0 && finite_roots[0] > finite_roots[1])
                || (a < 0.0 && finite_roots[0] < finite_roots[1])
            {
                finite_roots.swap(0, 1);
            }
            2
        }
    };
    if finite_roots[..finite_root_count]
        .iter()
        .any(|root| !root.is_finite())
    {
        return None;
    }
    Some(HarmonicHalfAngleRoots {
        finite_roots,
        finite_root_count: finite_root_count as u8,
        has_infinity_root: false,
        discriminant: sign,
        used_exact_fallback: discriminant.used_exact_fallback,
        identity: false,
    })
}

/// Machine epsilon in Shewchuk's convention: 2^-53, half a ulp of 1.0.
const EPS: f64 = f64::EPSILON / 2.0;
/// Stage-A error bound coefficient for `orient2d` (Shewchuk `ccwerrboundA`).
const CCW_ERRBOUND_A: f64 = (3.0 + 16.0 * EPS) * EPS;
/// Stage-A error bound coefficient for `orient3d` (Shewchuk `o3derrboundA`).
const O3D_ERRBOUND_A: f64 = (7.0 + 56.0 * EPS) * EPS;
/// Stage-A error bound coefficient for `incircle` (Shewchuk `iccerrboundA`).
const ICC_ERRBOUND_A: f64 = (10.0 + 96.0 * EPS) * EPS;

/// Orientation of point `c` relative to the directed line `a -> b`.
///
/// Returns [`Orientation::Positive`] when `a`, `b`, `c` wind counterclockwise
/// (i.e. `c` lies to the left of `a -> b`), the exact sign of
/// `det[[ax - cx, ay - cy], [bx - cx, by - cy]]`.
pub fn orient2d(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> Orientation {
    let det_left = (a[0] - c[0]) * (b[1] - c[1]);
    let det_right = (a[1] - c[1]) * (b[0] - c[0]);
    let det = det_left - det_right;

    let det_sum = if det_left > 0.0 {
        if det_right <= 0.0 {
            return Orientation::from_scalar(det);
        }
        det_left + det_right
    } else if det_left < 0.0 {
        if det_right >= 0.0 {
            return Orientation::from_scalar(det);
        }
        -det_left - det_right
    } else {
        return Orientation::from_scalar(det);
    };

    let errbound = CCW_ERRBOUND_A * det_sum;
    if det >= errbound || -det >= errbound {
        return Orientation::from_scalar(det);
    }
    orient2d_exact(a, b, c)
}

/// Exact orientation of a closed 2D polygon.
///
/// Returns the sign of the exact cyclic shoelace sum
/// `sum_i(x_i * y_(i+1) - y_i * x_(i+1))`: positive for counterclockwise
/// winding and negative for clockwise winding. The result is invariant under
/// cyclic rotation of the vertices, and reversing their order reverses every
/// nonzero result.
///
/// Fewer than three vertices, any non-finite coordinate, or an exactly zero
/// shoelace sum returns [`Orientation::Zero`]. Repeated vertices are allowed;
/// self-intersecting polygons retain the sign of their algebraic area.
pub fn polygon_orientation2d(points: &[[f64; 2]]) -> Orientation {
    polygon_orientation2d_iter(points.iter().copied())
}

/// Streaming form of [`polygon_orientation2d`].
///
/// This evaluates the same exact cyclic shoelace expansion without retaining
/// or allocating a copy of the input. It has the same winding convention and
/// returns [`Orientation::Zero`] for fewer than three vertices, any non-finite
/// coordinate, or an exactly zero shoelace sum.
pub fn polygon_orientation2d_iter(points: impl IntoIterator<Item = [f64; 2]>) -> Orientation {
    let mut twice_area = vec![0.0];
    let mut first = None;
    let mut previous = None;
    let mut count = 0_usize;
    for point in points {
        if point.iter().any(|coordinate| !coordinate.is_finite()) {
            return Orientation::Zero;
        }
        if let Some(previous) = previous {
            accumulate_shoelace_cross(&mut twice_area, previous, point);
        } else {
            first = Some(point);
        }
        previous = Some(point);
        count = count.saturating_add(1);
    }
    if count < 3 {
        return Orientation::Zero;
    }
    let (Some(previous), Some(first)) = (previous, first) else {
        return Orientation::Zero;
    };
    accumulate_shoelace_cross(&mut twice_area, previous, first);
    Orientation::from_sign(expansion::sign(&twice_area))
}

fn accumulate_shoelace_cross(twice_area: &mut Vec<f64>, point: [f64; 2], next: [f64; 2]) {
    let (product, error) = expansion::two_product(point[0], next[1]);
    let positive = expansion::from_two(product, error);
    let (product, error) = expansion::two_product(point[1], next[0]);
    let negative = expansion::from_two(product, error);
    let cross = expansion::sum(&positive, &expansion::negate(&negative));
    *twice_area = expansion::sum(twice_area, &cross);
}

/// Exact 2D orientation via the identity
/// `det = ax(by - cy) + bx(cy - ay) + cx(ay - by)`.
fn orient2d_exact(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> Orientation {
    let (x, y) = expansion::two_diff(b[1], c[1]);
    let t1 = expansion::scale(&expansion::from_two(x, y), a[0]);
    let (x, y) = expansion::two_diff(c[1], a[1]);
    let t2 = expansion::scale(&expansion::from_two(x, y), b[0]);
    let (x, y) = expansion::two_diff(a[1], b[1]);
    let t3 = expansion::scale(&expansion::from_two(x, y), c[0]);
    let det = expansion::sum(&expansion::sum(&t1, &t2), &t3);
    Orientation::from_sign(expansion::sign(&det))
}

/// Orientation of point `d` relative to the plane through `a`, `b`, `c`.
///
/// Returns the exact sign of
/// `det[[a - d], [b - d], [c - d]]` (rows are 3-vectors):
/// [`Orientation::Positive`] when `d` lies on the side of the plane from
/// which `a`, `b`, `c` appear in clockwise order (equivalently, "below" a
/// counterclockwise triangle).
pub fn orient3d(a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3]) -> Orientation {
    let adx = a[0] - d[0];
    let ady = a[1] - d[1];
    let adz = a[2] - d[2];
    let bdx = b[0] - d[0];
    let bdy = b[1] - d[1];
    let bdz = b[2] - d[2];
    let cdx = c[0] - d[0];
    let cdy = c[1] - d[1];
    let cdz = c[2] - d[2];

    let bdxcdy = bdx * cdy;
    let cdxbdy = cdx * bdy;
    let cdxady = cdx * ady;
    let adxcdy = adx * cdy;
    let adxbdy = adx * bdy;
    let bdxady = bdx * ady;

    let det = adz * (bdxcdy - cdxbdy) + bdz * (cdxady - adxcdy) + cdz * (adxbdy - bdxady);

    let permanent = (bdxcdy.abs() + cdxbdy.abs()) * adz.abs()
        + (cdxady.abs() + adxcdy.abs()) * bdz.abs()
        + (adxbdy.abs() + bdxady.abs()) * cdz.abs();
    let errbound = O3D_ERRBOUND_A * permanent;
    if det > errbound || -det > errbound {
        return Orientation::from_scalar(det);
    }
    orient3d_exact(a, b, c, d)
}

/// Exact 3D orientation: coordinate differences are computed exactly as
/// two-component expansions, then the determinant is expanded with exact
/// expansion algebra.
fn orient3d_exact(a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3]) -> Orientation {
    let diff = |p: f64, q: f64| {
        let (x, y) = expansion::two_diff(p, q);
        expansion::from_two(x, y)
    };
    let adx = diff(a[0], d[0]);
    let ady = diff(a[1], d[1]);
    let adz = diff(a[2], d[2]);
    let bdx = diff(b[0], d[0]);
    let bdy = diff(b[1], d[1]);
    let bdz = diff(b[2], d[2]);
    let cdx = diff(c[0], d[0]);
    let cdy = diff(c[1], d[1]);
    let cdz = diff(c[2], d[2]);

    // (p*q - r*s) * z, all exact.
    let term = |p: &[f64], q: &[f64], r: &[f64], s: &[f64], z: &[f64]| {
        let pq = expansion::mul(p, q);
        let rs = expansion::mul(r, s);
        expansion::mul(&expansion::sum(&pq, &expansion::negate(&rs)), z)
    };
    let t1 = term(&bdx, &cdy, &cdx, &bdy, &adz);
    let t2 = term(&cdx, &ady, &adx, &cdy, &bdz);
    let t3 = term(&adx, &bdy, &bdx, &ady, &cdz);
    let det = expansion::sum(&expansion::sum(&t1, &t2), &t3);
    Orientation::from_sign(expansion::sign(&det))
}

/// Position of point `d` relative to the oriented circumcircle through
/// `a`, `b`, and `c`.
///
/// For counterclockwise `a`, `b`, `c`, returns [`Orientation::Positive`] when
/// `d` lies inside the circumcircle, [`Orientation::Negative`] when it lies
/// outside, and [`Orientation::Zero`] when the four points are cocircular.
/// Reversing the orientation of `a`, `b`, `c` reverses the returned sign.
/// Degenerate inputs retain the exact sign of the same lifted determinant.
pub fn incircle(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> Orientation {
    let adx = a[0] - d[0];
    let ady = a[1] - d[1];
    let bdx = b[0] - d[0];
    let bdy = b[1] - d[1];
    let cdx = c[0] - d[0];
    let cdy = c[1] - d[1];

    let bdxcdy = bdx * cdy;
    let cdxbdy = cdx * bdy;
    let cdxady = cdx * ady;
    let adxcdy = adx * cdy;
    let adxbdy = adx * bdy;
    let bdxady = bdx * ady;

    let alift = adx * adx + ady * ady;
    let blift = bdx * bdx + bdy * bdy;
    let clift = cdx * cdx + cdy * cdy;
    let det = alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy) + clift * (adxbdy - bdxady);

    let permanent = (bdxcdy.abs() + cdxbdy.abs()) * alift
        + (cdxady.abs() + adxcdy.abs()) * blift
        + (adxbdy.abs() + bdxady.abs()) * clift;
    let errbound = ICC_ERRBOUND_A * permanent;
    if det > errbound || -det > errbound {
        return Orientation::from_scalar(det);
    }
    incircle_exact(a, b, c, d)
}

/// Exact 2D in-circle determinant. Coordinate differences, squared lifts,
/// oriented areas, and their products all remain expansions, so no rounded
/// intermediate decides the sign.
fn incircle_exact(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> Orientation {
    let diff = |p: f64, q: f64| {
        let (x, y) = expansion::two_diff(p, q);
        expansion::from_two(x, y)
    };
    let adx = diff(a[0], d[0]);
    let ady = diff(a[1], d[1]);
    let bdx = diff(b[0], d[0]);
    let bdy = diff(b[1], d[1]);
    let cdx = diff(c[0], d[0]);
    let cdy = diff(c[1], d[1]);

    let cross = |p: &[f64], q: &[f64], r: &[f64], s: &[f64]| {
        let pq = expansion::mul(p, q);
        let rs = expansion::mul(r, s);
        expansion::sum(&pq, &expansion::negate(&rs))
    };
    let lift = |x: &[f64], y: &[f64]| expansion::sum(&expansion::mul(x, x), &expansion::mul(y, y));

    let alift = lift(&adx, &ady);
    let blift = lift(&bdx, &bdy);
    let clift = lift(&cdx, &cdy);
    let bcdet = cross(&bdx, &cdy, &cdx, &bdy);
    let cadet = cross(&cdx, &ady, &adx, &cdy);
    let abdet = cross(&adx, &bdy, &bdx, &ady);

    let adet = expansion::mul(&alift, &bcdet);
    let bdet = expansion::mul(&blift, &cadet);
    let cdet = expansion::mul(&clift, &abdet);
    let det = expansion::sum(&expansion::sum(&adet, &bdet), &cdet);
    Orientation::from_sign(expansion::sign(&det))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Deterministic xorshift64 PRNG so tests never depend on external crates
    /// or platform randomness.
    struct Rng(u64);

    impl Rng {
        fn new(seed: u64) -> Self {
            Rng(seed.max(1))
        }
        fn next(&mut self) -> u64 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            self.0 = x;
            x
        }
        /// Uniform integer in `[-bound, bound]`.
        fn int(&mut self, bound: i64) -> i64 {
            let span = (2 * bound + 1) as u64;
            (self.next() % span) as i64 - bound
        }
    }

    /// Exact oracle: integer coordinates make the 2x2 determinant exactly
    /// computable in i128.
    fn orient2d_oracle(a: [i64; 2], b: [i64; 2], c: [i64; 2]) -> i8 {
        let det = (a[0] - c[0]) as i128 * (b[1] - c[1]) as i128
            - (a[1] - c[1]) as i128 * (b[0] - c[0]) as i128;
        det.signum() as i8
    }

    fn orient3d_oracle(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> i8 {
        let v = |p: [i64; 3]| {
            [
                (p[0] - d[0]) as i128,
                (p[1] - d[1]) as i128,
                (p[2] - d[2]) as i128,
            ]
        };
        let (r0, r1, r2) = (v(a), v(b), v(c));
        let det = r0[0] * (r1[1] * r2[2] - r1[2] * r2[1]) - r0[1] * (r1[0] * r2[2] - r1[2] * r2[0])
            + r0[2] * (r1[0] * r2[1] - r1[1] * r2[0]);
        det.signum() as i8
    }

    fn incircle_oracle(a: [i64; 2], b: [i64; 2], c: [i64; 2], d: [i64; 2]) -> i8 {
        let v = |p: [i64; 2]| [(p[0] - d[0]) as i128, (p[1] - d[1]) as i128];
        let (a, b, c) = (v(a), v(b), v(c));
        let cross = |p: [i128; 2], q: [i128; 2]| p[0] * q[1] - q[0] * p[1];
        let lift = |p: [i128; 2]| p[0] * p[0] + p[1] * p[1];
        let det = lift(a) * cross(b, c) + lift(b) * cross(c, a) + lift(c) * cross(a, b);
        det.signum() as i8
    }

    fn polygon_orientation2d_oracle(points: &[[i64; 2]]) -> i8 {
        if points.len() < 3 {
            return 0;
        }
        points
            .iter()
            .zip(points.iter().cycle().skip(1))
            .map(|(point, next)| {
                i128::from(point[0]) * i128::from(next[1])
                    - i128::from(point[1]) * i128::from(next[0])
            })
            .sum::<i128>()
            .signum() as i8
    }

    fn quadratic_discriminant_oracle(a: i128, b: i128, c: i128) -> i8 {
        (b * b - 4 * a * c).signum() as i8
    }

    #[test]
    fn quadratic_discriminant_exactly_classifies_large_cancellation() {
        const M: i128 = 1_i128 << 52;
        let cases = [
            (1_i128 << 51, M + 1, (1_i128 << 51) + 1, 1),
            (1, M, 1_i128 << 102, 0),
            (1, M, (1_i128 << 102) + (1_i128 << 50), -1),
        ];
        for (a, b, c, expected) in cases {
            assert_eq!(quadratic_discriminant_oracle(a, b, c), expected);
            let rounded = (b as f64) * (b as f64) - 4.0 * (a as f64) * (c as f64);
            if expected == 1 {
                assert_eq!(rounded, 0.0);
            }
            let classified = quadratic_discriminant(a as f64, b as f64, c as f64).unwrap();
            assert_eq!(classified.sign().as_i8(), expected);
            assert!(classified.used_exact_fallback());
            assert_eq!(
                Orientation::from_scalar(classified.approximation()).as_i8(),
                expected
            );
        }
    }

    #[test]
    fn quadratic_discriminant_retains_stable_root_magnitude_and_scale_sign() {
        const M: i128 = 1_i128 << 52;
        const C: i128 = (1_i128 << 102) + (1_i128 << 51);
        assert_eq!(quadratic_discriminant_oracle(1, M + 1, C), 1);
        let classified = quadratic_discriminant(1.0, (M + 1) as f64, C as f64).unwrap();
        assert_eq!(classified.sign(), Orientation::Positive);
        assert_eq!(classified.approximation(), 1.0);
        assert!(classified.used_exact_fallback());

        for scale in [2.0_f64.powi(-20), 2.0_f64.powi(20), -1.0] {
            let scaled =
                quadratic_discriminant(scale, scale * (M + 1) as f64, scale * C as f64).unwrap();
            assert_eq!(scaled.sign(), Orientation::Positive);
            assert!(scaled.used_exact_fallback());
            assert!(scaled.approximation() > 0.0);
        }
    }

    #[test]
    fn quadratic_discriminant_filter_and_failure_contracts() {
        let ordinary = quadratic_discriminant(1.0, -3.0, 2.0).unwrap();
        assert_eq!(ordinary.sign(), Orientation::Positive);
        assert_eq!(ordinary.approximation().to_bits(), 1.0_f64.to_bits());
        assert!(!ordinary.used_exact_fallback());

        for coefficients in [
            [f64::NAN, 1.0, 1.0],
            [1.0, f64::INFINITY, 1.0],
            [1.0, 1.0, f64::NEG_INFINITY],
            [f64::MAX, f64::MAX, f64::MAX],
            [0.0, f64::from_bits(1), 0.0],
            [2.0_f64.powi(-600), 0.0, 2.0_f64.powi(-600)],
        ] {
            assert_eq!(
                quadratic_discriminant(coefficients[0], coefficients[1], coefficients[2]),
                None
            );
        }
    }

    #[test]
    fn quadratic_discriminant_exact_product_envelope_stays_normal() {
        let low_a = 2.0_f64.powi(-250) * (1.0 + f64::EPSILON);
        let low_b = 2.0_f64.powi(-150) * (1.0 + f64::EPSILON);
        let high_a = 2.0_f64.powi(249) * (1.0 + f64::EPSILON);
        let high_b = 2.0_f64.powi(150) * (1.0 + f64::EPSILON);

        for (a, b) in [
            (EXACT_COMPONENT_MIN, 2.0_f64.powi(100)),
            (EXACT_COMPONENT_MAX, 2.0_f64.powi(-100)),
            (low_a, low_b),
            (high_a, high_b),
        ] {
            let product = exact_product_for_discriminant(a, b).unwrap();
            assert!(exact_components_are_normal(&product));
            let four_product = expansion::scale(&product, 4.0);
            assert!(exact_components_are_normal(&four_product));
        }

        let low_residue = exact_product_for_discriminant(low_a, low_b).unwrap();
        let high_residue = exact_product_for_discriminant(high_a, high_b).unwrap();
        assert_eq!(low_residue.len(), 2);
        assert_eq!(high_residue.len(), 2);
        assert!(low_residue[0].is_normal());
        assert!(high_residue[0].is_normal());

        for (a, b) in [
            (2.0_f64.powi(-501), 2.0_f64.powi(101)),
            (2.0_f64.powi(501), 2.0_f64.powi(-101)),
            (2.0_f64.powi(-200), 2.0_f64.powi(-201)),
            (2.0_f64.powi(200), 2.0_f64.powi(201)),
        ] {
            assert_eq!(exact_product_for_discriminant(a, b), None);
        }
    }

    #[test]
    fn harmonic_half_angle_degree_and_root_count_are_exact() {
        let secant = harmonic_half_angle_roots(1.0, 0.0, 0.991).unwrap();
        assert_eq!(secant.discriminant(), Orientation::Positive);
        assert_eq!(secant.finite_roots().len(), 2);
        assert!(!secant.has_infinity_root());
        for &root in secant.finite_roots() {
            let angle = 2.0 * crate::math::atan2(root, 1.0);
            assert!((crate::math::cos(angle) + 0.991).abs() < 2.0e-15);
        }
        let first = 2.0 * crate::math::atan2(secant.finite_roots()[0], 1.0);
        let second = 2.0 * crate::math::atan2(secant.finite_roots()[1], 1.0);
        assert!((first - second).abs() > 0.25);

        let tangent = harmonic_half_angle_roots(1.0, 0.0, 1.0).unwrap();
        assert_eq!(tangent.discriminant(), Orientation::Zero);
        assert!(tangent.finite_roots().is_empty());
        assert!(tangent.has_infinity_root());

        let projective_linear = harmonic_half_angle_roots(1.0, 0.5, 1.0).unwrap();
        assert_eq!(projective_linear.discriminant(), Orientation::Positive);
        assert_eq!(projective_linear.finite_roots(), &[-2.0]);
        assert!(projective_linear.has_infinity_root());

        let miss = harmonic_half_angle_roots(1.0, 0.0, 1.001).unwrap();
        assert_eq!(miss.discriminant(), Orientation::Negative);
        assert!(miss.finite_roots().is_empty());
        assert!(!miss.has_infinity_root());
    }

    #[test]
    fn harmonic_half_angle_normalization_preserves_ordinary_bits() {
        let expected = [1.0_f64.to_bits(), 2.0_f64.to_bits()];
        for scale in [1.0, 2.0_f64.powi(700), 2.0_f64.powi(-700)] {
            let roots = harmonic_half_angle_roots(0.5 * scale, -1.5 * scale, 1.5 * scale).unwrap();
            assert_eq!(roots.discriminant(), Orientation::Positive);
            assert_eq!(
                roots
                    .finite_roots()
                    .iter()
                    .map(|root| root.to_bits())
                    .collect::<Vec<_>>(),
                expected
            );
        }

        let reversed = harmonic_half_angle_roots(-0.5, 1.5, -1.5).unwrap();
        assert_eq!(reversed.finite_roots(), &[2.0, 1.0]);

        let identity = harmonic_half_angle_roots(0.0, 0.0, 0.0).unwrap();
        assert!(identity.is_identity());
        assert!(identity.finite_roots().is_empty());
        assert!(!identity.has_infinity_root());

        assert_eq!(
            harmonic_half_angle_roots(f64::MAX, f64::from_bits(1), 0.0),
            None
        );
    }

    #[test]
    fn harmonic_half_angle_routes_large_cancellation_to_exact_discriminant() {
        const M: i128 = 1_i128 << 52;
        let q2 = (M - 1) as f64;
        let q1 = (2 * M) as f64;
        let q0 = (M + 1) as f64;
        assert_eq!((2 * M) * (2 * M) - 4 * (M - 1) * (M + 1), 4);
        assert_eq!(q1 * q1 - 4.0 * q2 * q0, 0.0);

        let roots = harmonic_half_angle_roots(1.0, M as f64, M as f64).unwrap();
        assert_eq!(roots.discriminant(), Orientation::Positive);
        assert!(roots.used_exact_fallback());
        assert_eq!(roots.finite_roots().len(), 2);
        assert_ne!(
            roots.finite_roots()[0].to_bits(),
            roots.finite_roots()[1].to_bits()
        );
        assert_eq!(roots.finite_roots()[1], -1.0);
        let expected = roots
            .finite_roots()
            .iter()
            .map(|root| root.to_bits())
            .collect::<Vec<_>>();
        for scale in [2.0_f64.powi(700), 2.0_f64.powi(-700)] {
            let scaled =
                harmonic_half_angle_roots(scale, (M as f64) * scale, (M as f64) * scale).unwrap();
            assert!(scaled.used_exact_fallback());
            assert_eq!(
                scaled
                    .finite_roots()
                    .iter()
                    .map(|root| root.to_bits())
                    .collect::<Vec<_>>(),
                expected
            );
        }
    }

    #[test]
    fn harmonic_sign_precedes_rounded_half_angle_coefficient_sign() {
        let cosine = f64::from_bits(0x3fe4_babb_6e9f_fd16);
        let sine = f64::from_bits(0x3f94_45bb_238f_5480);
        let constant = f64::from_bits(0x3fe4_bd35_b29f_bd7d);
        let q2 = constant - cosine;
        let q1 = 2.0 * sine;
        let q0 = cosine + constant;
        assert_eq!(
            quadratic_discriminant(q2, q1, q0).unwrap().sign(),
            Orientation::Negative,
            "rounded half-angle coefficients exhibit the wrong exact sign"
        );

        let exact = harmonic_norm_difference(cosine, sine, constant).unwrap();
        assert_eq!(exact.sign, Orientation::Positive);
        assert!(exact.used_exact_fallback);
        let roots = harmonic_half_angle_roots(cosine, sine, constant).unwrap();
        assert_eq!(roots.discriminant(), Orientation::Positive);
        assert_eq!(roots.finite_roots().len(), 2);
        for &root in roots.finite_roots() {
            let angle = 2.0 * crate::math::atan2(root, 1.0);
            let residual =
                cosine * crate::math::cos(angle) + sine * crate::math::sin(angle) + constant;
            assert!(residual.abs() < 2.0e-15, "residual {residual:e}");
        }
    }

    fn to2(p: [i64; 2]) -> [f64; 2] {
        [p[0] as f64, p[1] as f64]
    }
    fn to3(p: [i64; 3]) -> [f64; 3] {
        [p[0] as f64, p[1] as f64, p[2] as f64]
    }

    #[test]
    fn orient2d_matches_oracle_on_random_points() {
        let mut rng = Rng::new(0x9E37_79B9_7F4A_7C15);
        for _ in 0..20_000 {
            let a = [rng.int(1 << 20), rng.int(1 << 20)];
            let b = [rng.int(1 << 20), rng.int(1 << 20)];
            let c = [rng.int(1 << 20), rng.int(1 << 20)];
            assert_eq!(
                orient2d(to2(a), to2(b), to2(c)).as_i8(),
                orient2d_oracle(a, b, c),
                "a={a:?} b={b:?} c={c:?}"
            );
        }
    }

    #[test]
    fn orient2d_matches_oracle_near_degeneracy() {
        // Collinear triples with unit-scale perturbations: large coordinates,
        // tiny (or zero) determinants — exactly where the filter must punt to
        // the exact path and the exact path must be right.
        let mut rng = Rng::new(0xDEAD_BEEF_CAFE_F00D);
        for _ in 0..20_000 {
            let a = [rng.int(1 << 20), rng.int(1 << 20)];
            let dir = [rng.int(1 << 10), rng.int(1 << 10)];
            let b = [a[0] + dir[0] * rng.int(8), a[1] + dir[1] * rng.int(8)];
            let mut c = [a[0] + dir[0] * rng.int(8), a[1] + dir[1] * rng.int(8)];
            c[0] += rng.int(1);
            c[1] += rng.int(1);
            assert_eq!(
                orient2d(to2(a), to2(b), to2(c)).as_i8(),
                orient2d_oracle(a, b, c),
                "a={a:?} b={b:?} c={c:?}"
            );
        }
    }

    #[test]
    fn polygon_orientation2d_matches_random_integer_oracle() {
        let mut rng = Rng::new(0xA11C_E5E7_0ACE_2D00);
        for _ in 0..20_000 {
            let len = 3 + (rng.next() % 10) as usize;
            let points = (0..len)
                .map(|_| [rng.int(1 << 20), rng.int(1 << 20)])
                .collect::<Vec<_>>();
            let coordinates = points.iter().copied().map(to2).collect::<Vec<_>>();
            assert_eq!(
                polygon_orientation2d(&coordinates).as_i8(),
                polygon_orientation2d_oracle(&points),
                "points={points:?}"
            );
        }
    }

    #[test]
    fn polygon_orientation2d_resolves_large_cancellation_deterministically() {
        let magnitude = 2f64.powi(52);
        let points = vec![
            [magnitude, magnitude],
            [magnitude + 1.0, magnitude],
            [magnitude + 1.0, magnitude + 1.0],
            [magnitude, magnitude + 1.0],
        ];
        let naive_twice_area = points
            .iter()
            .zip(points.iter().cycle().skip(1))
            .map(|(point, next)| point[0] * next[1] - point[1] * next[0])
            .sum::<f64>();
        assert_eq!(naive_twice_area, 0.0);

        for rotation in 0..points.len() {
            let mut rotated = points.clone();
            rotated.rotate_left(rotation);
            for _ in 0..32 {
                assert_eq!(polygon_orientation2d(&rotated), Orientation::Positive);
                assert_eq!(
                    polygon_orientation2d_iter(rotated.iter().copied()),
                    Orientation::Positive
                );
            }

            rotated.reverse();
            assert_eq!(polygon_orientation2d(&rotated), Orientation::Negative);
        }
    }

    #[test]
    fn polygon_orientation2d_fails_closed_on_invalid_or_exact_zero_input() {
        assert_eq!(polygon_orientation2d(&[]), Orientation::Zero);
        assert_eq!(
            polygon_orientation2d(&[[0.0, 0.0], [1.0, 0.0]]),
            Orientation::Zero
        );
        assert_eq!(
            polygon_orientation2d(&[[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]]),
            Orientation::Zero
        );
        assert_eq!(
            polygon_orientation2d(&[[0.0, 0.0], [f64::NAN, 1.0], [1.0, 0.0]]),
            Orientation::Zero
        );
        assert_eq!(
            polygon_orientation2d(&[[0.0, 0.0], [f64::INFINITY, 1.0], [1.0, 0.0]]),
            Orientation::Zero
        );
    }

    #[test]
    fn orient3d_matches_oracle_on_random_points() {
        let mut rng = Rng::new(0x0123_4567_89AB_CDEF);
        for _ in 0..20_000 {
            let p = |rng: &mut Rng| [rng.int(1 << 18), rng.int(1 << 18), rng.int(1 << 18)];
            let (a, b, c, d) = (p(&mut rng), p(&mut rng), p(&mut rng), p(&mut rng));
            assert_eq!(
                orient3d(to3(a), to3(b), to3(c), to3(d)).as_i8(),
                orient3d_oracle(a, b, c, d),
                "a={a:?} b={b:?} c={c:?} d={d:?}"
            );
        }
    }

    #[test]
    fn orient3d_matches_oracle_near_coplanarity() {
        // d starts in the plane spanned by (a, b, c) via integer combinations,
        // then gets a perturbation in {-1, 0, 1}^3.
        let mut rng = Rng::new(0xFEED_FACE_0BAD_F00D);
        for _ in 0..20_000 {
            let p = |rng: &mut Rng| [rng.int(1 << 16), rng.int(1 << 16), rng.int(1 << 16)];
            let (a, b, c) = (p(&mut rng), p(&mut rng), p(&mut rng));
            let (u, v) = (rng.int(4), rng.int(4));
            let mut d = [0_i64; 3];
            for k in 0..3 {
                d[k] = a[k] + u * (b[k] - a[k]) + v * (c[k] - a[k]) + rng.int(1);
            }
            assert_eq!(
                orient3d(to3(a), to3(b), to3(c), to3(d)).as_i8(),
                orient3d_oracle(a, b, c, d),
                "a={a:?} b={b:?} c={c:?} d={d:?}"
            );
        }
    }

    #[test]
    fn incircle_matches_oracle_on_random_integer_points() {
        let mut rng = Rng::new(0xC1AC_1E00_51A7_0BAD);
        for _ in 0..20_000 {
            let p = |rng: &mut Rng| [rng.int(1 << 15), rng.int(1 << 15)];
            let (a, b, c, d) = (p(&mut rng), p(&mut rng), p(&mut rng), p(&mut rng));
            assert_eq!(
                incircle(to2(a), to2(b), to2(c), to2(d)).as_i8(),
                incircle_oracle(a, b, c, d),
                "a={a:?} b={b:?} c={c:?} d={d:?}"
            );
        }
    }

    #[test]
    fn incircle_sign_convention_and_permutations() {
        let a = [0.0, 0.0];
        let b = [4.0, 0.0];
        let c = [0.0, 4.0];
        let inside = [1.0, 1.0];
        let outside = [5.0, 5.0];
        let boundary = [4.0, 4.0];

        assert_eq!(orient2d(a, b, c), Orientation::Positive);
        for ([a, b, c], winding) in [
            ([a, b, c], Orientation::Positive),
            ([b, c, a], Orientation::Positive),
            ([c, a, b], Orientation::Positive),
            ([a, c, b], Orientation::Negative),
            ([c, b, a], Orientation::Negative),
            ([b, a, c], Orientation::Negative),
        ] {
            let opposite = match winding {
                Orientation::Positive => Orientation::Negative,
                Orientation::Negative => Orientation::Positive,
                Orientation::Zero => unreachable!("the fixture triangle is not degenerate"),
            };
            assert_eq!(orient2d(a, b, c), winding);
            assert_eq!(incircle(a, b, c, inside), winding);
            assert_eq!(incircle(a, b, c, outside), opposite);
            assert_eq!(incircle(a, b, c, boundary), Orientation::Zero);
        }
    }

    #[test]
    fn incircle_exact_fallback_resolves_near_cocircular_points() {
        // At this scale, moving the fourth point by one changes the exact
        // determinant, but the stage-A bound deliberately cannot certify the
        // rounded determinant. The full expansion path must retain the sign.
        let radius = 2f64.powi(51);
        let a = [radius, 0.0];
        let b = [0.0, radius];
        let c = [-radius, 0.0];
        let stage_a_is_uncertain = |d: [f64; 2]| {
            let adx = a[0] - d[0];
            let ady = a[1] - d[1];
            let bdx = b[0] - d[0];
            let bdy = b[1] - d[1];
            let cdx = c[0] - d[0];
            let cdy = c[1] - d[1];
            let bdxcdy = bdx * cdy;
            let cdxbdy = cdx * bdy;
            let cdxady = cdx * ady;
            let adxcdy = adx * cdy;
            let adxbdy = adx * bdy;
            let bdxady = bdx * ady;
            let alift = adx * adx + ady * ady;
            let blift = bdx * bdx + bdy * bdy;
            let clift = cdx * cdx + cdy * cdy;
            let det =
                alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy) + clift * (adxbdy - bdxady);
            let permanent = (bdxcdy.abs() + cdxbdy.abs()) * alift
                + (cdxady.abs() + adxcdy.abs()) * blift
                + (adxbdy.abs() + bdxady.abs()) * clift;
            det.abs() <= ICC_ERRBOUND_A * permanent
        };
        for d in [[0.0, -radius + 1.0], [0.0, -radius], [0.0, -radius - 1.0]] {
            assert!(
                stage_a_is_uncertain(d),
                "fixture must exercise exact fallback"
            );
        }
        assert_eq!(
            incircle(a, b, c, [0.0, -radius + 1.0]),
            Orientation::Positive
        );
        assert_eq!(incircle(a, b, c, [0.0, -radius]), Orientation::Zero);
        assert_eq!(
            incircle(a, b, c, [0.0, -radius - 1.0]),
            Orientation::Negative
        );
    }

    #[test]
    fn incircle_degenerate_and_nonfinite_inputs_do_not_panic() {
        let a = [0.0, 0.0];
        let b = [1.0, 0.0];
        let d = [0.0, 1.0];
        assert_eq!(incircle(a, a, b, d), Orientation::Zero);
        assert_eq!(incircle(a, b, [2.0, 0.0], [3.0, 0.0]), Orientation::Zero);
        // A collinear defining triple has no geometric circumcircle, but the
        // predicate still returns the exact sign of its lifted determinant.
        assert_eq!(incircle(a, b, [2.0, 0.0], d), Orientation::Positive);
        assert_eq!(incircle(a, [2.0, 0.0], b, d), Orientation::Negative);

        // Like the existing orientation predicates, invalid non-finite input
        // is not ordered and therefore maps to the neutral sign without a
        // panic; callers validate finite geometry at their API boundary.
        assert_eq!(incircle([f64::NAN, 0.0], a, b, d), Orientation::Zero);
        assert_eq!(incircle([f64::INFINITY, 0.0], a, b, d), Orientation::Zero);
        assert_eq!(orient2d([f64::NAN, 0.0], a, b), Orientation::Zero);
        assert_eq!(
            orient3d(
                [f64::INFINITY, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0]
            ),
            Orientation::Zero
        );
    }

    #[test]
    fn orient2d_exact_zero_on_collinear() {
        assert_eq!(
            orient2d([0.0, 0.0], [1e10, 1e10], [2e10, 2e10]),
            Orientation::Zero
        );
    }

    #[test]
    fn orient3d_sign_convention() {
        // Unit triangle in z = 0, counterclockwise seen from +z;
        // d below the plane (negative z) must be Positive.
        let a = [0.0, 0.0, 0.0];
        let b = [1.0, 0.0, 0.0];
        let c = [0.0, 1.0, 0.0];
        assert_eq!(orient3d(a, b, c, [0.3, 0.3, -1.0]), Orientation::Positive);
        assert_eq!(orient3d(a, b, c, [0.3, 0.3, 1.0]), Orientation::Negative);
        assert_eq!(orient3d(a, b, c, [5.0, -3.0, 0.0]), Orientation::Zero);
    }
}
