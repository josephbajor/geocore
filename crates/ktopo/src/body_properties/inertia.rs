//! Fixed-degree second-moment forms used by certified body properties.

use super::{Interval, Laurent, Poly, finite_interval, interval_ratio};

pub(super) const SYMMETRIC_COMPONENTS: [(usize, usize); 6] =
    [(0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2)];

pub(super) fn plane_poly_primitives(
    u: Poly,
    v: Poly,
    origin: [Interval; 3],
    x: [Interval; 3],
    y: [Interval; 3],
) -> [Poly; 6] {
    let zero = Interval::point(0.0);
    let base: [Poly; 3] = core::array::from_fn(|coordinate| {
        Poly::affine_interval(origin[coordinate], zero).add(u.scale(x[coordinate]))
    });
    let v2 = v.mul(v);
    let v3 = v2.mul(v);
    core::array::from_fn(|component| {
        let (left, right) = SYMMETRIC_COMPONENTS[component];
        base[left]
            .mul(base[right])
            .mul(v)
            .add(
                base[left]
                    .scale(y[right])
                    .add(base[right].scale(y[left]))
                    .mul(v2)
                    .scale(interval_ratio(1.0, 2.0)),
            )
            .add(v3.scale(y[left] * y[right] * interval_ratio(1.0, 3.0)))
    })
}

pub(super) fn plane_laurent_primitives(
    u: Laurent,
    v: Laurent,
    origin: [Interval; 3],
    x: [Interval; 3],
    y: [Interval; 3],
) -> [Laurent; 6] {
    let base: [Laurent; 3] = core::array::from_fn(|coordinate| {
        Laurent::constant(origin[coordinate]).add(u.scale_interval(x[coordinate]))
    });
    let v2 = v.mul(v);
    let v3 = v2.mul(v);
    core::array::from_fn(|component| {
        let (left, right) = SYMMETRIC_COMPONENTS[component];
        base[left]
            .mul(base[right])
            .mul(v)
            .add(
                base[left]
                    .scale_interval(y[right])
                    .add(base[right].scale_interval(y[left]))
                    .mul(v2)
                    .scale_interval(interval_ratio(1.0, 2.0)),
            )
            .add(v3.scale_interval(y[left] * y[right] * interval_ratio(1.0, 3.0)))
    })
}

#[derive(Debug, Clone, Copy)]
pub(super) struct CylinderSecondTerms {
    pub(super) linear: Laurent,
    pub(super) quadratic: Laurent,
    pub(super) cubic: Laurent,
}

pub(super) fn cylinder_second_terms(
    h: Laurent,
    base: [Laurent; 3],
    axis: [Interval; 3],
) -> [CylinderSecondTerms; 6] {
    core::array::from_fn(|component| {
        let (left, right) = SYMMETRIC_COMPONENTS[component];
        CylinderSecondTerms {
            linear: h.mul(base[left]).mul(base[right]),
            quadratic: h
                .mul(
                    base[left]
                        .scale_interval(axis[right])
                        .add(base[right].scale_interval(axis[left])),
                )
                .scale_interval(interval_ratio(1.0, 2.0)),
            cubic: h.scale_interval(axis[left] * axis[right] * interval_ratio(1.0, 3.0)),
        }
    })
}

pub(super) fn centroidal_inertia(
    volume: Interval,
    moment: [Interval; 3],
    second_moment: [Interval; 6],
) -> Option<[Interval; 6]> {
    let mut covariance = [Interval::point(0.0); 6];
    for component in 0..6 {
        let (left, right) = SYMMETRIC_COMPONENTS[component];
        let product = if left == right {
            moment[left].square()
        } else {
            moment[left] * moment[right]
        };
        covariance[component] = second_moment[component] - product.checked_div(volume)?;
    }
    if !covariance.into_iter().all(finite_interval) {
        return None;
    }
    let result = [
        covariance[1] + covariance[2],
        covariance[0] + covariance[2],
        covariance[0] + covariance[1],
        -covariance[3],
        -covariance[4],
        -covariance[5],
    ];
    result.into_iter().all(finite_interval).then_some(result)
}
