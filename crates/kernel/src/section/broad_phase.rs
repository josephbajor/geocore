//! Certified face envelopes for section-pair rejection.
//!
//! A face domain is a conservative parameter-space superset of its trim.
//! For the analytic classes used by the block/cylinder rung, interval
//! evaluation of that domain therefore gives a conservative model-space
//! envelope.  These envelopes may prove a pair empty before a trim class is
//! admitted; failure to construct one is never evidence of intersection.

use kcore::interval::Interval;
use kcore::operation::OperationScope;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Plane};
use ktopo::entity::FaceId as RawFaceId;
use ktopo::geom::SurfaceGeom;
use ktopo::store::Store;

use crate::error::Result;

use super::{charge, read};

/// One source face and its optional certified model-space envelope.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct FaceEnvelope {
    pub raw: RawFaceId,
    pub class: FaceSurfaceClass,
    pub bounds: Option<[Interval; 3]>,
    support: Option<FaceSupport>,
}

/// Source-derived support function for the two analytic domain classes.
/// Candidate axes come from the authored frames, so rigid transforms do not
/// weaken pair rejection to a world-axis AABB accident.
#[derive(Debug, Clone, Copy, PartialEq)]
enum FaceSupport {
    Plane {
        origin: [f64; 3],
        x: [f64; 3],
        y: [f64; 3],
        axes: [[f64; 3]; 3],
        u: ParamRange,
        v: ParamRange,
    },
    FullCylinder {
        origin: [f64; 3],
        x: [f64; 3],
        y: [f64; 3],
        z: [f64; 3],
        axes: [[f64; 3]; 3],
        radius: f64,
        v: ParamRange,
    },
}

impl FaceSupport {
    const fn axes(self) -> [[f64; 3]; 3] {
        match self {
            Self::Plane { axes, .. } | Self::FullCylinder { axes, .. } => axes,
        }
    }

    fn project(self, axis: [f64; 3]) -> Option<Interval> {
        match self {
            Self::Plane {
                origin, x, y, u, v, ..
            } => Some(
                interval_dot(axis, origin)
                    + interval_dot(axis, x) * Interval::new(u.lo, u.hi)
                    + interval_dot(axis, y) * Interval::new(v.lo, v.hi),
            ),
            Self::FullCylinder {
                origin,
                x,
                y,
                z,
                radius,
                v,
                ..
            } => {
                let x = interval_dot(axis, x);
                let y = interval_dot(axis, y);
                let amplitude = (x.square() + y.square()).sqrt()? * Interval::point(radius);
                let center =
                    interval_dot(axis, origin) + interval_dot(axis, z) * Interval::new(v.lo, v.hi);
                Some(center + Interval::new(-amplitude.hi(), amplitude.hi()))
            }
        }
        .filter(|bound| bound.lo().is_finite() && bound.hi().is_finite())
    }
}

/// Analytic surface class relevant to the current section dispatcher.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum FaceSurfaceClass {
    Plane,
    Cylinder,
    Other,
}

/// Build envelopes in stored face order.
pub(crate) fn prepare_face_envelopes(
    store: &Store,
    faces: &[RawFaceId],
    scope: &mut OperationScope<'_, '_>,
) -> Result<Vec<FaceEnvelope>> {
    let mut prepared = Vec::with_capacity(faces.len());
    for &raw in faces {
        charge(scope, 1)?;
        let face = read(store.get(raw))?;
        let surface = read(store.surface(face.surface))?;
        let class = match surface {
            SurfaceGeom::Plane(_) => FaceSurfaceClass::Plane,
            SurfaceGeom::Cylinder(_) => FaceSurfaceClass::Cylinder,
            _ => FaceSurfaceClass::Other,
        };
        let bounds = match (surface, face.domain()) {
            (SurfaceGeom::Plane(plane), Some(domain)) => {
                plane_domain_bounds(plane, domain.u, domain.v, scope)?
            }
            (SurfaceGeom::Cylinder(cylinder), Some(domain))
                if domain.u.width() == core::f64::consts::TAU =>
            {
                full_cylinder_domain_bounds(cylinder, domain.v, scope)?
            }
            _ => None,
        };
        let support = match (surface, face.domain()) {
            (SurfaceGeom::Plane(plane), Some(domain)) => plane_support(plane, domain.u, domain.v),
            (SurfaceGeom::Cylinder(cylinder), Some(domain))
                if domain.u.width() == core::f64::consts::TAU =>
            {
                full_cylinder_support(cylinder, domain.v)
            }
            _ => None,
        };
        prepared.push(FaceEnvelope {
            raw,
            class,
            bounds,
            support,
        });
    }
    Ok(prepared)
}

/// Prove that two conservative envelopes remain separated after the session
/// linear tolerance is applied to both.  Missing/non-finite evidence fails
/// closed and returns `false`.
pub(crate) fn certifiably_disjoint(a: FaceEnvelope, b: FaceEnvelope, linear: f64) -> bool {
    if !linear.is_finite() || linear < 0.0 {
        return false;
    }
    let (Some(a_bounds), Some(b_bounds)) = (a.bounds, b.bounds) else {
        return false;
    };
    let pad = Interval::new(-linear, linear);
    if (0..3).any(|axis| {
        let a = a_bounds[axis] + pad;
        let b = b_bounds[axis] + pad;
        a.hi() < b.lo() || b.hi() < a.lo()
    }) {
        return true;
    }
    let (Some(a_support), Some(b_support)) = (a.support, b.support) else {
        return false;
    };
    a_support
        .axes()
        .into_iter()
        .chain(b_support.axes())
        .any(|axis| {
            let (Some(a), Some(b)) = (a_support.project(axis), b_support.project(axis)) else {
                return false;
            };
            let a = a + pad;
            let b = b + pad;
            a.hi() < b.lo() || b.hi() < a.lo()
        })
}

fn plane_support(plane: &Plane, u: ParamRange, v: ParamRange) -> Option<FaceSupport> {
    if !finite_range(u) || !finite_range(v) {
        return None;
    }
    let frame = plane.frame();
    Some(FaceSupport::Plane {
        origin: frame.origin().to_array(),
        x: frame.x().to_array(),
        y: frame.y().to_array(),
        axes: [
            frame.x().to_array(),
            frame.y().to_array(),
            frame.z().to_array(),
        ],
        u,
        v,
    })
}

fn full_cylinder_support(cylinder: &Cylinder, v: ParamRange) -> Option<FaceSupport> {
    if !finite_range(v) || !cylinder.radius().is_finite() || cylinder.radius() <= 0.0 {
        return None;
    }
    let frame = cylinder.frame();
    Some(FaceSupport::FullCylinder {
        origin: frame.origin().to_array(),
        x: frame.x().to_array(),
        y: frame.y().to_array(),
        z: frame.z().to_array(),
        axes: [
            frame.x().to_array(),
            frame.y().to_array(),
            frame.z().to_array(),
        ],
        radius: cylinder.radius(),
        v,
    })
}

fn interval_dot(a: [f64; 3], b: [f64; 3]) -> Interval {
    let mut dot = Interval::point(0.0);
    for axis in 0..3 {
        dot = dot + Interval::point(a[axis]) * Interval::point(b[axis]);
    }
    dot
}

fn plane_domain_bounds(
    plane: &Plane,
    u: ParamRange,
    v: ParamRange,
    scope: &mut OperationScope<'_, '_>,
) -> Result<Option<[Interval; 3]>> {
    if !finite_range(u) || !finite_range(v) {
        return Ok(None);
    }
    charge(scope, 3)?;
    let frame = plane.frame();
    let u = Interval::new(u.lo, u.hi);
    let v = Interval::new(v.lo, v.hi);
    let bounds = (0..3).map(|axis| {
        Interval::point(component(frame.origin().to_array(), axis))
            + Interval::point(component(frame.x().to_array(), axis)) * u
            + Interval::point(component(frame.y().to_array(), axis)) * v
    });
    finite_bounds(bounds)
}

fn full_cylinder_domain_bounds(
    cylinder: &Cylinder,
    v: ParamRange,
    scope: &mut OperationScope<'_, '_>,
) -> Result<Option<[Interval; 3]>> {
    if !finite_range(v) {
        return Ok(None);
    }
    charge(scope, 3)?;
    let frame = cylinder.frame();
    let v = Interval::new(v.lo, v.hi);
    let radius = Interval::point(cylinder.radius());
    let bounds = (0..3).map(|axis| {
        let x = Interval::point(component(frame.x().to_array(), axis));
        let y = Interval::point(component(frame.y().to_array(), axis));
        let radial = (x.square() + y.square()).sqrt()? * radius;
        let center = Interval::point(component(frame.origin().to_array(), axis))
            + Interval::point(component(frame.z().to_array(), axis)) * v;
        Some(center + Interval::new(-radial.hi(), radial.hi()))
    });
    let Some(bounds) = bounds.collect::<Option<Vec<_>>>() else {
        return Ok(None);
    };
    finite_bounds(bounds.into_iter())
}

fn finite_bounds(mut bounds: impl Iterator<Item = Interval>) -> Result<Option<[Interval; 3]>> {
    let Some(x) = bounds.next() else {
        return Ok(None);
    };
    let Some(y) = bounds.next() else {
        return Ok(None);
    };
    let Some(z) = bounds.next() else {
        return Ok(None);
    };
    if bounds.next().is_some()
        || [x, y, z]
            .iter()
            .any(|value| !value.lo().is_finite() || !value.hi().is_finite())
    {
        return Ok(None);
    }
    Ok(Some([x, y, z]))
}

const fn finite_range(range: ParamRange) -> bool {
    range.lo.is_finite() && range.hi.is_finite() && range.lo < range.hi
}

const fn component(values: [f64; 3], axis: usize) -> f64 {
    values[axis]
}

#[cfg(test)]
mod tests {
    use kcore::operation::{OperationContext, OperationScope, SessionPolicy};
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;
    use kgeom::surface::{Cylinder, Plane};
    use kgeom::vec::{Point3, Vec3};

    use super::*;
    use crate::section::BodySectionBudgetProfile;

    fn with_scope<T>(f: impl FnOnce(&mut OperationScope<'_, '_>) -> T) -> T {
        let policy = SessionPolicy::v1();
        let context = OperationContext::new(&policy, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(BodySectionBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        f(&mut scope)
    }

    #[test]
    fn tilted_plane_and_full_cylinder_envelopes_contain_independent_samples() {
        let frame = Frame::new(
            Point3::new(1.0, -2.0, 0.5),
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(2.0, -1.0, 0.0),
        )
        .unwrap();
        let plane = Plane::new(frame);
        let cylinder = Cylinder::new(frame, 0.75).unwrap();
        with_scope(|scope| {
            let plane_bounds = plane_domain_bounds(
                &plane,
                ParamRange::new(-2.0, 1.0),
                ParamRange::new(-1.0, 3.0),
                scope,
            )
            .unwrap()
            .unwrap();
            for uv in [[-2.0, -1.0], [-0.5, 1.25], [1.0, 3.0]] {
                let p = kgeom::surface::Surface::eval(&plane, uv).to_array();
                assert!((0..3).all(|axis| plane_bounds[axis].contains(p[axis])));
            }

            let cylinder_bounds =
                full_cylinder_domain_bounds(&cylinder, ParamRange::new(-1.0, 2.5), scope)
                    .unwrap()
                    .unwrap();
            for uv in [
                [0.0, -1.0],
                [core::f64::consts::FRAC_PI_2, 0.25],
                [core::f64::consts::PI, 2.5],
                [1.75 * core::f64::consts::PI, 1.0],
            ] {
                let p = kgeom::surface::Surface::eval(&cylinder, uv).to_array();
                assert!((0..3).all(|axis| cylinder_bounds[axis].contains(p[axis])));
            }
        });
    }

    #[test]
    fn disjoint_requires_a_certified_gap_beyond_both_tolerance_pads() {
        let a = FaceEnvelope {
            raw: raw_face(),
            class: FaceSurfaceClass::Plane,
            bounds: Some([
                Interval::point(0.0),
                Interval::new(-1.0, 1.0),
                Interval::new(-1.0, 1.0),
            ]),
            support: None,
        };
        let mut b = a;
        b.bounds.as_mut().unwrap()[0] = Interval::point(3.0);
        assert!(certifiably_disjoint(a, b, 1.0));
        b.bounds.as_mut().unwrap()[0] = Interval::point(2.0);
        assert!(!certifiably_disjoint(a, b, 1.0));
        assert!(!certifiably_disjoint(
            FaceEnvelope {
                raw: a.raw,
                class: a.class,
                bounds: None,
                support: None,
            },
            b,
            0.0,
        ));
    }

    fn raw_face() -> RawFaceId {
        let mut store = Store::new();
        let body = ktopo::make::block(&mut store, &Frame::world(), [1.0; 3]).unwrap();
        store.faces_of_body(body).unwrap()[0]
    }
}
