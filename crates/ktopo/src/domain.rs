//! Certified conservative face-domain construction.
//!
//! This module derives finite UV work boxes only from bounds that already
//! carry containment guarantees. Fin pcurves provide native analytic or
//! positive-weight NURBS control-hull boxes. Legacy exact edge curves provide
//! equivalent 3D boxes that are projected analytically into plane/cylinder/
//! cone parameters. There is deliberately no sampled fallback.

use crate::entity::{FaceDomain, FaceId, FinPcurve};
use crate::geom::SurfaceGeom;
use crate::store::Store;
use kcore::error::{Error, Result};
use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::aabb::{Aabb2, Aabb3};
use kgeom::curve::Curve;
use kgeom::param::ParamRange;
use kgeom::vec::{Point3, Vec2, Vec3};

/// Proof status for whether a declared face domain contains its boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaceDomainContainment {
    /// Conservative boxes prove the entire represented boundary is inside.
    Certified,
    /// An evaluated boundary endpoint is provably outside the domain.
    Outside,
    /// No point is proven outside, but current bounds are too loose or
    /// incomplete to prove full containment.
    Indeterminate,
}

/// Derive a certified conservative finite UV work box for `face`.
///
/// Finite natural surface domains (sphere, torus, current NURBS) are
/// returned directly. Plane/cylinder/cone domains prefer each fin's pcurve
/// bounds and project a 3D curve box only for a legacy exact fin without a
/// pcurve. `Ok(None)` means the available representations cannot prove one
/// finite chart, not that the face is unbounded.
pub fn derive_face_domain(store: &Store, face_id: FaceId) -> Result<Option<FaceDomain>> {
    let face = store.get(face_id)?;
    let surface = store.get(face.surface)?;
    if let Some(domain) = FaceDomain::natural(surface) {
        return Ok(Some(domain));
    }
    if !matches!(
        surface,
        SurfaceGeom::Plane(_) | SurfaceGeom::Cylinder(_) | SurfaceGeom::Cone(_)
    ) {
        return Ok(None);
    }

    let mut uv_bounds = Aabb2::empty();
    let mut xyz_bounds = Aabb3::empty();
    let mut tolerance = face
        .tolerance
        .map(crate::tolerance::EntityTolerance::value)
        .unwrap_or(0.0)
        .max(LINEAR_RESOLUTION);
    let mut needs_full_period_u = false;
    let mut found_edge = false;
    let periods = surface.as_surface().periodicity();
    for &loop_id in &face.loops {
        for &fin_id in &store.get(loop_id)?.fins {
            let fin = store.get(fin_id)?;
            let edge = store.get(fin.edge)?;
            if let Some(pcurve) = fin.pcurve {
                uv_bounds = uv_bounds.union(pcurve_box(store, pcurve, periods)?);
                found_edge = true;
                continue;
            }
            let Some(curve_id) = edge.curve else {
                return Ok(None);
            };
            let curve = store.get(curve_id)?.as_curve();
            let range = match edge.bounds {
                Some((lo, hi)) if lo.is_finite() && hi.is_finite() && lo < hi => {
                    ParamRange::new(lo, hi)
                }
                Some(_) => {
                    return Err(Error::InvalidGeometry {
                        reason: "cannot derive face domain from invalid edge bounds",
                    });
                }
                None => {
                    let natural = curve.param_range();
                    if curve.periodicity().is_none() || !natural.is_finite() {
                        return Err(Error::InvalidGeometry {
                            reason: "cannot derive face domain from non-periodic ring edge",
                        });
                    }
                    natural
                }
            };
            xyz_bounds = xyz_bounds.union(curve_box(curve, range));
            tolerance = tolerance.max(
                edge.tolerance
                    .map(crate::tolerance::EntityTolerance::value)
                    .unwrap_or(0.0),
            );
            needs_full_period_u |=
                matches!(surface, SurfaceGeom::Cylinder(_) | SurfaceGeom::Cone(_));
            found_edge = true;
        }
    }
    if !found_edge {
        return Ok(None);
    }
    if !xyz_bounds.is_empty() {
        let bounds = xyz_bounds.inflated(tolerance);
        match surface {
            SurfaceGeom::Plane(plane) => {
                let (u_min, u_max) = project_box(bounds, plane.frame().origin(), plane.frame().x());
                let (v_min, v_max) = project_box(bounds, plane.frame().origin(), plane.frame().y());
                uv_bounds = uv_bounds.union(Aabb2::from_points(&[
                    Vec2::new(u_min, v_min),
                    Vec2::new(u_max, v_max),
                ]));
            }
            SurfaceGeom::Cylinder(cylinder) => {
                let (v_min, v_max) =
                    project_box(bounds, cylinder.frame().origin(), cylinder.frame().z());
                include_v_range(&mut uv_bounds, v_min, v_max);
            }
            SurfaceGeom::Cone(cone) => {
                let (z_min, z_max) = project_box(bounds, cone.frame().origin(), cone.frame().z());
                let cos = kcore::math::cos(cone.half_angle());
                include_v_range(&mut uv_bounds, z_min / cos, z_max / cos);
            }
            _ => unreachable!("filtered above"),
        }
    }
    domain_from_box(surface, uv_bounds, needs_full_period_u)
}

/// Certify containment of a face boundary in its declared UV work box.
///
/// `Outside` is returned only for an actual charted pcurve endpoint.
/// Failure to contain a conservative box is `Indeterminate`, because a
/// control hull or projected 3D box may be looser than the represented
/// curve. This distinction prevents checker-v2 from turning a missing proof
/// into a false invalidity claim.
pub fn certify_face_domain_containment(
    store: &Store,
    face_id: FaceId,
) -> Result<FaceDomainContainment> {
    let face = store.get(face_id)?;
    let Some(domain) = face.domain else {
        return Ok(FaceDomainContainment::Indeterminate);
    };
    let surface = store.get(face.surface)?;
    let periods = surface.as_surface().periodicity();
    for &loop_id in &face.loops {
        for &fin_id in &store.get(loop_id)?.fins {
            let fin = store.get(fin_id)?;
            let Some(use_) = fin.pcurve else { continue };
            let curve = store.get(use_.curve())?.as_curve();
            for q in [use_.range().lo, use_.range().hi] {
                let uv = use_.chart().apply(curve.eval(q), periods)?;
                if !domain_contains_uv(domain, uv) {
                    return Ok(FaceDomainContainment::Outside);
                }
            }
        }
    }
    let Some(required) = derive_face_domain(store, face_id)? else {
        return Ok(FaceDomainContainment::Indeterminate);
    };
    if domain_contains_domain(domain, required) {
        Ok(FaceDomainContainment::Certified)
    } else {
        Ok(FaceDomainContainment::Indeterminate)
    }
}

fn domain_contains_uv(domain: FaceDomain, uv: Vec2) -> bool {
    range_contains_value(domain.u, uv.x) && range_contains_value(domain.v, uv.y)
}

fn domain_contains_domain(outer: FaceDomain, inner: FaceDomain) -> bool {
    range_contains_value(outer.u, inner.u.lo)
        && range_contains_value(outer.u, inner.u.hi)
        && range_contains_value(outer.v, inner.v.lo)
        && range_contains_value(outer.v, inner.v.hi)
}

fn range_contains_value(range: ParamRange, value: f64) -> bool {
    let slack = 256.0 * f64::EPSILON * (1.0 + range.lo.abs().max(range.hi.abs()).max(value.abs()));
    value >= range.lo - slack && value <= range.hi + slack
}

/// Bounding periodic geometry over its full natural period avoids relying
/// on a particular unwrapped parameter magnitude while remaining
/// conservative for the active range.
fn curve_box(curve: &dyn Curve, range: ParamRange) -> Aabb3 {
    let natural = curve.param_range();
    let range = if curve.periodicity().is_some() && natural.is_finite() {
        natural
    } else {
        range
    };
    curve.bounding_box(range)
}

fn pcurve_box(store: &Store, pcurve: FinPcurve, periods: [Option<f64>; 2]) -> Result<Aabb2> {
    let curve = store.get(pcurve.curve())?.as_curve();
    let natural = curve.param_range();
    let range = if curve.periodicity().is_some() && natural.is_finite() {
        natural
    } else {
        pcurve.range()
    };
    let bounds = curve.bounding_box(range);
    let min = pcurve.chart().apply(bounds.min, periods)?;
    let max = pcurve.chart().apply(bounds.max, periods)?;
    Ok(Aabb2 { min, max })
}

fn include_v_range(bounds: &mut Aabb2, v_min: f64, v_max: f64) {
    let u = if bounds.is_empty() { 0.0 } else { bounds.min.x };
    *bounds = bounds
        .including(Vec2::new(u, v_min))
        .including(Vec2::new(u, v_max));
}

fn domain_from_box(
    surface: &SurfaceGeom,
    mut bounds: Aabb2,
    needs_full_period_u: bool,
) -> Result<Option<FaceDomain>> {
    if bounds.is_empty()
        || !bounds.min.x.is_finite()
        || !bounds.min.y.is_finite()
        || !bounds.max.x.is_finite()
        || !bounds.max.y.is_finite()
    {
        return Ok(None);
    }
    let periods = surface.as_surface().periodicity();
    for (direction, period) in periods.into_iter().enumerate() {
        let Some(period) = period else { continue };
        let (lo, hi) = if direction == 0 {
            (bounds.min.x, bounds.max.x)
        } else {
            (bounds.min.y, bounds.max.y)
        };
        let slack = 256.0 * f64::EPSILON * (1.0 + lo.abs().max(hi.abs()).max(period));
        if hi - lo > period + slack {
            // The pcurve uses do not fit one periodic chart. Explicit seam
            // metadata must resolve the branches before a domain is known.
            return Ok(None);
        }
        if direction == 0 && needs_full_period_u {
            bounds.max.x = bounds.max.x.max(bounds.min.x + period);
        }
    }
    if bounds.min.x >= bounds.max.x || bounds.min.y >= bounds.max.y {
        return Ok(None);
    }
    FaceDomain::from_bounds(bounds.min.x, bounds.max.x, bounds.min.y, bounds.max.y).map(Some)
}

/// Range of `(point - origin) · axis` over an axis-aligned 3D box.
fn project_box(bounds: Aabb3, origin: Point3, axis: Vec3) -> (f64, f64) {
    let center = (bounds.min + bounds.max) / 2.0;
    let half = (bounds.max - bounds.min) / 2.0;
    let midpoint = (center - origin).dot(axis);
    let radius = half.x * axis.x.abs() + half.y * axis.y.abs() + half.z * axis.z.abs();
    (midpoint - radius, midpoint + radius)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::check::check_body;
    use crate::entity::{EdgeId, ParamMap1d};
    use crate::geom::Curve2dGeom;
    use crate::make::{block, cylinder};
    use kgeom::frame::Frame;
    use kgeom::vec::{Point3, Vec3};

    fn tilted() -> Frame {
        Frame::new(
            Point3::new(0.3, -1.2, 2.1),
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap()
    }

    #[test]
    fn exact_analytic_boundaries_produce_conservative_domains() {
        let mut store = Store::new();
        let body = block(&mut store, &tilted(), [2.0, 3.0, 4.0]).unwrap();
        for face_id in store.faces_of_body(body).unwrap() {
            let authored = store.get(face_id).unwrap().domain.unwrap();
            let derived = derive_face_domain(&store, face_id).unwrap().unwrap();
            assert!(derived.u.lo <= authored.u.lo && derived.u.hi >= authored.u.hi);
            assert!(derived.v.lo <= authored.v.lo && derived.v.hi >= authored.v.hi);
        }

        let body = cylinder(&mut store, &tilted(), 1.2, 2.5).unwrap();
        assert!(
            store
                .faces_of_body(body)
                .unwrap()
                .into_iter()
                .all(|face| derive_face_domain(&store, face).unwrap().is_some())
        );
    }

    #[test]
    fn containment_evidence_distinguishes_proof_outside_and_unknown() {
        let mut store = Store::new();
        let body = block(&mut store, &Frame::world(), [2.0, 3.0, 4.0]).unwrap();
        let face = store.faces_of_body(body).unwrap()[0];
        assert_eq!(
            certify_face_domain_containment(&store, face).unwrap(),
            FaceDomainContainment::Certified
        );

        let original = store.get(face).unwrap().domain.unwrap();
        store.get_mut(face).unwrap().domain = Some(
            FaceDomain::new(
                ParamRange::new(original.u.lo + 0.25, original.u.hi),
                original.v,
            )
            .unwrap(),
        );
        assert_eq!(
            certify_face_domain_containment(&store, face).unwrap(),
            FaceDomainContainment::Outside
        );

        store.get_mut(face).unwrap().domain = None;
        assert_eq!(
            certify_face_domain_containment(&store, face).unwrap(),
            FaceDomainContainment::Indeterminate
        );
    }

    fn make_curve_less(store: &mut Store, body: crate::entity::BodyId) -> EdgeId {
        let edge_id = store.edges_of_body(body).unwrap()[0];
        let edge = store.get(edge_id).unwrap();
        let old_bounds = edge.bounds.unwrap();
        let fins = edge.fins.clone();
        for fin_id in fins {
            let old = store.get(fin_id).unwrap().pcurve.unwrap();
            let q0 = old.parameter_at_edge(old_bounds.0);
            let q1 = old.parameter_at_edge(old_bounds.1);
            let map = ParamMap1d::affine(q1 - q0, q0).unwrap();
            store.get_mut(fin_id).unwrap().pcurve =
                Some(FinPcurve::new(old.curve(), old.range(), map).unwrap());
        }
        let edge = store.get_mut(edge_id).unwrap();
        edge.curve = None;
        edge.bounds = Some((0.0, 1.0));
        edge.tolerance = Some(
            crate::tolerance::EntityTolerance::operation(LINEAR_RESOLUTION, "domain-test").unwrap(),
        );
        edge_id
    }

    #[test]
    fn curve_less_boundary_uses_certified_pcurve_bounds() {
        let mut store = Store::new();
        let body = block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        let edge = make_curve_less(&mut store, body);
        for &fin_id in &store.get(edge).unwrap().fins {
            let loop_id = store.get(fin_id).unwrap().parent;
            let face = store.get(loop_id).unwrap().face;
            let domain = derive_face_domain(&store, face).unwrap().unwrap();
            let pcurve = store.get(fin_id).unwrap().pcurve.unwrap();
            let periods = store
                .get(store.get(face).unwrap().surface)
                .unwrap()
                .as_surface()
                .periodicity();
            let bounds = pcurve_box(&store, pcurve, periods).unwrap();
            assert!(domain.contains([bounds.min.x, bounds.min.y]));
            assert!(domain.contains([bounds.max.x, bounds.max.y]));
        }
    }

    #[test]
    fn curve_less_boundary_without_a_pcurve_remains_unknown() {
        let mut store = Store::new();
        let body = block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        let edge = make_curve_less(&mut store, body);
        let fin = store.get(edge).unwrap().fins[0];
        let face = store.get(store.get(fin).unwrap().parent).unwrap().face;
        store.get_mut(fin).unwrap().pcurve = None;
        assert_eq!(derive_face_domain(&store, face).unwrap(), None);
    }

    #[test]
    fn periodic_pcurve_branch_selects_the_domain_chart() {
        let mut store = Store::new();
        let body = cylinder(&mut store, &Frame::world(), 1.0, 2.0).unwrap();
        let side = store
            .faces_of_body(body)
            .unwrap()
            .into_iter()
            .find(|&face| {
                matches!(
                    store.get(store.get(face).unwrap().surface).unwrap(),
                    SurfaceGeom::Cylinder(_)
                )
            })
            .unwrap();
        let loops = store.get(side).unwrap().loops.clone();
        let mut side_fins = Vec::new();
        for loop_id in loops {
            let fins = store.get(loop_id).unwrap().fins.clone();
            for fin_id in fins {
                let use_ = store.get(fin_id).unwrap().pcurve.unwrap();
                side_fins.push(fin_id);
                let Curve2dGeom::Line(_) = *store.get(use_.curve()).unwrap() else {
                    panic!("cylinder side pcurve must be linear");
                };
                store.get_mut(fin_id).unwrap().pcurve =
                    Some(use_.with_chart(crate::entity::PcurveChart::shifted([2, 0])));
            }
        }
        let domain = derive_face_domain(&store, side).unwrap().unwrap();
        assert!((domain.u.lo - 2.0 * core::f64::consts::TAU).abs() < 1e-14);
        assert!((domain.u.hi - 3.0 * core::f64::consts::TAU).abs() < 1e-14);
        store.get_mut(side).unwrap().domain = Some(domain);
        assert!(check_body(&store, body).unwrap().is_empty());

        let first = side_fins[0];
        let use_ = store.get(first).unwrap().pcurve.unwrap();
        store.get_mut(first).unwrap().pcurve =
            Some(use_.with_chart(crate::entity::PcurveChart::identity()));
        assert_eq!(
            derive_face_domain(&store, side).unwrap(),
            None,
            "inconsistent periodic branches require explicit seam metadata"
        );
    }
}
