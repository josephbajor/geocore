//! Certified conservative face-domain construction.
//!
//! This module derives finite UV work boxes only from bounds that already
//! carry containment guarantees. Exact edge curves provide analytic boxes or
//! positive-weight NURBS control-hull boxes through `kgeom`; those 3D boxes
//! are projected analytically into plane/cylinder/cone parameters. A face
//! with any curve-less tolerant boundary returns `None` until certified 2D
//! pcurve bounding lands—there is deliberately no sampled fallback.

use crate::entity::{FaceDomain, FaceId};
use crate::geom::SurfaceGeom;
use crate::store::Store;
use kcore::error::{Error, Result};
use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::aabb::Aabb3;
use kgeom::param::ParamRange;
use kgeom::vec::{Point3, Vec3};

/// Derive a certified conservative finite UV work box for `face`.
///
/// Finite natural surface domains (sphere, torus, current NURBS) are
/// returned directly. Plane/cylinder/cone domains are derived from every
/// exact boundary edge. `Ok(None)` means the available representation is
/// insufficient for a proof, not that the face is unbounded.
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

    let mut bounds = Aabb3::empty();
    let mut tolerance = face.tolerance.unwrap_or(0.0).max(LINEAR_RESOLUTION);
    let mut found_edge = false;
    for &loop_id in &face.loops {
        for &fin_id in &store.get(loop_id)?.fins {
            let edge = store.get(store.get(fin_id)?.edge)?;
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
            bounds = bounds.union(curve.bounding_box(range));
            tolerance = tolerance.max(edge.tolerance.unwrap_or(0.0));
            found_edge = true;
        }
    }
    if !found_edge || bounds.is_empty() {
        return Ok(None);
    }
    let bounds = bounds.inflated(tolerance);
    let domain = match surface {
        SurfaceGeom::Plane(plane) => {
            let (u_min, u_max) = project_box(bounds, plane.frame().origin(), plane.frame().x());
            let (v_min, v_max) = project_box(bounds, plane.frame().origin(), plane.frame().y());
            FaceDomain::from_bounds(u_min, u_max, v_min, v_max)?
        }
        SurfaceGeom::Cylinder(cylinder) => {
            let (v_min, v_max) =
                project_box(bounds, cylinder.frame().origin(), cylinder.frame().z());
            FaceDomain::from_bounds(0.0, core::f64::consts::TAU, v_min, v_max)?
        }
        SurfaceGeom::Cone(cone) => {
            let (z_min, z_max) = project_box(bounds, cone.frame().origin(), cone.frame().z());
            let cos = kcore::math::cos(cone.half_angle());
            FaceDomain::from_bounds(0.0, core::f64::consts::TAU, z_min / cos, z_max / cos)?
        }
        _ => unreachable!("filtered above"),
    };
    Ok(Some(domain))
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
    use crate::entity::Edge;
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
    fn curve_less_boundary_remains_explicitly_unknown() {
        let mut store = Store::new();
        let body = block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
        let face = store.faces_of_body(body).unwrap()[0];
        let edge = store.get(store.get(face).unwrap().loops[0]).unwrap().fins[0];
        let edge = store.get(edge).unwrap().edge;
        let edge_data: &mut Edge = store.get_mut(edge).unwrap();
        edge_data.curve = None;
        edge_data.tolerance = Some(LINEAR_RESOLUTION);
        assert_eq!(derive_face_domain(&store, face).unwrap(), None);
    }
}
