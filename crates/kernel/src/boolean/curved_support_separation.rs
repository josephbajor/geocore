//! Exact/certified support separation for convex-planar/cylinder intersection.
//!
//! A convex host is the intersection of its certified material half-spaces.
//! Consequently, one host support containing the complete finite cylinder in
//! its closed exterior half-space proves that the regularized intersection is
//! empty. The certificate is read-only and scans supports without allocating.

use kcore::interval::Interval;
use kcore::operation::OperationScope;
use kcore::predicates::{Orientation, affine_dot3};
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::Point3;
use ktopo::geom::SurfaceGeom;
use ktopo::store::Store;

use super::curved_pipeline::{CurvedBooleanPipelineRefusal, PipelineFailure, StageResult};
use super::curved_source::CertifiedCylinderSource;
use super::extract::ExtractedPlanarSourceBody;
use super::pipeline::PLANAR_BOOLEAN_BSP_WORK;
use super::planar_bsp::SourcePlane;
use crate::error::Error;

const WORK_PER_SUPPORT: u64 = 4;

/// Prove that one convex-host support separates the complete finite cylinder.
///
/// The `4 * face_count` BSP-work precharge covers, per support, three exact
/// analytic-carrier witness bindings and one complete finite-cylinder support
/// evaluation. It is charged before the first support is inspected, so early
/// success and support storage order cannot affect accounting.
pub(super) fn certify_cylinder_in_closed_host_exterior(
    store: &Store,
    host: &ExtractedPlanarSourceBody,
    cylinder: &CertifiedCylinderSource,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<bool> {
    let work = support_scan_work(host.faces().len()).ok_or_else(work_overflow)?;
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_BSP_WORK, work)
        .map_err(Error::from)?;

    for (source_face, support) in host.faces().iter().zip(host.planes()) {
        if source_face.plane() != support.id() {
            return Ok(false);
        }
        let surface = store
            .surface(source_face.surface())
            .map_err(|source| Error::InconsistentTopology { source })?;
        let SurfaceGeom::Plane(plane) = surface else {
            // The proof-bearing planar extractor already excludes this, but
            // fail closed if live source geometry ever violates that contract.
            return Ok(false);
        };
        if !analytic_plane_binds_exact_support(plane, *support) {
            return Ok(false);
        }
        if plane_separates_finite_cylinder(
            plane,
            host.interior_sample(),
            cylinder.cylinder(),
            [
                cylinder.boundaries()[0].center(),
                cylinder.boundaries()[1].center(),
            ],
        ) {
            return Ok(true);
        }
    }
    Ok(false)
}

fn analytic_plane_binds_exact_support(plane: &Plane, support: SourcePlane) -> bool {
    let normal = plane.frame().z().to_array();
    let origin = plane.frame().origin().to_array();
    support.points().into_iter().all(|point| {
        affine_dot3(normal, point, origin, 0.0)
            .is_some_and(|value| value.sign() == Orientation::Zero)
    })
}

fn support_scan_work(face_count: usize) -> Option<u64> {
    u64::try_from(face_count)
        .ok()?
        .checked_mul(WORK_PER_SUPPORT)
}

/// Certify a complete finite cylinder on the non-material side of one plane.
///
/// The interval projection proves strict separation for arbitrary relative
/// orientations. The exact axial branch additionally admits a zero-distance
/// cap: exact dot predicates prove both cylinder radial axes parallel to the
/// plane and both cap centers in its closed exterior half-space.
fn plane_separates_finite_cylinder(
    plane: &Plane,
    host_interior: Point3,
    cylinder: Cylinder,
    endpoints: [Point3; 2],
) -> bool {
    let normal = plane.frame().z().to_array();
    let origin = plane.frame().origin().to_array();
    let Some(material_side) = affine_dot3(normal, host_interior.to_array(), origin, 0.0)
        .map(|value| value.sign())
        .filter(|side| *side != Orientation::Zero)
    else {
        return false;
    };

    if exact_axial_closed_exterior(normal, origin, material_side, cylinder, endpoints) {
        return true;
    }

    let Some(projection) = finite_cylinder_projection(normal, origin, cylinder, endpoints) else {
        return false;
    };
    match material_side {
        Orientation::Positive => projection.hi() < 0.0,
        Orientation::Negative => projection.lo() > 0.0,
        Orientation::Zero => false,
    }
}

fn exact_axial_closed_exterior(
    normal: [f64; 3],
    plane_origin: [f64; 3],
    material_side: Orientation,
    cylinder: Cylinder,
    endpoints: [Point3; 2],
) -> bool {
    let zero = [0.0; 3];
    let radial_is_parallel = [cylinder.frame().x(), cylinder.frame().y()]
        .into_iter()
        .all(|axis| {
            affine_dot3(normal, axis.to_array(), zero, 0.0)
                .is_some_and(|value| value.sign() == Orientation::Zero)
        });
    radial_is_parallel
        && endpoints.into_iter().all(|endpoint| {
            affine_dot3(normal, endpoint.to_array(), plane_origin, 0.0).is_some_and(|value| {
                value.sign() == Orientation::Zero || value.sign() != material_side
            })
        })
}

fn finite_cylinder_projection(
    normal: [f64; 3],
    plane_origin: [f64; 3],
    cylinder: Cylinder,
    endpoints: [Point3; 2],
) -> Option<Interval> {
    let endpoint_projection =
        endpoints.map(|endpoint| affine_interval(normal, endpoint.to_array(), plane_origin));
    if endpoint_projection
        .iter()
        .any(|value| !value.lo().is_finite() || !value.hi().is_finite())
    {
        return None;
    }
    let axial = Interval::new(
        endpoint_projection[0].lo().min(endpoint_projection[1].lo()),
        endpoint_projection[0].hi().max(endpoint_projection[1].hi()),
    );
    let radial_sq = [cylinder.frame().x(), cylinder.frame().y()]
        .into_iter()
        .map(|axis| vector_dot_interval(normal, axis.to_array()).square())
        .fold(Interval::point(0.0), |sum, term| sum + term);
    if !radial_sq.lo().is_finite() || !radial_sq.hi().is_finite() {
        return None;
    }
    let amplitude = radial_sq.sqrt()? * Interval::point(cylinder.radius());
    let amplitude = amplitude.hi();
    if !amplitude.is_finite() {
        return None;
    }
    let projection = axial + Interval::new(-amplitude, amplitude);
    (projection.lo().is_finite() && projection.hi().is_finite()).then_some(projection)
}

fn affine_interval(normal: [f64; 3], point: [f64; 3], origin: [f64; 3]) -> Interval {
    (0..3).fold(Interval::point(0.0), |sum, axis| {
        sum + Interval::point(normal[axis])
            * (Interval::point(point[axis]) - Interval::point(origin[axis]))
    })
}

fn vector_dot_interval(first: [f64; 3], second: [f64; 3]) -> Interval {
    (0..3).fold(Interval::point(0.0), |sum, axis| {
        sum + Interval::point(first[axis]) * Interval::point(second[axis])
    })
}

fn work_overflow() -> PipelineFailure {
    PipelineFailure::Refused(CurvedBooleanPipelineRefusal::WorkCountOverflow)
}

#[cfg(test)]
mod tests {
    use kgeom::frame::Frame;
    use kgeom::vec::Vec3;

    use super::*;

    fn rigid_frame() -> Frame {
        Frame::new(
            Point3::new(1.25, -0.75, 0.5),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap()
    }

    #[test]
    fn exact_axial_support_admits_either_flush_cap_and_rejects_overlap() {
        let frame = rigid_frame();
        let plane = Plane::new(frame);
        let interior = frame.point_at(0.0, 0.0, 1.0);
        let cylinder = Cylinder::new(frame, 0.75).unwrap();

        for endpoints in [
            [frame.point_at(0.0, 0.0, -2.0), frame.origin()],
            [frame.origin(), frame.point_at(0.0, 0.0, -2.0)],
        ] {
            assert!(plane_separates_finite_cylinder(
                &plane, interior, cylinder, endpoints
            ));
        }
        assert!(!plane_separates_finite_cylinder(
            &plane,
            interior,
            cylinder,
            [
                frame.point_at(0.0, 0.0, -0.5),
                frame.point_at(0.0, 0.0, 1.5),
            ],
        ));
    }

    #[test]
    fn tilted_strict_support_uses_conservative_radial_projection() {
        let plane = Plane::new(Frame::world());
        let interior = Point3::new(0.0, 0.0, 1.0);
        let cylinder_frame = Frame::new(
            Point3::new(0.0, 0.0, -4.0),
            Vec3::new(0.0, 1.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let cylinder = Cylinder::new(cylinder_frame, 0.5).unwrap();
        assert!(plane_separates_finite_cylinder(
            &plane,
            interior,
            cylinder,
            [
                cylinder_frame.origin(),
                cylinder_frame.origin() + cylinder_frame.z(),
            ],
        ));
    }

    #[test]
    fn analytic_carrier_requires_exact_support_witness_binding() {
        let support = SourcePlane::from_interior_sample(
            super::super::planar_bsp::SourcePlaneRef::new(0, 0),
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [0.0, 0.0, 1.0],
        )
        .unwrap();
        assert!(analytic_plane_binds_exact_support(
            &Plane::new(Frame::world()),
            support
        ));
        assert!(!analytic_plane_binds_exact_support(
            &Plane::new(Frame::world().with_origin(Point3::new(0.0, 0.0, 0.25))),
            support
        ));
    }

    #[test]
    fn support_work_is_checked_and_source_size_exact() {
        assert_eq!(support_scan_work(6), Some(24));
        if usize::BITS == u64::BITS {
            assert_eq!(support_scan_work(usize::MAX), None);
        }
    }
}
