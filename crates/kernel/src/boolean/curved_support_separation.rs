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
use ktopo::entity::FaceId as RawFaceId;
use ktopo::geom::SurfaceGeom;
use ktopo::store::Store;

use super::curved_pipeline::{CurvedBooleanPipelineRefusal, PipelineFailure, StageResult};
use super::curved_source::CertifiedCylinderSource;
use super::extract::ExtractedPlanarSourceBody;
use super::face_partition::PlanarCircleRepresentative;
use super::pipeline::PLANAR_BOOLEAN_BSP_WORK;
use super::planar_bsp::{SourcePlane, SourcePlaneRef};
use crate::error::Error;

const WORK_PER_SUPPORT: u64 = 4;
const WORK_PER_PORT_USE: u64 = 4;

/// Complete convex-host relation proven by one separating support.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum ConvexHostCylinderSupportRelation {
    /// The complete finite cylinder is strictly outside one host support.
    StrictExterior,
    /// Exactly one axial cap is incident and the rest of the cylinder is
    /// strictly on the non-material side of the support.
    CertifiedAxialSingleCap {
        host_face: RawFaceId,
        support: SourcePlane,
        boundary: usize,
    },
}

/// Strict full-disk contact retained for partition and realization.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct CertifiedAxialCapContact {
    host_face: RawFaceId,
    support: SourcePlane,
    boundary: usize,
    planar_representative: PlanarCircleRepresentative,
}

impl CertifiedAxialCapContact {
    pub(super) const fn key(self) -> usize {
        0
    }

    pub(super) const fn host_face(self) -> RawFaceId {
        self.host_face
    }

    pub(super) const fn support(self) -> SourcePlane {
        self.support
    }

    pub(super) const fn boundary(self) -> usize {
        self.boundary
    }

    pub(super) const fn planar_representative(self) -> PlanarCircleRepresentative {
        self.planar_representative
    }
}

/// Prove that one convex-host support separates the complete finite cylinder.
///
/// The `4 * face_count` BSP-work precharge covers, per support, three
/// outward-rounded analytic-carrier residual bounds and one complete
/// finite-cylinder support evaluation. Structural face/support identity is
/// exact; analytic coincidence is admitted only inside the operation's linear
/// envelope. The charge precedes inspection, so early success and storage
/// order cannot affect accounting.
pub(super) fn certify_convex_host_cylinder_support_relation(
    store: &Store,
    host: &ExtractedPlanarSourceBody,
    cylinder: &CertifiedCylinderSource,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<Option<ConvexHostCylinderSupportRelation>> {
    let work = support_scan_work(host.faces().len()).ok_or_else(work_overflow)?;
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_BSP_WORK, work)
        .map_err(Error::from)?;

    let mut contact = None;
    let linear = scope.context().tolerances().linear();
    for (source_face, support) in host.faces().iter().zip(host.planes()) {
        if source_face.plane() != support.id() {
            return Ok(None);
        }
        let surface = store
            .surface(source_face.surface())
            .map_err(|source| Error::InconsistentTopology { source })?;
        let SurfaceGeom::Plane(plane) = surface else {
            // The proof-bearing planar extractor already excludes this, but
            // fail closed if live source geometry ever violates that contract.
            return Ok(None);
        };
        let Some(binding_error) =
            analytic_plane_support_residual(plane, *support).filter(|residual| *residual <= linear)
        else {
            return Ok(None);
        };
        let Some(relation) = plane_finite_cylinder_support_relation(
            plane,
            *support,
            host.interior_sample(),
            cylinder.cylinder(),
            [
                cylinder.boundaries()[0].center(),
                cylinder.boundaries()[1].center(),
            ],
            linear,
            binding_error,
        ) else {
            continue;
        };
        match relation {
            PlaneSupportRelation::StrictExterior => {
                return Ok(Some(ConvexHostCylinderSupportRelation::StrictExterior));
            }
            PlaneSupportRelation::CertifiedAxialSingleCap { boundary } => {
                let candidate = ConvexHostCylinderSupportRelation::CertifiedAxialSingleCap {
                    host_face: source_face.face().raw(),
                    support: *support,
                    boundary,
                };
                if contact.replace(candidate).is_some() {
                    return Ok(None);
                }
            }
        }
    }
    Ok(contact)
}

/// Strengthen a certified cap/support relation to strict full-disk containment.
pub(super) fn certify_strict_axial_cap_contact(
    store: &Store,
    host: &ExtractedPlanarSourceBody,
    cylinder: &CertifiedCylinderSource,
    relation: ConvexHostCylinderSupportRelation,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<Option<CertifiedAxialCapContact>> {
    let ConvexHostCylinderSupportRelation::CertifiedAxialSingleCap {
        host_face,
        support,
        boundary,
    } = relation
    else {
        return Ok(None);
    };
    let uses = host
        .edges()
        .len()
        .checked_mul(2)
        .and_then(|uses| u64::try_from(uses).ok())
        .and_then(|uses| uses.checked_mul(WORK_PER_PORT_USE))
        .ok_or_else(work_overflow)?;
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_BSP_WORK, uses)
        .map_err(Error::from)?;

    let Some(contact_face_index) = source_plane_index(host, support.id()) else {
        return Ok(None);
    };
    let Some(fragment) = host
        .fragments()
        .iter()
        .find(|fragment| fragment.source_face() == support.id())
    else {
        return Ok(None);
    };
    if host
        .fragments()
        .iter()
        .filter(|fragment| fragment.source_face() == support.id())
        .count()
        != 1
    {
        return Ok(None);
    }
    let center = cylinder.boundaries()[boundary].center();
    let linear = scope.context().tolerances().linear();
    for edge_support in fragment.edge_planes() {
        let Some(index) = source_plane_index(host, *edge_support) else {
            return Ok(None);
        };
        let source_face = &host.faces()[index];
        let source_support = host.planes()[index];
        let SurfaceGeom::Plane(plane) = store
            .surface(source_face.surface())
            .map_err(|source| Error::InconsistentTopology { source })?
        else {
            return Ok(None);
        };
        let Some(binding_error) = analytic_plane_support_residual(plane, source_support)
            .filter(|residual| *residual <= linear)
        else {
            return Ok(None);
        };
        if !circle_strictly_inside_support(
            plane,
            source_support,
            host.interior_sample(),
            cylinder.cylinder(),
            center,
            linear,
            binding_error,
        ) {
            return Ok(None);
        }
    }
    let source_face = &host.faces()[contact_face_index];
    if source_face.face().raw() != host_face {
        return Err(PipelineFailure::Refused(
            CurvedBooleanPipelineRefusal::SectionIncomplete,
        ));
    }
    let SurfaceGeom::Plane(port_plane) = store
        .surface(source_face.surface())
        .map_err(|source| Error::InconsistentTopology { source })?
    else {
        return Ok(None);
    };
    let offset = center - port_plane.frame().origin();
    let planar_representative = PlanarCircleRepresentative::new(
        [
            offset.dot(port_plane.frame().x()),
            offset.dot(port_plane.frame().y()),
        ],
        cylinder.cylinder().radius(),
    );
    Ok(Some(CertifiedAxialCapContact {
        host_face,
        support,
        boundary,
        planar_representative,
    }))
}

fn analytic_plane_support_residual(plane: &Plane, support: SourcePlane) -> Option<f64> {
    let normal = plane.frame().z().to_array();
    let origin = plane.frame().origin().to_array();
    support
        .points()
        .into_iter()
        .try_fold(0.0_f64, |maximum, point| {
            let residual = affine_interval(normal, point, origin);
            let bound = residual.lo().abs().max(residual.hi().abs());
            bound.is_finite().then_some(maximum.max(bound))
        })
}

fn support_scan_work(face_count: usize) -> Option<u64> {
    u64::try_from(face_count)
        .ok()?
        .checked_mul(WORK_PER_SUPPORT)
}

fn source_plane_index(host: &ExtractedPlanarSourceBody, id: SourcePlaneRef) -> Option<usize> {
    let index = usize::try_from(id.face()).ok()?;
    (host.faces().get(index)?.plane() == id && host.planes().get(index)?.id() == id)
        .then_some(index)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PlaneSupportRelation {
    StrictExterior,
    CertifiedAxialSingleCap { boundary: usize },
}

/// Classify one complete finite cylinder against one host support.
fn plane_finite_cylinder_support_relation(
    plane: &Plane,
    support: SourcePlane,
    host_interior: Point3,
    cylinder: Cylinder,
    endpoints: [Point3; 2],
    linear: f64,
    binding_error: f64,
) -> Option<PlaneSupportRelation> {
    let normal = plane.frame().z().to_array();
    let origin = plane.frame().origin().to_array();
    let material_side = affine_dot3(normal, host_interior.to_array(), origin, 0.0)
        .map(|value| value.sign())
        .filter(|side| *side != Orientation::Zero)?;

    let envelope = tolerance_envelope(linear, binding_error)?;
    if let Some(boundary) = certified_axial_single_cap_contact(
        normal,
        origin,
        material_side,
        support,
        cylinder,
        endpoints,
        envelope,
    ) {
        return Some(PlaneSupportRelation::CertifiedAxialSingleCap { boundary });
    }

    let projection = finite_cylinder_projection(normal, origin, cylinder, endpoints)?;
    let separated = match material_side {
        Orientation::Positive => projection.hi() < -envelope,
        Orientation::Negative => projection.lo() > envelope,
        Orientation::Zero => false,
    };
    separated.then_some(PlaneSupportRelation::StrictExterior)
}

fn certified_axial_single_cap_contact(
    normal: [f64; 3],
    plane_origin: [f64; 3],
    material_side: Orientation,
    support: SourcePlane,
    cylinder: Cylinder,
    endpoints: [Point3; 2],
    envelope: f64,
) -> Option<usize> {
    let zero = [0.0; 3];
    let plane_axis = normal;
    let cylinder_axis = cylinder.frame().z().to_array();
    let shared_axis =
        plane_axis == cylinder_axis || plane_axis == cylinder_axis.map(|value| -value);
    let radial_is_parallel = shared_axis
        || [cylinder.frame().x(), cylinder.frame().y()]
            .into_iter()
            .all(|axis| {
                affine_dot3(normal, axis.to_array(), zero, 0.0)
                    .is_some_and(|value| value.sign() == Orientation::Zero)
            });
    if !radial_is_parallel {
        return None;
    }
    let projections = endpoints.map(|endpoint| {
        let projection = affine_interval(normal, endpoint.to_array(), plane_origin);
        (projection.lo().is_finite() && projection.hi().is_finite()).then_some(projection)
    });
    let [Some(first), Some(second)] = projections else {
        return None;
    };
    let incident = [first, second]
        .map(|projection| projection.lo() >= -envelope && projection.hi() <= envelope);
    let boundary = match incident {
        [true, false] => 0,
        [false, true] => 1,
        _ => return None,
    };
    let far_projection = [first, second][1 - boundary];
    let far_is_exterior = match material_side {
        Orientation::Positive => far_projection.hi() < -envelope,
        Orientation::Negative => far_projection.lo() > envelope,
        Orientation::Zero => false,
    };
    if !far_is_exterior {
        return None;
    }
    let witness = support.points();
    let far_side = kcore::predicates::orient3d(
        witness[0],
        witness[1],
        witness[2],
        endpoints[1 - boundary].to_array(),
    );
    if far_side == Orientation::Zero || far_side == support.interior_side() {
        return None;
    }
    Some(boundary)
}

fn circle_strictly_inside_support(
    plane: &Plane,
    support: SourcePlane,
    host_interior: Point3,
    cylinder: Cylinder,
    center: Point3,
    linear: f64,
    binding_error: f64,
) -> bool {
    let normal = plane.frame().z().to_array();
    let origin = plane.frame().origin().to_array();
    let Some(material_side) = affine_dot3(normal, host_interior.to_array(), origin, 0.0)
        .map(|value| value.sign())
        .filter(|side| *side != Orientation::Zero)
    else {
        return false;
    };
    let witness = support.points();
    if kcore::predicates::orient3d(witness[0], witness[1], witness[2], center.to_array())
        != support.interior_side()
    {
        return false;
    }
    let center_projection = affine_interval(normal, center.to_array(), origin);
    let radial_sq = [cylinder.frame().x(), cylinder.frame().y()]
        .into_iter()
        .map(|axis| vector_dot_interval(normal, axis.to_array()).square())
        .fold(Interval::point(0.0), |sum, term| sum + term);
    let Some(amplitude) = radial_sq.sqrt() else {
        return false;
    };
    let amplitude = amplitude * Interval::point(cylinder.radius());
    let disk = center_projection + Interval::new(-amplitude.hi(), amplitude.hi());
    let Some(envelope) = tolerance_envelope(linear, binding_error) else {
        return false;
    };
    match material_side {
        Orientation::Positive => disk.lo() > envelope,
        Orientation::Negative => disk.hi() < -envelope,
        Orientation::Zero => false,
    }
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

fn tolerance_envelope(linear: f64, binding_error: f64) -> Option<f64> {
    let envelope = (Interval::point(linear) + Interval::point(binding_error)).hi();
    envelope.is_finite().then_some(envelope)
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

    fn support(frame: Frame, interior: Point3) -> SourcePlane {
        SourcePlane::from_interior_sample(
            super::super::planar_bsp::SourcePlaneRef::new(0, 0),
            [
                frame.origin().to_array(),
                frame.point_at(1.0, 0.0, 0.0).to_array(),
                frame.point_at(0.0, 1.0, 0.0).to_array(),
            ],
            interior.to_array(),
        )
        .unwrap()
    }

    #[test]
    fn certified_axial_support_admits_either_flush_cap_and_rejects_overlap() {
        let frame = rigid_frame();
        let plane = Plane::new(frame);
        let interior = frame.point_at(0.0, 0.0, 1.0);
        let support = support(frame, interior);
        let cylinder = Cylinder::new(frame, 0.75).unwrap();

        for (endpoints, boundary) in [
            ([frame.point_at(0.0, 0.0, -2.0), frame.origin()], 1),
            ([frame.origin(), frame.point_at(0.0, 0.0, -2.0)], 0),
        ] {
            assert_eq!(
                plane_finite_cylinder_support_relation(
                    &plane, support, interior, cylinder, endpoints, 1.0e-7, 0.0,
                ),
                Some(PlaneSupportRelation::CertifiedAxialSingleCap { boundary })
            );
        }
        assert_eq!(
            plane_finite_cylinder_support_relation(
                &plane,
                support,
                interior,
                cylinder,
                [
                    frame.point_at(0.0, 0.0, -0.5),
                    frame.point_at(0.0, 0.0, 1.5),
                ],
                1.0e-7,
                0.0,
            ),
            None
        );
    }

    #[test]
    fn tilted_strict_support_uses_conservative_radial_projection() {
        let plane = Plane::new(Frame::world());
        let interior = Point3::new(0.0, 0.0, 1.0);
        let support = support(Frame::world(), interior);
        let cylinder_frame = Frame::new(
            Point3::new(0.0, 0.0, -4.0),
            Vec3::new(0.0, 1.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let cylinder = Cylinder::new(cylinder_frame, 0.5).unwrap();
        assert_eq!(
            plane_finite_cylinder_support_relation(
                &plane,
                support,
                interior,
                cylinder,
                [
                    cylinder_frame.origin(),
                    cylinder_frame.origin() + cylinder_frame.z(),
                ],
                1.0e-7,
                0.0,
            ),
            Some(PlaneSupportRelation::StrictExterior)
        );
    }

    #[test]
    fn endpoint_center_contact_without_radial_parallelism_is_not_full_cap_contact() {
        let plane = Plane::new(Frame::world());
        let interior = Point3::new(0.0, 0.0, 1.0);
        let support = support(Frame::world(), interior);
        let frame = Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, -1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let cylinder = Cylinder::new(frame, 0.75).unwrap();
        assert_eq!(
            plane_finite_cylinder_support_relation(
                &plane,
                support,
                interior,
                cylinder,
                [frame.origin(), frame.origin() + frame.z()],
                1.0e-7,
                0.0,
            ),
            None
        );
    }

    #[test]
    fn analytic_carrier_binding_has_a_certified_residual_envelope() {
        let support = SourcePlane::from_interior_sample(
            super::super::planar_bsp::SourcePlaneRef::new(0, 0),
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [0.0, 0.0, 1.0],
        )
        .unwrap();
        assert!(
            analytic_plane_support_residual(&Plane::new(Frame::world()), support)
                .is_some_and(|residual| residual <= f64::MIN_POSITIVE)
        );
        assert!(
            analytic_plane_support_residual(
                &Plane::new(Frame::world().with_origin(Point3::new(0.0, 0.0, 0.25))),
                support,
            )
            .is_some_and(|residual| residual >= 0.25)
        );
    }

    #[test]
    fn support_work_is_checked_and_source_size_exact() {
        assert_eq!(support_scan_work(6), Some(24));
        if usize::BITS == u64::BITS {
            assert_eq!(support_scan_work(usize::MAX), None);
        }
    }
}
