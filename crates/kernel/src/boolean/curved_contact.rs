//! Selected-boundary adapter for certified full-disk support contact.
//!
//! The adapter is operation-neutral: it receives generic truth-selected
//! fragments plus a certified support witness. One complete cylinder source band
//! is attached to the witnessed planar port; the opposite source cap closes
//! the band. Admission uses source dimensions only, before semantic host
//! vectors are materialized.

use kgeom::param::ParamRange;
use ktopo::cylindrical_host::{
    CylindricalHostBandInput, CylindricalHostEndpoint, CylindricalHostSolidInput,
    cylindrical_host_dimension_work, cylindrical_host_preflight_work,
};

use super::boundary_select::{OperandSide, SelectedBoundaryFragment, SelectedOrientation};
use super::curved_host::{prepare_curved_host, source_operand, source_plane_for_face};
use super::curved_pipeline::{CurvedFragment, CurvedFragmentKey};
use super::curved_source::CertifiedCylinderSource;
use super::curved_support_separation::CertifiedAxialCapContact;
use super::extract::ExtractedPlanarSourceBody;
use super::face_partition::{AxialBoundary, FaceRegionKey};

type SelectedCurvedFragment = SelectedBoundaryFragment<CurvedFragmentKey, CurvedFragment>;

/// Allocation-free source dimensions for one selected contact result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct SupportContactAdmission {
    host_vertices: usize,
    host_faces: usize,
    host_face_uses: usize,
    semantic_preflight_work: u64,
}

impl SupportContactAdmission {
    pub(super) const fn host_vertices(self) -> usize {
        self.host_vertices
    }

    pub(super) const fn host_faces(self) -> usize {
        self.host_faces
    }

    pub(super) const fn host_face_uses(self) -> usize {
        self.host_face_uses
    }

    pub(super) const fn semantic_preflight_work(self) -> u64 {
        self.semantic_preflight_work
    }
}

/// Materialized one-port host input plus verified accounting dimensions.
#[derive(Debug, Clone)]
pub(super) struct PreparedSupportContact {
    input: CylindricalHostSolidInput,
}

impl PreparedSupportContact {
    pub(super) fn into_input(self) -> CylindricalHostSolidInput {
        self.input
    }
}

/// Recognize the certified selected truth and compute realization work without scratch.
pub(super) fn admit_support_contact(
    planar: &ExtractedPlanarSourceBody,
    cylinder: &CertifiedCylinderSource,
    contact: CertifiedAxialCapContact,
    selected: &[SelectedCurvedFragment],
) -> Result<Option<SupportContactAdmission>, &'static str> {
    if contact.boundary() >= cylinder.boundaries().len()
        || source_plane_for_face(planar, contact.host_face())? != contact.support()
    {
        return Ok(None);
    }
    let planar_side = operand_side(source_operand(planar)?);
    let cylinder_side = opposite(planar_side);
    let far_boundary = 1_usize
        .checked_sub(contact.boundary())
        .ok_or("support contact boundary is invalid")?;
    let mut planar_faces = 0_usize;
    let mut side_band = false;
    let mut far_cap = false;
    for (position, fragment) in selected.iter().enumerate() {
        match fragment.fragment() {
            CurvedFragment::Planar {
                face,
                region: FaceRegionKey::PlanarOuter,
            } if fragment.operand() == planar_side
                && fragment.orientation() == SelectedOrientation::Preserved
                && planar
                    .faces()
                    .iter()
                    .any(|source| source.face().raw() == *face) =>
            {
                if selected[..position].iter().any(|prior| {
                    matches!(
                        prior.fragment(),
                        CurvedFragment::Planar {
                            face: prior_face,
                            region: FaceRegionKey::PlanarOuter,
                        } if prior.operand() == planar_side && prior_face == face
                    )
                }) {
                    return Ok(None);
                }
                planar_faces = planar_faces
                    .checked_add(1)
                    .ok_or("support contact planar face count overflow")?;
            }
            CurvedFragment::CylinderSide {
                region:
                    FaceRegionKey::AxialBand {
                        lower: AxialBoundary::LowerSource,
                        upper: AxialBoundary::UpperSource,
                    },
            } if fragment.operand() == cylinder_side
                && fragment.orientation() == SelectedOrientation::Preserved
                && !side_band =>
            {
                side_band = true;
            }
            CurvedFragment::CylinderCap { face, boundary }
                if fragment.operand() == cylinder_side
                    && fragment.orientation() == SelectedOrientation::Preserved
                    && *boundary == far_boundary
                    && *face == cylinder.boundaries()[far_boundary].cap_face()
                    && !far_cap =>
            {
                far_cap = true;
            }
            _ => return Ok(None),
        }
    }
    let expected_selected = planar
        .faces()
        .len()
        .checked_add(2)
        .ok_or("support contact selected count overflow")?;
    if planar_faces != planar.faces().len()
        || !side_band
        || !far_cap
        || selected.len() != expected_selected
    {
        return Ok(None);
    }

    let port_uses = planar
        .fragments()
        .iter()
        .find(|fragment| fragment.source_face() == contact.support().id())
        .map(|fragment| fragment.vertices().len())
        .ok_or("support contact port has no source fragment")?;
    if planar
        .fragments()
        .iter()
        .filter(|fragment| fragment.source_face() == contact.support().id())
        .count()
        != 1
    {
        return Ok(None);
    }
    let host_vertices = planar.vertices().len();
    let host_faces = planar.faces().len();
    let host_face_uses = planar
        .edges()
        .len()
        .checked_mul(2)
        .ok_or("support contact face-use count overflow")?;
    let semantic_preflight_work =
        cylindrical_host_dimension_work(host_faces, host_vertices, 1, port_uses)
            .map_err(|_| "support contact semantic work is invalid")?
            .total();
    Ok(Some(SupportContactAdmission {
        host_vertices,
        host_faces,
        host_face_uses,
        semantic_preflight_work,
    }))
}

/// Build the semantic host input after work admission and verify dimensions.
pub(super) fn prepare_admitted_support_contact(
    admission: SupportContactAdmission,
    planar: &ExtractedPlanarSourceBody,
    cylinder: &CertifiedCylinderSource,
    contact: CertifiedAxialCapContact,
) -> Result<PreparedSupportContact, &'static str> {
    let (host, ports) = prepare_curved_host(planar, &[contact.host_face()])?;
    let [port_face] = ports.as_slice() else {
        return Err("support contact did not prepare exactly one host port");
    };
    let boundaries = cylinder.boundaries();
    if contact.boundary() >= boundaries.len() {
        return Err("support contact boundary is invalid");
    }
    let parameters = boundaries.map(|boundary| {
        axial_parameter(cylinder, boundary.center())
            .ok_or("support contact endpoint has no finite axial parameter")
    });
    let [low, high] = parameters;
    let (low, high) = (low?, high?);
    if low >= high {
        return Err("support contact cylinder range must be increasing");
    }
    let far_boundary = 1_usize
        .checked_sub(contact.boundary())
        .ok_or("support contact boundary is invalid")?;
    let endpoints = core::array::from_fn(|boundary| {
        if boundary == contact.boundary() {
            CylindricalHostEndpoint::port(*port_face)
        } else {
            CylindricalHostEndpoint::cap_with_source(boundaries[far_boundary].cap_face())
        }
    });
    let geometry = cylinder.cylinder();
    let band = CylindricalHostBandInput::new(
        *geometry.frame(),
        geometry.radius(),
        ParamRange::new(low, high),
        endpoints,
    )
    .with_side_source(cylinder.side_face());
    let input = CylindricalHostSolidInput::new(host, vec![band]);
    let host_vertices = input.host().vertices().len();
    let host_faces = input.host().faces().len();
    let host_face_uses = input
        .host()
        .faces()
        .iter()
        .try_fold(0_usize, |uses, face| {
            uses.checked_add(face.vertices().len())
        })
        .ok_or("support contact face-use count overflow")?;
    let semantic_preflight_work = cylindrical_host_preflight_work(&input)
        .map_err(|_| "support contact semantic work is invalid")?
        .total();
    if host_vertices != admission.host_vertices()
        || host_faces != admission.host_faces()
        || host_face_uses != admission.host_face_uses()
        || semantic_preflight_work != admission.semantic_preflight_work()
    {
        return Err("support contact dimensions changed after admission");
    }
    Ok(PreparedSupportContact { input })
}

fn operand_side(operand: u8) -> OperandSide {
    if operand == 0 {
        OperandSide::Left
    } else {
        OperandSide::Right
    }
}

fn opposite(side: OperandSide) -> OperandSide {
    match side {
        OperandSide::Left => OperandSide::Right,
        OperandSide::Right => OperandSide::Left,
    }
}

fn axial_parameter(source: &CertifiedCylinderSource, point: kgeom::vec::Point3) -> Option<f64> {
    let cylinder = source.cylinder();
    let frame = cylinder.frame();
    let parameter = (point - frame.origin()).dot(frame.z());
    parameter.is_finite().then_some(parameter)
}

#[cfg(test)]
mod tests {
    use kgeom::frame::Frame;
    use kgeom::vec::{Point3, Vec3};
    use ktopo::check::CheckOutcome;

    use crate::boolean::curved_pipeline::CurvedBooleanPipelineOutcome;
    use crate::boolean::dispatch::{BooleanPipelineOutcome, execute_boolean};
    use crate::boolean::pipeline::PLANAR_BOOLEAN_REALIZATION_WORK;
    use crate::boolean::select::PlanarBooleanOperation;
    use crate::{BlockRequest, CylinderRequest, Kernel, OperationSettings, ResourceKind};

    fn rigid_frame() -> Frame {
        Frame::new(
            Point3::new(1.25, -0.75, 0.5),
            Vec3::new(0.0, 0.6, 0.8),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap()
    }

    #[test]
    fn flush_support_contact_commits_for_both_caps_orders_and_rigid_frames() {
        for block_frame in [Frame::world(), rigid_frame()] {
            for contact_boundary in [0_usize, 1] {
                for swapped in [false, true] {
                    let mut session = Kernel::new().create_session();
                    let part = session.create_part();
                    let (block, cylinder) = {
                        let mut edit = session.edit_part(part.clone()).unwrap();
                        let block = edit
                            .create_block(BlockRequest::new(block_frame, [4.0, 4.0, 2.0]))
                            .unwrap()
                            .into_result()
                            .unwrap()
                            .body();
                        let cylinder_origin = if contact_boundary == 0 {
                            block_frame.point_at(0.3, -0.2, 1.0)
                        } else {
                            block_frame.point_at(0.3, -0.2, -2.5)
                        };
                        let cylinder = edit
                            .create_cylinder(CylinderRequest::new(
                                block_frame.with_origin(cylinder_origin),
                                0.5,
                                1.5,
                            ))
                            .unwrap()
                            .into_result()
                            .unwrap()
                            .body();
                        (block, cylinder)
                    };
                    let (left, right) = if swapped {
                        (cylinder, block)
                    } else {
                        (block, cylinder)
                    };
                    let outcome = execute_boolean(
                        &mut session.edit_part(part.clone()).unwrap(),
                        PlanarBooleanOperation::Unite,
                        left,
                        right,
                        OperationSettings::new(),
                    )
                    .unwrap();
                    let realization_work = outcome
                        .report()
                        .usage()
                        .iter()
                        .find(|usage| {
                            usage.stage == PLANAR_BOOLEAN_REALIZATION_WORK
                                && usage.resource == ResourceKind::Work
                        })
                        .map(|usage| usage.consumed);
                    assert_eq!(
                        realization_work,
                        Some(280),
                        "unexpected contact outcome: {:?}",
                        outcome.result()
                    );
                    let result = outcome.into_result().unwrap();
                    let BooleanPipelineOutcome::Curved(CurvedBooleanPipelineOutcome::Committed(
                        committed,
                    )) = result
                    else {
                        panic!("flush support contact did not commit: {result:?}")
                    };
                    let (bodies, _, checks) = committed.into_parts();
                    assert_eq!(bodies.len(), 1);
                    assert!(
                        checks
                            .iter()
                            .all(|check| check.report().outcome() == CheckOutcome::Valid)
                    );
                    let part_view = session.part(part.clone()).unwrap();
                    let body = part_view.body(bodies[0].clone()).unwrap();
                    assert_eq!(body.faces().unwrap().count(), 8);
                    assert_eq!(body.edges().unwrap().count(), 14);
                    assert_eq!(body.vertices().unwrap().count(), 8);
                }
            }
        }
    }
}
