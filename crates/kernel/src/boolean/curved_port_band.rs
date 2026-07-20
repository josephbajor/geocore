//! Selected-boundary adapter for one finite cylinder band joining two host ports.
//!
//! Recognition is driven by the proof-selected boundary topology: a complete
//! convex-planar host, two exact port cuts, and one reversed cylinder band
//! bounded by those cuts. Boolean operation and primitive fixture identities
//! are deliberately absent.

use kcore::predicates::{Orientation, orient3d};
use kgeom::param::ParamRange;
use kgeom::vec::Point3;
use ktopo::cylindrical_ports::TwoPortCylinderSolidInput;

use super::boundary_select::{OperandSide, SelectedBoundaryFragment, SelectedOrientation};
use super::curved_host::{prepare_curved_host, source_operand, source_plane_for_face};
use super::curved_pipeline::{CertifiedRingCut, CurvedFragment, CurvedFragmentKey};
use super::curved_source::CertifiedCylinderSource;
use super::extract::ExtractedPlanarSourceBody;
use super::face_partition::{AxialBoundary, FaceRegionKey};
use super::planar_bsp::SourcePlane;

type SelectedCurvedFragment = SelectedBoundaryFragment<CurvedFragmentKey, CurvedFragment>;

/// Semantic two-port proposal plus exact source-size accounting inputs.
#[derive(Debug, Clone)]
pub(super) struct PreparedCylindricalPortBand {
    input: TwoPortCylinderSolidInput,
    host_vertices: usize,
    host_faces: usize,
    host_face_uses: usize,
}

impl PreparedCylindricalPortBand {
    pub(super) fn into_input(self) -> TwoPortCylinderSolidInput {
        self.input
    }

    pub(super) const fn host_vertices(&self) -> usize {
        self.host_vertices
    }

    pub(super) const fn host_faces(&self) -> usize {
        self.host_faces
    }

    pub(super) const fn host_face_uses(&self) -> usize {
        self.host_face_uses
    }
}

/// Recognize one reversed cylinder band whose endpoints are two host ports.
///
/// `Ok(None)` means another result topology owns the selected truth. `Err`
/// reports inconsistency in already-certified source evidence.
pub(super) fn prepare_cylindrical_port_band(
    planar: &ExtractedPlanarSourceBody,
    cylinder: &CertifiedCylinderSource,
    cuts: &[CertifiedRingCut],
    selected: &[SelectedCurvedFragment],
) -> Result<Option<PreparedCylindricalPortBand>, &'static str> {
    let [low, high] = cuts else {
        return Ok(None);
    };
    if low.exact_order != 0 || high.exact_order != 1 || low.key == high.key {
        return Err("two-port cylinder cuts have inconsistent exact order");
    }
    if low.planar_face == high.planar_face {
        return Ok(None);
    }

    let planar_operand = source_operand(planar)?;
    let planar_side = operand_side(planar_operand);
    let cylinder_side = operand_side(planar_operand ^ 1);
    if selected.len() != planar.faces().len().saturating_add(1) {
        return Ok(None);
    }

    let expected_faces = planar
        .faces()
        .iter()
        .map(|face| face.face().raw())
        .collect::<Vec<_>>();
    let mut planar_faces = Vec::with_capacity(expected_faces.len());
    let mut side_seen = false;
    for selected in selected {
        match selected.fragment() {
            CurvedFragment::Planar {
                face,
                region: FaceRegionKey::PlanarOuter,
            } if selected.operand() == planar_side
                && selected.orientation() == SelectedOrientation::Preserved
                && expected_faces.contains(face) =>
            {
                if planar_faces.contains(face) {
                    return Ok(None);
                }
                planar_faces.push(*face);
            }
            CurvedFragment::CylinderSide {
                region:
                    FaceRegionKey::AxialBand {
                        lower: AxialBoundary::Cut(lower),
                        upper: AxialBoundary::Cut(upper),
                    },
            } if selected.operand() == cylinder_side
                && selected.orientation() == SelectedOrientation::Reversed
                && *lower == low.key
                && *upper == high.key
                && !side_seen =>
            {
                side_seen = true;
            }
            _ => return Ok(None),
        }
    }
    if !side_seen
        || planar_faces.len() != expected_faces.len()
        || expected_faces
            .iter()
            .any(|face| !planar_faces.contains(face))
    {
        return Ok(None);
    }

    let planes = [
        source_plane_for_face(planar, low.planar_face)?,
        source_plane_for_face(planar, high.planar_face)?,
    ];
    if !certify_two_port_segment(planes, [low.center, high.center]) {
        return Ok(None);
    }

    let (host, port_faces) = prepare_curved_host(planar, &[low.planar_face, high.planar_face])?;
    let [low_port, high_port] = port_faces.as_slice() else {
        return Err("two-port cylinder requires exactly two host ports");
    };
    let axial_range = ordered_cut_range(low.axial_parameter, high.axial_parameter)?;
    let host_vertices = host.vertices().len();
    let host_faces = host.faces().len();
    let host_face_uses = host.faces().iter().map(|face| face.vertices().len()).sum();
    Ok(Some(PreparedCylindricalPortBand {
        input: TwoPortCylinderSolidInput::new(
            host,
            [*low_port, *high_port],
            *cylinder.cylinder().frame(),
            cylinder.cylinder().radius(),
            axial_range,
        )
        .with_side_source(cylinder.side_face()),
        host_vertices,
        host_faces,
        host_face_uses,
    }))
}

fn certify_two_port_segment(planes: [SourcePlane; 2], centers: [Point3; 2]) -> bool {
    let incidence = planes
        .iter()
        .zip(centers)
        .all(|(plane, center)| point_side(*plane, center) == Orientation::Zero);
    let inward = [
        point_side(planes[0], centers[1]) == planes[0].interior_side(),
        point_side(planes[1], centers[0]) == planes[1].interior_side(),
    ];
    incidence && inward.into_iter().all(|value| value)
}

fn point_side(plane: SourcePlane, point: Point3) -> Orientation {
    let witness = plane.points();
    orient3d(witness[0], witness[1], witness[2], point.to_array())
}

fn ordered_cut_range(low: f64, high: f64) -> Result<ParamRange, &'static str> {
    if !low.is_finite() || !high.is_finite() || low >= high {
        return Err("two-port cylinder cut range must be finite and increasing");
    }
    Ok(ParamRange::new(low, high))
}

fn operand_side(operand: u8) -> OperandSide {
    if operand == 0 {
        OperandSide::Left
    } else {
        OperandSide::Right
    }
}

#[cfg(test)]
mod tests {
    use super::super::planar_bsp::SourcePlaneRef;
    use super::*;
    use kgeom::frame::Frame;
    use ktopo::check::CheckOutcome;
    use ktopo::entity::RegionKind;

    use crate::{BlockRequest, BodyId, CylinderRequest, Kernel};

    fn port_plane(face: u32, z: f64, interior_z: f64) -> SourcePlane {
        SourcePlane::from_interior_sample(
            SourcePlaneRef::new(0, face),
            [[-1.0, -1.0, z], [1.0, -1.0, z], [1.0, 1.0, z]],
            [0.0, 0.0, interior_z],
        )
        .unwrap()
    }

    #[test]
    fn exact_supports_certify_two_incident_inward_ports() {
        let planes = [port_plane(0, -1.0, 0.0), port_plane(1, 1.0, 0.0)];
        let centers = [Point3::new(0.0, 0.0, -1.0), Point3::new(0.0, 0.0, 1.0)];
        assert!(certify_two_port_segment(planes, centers));
        assert!(!certify_two_port_segment(
            planes,
            [centers[0], Point3::new(0.0, 0.0, 1.25)]
        ));
        assert!(!certify_two_port_segment(
            [planes[0], port_plane(1, 1.0, 2.0)],
            centers
        ));
    }

    #[test]
    fn exact_cut_range_rejects_reordered_or_collapsed_parameters() {
        assert_eq!(
            ordered_cut_range(-0.5, 1.25).unwrap(),
            ParamRange::new(-0.5, 1.25)
        );
        assert!(ordered_cut_range(1.25, -0.5).is_err());
        assert!(ordered_cut_range(0.0, 0.0).is_err());
        assert!(ordered_cut_range(f64::NAN, 1.0).is_err());
    }

    fn reverse_body_face_storage(edit: &mut crate::session::PartEdit<'_>, body: &BodyId) {
        let store = edit.store_mut_for_test();
        let material = store
            .get(body.raw())
            .unwrap()
            .regions()
            .iter()
            .copied()
            .find(|region| store.get(*region).unwrap().kind() == RegionKind::Solid)
            .unwrap();
        let shell = store.get(material).unwrap().shells()[0];
        let mut transaction = store.transaction().unwrap();
        transaction
            .assembly()
            .get_mut(shell)
            .unwrap()
            .faces
            .reverse();
        transaction.commit_checked_body(body.raw()).unwrap();
    }

    #[test]
    fn two_port_result_and_reverse_subtraction_ignore_face_storage_order() {
        for (cylinder_first, expected_bodies) in [(false, 1), (true, 2)] {
            let mut session = Kernel::new().create_session();
            let part = session.create_part();
            let (block, cylinder) = {
                let mut edit = session.edit_part(part.clone()).unwrap();
                let block = edit
                    .create_block(BlockRequest::new(
                        Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0)),
                        [4.0, 4.0, 1.0],
                    ))
                    .unwrap()
                    .into_result()
                    .unwrap()
                    .body();
                let cylinder = edit
                    .create_cylinder(CylinderRequest::new(Frame::world(), 0.75, 2.0))
                    .unwrap()
                    .into_result()
                    .unwrap()
                    .body();
                reverse_body_face_storage(&mut edit, &block);
                reverse_body_face_storage(&mut edit, &cylinder);
                (block, cylinder)
            };
            let (left, right) = if cylinder_first {
                (cylinder, block)
            } else {
                (block, cylinder)
            };
            let outcome = super::super::dispatch::execute_boolean(
                &mut session.edit_part(part.clone()).unwrap(),
                super::super::select::PlanarBooleanOperation::Subtract,
                left,
                right,
                crate::OperationSettings::new(),
            )
            .unwrap()
            .into_result()
            .unwrap();
            let super::super::dispatch::BooleanPipelineOutcome::Curved(
                super::super::curved_pipeline::CurvedBooleanPipelineOutcome::Committed(committed),
            ) = outcome
            else {
                panic!("expected committed curved subtraction, got {outcome:?}")
            };
            let (bodies, _, full_checks) = committed.into_parts();
            assert_eq!(bodies.len(), expected_bodies);
            assert!(
                full_checks
                    .iter()
                    .all(|check| check.report().outcome() == CheckOutcome::Valid)
            );

            if !cylinder_first {
                let view = session.part(part).unwrap();
                let body = view.body(bodies[0].clone()).unwrap();
                assert_eq!(body.faces().unwrap().len(), 7);
                assert_eq!(body.edges().unwrap().len(), 14);
                assert_eq!(body.vertices().unwrap().len(), 8);
            }
        }
    }
}
