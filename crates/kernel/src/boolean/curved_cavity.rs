//! Selected-boundary adapter for one complete cylindrical cavity.
//!
//! Recognition is operation-neutral: one complete preserved convex-planar
//! boundary surrounds one complete reversed finite-cylinder boundary. Source
//! constructor identity, Boolean operation, and topology storage order do not
//! participate.

use kgeom::param::ParamRange;
use ktopo::cylindrical_band::CylindricalBandSolidInput;
use ktopo::cylindrical_multishell::CylindricalCavitySolidInput;

use super::boundary_select::{OperandSide, SelectedBoundaryFragment, SelectedOrientation};
use super::curved_host::{prepare_curved_host, source_operand};
use super::curved_pipeline::{CertifiedRingCut, CurvedFragment, CurvedFragmentKey};
use super::curved_source::CertifiedCylinderSource;
use super::extract::ExtractedPlanarSourceBody;
use super::face_partition::{AxialBoundary, FaceRegionKey};

type SelectedCurvedFragment = SelectedBoundaryFragment<CurvedFragmentKey, CurvedFragment>;

/// Semantic cavity proposal plus exact source-size accounting inputs.
#[derive(Debug, Clone)]
pub(super) struct PreparedCylindricalCavity {
    input: CylindricalCavitySolidInput,
    host_vertices: usize,
    host_faces: usize,
    host_face_uses: usize,
}

impl PreparedCylindricalCavity {
    pub(super) fn into_input(self) -> CylindricalCavitySolidInput {
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

/// Recognize a preserved whole host plus one reversed whole cylinder boundary.
///
/// `Ok(None)` means another result topology owns the selected truth. `Err`
/// reports inconsistency in already-certified source evidence.
pub(super) fn prepare_cylindrical_cavity(
    planar: &ExtractedPlanarSourceBody,
    cylinder: &CertifiedCylinderSource,
    cuts: &[CertifiedRingCut],
    selected: &[SelectedCurvedFragment],
) -> Result<Option<PreparedCylindricalCavity>, &'static str> {
    if !cuts.is_empty() {
        return Ok(None);
    }
    let planar_operand = source_operand(planar)?;
    let planar_side = operand_side(planar_operand);
    let cylinder_side = operand_side(planar_operand ^ 1);
    if selected.len() != planar.faces().len().saturating_add(3) {
        return Ok(None);
    }

    let expected_faces = planar
        .faces()
        .iter()
        .map(|face| face.face().raw())
        .collect::<Vec<_>>();
    let mut planar_faces = Vec::with_capacity(expected_faces.len());
    let mut side_seen = false;
    let mut caps_seen = [false; 2];
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
                        lower: AxialBoundary::LowerSource,
                        upper: AxialBoundary::UpperSource,
                    },
            } if selected.operand() == cylinder_side
                && selected.orientation() == SelectedOrientation::Reversed
                && !side_seen =>
            {
                side_seen = true;
            }
            CurvedFragment::CylinderCap { face, boundary }
                if selected.operand() == cylinder_side
                    && selected.orientation() == SelectedOrientation::Reversed
                    && *boundary < cylinder.boundaries().len()
                    && *face == cylinder.boundaries()[*boundary].cap_face()
                    && !caps_seen[*boundary] =>
            {
                caps_seen[*boundary] = true;
            }
            _ => return Ok(None),
        }
    }
    if !side_seen
        || caps_seen != [true; 2]
        || planar_faces.len() != expected_faces.len()
        || expected_faces
            .iter()
            .any(|face| !planar_faces.contains(face))
    {
        return Ok(None);
    }

    let (host, ports) = prepare_curved_host(planar, &[])?;
    if !ports.is_empty() {
        return Err("cylindrical cavity host unexpectedly prepared ports");
    }
    let bounds = cylinder.boundaries();
    let low = axial_parameter(cylinder, bounds[0].center())
        .ok_or("cylindrical cavity lower endpoint has no finite parameter")?;
    let high = axial_parameter(cylinder, bounds[1].center())
        .ok_or("cylindrical cavity upper endpoint has no finite parameter")?;
    if low >= high {
        return Err("cylindrical cavity source range must be increasing");
    }
    let cylinder_geom = cylinder.cylinder();
    let cavity = CylindricalBandSolidInput::new(
        *cylinder_geom.frame(),
        cylinder_geom.radius(),
        ParamRange::new(low, high),
    )
    .with_side_source(cylinder.side_face())
    .with_cap_sources([Some(bounds[0].cap_face()), Some(bounds[1].cap_face())]);
    let host_vertices = host.vertices().len();
    let host_faces = host.faces().len();
    let host_face_uses = host.faces().iter().map(|face| face.vertices().len()).sum();
    Ok(Some(PreparedCylindricalCavity {
        input: CylindricalCavitySolidInput::new(host, cavity),
        host_vertices,
        host_faces,
        host_face_uses,
    }))
}

fn operand_side(operand: u8) -> OperandSide {
    if operand == 0 {
        OperandSide::Left
    } else {
        OperandSide::Right
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
    use kgeom::vec::Point3;
    use ktopo::check::CheckOutcome;
    use ktopo::entity::{EntityRef, RegionKind, Sense};
    use ktopo::geom::SurfaceGeom;
    use ktopo::transaction::LineageEvent;

    use super::super::curved_pipeline::CurvedBooleanPipelineOutcome;
    use super::super::select::PlanarBooleanOperation;
    use crate::{BlockRequest, BodyId, CylinderRequest, Kernel};

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

    fn fixture(reverse_storage: bool) -> (crate::Session, crate::PartId, BodyId, BodyId) {
        let mut session = Kernel::new().create_session();
        let part = session.create_part();
        let (block, cylinder) = {
            let mut edit = session.edit_part(part.clone()).unwrap();
            let block = edit
                .create_block(BlockRequest::new(Frame::world(), [6.0, 5.0, 4.0]))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let cylinder = edit
                .create_cylinder(CylinderRequest::new(
                    Frame::world().with_origin(Point3::new(0.25, -0.2, -0.7)),
                    0.75,
                    1.6,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            if reverse_storage {
                reverse_body_face_storage(&mut edit, &block);
                reverse_body_face_storage(&mut edit, &cylinder);
            }
            (block, cylinder)
        };
        (session, part, block, cylinder)
    }

    fn subtract(
        session: &mut crate::Session,
        part: crate::PartId,
        left: BodyId,
        right: BodyId,
    ) -> CurvedBooleanPipelineOutcome {
        let outcome = super::super::dispatch::execute_boolean(
            &mut session.edit_part(part).unwrap(),
            PlanarBooleanOperation::Subtract,
            left,
            right,
            crate::OperationSettings::new(),
        )
        .unwrap()
        .into_result()
        .unwrap();
        let super::super::dispatch::BooleanPipelineOutcome::Curved(outcome) = outcome else {
            panic!("contained finite cylinder must use the curved pipeline")
        };
        outcome
    }

    #[test]
    fn complete_zero_cut_boundary_is_recognized_as_one_exact_cavity() {
        let (mut session, part, block, cylinder) = fixture(false);
        let outcome = subtract(&mut session, part.clone(), block.clone(), cylinder.clone());
        let CurvedBooleanPipelineOutcome::Committed(committed) = outcome else {
            panic!("complete reversed cylinder boundary must commit, got {outcome:?}")
        };
        let (bodies, journal, checks) = committed.into_parts();
        assert_eq!(bodies.len(), 1);
        assert!(
            checks
                .iter()
                .all(|check| check.report().outcome() == CheckOutcome::Valid)
        );

        let part_view = session.part(part).unwrap();
        let store = &part_view.state.store;
        let result = bodies[0].raw();
        assert_eq!(store.get(result).unwrap().regions().len(), 3);
        let solid = store
            .get(result)
            .unwrap()
            .regions()
            .iter()
            .copied()
            .find(|region| store.get(*region).unwrap().kind() == RegionKind::Solid)
            .unwrap();
        assert_eq!(store.get(solid).unwrap().shells().len(), 2);
        assert_eq!(store.faces_of_body(result).unwrap().len(), 9);
        assert_eq!(store.edges_of_body(result).unwrap().len(), 14);
        assert_eq!(store.vertices_of_body(result).unwrap().len(), 8);

        let result_faces = store.faces_of_body(result).unwrap();
        let block_faces = store.faces_of_body(block.raw()).unwrap();
        let cylinder_faces = store.faces_of_body(cylinder.raw()).unwrap();
        let mut derived = Vec::new();
        let mut block_sources = 0;
        let mut cylinder_sources = 0;
        for event in journal.lineage() {
            let LineageEvent::DerivedFrom {
                derived: EntityRef::Face(face),
                source: EntityRef::Face(source),
            } = event
            else {
                panic!("cavity lineage must be face-only DerivedFrom")
            };
            assert!(result_faces.contains(face));
            assert!(!derived.contains(face));
            derived.push(*face);
            if block_faces.contains(source) {
                block_sources += 1;
            } else if cylinder_faces.contains(source) {
                cylinder_sources += 1;
            } else {
                panic!("cavity lineage escaped both selected sources")
            }
        }
        assert_eq!(derived.len(), result_faces.len());
        assert_eq!((block_sources, cylinder_sources), (6, 3));

        let mut planes = 0;
        let mut reversed_cylinders = 0;
        for face in result_faces {
            let value = store.get(face).unwrap();
            match store.surface(value.surface()).unwrap() {
                SurfaceGeom::Plane(_) => planes += 1,
                SurfaceGeom::Cylinder(_) if value.sense() == Sense::Reversed => {
                    reversed_cylinders += 1;
                }
                surface => panic!("unexpected cavity result surface: {surface:?}"),
            }
        }
        assert_eq!((planes, reversed_cylinders), (8, 1));
    }

    #[test]
    fn storage_order_is_irrelevant_and_reverse_subtraction_stays_empty() {
        let (mut session, part, block, cylinder) = fixture(true);
        let cavity = subtract(&mut session, part.clone(), block.clone(), cylinder.clone());
        let CurvedBooleanPipelineOutcome::Committed(committed) = cavity else {
            panic!("reordered sources must still commit a cavity, got {cavity:?}")
        };
        assert_eq!(committed.into_parts().0.len(), 1);

        let reverse = subtract(&mut session, part, cylinder, block);
        assert!(
            matches!(reverse, CurvedBooleanPipelineOutcome::ProvenEmpty),
            "cylinder minus its containing host must be empty, got {reverse:?}"
        );
    }
}
