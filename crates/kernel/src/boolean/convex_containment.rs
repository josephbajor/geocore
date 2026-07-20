//! Complete-boundary recognition for mixed convex containment results.
//!
//! Recognition is independent of Boolean operation and source constructor. It
//! accepts only two complete, unsplit source boundaries selected with opposite
//! winding: the finite-cylinder boundary is preserved as the positive outer
//! shell and the convex-planar boundary is reversed as the negative cavity
//! shell. Semantic geometry preparation remains separate from this exact
//! identity-and-orientation proof.

use std::collections::{BTreeMap, BTreeSet};

use kgeom::param::ParamRange;
use ktopo::convex_multishell::{
    MixedConvexMultiShellSolidInput, OrientedWholeShellInput,
    mixed_convex_multishell_dimension_work, mixed_convex_multishell_preflight_work,
};

use super::boundary_select::{OperandSide, SelectedBoundaryFragment, SelectedOrientation};
use super::curved_host::{prepare_curved_host, source_operand};
use super::curved_pipeline::{CertifiedRingCut, CurvedFragment, CurvedFragmentKey};
use super::curved_source::CertifiedCylinderSource;
use super::extract::ExtractedPlanarSourceBody;

type SelectedCurvedFragment = SelectedBoundaryFragment<CurvedFragmentKey, CurvedFragment>;

/// Recognize two complete source boundaries and return their selected winding.
///
/// `None` means the selected truth is partial, repeats a cell, omits one
/// source, or mixes winding within a source boundary. The comparison is over
/// canonical source-cell identities and is therefore independent of storage
/// and traversal order.
fn complete_source_orientations<K: Ord, F>(
    source_boundary_keys: &BTreeMap<OperandSide, BTreeSet<K>>,
    selected: &[SelectedBoundaryFragment<K, F>],
) -> Option<[SelectedOrientation; 2]> {
    if source_boundary_keys.len() != 2 || source_boundary_keys.values().any(BTreeSet::is_empty) {
        return None;
    }
    let registries = [
        source_boundary_keys.get(&OperandSide::Left)?,
        source_boundary_keys.get(&OperandSide::Right)?,
    ];
    let mut selected_counts = [0_usize; 2];
    let mut orientations = [None; 2];
    for (position, fragment) in selected.iter().enumerate() {
        if selected[..position]
            .iter()
            .any(|prior| prior.operand() == fragment.operand() && prior.key() == fragment.key())
        {
            return None;
        }
        let index = operand_index(fragment.operand());
        if !registries[index].contains(fragment.key()) {
            return None;
        }
        selected_counts[index] = selected_counts[index].checked_add(1)?;
        if orientations[index].is_some_and(|value| value != fragment.orientation()) {
            return None;
        }
        orientations[index] = Some(fragment.orientation());
    }
    if selected_counts != [registries[0].len(), registries[1].len()] {
        return None;
    }
    Some([orientations[0]?, orientations[1]?])
}

/// Operand roles proven by one mixed convex-containment boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct MixedConvexContainment {
    outer: OperandSide,
    cavity: OperandSide,
}

/// Allocation-free result-admission accounting for one recognized relation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct MixedConvexContainmentAdmission {
    planar_vertices: usize,
    planar_faces: usize,
    planar_face_uses: usize,
    semantic_preflight_work: u64,
}

impl MixedConvexContainmentAdmission {
    pub(super) const fn planar_vertices(self) -> usize {
        self.planar_vertices
    }

    pub(super) const fn planar_faces(self) -> usize {
        self.planar_faces
    }

    pub(super) const fn planar_face_uses(self) -> usize {
        self.planar_face_uses
    }

    pub(super) const fn semantic_preflight_work(self) -> u64 {
        self.semantic_preflight_work
    }
}

/// Prepared semantic assembly and its source-size accounting inputs.
#[derive(Debug, Clone)]
pub(super) struct PreparedMixedConvexContainment {
    input: MixedConvexMultiShellSolidInput,
    planar_vertices: usize,
    planar_faces: usize,
    planar_face_uses: usize,
    semantic_preflight_work: u64,
}

impl PreparedMixedConvexContainment {
    pub(super) const fn input(&self) -> &MixedConvexMultiShellSolidInput {
        &self.input
    }

    pub(super) fn into_input(self) -> MixedConvexMultiShellSolidInput {
        self.input
    }

    pub(super) const fn planar_vertices(&self) -> usize {
        self.planar_vertices
    }

    pub(super) const fn planar_faces(&self) -> usize {
        self.planar_faces
    }

    pub(super) const fn planar_face_uses(&self) -> usize {
        self.planar_face_uses
    }

    pub(super) const fn semantic_preflight_work(&self) -> u64 {
        self.semantic_preflight_work
    }
}

impl MixedConvexContainment {
    /// Operand whose complete source boundary keeps positive winding.
    pub(super) const fn outer(self) -> OperandSide {
        self.outer
    }

    /// Operand whose complete source boundary supplies the negative shell.
    pub(super) const fn cavity(self) -> OperandSide {
        self.cavity
    }
}

/// Recognize one preserved whole shell and one reversed whole shell.
pub(super) fn recognize_mixed_convex_containment<K: Ord, F>(
    source_boundary_keys: &BTreeMap<OperandSide, BTreeSet<K>>,
    selected: &[SelectedBoundaryFragment<K, F>],
) -> Option<MixedConvexContainment> {
    let orientations = complete_source_orientations(source_boundary_keys, selected)?;
    match orientations {
        [
            SelectedOrientation::Preserved,
            SelectedOrientation::Reversed,
        ] => Some(MixedConvexContainment {
            outer: OperandSide::Left,
            cavity: OperandSide::Right,
        }),
        [
            SelectedOrientation::Reversed,
            SelectedOrientation::Preserved,
        ] => Some(MixedConvexContainment {
            outer: OperandSide::Right,
            cavity: OperandSide::Left,
        }),
        _ => None,
    }
}

/// Prepare the carrier inputs for a recognized mixed whole-shell containment.
///
/// `Ok(None)` delegates every other selected boundary class. The only carrier
/// specialization here maps the generic shell roles onto the currently
/// admitted positive finite-cylinder and negative convex-planar assemblers.
pub(super) fn admit_mixed_convex_containment(
    source_boundary_keys: &BTreeMap<OperandSide, BTreeSet<CurvedFragmentKey>>,
    planar: &ExtractedPlanarSourceBody,
    cuts: &[CertifiedRingCut],
    selected: &[SelectedCurvedFragment],
) -> Result<Option<MixedConvexContainmentAdmission>, &'static str> {
    if !cuts.is_empty() {
        return Ok(None);
    }
    let planar_side = operand_side(source_operand(planar)?);
    let cylinder_side = opposite(planar_side);
    let Some(roles) = recognize_mixed_convex_containment(source_boundary_keys, selected) else {
        return Ok(None);
    };
    if roles.outer() != cylinder_side || roles.cavity() != planar_side {
        return Ok(None);
    }
    let planar_vertices = planar.vertices().len();
    let planar_faces = planar.faces().len();
    // Extraction has already certified a closed two-manifold. Every source
    // edge therefore contributes exactly two face uses, so admission can use
    // source dimensions without scanning semantic fragment payloads.
    let planar_face_uses = planar
        .edges()
        .len()
        .checked_mul(2)
        .ok_or("mixed convex containment face-use count overflow")?;
    let semantic_preflight_work =
        mixed_convex_multishell_dimension_work(&[(planar_faces, planar_vertices)])
            .map_err(|_| "mixed convex containment preflight work is invalid")?;
    Ok(Some(MixedConvexContainmentAdmission {
        planar_vertices,
        planar_faces,
        planar_face_uses,
        semantic_preflight_work,
    }))
}

/// Materialize semantic inputs only after result work admission succeeds.
pub(super) fn prepare_admitted_mixed_convex_containment(
    admission: MixedConvexContainmentAdmission,
    planar: &ExtractedPlanarSourceBody,
    cylinder: &CertifiedCylinderSource,
) -> Result<PreparedMixedConvexContainment, &'static str> {
    let prepared = prepare_mixed_convex_containment_input(planar, cylinder)?;
    if prepared.planar_vertices() != admission.planar_vertices()
        || prepared.planar_faces() != admission.planar_faces()
        || prepared.planar_face_uses() != admission.planar_face_uses()
        || prepared.semantic_preflight_work() != admission.semantic_preflight_work()
    {
        return Err("mixed convex containment dimensions changed after admission");
    }
    Ok(prepared)
}

/// Prepare canonical positive cylinder and planar shell inputs without
/// assigning them a Boolean result role.
pub(super) fn prepare_mixed_convex_containment_input(
    planar: &ExtractedPlanarSourceBody,
    cylinder: &CertifiedCylinderSource,
) -> Result<PreparedMixedConvexContainment, &'static str> {
    let (planar_input, ports) = prepare_curved_host(planar, &[])?;
    if !ports.is_empty() {
        return Err("mixed convex containment unexpectedly prepared planar ports");
    }
    let planar_vertices = planar_input.vertices().len();
    let planar_faces = planar_input.faces().len();
    let planar_face_uses = planar_input
        .faces()
        .iter()
        .try_fold(0_usize, |uses, face| {
            uses.checked_add(face.vertices().len())
        })
        .ok_or("mixed convex containment face-use count overflow")?;
    let bounds = cylinder.boundaries();
    let low = axial_parameter(cylinder, bounds[0].center())
        .ok_or("mixed convex containment lower endpoint has no finite parameter")?;
    let high = axial_parameter(cylinder, bounds[1].center())
        .ok_or("mixed convex containment upper endpoint has no finite parameter")?;
    if low >= high {
        return Err("mixed convex containment cylinder range must be increasing");
    }
    let geometry = cylinder.cylinder();
    let cylinder_input = ktopo::cylindrical_band::CylindricalBandSolidInput::new(
        *geometry.frame(),
        geometry.radius(),
        ParamRange::new(low, high),
    )
    .with_side_source(cylinder.side_face())
    .with_cap_sources([Some(bounds[0].cap_face()), Some(bounds[1].cap_face())]);
    let input = MixedConvexMultiShellSolidInput::new(
        OrientedWholeShellInput::Cylindrical(cylinder_input),
        vec![OrientedWholeShellInput::Planar(planar_input)],
    );
    let semantic_preflight_work = mixed_convex_multishell_preflight_work(&input)
        .map_err(|_| "mixed convex containment preflight work is invalid")?;
    Ok(PreparedMixedConvexContainment {
        input,
        planar_vertices,
        planar_faces,
        planar_face_uses,
        semantic_preflight_work,
    })
}

fn operand_side(operand: u8) -> OperandSide {
    if operand == 0 {
        OperandSide::Left
    } else {
        OperandSide::Right
    }
}

const fn operand_index(operand: OperandSide) -> usize {
    match operand {
        OperandSide::Left => 0,
        OperandSide::Right => 1,
    }
}

fn opposite(operand: OperandSide) -> OperandSide {
    match operand {
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
    use kgeom::vec::Point3;

    use super::*;
    use crate::boolean::boundary_select::{
        BoundaryFragmentClassification, ClassifiedBoundaryFragment, RegularizedBooleanOperation,
        select_boundary_fragments,
    };

    fn selected(
        operation: RegularizedBooleanOperation,
        fragments: &[(OperandSide, u8, BoundaryFragmentClassification)],
    ) -> Vec<SelectedBoundaryFragment<u8, ()>> {
        select_boundary_fragments(
            operation,
            fragments.iter().copied().map(|(operand, key, class)| {
                ClassifiedBoundaryFragment::new(key, operand, (), class)
            }),
        )
        .unwrap()
    }

    #[test]
    fn complete_boundary_recognition_is_order_and_count_neutral() {
        use BoundaryFragmentClassification::{Exterior, Interior};
        use OperandSide::{Left, Right};
        let source = BTreeMap::from([
            (Left, BTreeSet::from([1, 2, 3])),
            (Right, BTreeSet::from([4, 5])),
        ]);
        let selected = selected(
            RegularizedBooleanOperation::Subtract,
            &[
                (Right, 5, Interior),
                (Left, 2, Exterior),
                (Left, 1, Exterior),
                (Right, 4, Interior),
                (Left, 3, Exterior),
            ],
        );
        assert_eq!(
            complete_source_orientations(&source, &selected),
            Some([
                SelectedOrientation::Preserved,
                SelectedOrientation::Reversed,
            ])
        );
        assert_eq!(
            recognize_mixed_convex_containment(&source, &selected),
            Some(MixedConvexContainment {
                outer: Left,
                cavity: Right,
            })
        );
    }

    #[test]
    fn partial_or_same_winding_boundaries_are_not_containment_proofs() {
        use BoundaryFragmentClassification::{Exterior, Interior};
        use OperandSide::{Left, Right};
        let source = BTreeMap::from([
            (Left, BTreeSet::from([1, 2])),
            (Right, BTreeSet::from([3, 4])),
        ]);
        let partial = selected(
            RegularizedBooleanOperation::Subtract,
            &[
                (Left, 1, Exterior),
                (Right, 3, Interior),
                (Right, 4, Interior),
            ],
        );
        assert_eq!(complete_source_orientations(&source, &partial), None);

        let preserved = selected(
            RegularizedBooleanOperation::Unite,
            &[
                (Left, 1, Exterior),
                (Left, 2, Exterior),
                (Right, 3, Exterior),
                (Right, 4, Exterior),
            ],
        );
        let orientations = complete_source_orientations(&source, &preserved).unwrap();
        assert_eq!(orientations, [SelectedOrientation::Preserved; 2]);
        assert_eq!(
            recognize_mixed_convex_containment(&source, &preserved),
            None
        );
    }

    #[test]
    fn incomplete_or_empty_source_registries_fail_closed() {
        use BoundaryFragmentClassification::Exterior;
        use OperandSide::{Left, Right};
        let one_source = BTreeMap::from([(Left, BTreeSet::from([1]))]);
        let selected = selected(RegularizedBooleanOperation::Unite, &[(Left, 1, Exterior)]);
        assert_eq!(complete_source_orientations(&one_source, &selected), None);

        let empty_source = BTreeMap::from([(Left, BTreeSet::from([1])), (Right, BTreeSet::new())]);
        assert_eq!(complete_source_orientations(&empty_source, &selected), None);
    }

    #[test]
    fn contained_planar_boundary_reaches_mixed_shell_realization() {
        use crate::{BlockRequest, CylinderRequest, Kernel, OperationSettings};

        let rigid = Frame::new(
            Point3::new(3.0, -2.0, 1.25),
            kgeom::vec::Vec3::new(0.0, 0.6, 0.8),
            kgeom::vec::Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        for frame in [Frame::world(), rigid] {
            let mut session = Kernel::new().create_session();
            let part = session.create_part();
            let (cylinder, block) = {
                let mut edit = session.edit_part(part.clone()).unwrap();
                let cylinder = edit
                    .create_cylinder(CylinderRequest::new(
                        frame.with_origin(frame.point_at(0.0, 0.0, -2.0)),
                        2.0,
                        4.0,
                    ))
                    .unwrap()
                    .into_result()
                    .unwrap()
                    .body();
                let block = edit
                    .create_block(BlockRequest::new(frame, [1.0, 1.0, 1.0]))
                    .unwrap()
                    .into_result()
                    .unwrap()
                    .body();
                (cylinder, block)
            };
            let outcome = super::super::dispatch::execute_boolean(
                &mut session.edit_part(part).unwrap(),
                super::super::select::PlanarBooleanOperation::Subtract,
                cylinder,
                block,
                OperationSettings::new(),
            )
            .unwrap()
            .into_result()
            .unwrap();
            let super::super::dispatch::BooleanPipelineOutcome::Curved(outcome) = outcome else {
                panic!("mixed containment must use the curved path")
            };
            assert!(
                matches!(
                    outcome,
                    super::super::curved_pipeline::CurvedBooleanPipelineOutcome::Committed(_)
                ),
                "mixed containment must commit, got {outcome:?}"
            );
        }
    }
}
