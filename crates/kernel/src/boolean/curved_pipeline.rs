//! Proof-bearing convex-planar/finite-cylinder Boolean pipeline.
//!
//! The first curved realization slice is not a primitive case switch. It
//! consumes complete Plane/Cylinder section rings, partitions every affected
//! source face into two-dimensional cells, classifies one anchor per exact
//! dual component, and propagates occupancy across transverse cuts. Generic
//! boundary truth selection then decides which cells survive. The current
//! topology adapters commit selected axial cylinder bands, one-port capped
//! features, two-port holes, contained cylindrical cavities, or proof-matched
//! complete source boundaries. Other partial and mixed boundary classes remain
//! explicit typed refusals.

use std::collections::{BTreeMap, BTreeSet};

use kcore::operation::{AccountingMode, OperationScope, ResourceKind};
use kcore::predicates::{Orientation, affine_dot3};
use kgeom::vec::Point3;
use ktopo::convex_multishell::{
    certify_mixed_convex_multishell_input, mixed_convex_multishell_dimension_work,
};
use ktopo::entity::FaceId as RawFaceId;
use ktopo::geom::SurfaceGeom;
use ktopo::transaction::{FullBodyCheck, Journal};

use super::boundary_select::{
    BoundaryFragmentClassification, BoundarySelectionError, ClassifiedBoundaryFragment,
    OperandSide, RegularizedBooleanOperation, select_boundary_fragments,
};
use super::convex_containment::prepare_mixed_convex_containment_input;
use super::curved_realize::{CurvedRealizationRequest, realize_selected_result};
use super::curved_source::{
    CertifiedCylinderSource, CylinderSourceGap, CylinderSourceOutcome, extract_cylinder_source,
};
use super::curved_support_separation::{
    CertifiedAxialCapContact, ConvexHostCylinderSupportRelation,
    certify_convex_host_cylinder_support_relation, certify_strict_axial_cap_contact,
};
use super::extract::{
    ExtractedPlanarSourceBody, PlanarSourceExtractionError, PlanarSourceGap,
    PlanarSourceProofFailure, extract_planar_source_body,
};
use super::face_partition::{
    AxialBoundary, CertifiedAxialRingCut, CertifiedPlanarCircleCut, FaceCellClassificationError,
    FaceCellKey, FaceCellOpenClassification, FacePartitionError, FaceRegionKey,
    PlanarCircleRepresentative, classify_face_partition_from_anchor, partition_convex_planar_face,
    partition_periodic_cylinder_face,
};
use super::pipeline::{PLANAR_BOOLEAN_BSP_FRAGMENTS, PLANAR_BOOLEAN_BSP_WORK};
use super::planar_bsp::SourcePlaneRef;
use super::select::PlanarBooleanOperation;
use crate::BodyId;
use crate::classify::{PointBodyVerdict, classify_point_in_body_in_scope};
use crate::error::{Error, Result};
use crate::operation::{BodyCheckReport, adapt_live_body_check};
use crate::section::{
    BodySectionGraph, SectionCarrier, SectionCompletion, SectionCurveFragmentSpan, SectionUvCurve,
    section_bodies_in_scope,
};
use crate::session::PartEdit;

/// One curved result that survived Full checking and committed atomically.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct CommittedCurvedBoolean {
    bodies: Vec<BodyId>,
    journal: Journal,
    full_checks: Vec<FullBodyCheck>,
}

impl CommittedCurvedBoolean {
    pub(super) fn new(
        bodies: Vec<BodyId>,
        journal: Journal,
        full_checks: Vec<FullBodyCheck>,
    ) -> Self {
        Self {
            bodies,
            journal,
            full_checks,
        }
    }

    pub(crate) fn into_parts(self) -> (Vec<BodyId>, Journal, Vec<FullBodyCheck>) {
        (self.bodies, self.journal, self.full_checks)
    }
}

/// Typed, non-persistent refusal from the curved pipeline.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum CurvedBooleanPipelineRefusal {
    PlanarSourceNotFastValid {
        operand: u8,
        report: BodyCheckReport,
    },
    PlanarSourceUnsupported {
        operand: u8,
        gap: PlanarSourceGap,
    },
    PlanarSourceUncertified {
        operand: u8,
        failure: PlanarSourceProofFailure,
    },
    CylinderSourceNotFullValid {
        operand: u8,
        report: BodyCheckReport,
    },
    CylinderSourceUnsupported {
        operand: u8,
        gap: CylinderSourceGap,
    },
    SectionIncomplete,
    Partition(FacePartitionError),
    CellClassification(FaceCellClassificationError),
    ClassificationBoundaryContact,
    ClassificationIndeterminate {
        reason: &'static str,
    },
    Selection(BoundarySelectionError),
    ResultTopologyUnsupported,
    AssemblyContract(&'static str),
    FullTopologyFault {
        fault_count: usize,
    },
    FullProofRejected(Vec<FullBodyCheck>),
    WorkCountOverflow,
}

/// Complete internal outcome from the curved path.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum CurvedBooleanPipelineOutcome {
    ProvenEmpty,
    Committed(CommittedCurvedBoolean),
    Refused(CurvedBooleanPipelineRefusal),
}

#[derive(Debug)]
pub(super) enum PipelineFailure {
    Execution(Error),
    Refused(CurvedBooleanPipelineRefusal),
}

impl From<Error> for PipelineFailure {
    fn from(error: Error) -> Self {
        Self::Execution(error)
    }
}

impl From<kcore::error::Error> for PipelineFailure {
    fn from(error: kcore::error::Error) -> Self {
        Self::Execution(error.into())
    }
}

pub(super) type StageResult<T> = core::result::Result<T, PipelineFailure>;

/// Detect cylinder carriers under the enclosing operation's source budget.
pub(crate) fn cylinder_operand_mask_in_scope(
    edit: &PartEdit<'_>,
    bodies: [&BodyId; 2],
    scope: &mut OperationScope<'_, '_>,
) -> Result<[bool; 2]> {
    scope
        .ledger()
        .require_limit(
            super::extract::PLANAR_SOURCE_EXTRACTION_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )
        .map_err(Error::from)?;
    let mut result = [false; 2];
    for (operand, body) in bodies.into_iter().enumerate() {
        let faces = edit
            .state
            .store
            .faces_of_body(body.raw())
            .map_err(|source| Error::InconsistentTopology { source })?;
        charge_source_scan(scope, faces.len())?;
        for face_id in faces {
            let face = edit
                .state
                .store
                .get(face_id)
                .map_err(|source| Error::InconsistentTopology { source })?;
            if matches!(
                edit.state
                    .store
                    .surface(face.surface())
                    .map_err(|source| Error::InconsistentTopology { source })?,
                SurfaceGeom::Cylinder(_)
            ) {
                result[operand] = true;
            }
        }
    }
    Ok(result)
}

/// Execute the curved stages inside the dispatcher-owned operation scope.
pub(crate) fn execute_curved_in_scope(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    left: BodyId,
    right: BodyId,
    cylinder_mask: [bool; 2],
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<CurvedBooleanPipelineOutcome> {
    match execute_stages(edit, operation, [left, right], cylinder_mask, linear, scope) {
        Ok(outcome) => Ok(outcome),
        Err(PipelineFailure::Execution(error)) => Err(error),
        Err(PipelineFailure::Refused(refusal)) => {
            Ok(CurvedBooleanPipelineOutcome::Refused(refusal))
        }
    }
}

#[derive(Debug, Clone)]
pub(super) struct CertifiedRingCut {
    pub(super) key: usize,
    pub(super) planar_face: RawFaceId,
    pub(super) center: Point3,
    pub(super) planar_representative: PlanarCircleRepresentative,
    pub(super) axial_parameter: f64,
    pub(super) exact_order: usize,
}

fn certify_section_rings(
    graph: &BodySectionGraph,
    planar_operand: usize,
    cylinder_operand: usize,
    cylinder: &CertifiedCylinderSource,
) -> StageResult<Vec<CertifiedRingCut>> {
    if graph.completion() != SectionCompletion::Complete
        || !graph.gaps().is_empty()
        || !graph.edges().is_empty()
        || !graph.loops().is_empty()
        || !graph.curve_endpoints().is_empty()
        || graph.rings().len() != graph.branches().len()
        || graph.curve_fragments().len() != graph.branches().len()
        || graph.curve_components().len() != graph.branches().len()
    {
        return refused(CurvedBooleanPipelineRefusal::SectionIncomplete);
    }
    let ring_branches = graph
        .rings()
        .iter()
        .map(|ring| ring.branch())
        .collect::<BTreeSet<_>>();
    if ring_branches.len() != graph.branches().len()
        || graph
            .curve_components()
            .iter()
            .any(|component| !component.closed() || component.fragments().len() != 1)
    {
        return refused(CurvedBooleanPipelineRefusal::SectionIncomplete);
    }

    let axis = cylinder.cylinder().frame().z().to_array();
    let boundaries = cylinder.boundaries();
    let mut cuts = Vec::with_capacity(graph.branches().len());
    for (branch_index, branch) in graph.branches().iter().enumerate() {
        if !ring_branches.contains(&branch_index)
            || branch.faces()[cylinder_operand].raw() != cylinder.side_face()
        {
            return refused(CurvedBooleanPipelineRefusal::SectionIncomplete);
        }
        let fragments = graph
            .curve_fragments()
            .iter()
            .filter(|fragment| fragment.branch() == branch_index)
            .collect::<Vec<_>>();
        let [fragment] = fragments.as_slice() else {
            return refused(CurvedBooleanPipelineRefusal::SectionIncomplete);
        };
        if !matches!(fragment.span(), SectionCurveFragmentSpan::Whole) {
            return refused(CurvedBooleanPipelineRefusal::SectionIncomplete);
        }
        let SectionCarrier::Circle { center, radius, .. } = branch.carrier();
        if radius != cylinder.cylinder().radius()
            || axis_order(axis, center, boundaries[0].center()) != Some(Orientation::Positive)
            || axis_order(axis, center, boundaries[1].center()) != Some(Orientation::Negative)
        {
            return refused(CurvedBooleanPipelineRefusal::SectionIncomplete);
        }
        let (SectionUvCurve::Circle(plane), SectionUvCurve::Line(side)) = (
            branch.pcurves()[planar_operand],
            branch.pcurves()[cylinder_operand],
        ) else {
            return refused(CurvedBooleanPipelineRefusal::SectionIncomplete);
        };
        if side.direction().y != 0.0 || side.direction().x == 0.0 || !side.origin().y.is_finite() {
            return refused(CurvedBooleanPipelineRefusal::SectionIncomplete);
        }
        cuts.push(CertifiedRingCut {
            key: branch_index,
            planar_face: branch.faces()[planar_operand].raw(),
            center,
            planar_representative: PlanarCircleRepresentative::new(
                [plane.center().x, plane.center().y],
                plane.radius(),
            ),
            axial_parameter: side.origin().y,
            exact_order: 0,
        });
    }
    exact_axis_sort(&mut cuts, axis)?;
    for (exact_order, cut) in cuts.iter_mut().enumerate() {
        cut.exact_order = exact_order;
    }
    Ok(cuts)
}

fn exact_axis_sort(cuts: &mut [CertifiedRingCut], axis: [f64; 3]) -> StageResult<()> {
    for index in 1..cuts.len() {
        let mut cursor = index;
        while cursor > 0 {
            match axis_order(axis, cuts[cursor].center, cuts[cursor - 1].center) {
                Some(Orientation::Negative) => cuts.swap(cursor, cursor - 1),
                Some(Orientation::Positive) => break,
                _ => return refused(CurvedBooleanPipelineRefusal::SectionIncomplete),
            }
            cursor -= 1;
        }
    }
    Ok(())
}

fn axis_order(axis: [f64; 3], point: Point3, origin: Point3) -> Option<Orientation> {
    affine_dot3(axis, point.to_array(), origin.to_array(), 0.0).map(|sign| sign.sign())
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(super) enum CurvedFragmentKey {
    Planar(FaceCellKey<SourcePlaneRef, usize>),
    CylinderSide(FaceCellKey<u8, usize>),
    CylinderCap { boundary: usize },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum CurvedFragment {
    Planar {
        face: RawFaceId,
        region: FaceRegionKey<usize>,
    },
    CylinderSide {
        region: FaceRegionKey<usize>,
    },
    CylinderCap {
        face: RawFaceId,
        boundary: usize,
    },
}

fn execute_stages(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    bodies: [BodyId; 2],
    cylinder_mask: [bool; 2],
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    super::pipeline::validate_pipeline_budget(scope)?;
    let (planar_operand, cylinder_operand) = match cylinder_mask {
        [true, false] => (1_usize, 0_usize),
        [false, true] => (0_usize, 1_usize),
        _ => return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported),
    };

    let (planar_source, cylinder_source) = if cylinder_operand == 0 {
        let cylinder = extract_cylinder_operand(
            edit,
            bodies[cylinder_operand].clone(),
            cylinder_operand as u8,
            scope,
        )?;
        let planar = extract_planar_operand(
            edit,
            bodies[planar_operand].clone(),
            planar_operand as u8,
            scope,
        )?;
        (planar, cylinder)
    } else {
        let planar = extract_planar_operand(
            edit,
            bodies[planar_operand].clone(),
            planar_operand as u8,
            scope,
        )?;
        let cylinder = extract_cylinder_operand(
            edit,
            bodies[cylinder_operand].clone(),
            cylinder_operand as u8,
            scope,
        )?;
        (planar, cylinder)
    };

    if operation == PlanarBooleanOperation::Intersect {
        let relation = certify_convex_host_cylinder_support_relation(
            &edit.state.store,
            &planar_source,
            &cylinder_source,
            scope,
        )?;
        if relation.is_some() {
            return Ok(CurvedBooleanPipelineOutcome::ProvenEmpty);
        }
    }

    let mut contact = None;
    let cuts = if certify_zero_cut_mixed_containment(
        &edit.state.store,
        &planar_source,
        &cylinder_source,
        scope,
    )? {
        Vec::new()
    } else {
        let graph =
            section_bodies_in_scope(&edit.as_part(), &bodies[0], &bodies[1], linear, scope)?;
        match certify_section_rings(&graph, planar_operand, cylinder_operand, &cylinder_source) {
            Ok(cuts) => cuts,
            Err(
                failure @ PipelineFailure::Refused(CurvedBooleanPipelineRefusal::SectionIncomplete),
            ) if operation == PlanarBooleanOperation::Unite => {
                let relation = certify_convex_host_cylinder_support_relation(
                    &edit.state.store,
                    &planar_source,
                    &cylinder_source,
                    scope,
                )?;
                let Some(
                    relation @ ConvexHostCylinderSupportRelation::CertifiedAxialSingleCap { .. },
                ) = relation
                else {
                    return Err(failure);
                };
                let Some(certified) = certify_strict_axial_cap_contact(
                    &edit.state.store,
                    &planar_source,
                    &cylinder_source,
                    relation,
                    scope,
                )?
                else {
                    return Err(failure);
                };
                contact = Some(certified);
                Vec::new()
            }
            Err(failure) => return Err(failure),
        }
    };
    let interfaces = cuts
        .len()
        .checked_add(usize::from(contact.is_some()))
        .ok_or_else(work_overflow)?;
    precharge_curved_partition(planar_source.faces().len(), interfaces, scope)?;
    let classified = build_classified_fragments(
        edit,
        &bodies,
        planar_operand,
        cylinder_operand,
        &planar_source,
        &cylinder_source,
        &cuts,
        contact.as_ref(),
        linear,
        scope,
    )?;
    let source_boundary_keys = classified.iter().fold(
        BTreeMap::<OperandSide, BTreeSet<CurvedFragmentKey>>::new(),
        |mut keys, fragment| {
            keys.entry(fragment.operand())
                .or_default()
                .insert(fragment.key().clone());
            keys
        },
    );
    let selected =
        select_boundary_fragments(adapt_operation(operation), classified).map_err(|error| {
            PipelineFailure::Refused(CurvedBooleanPipelineRefusal::Selection(error))
        })?;
    realize_selected_result(
        edit,
        CurvedRealizationRequest::new(
            &bodies,
            &source_boundary_keys,
            &planar_source,
            &cylinder_source,
            &cuts,
            contact.as_ref(),
            selected,
        ),
        scope,
    )
}

/// Prove the complete planar source strictly inside the convex cylinder.
///
/// This certificate excludes every boundary intersection before the general
/// section path. Failure to establish this optional relation delegates to
/// sectioning; topology/store errors other than a negative semantic relation
/// remain execution failures.
fn certify_zero_cut_mixed_containment(
    store: &ktopo::store::Store,
    planar: &ExtractedPlanarSourceBody,
    cylinder: &CertifiedCylinderSource,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<bool> {
    let preflight_work =
        mixed_convex_multishell_dimension_work(&[(planar.faces().len(), planar.vertices().len())])
            .map_err(|_| {
                PipelineFailure::Refused(CurvedBooleanPipelineRefusal::WorkCountOverflow)
            })?;
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_BSP_WORK, preflight_work)
        .map_err(Error::from)?;
    let prepared = prepare_mixed_convex_containment_input(planar, cylinder).map_err(|reason| {
        PipelineFailure::Refused(CurvedBooleanPipelineRefusal::AssemblyContract(reason))
    })?;
    if prepared.semantic_preflight_work() != preflight_work {
        return refused(CurvedBooleanPipelineRefusal::AssemblyContract(
            "mixed containment semantic work changed after admission",
        ));
    }
    match certify_mixed_convex_multishell_input(prepared.input(), store) {
        Ok(()) => Ok(true),
        Err(kcore::error::Error::InvalidGeometry { .. }) => Ok(false),
        Err(source) => Err(source.into()),
    }
}

fn extract_planar_operand(
    edit: &PartEdit<'_>,
    body: BodyId,
    operand: u8,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<ExtractedPlanarSourceBody> {
    match extract_planar_source_body(&edit.as_part(), body.clone(), operand, scope) {
        Ok(source) => Ok(source),
        Err(PlanarSourceExtractionError::NotFastValid(report)) => {
            let report = adapt_live_body_check(&edit.id, &edit.state.store, body.raw(), report)?;
            refused(CurvedBooleanPipelineRefusal::PlanarSourceNotFastValid { operand, report })
        }
        Err(PlanarSourceExtractionError::Unsupported(gap)) => {
            refused(CurvedBooleanPipelineRefusal::PlanarSourceUnsupported { operand, gap })
        }
        Err(PlanarSourceExtractionError::Uncertified(failure)) => {
            refused(CurvedBooleanPipelineRefusal::PlanarSourceUncertified { operand, failure })
        }
        Err(PlanarSourceExtractionError::Topology(source)) => Err(source.into()),
        Err(PlanarSourceExtractionError::WrongPart) => Err(kcore::error::Error::InvalidGeometry {
            reason: "prevalidated curved Boolean operand changed part",
        }
        .into()),
        Err(PlanarSourceExtractionError::InvalidOperand) => {
            Err(kcore::error::Error::InvalidGeometry {
                reason: "internal curved Boolean operand index is invalid",
            }
            .into())
        }
    }
}

fn extract_cylinder_operand(
    edit: &PartEdit<'_>,
    body: BodyId,
    operand: u8,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CertifiedCylinderSource> {
    match extract_cylinder_source(&edit.state.store, body.raw(), scope)? {
        CylinderSourceOutcome::Ready(source) => Ok(source),
        CylinderSourceOutcome::NotFullValid(report) => {
            let report = adapt_live_body_check(&edit.id, &edit.state.store, body.raw(), report)?;
            refused(CurvedBooleanPipelineRefusal::CylinderSourceNotFullValid { operand, report })
        }
        CylinderSourceOutcome::Unsupported(gap) => {
            refused(CurvedBooleanPipelineRefusal::CylinderSourceUnsupported { operand, gap })
        }
    }
}

fn precharge_curved_partition(
    planar_faces: usize,
    cuts: usize,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<()> {
    let faces = u64::try_from(planar_faces)
        .map_err(|_| refused_error(CurvedBooleanPipelineRefusal::WorkCountOverflow))?;
    let cuts = u64::try_from(cuts)
        .map_err(|_| refused_error(CurvedBooleanPipelineRefusal::WorkCountOverflow))?;
    let cells = faces
        .checked_add(cuts.checked_mul(2).ok_or_else(work_overflow)?)
        .and_then(|value| value.checked_add(3))
        .ok_or_else(work_overflow)?;
    let visits = faces
        .checked_add(cuts)
        .and_then(|value| value.checked_add(1))
        .and_then(|value| value.checked_mul(value))
        .and_then(|value| value.checked_mul(16))
        .ok_or_else(work_overflow)?;
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_BSP_WORK, visits)
        .map_err(Error::from)?;
    scope
        .ledger_mut()
        .observe(
            PLANAR_BOOLEAN_BSP_FRAGMENTS,
            ResourceKind::Items,
            cells.checked_mul(3).ok_or_else(work_overflow)?,
        )
        .map_err(Error::from)?;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn build_classified_fragments(
    edit: &PartEdit<'_>,
    bodies: &[BodyId; 2],
    planar_operand: usize,
    cylinder_operand: usize,
    planar: &ExtractedPlanarSourceBody,
    cylinder: &CertifiedCylinderSource,
    cuts: &[CertifiedRingCut],
    contact: Option<&CertifiedAxialCapContact>,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<Vec<ClassifiedBoundaryFragment<CurvedFragmentKey, CurvedFragment>>> {
    let mut classified = Vec::new();
    append_planar_fragments(
        edit,
        &bodies[cylinder_operand],
        planar_operand as u8,
        planar,
        cuts,
        contact,
        linear,
        scope,
        &mut classified,
    )?;
    append_cylinder_fragments(
        edit,
        &bodies[planar_operand],
        cylinder_operand as u8,
        cylinder,
        cuts,
        contact,
        linear,
        scope,
        &mut classified,
    )?;
    Ok(classified)
}

#[allow(clippy::too_many_arguments)]
fn append_planar_fragments(
    edit: &PartEdit<'_>,
    cylinder_body: &BodyId,
    operand: u8,
    source: &ExtractedPlanarSourceBody,
    cuts: &[CertifiedRingCut],
    contact: Option<&CertifiedAxialCapContact>,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
    output: &mut Vec<ClassifiedBoundaryFragment<CurvedFragmentKey, CurvedFragment>>,
) -> StageResult<()> {
    for source_face in source.faces() {
        let raw_face = source_face.face().raw();
        let source_fragment = source
            .fragments()
            .iter()
            .find(|fragment| fragment.source_face() == source_face.plane())
            .ok_or_else(|| fragment_contract(operand))?;
        let mut face_cuts = cuts
            .iter()
            .filter(|cut| cut.planar_face == raw_face)
            .map(|cut| CertifiedPlanarCircleCut::new(cut.key, cut.planar_representative))
            .collect::<Vec<_>>();
        if let Some(contact) = contact.filter(|contact| contact.host_face() == raw_face) {
            face_cuts.push(CertifiedPlanarCircleCut::new(
                contact.key(),
                contact.planar_representative(),
            ));
        }
        let partition = partition_convex_planar_face(
            source_face.plane(),
            source_fragment.edge_planes().iter().copied(),
            face_cuts,
        )
        .map_err(partition_failure)?;
        let anchor = partition
            .cells()
            .iter()
            .find(|cell| cell.key().region() == &FaceRegionKey::PlanarOuter)
            .ok_or_else(partition_contract)?
            .key()
            .clone();
        let anchor_vertex = *source_fragment
            .vertices()
            .first()
            .ok_or_else(|| fragment_contract(operand))?;
        let anchor_point = source
            .vertices()
            .iter()
            .find(|vertex| vertex.key() == anchor_vertex)
            .map(|vertex| vertex.position())
            .ok_or_else(|| fragment_contract(operand))?;
        let anchor_class = classify_anchor(edit, cylinder_body, anchor_point, linear, scope)?;
        let classes = classify_face_partition_from_anchor(&partition, &anchor, anchor_class)
            .map_err(cell_classification_failure)?;
        for cell in partition.cells() {
            let classification = if contact.is_some_and(|contact| {
                contact.host_face() == raw_face
                    && cell.key().region() == &FaceRegionKey::PlanarDisk(contact.key())
            }) {
                BoundaryFragmentClassification::TwoSided {
                    other_on_source_interior: false,
                    other_on_source_exterior: true,
                }
            } else {
                adapt_cell_classification(
                    *classes
                        .get(cell.key())
                        .ok_or_else(cell_classification_contract)?,
                )
            };
            output.push(ClassifiedBoundaryFragment::new(
                CurvedFragmentKey::Planar(cell.key().clone()),
                operand_side(operand),
                CurvedFragment::Planar {
                    face: raw_face,
                    region: cell.key().region().clone(),
                },
                classification,
            ));
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn append_cylinder_fragments(
    edit: &PartEdit<'_>,
    planar_body: &BodyId,
    operand: u8,
    source: &CertifiedCylinderSource,
    cuts: &[CertifiedRingCut],
    contact: Option<&CertifiedAxialCapContact>,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
    output: &mut Vec<ClassifiedBoundaryFragment<CurvedFragmentKey, CurvedFragment>>,
) -> StageResult<()> {
    let boundaries = source.boundaries();
    let partition = partition_periodic_cylinder_face(
        0_u8,
        0_u8,
        1_u8,
        cuts.iter()
            .map(|cut| CertifiedAxialRingCut::new(cut.key, cut.exact_order, cut.axial_parameter)),
    )
    .map_err(partition_failure)?;
    let first_upper = cuts
        .first()
        .map_or(boundaries[1].center(), |cut| cut.center);
    let anchor = partition
        .cells()
        .iter()
        .find(|cell| {
            cell.key().region()
                == &FaceRegionKey::AxialBand {
                    lower: AxialBoundary::LowerSource,
                    upper: cuts.first().map_or(AxialBoundary::UpperSource, |cut| {
                        AxialBoundary::Cut(cut.key)
                    }),
                }
        })
        .ok_or_else(partition_contract)?
        .key()
        .clone();
    let low_v = axial_parameter(source, boundaries[0].center()).ok_or_else(section_contract)?;
    let upper_v = cuts
        .first()
        .map_or_else(
            || axial_parameter(source, boundaries[1].center()),
            |cut| Some(cut.axial_parameter),
        )
        .ok_or_else(section_contract)?;
    let midpoint = low_v + (upper_v - low_v) * 0.5;
    if !(midpoint.is_finite() && low_v < midpoint && midpoint < upper_v) {
        return refused(CurvedBooleanPipelineRefusal::SectionIncomplete);
    }
    let cylinder = source.cylinder();
    let anchor_point = cylinder.frame().origin()
        + cylinder.frame().z() * midpoint
        + cylinder.frame().x() * cylinder.radius();
    let axis = cylinder.frame().z().to_array();
    if axis_order(axis, anchor_point, boundaries[0].center()) != Some(Orientation::Positive)
        || axis_order(axis, anchor_point, first_upper) != Some(Orientation::Negative)
    {
        return refused(CurvedBooleanPipelineRefusal::SectionIncomplete);
    }
    let anchor_class = classify_anchor(edit, planar_body, anchor_point, linear, scope)?;
    let classes = classify_face_partition_from_anchor(&partition, &anchor, anchor_class)
        .map_err(cell_classification_failure)?;
    for cell in partition.cells() {
        let classification = *classes
            .get(cell.key())
            .ok_or_else(cell_classification_contract)?;
        output.push(ClassifiedBoundaryFragment::new(
            CurvedFragmentKey::CylinderSide(cell.key().clone()),
            operand_side(operand),
            CurvedFragment::CylinderSide {
                region: cell.key().region().clone(),
            },
            adapt_cell_classification(classification),
        ));
    }

    for (boundary, evidence) in boundaries.iter().enumerate() {
        let classification = if contact.is_some_and(|contact| contact.boundary() == boundary) {
            BoundaryFragmentClassification::TwoSided {
                other_on_source_interior: false,
                other_on_source_exterior: true,
            }
        } else {
            adapt_cell_classification(classify_anchor(
                edit,
                planar_body,
                evidence.center(),
                linear,
                scope,
            )?)
        };
        output.push(ClassifiedBoundaryFragment::new(
            CurvedFragmentKey::CylinderCap { boundary },
            operand_side(operand),
            CurvedFragment::CylinderCap {
                face: evidence.cap_face(),
                boundary,
            },
            classification,
        ));
    }
    Ok(())
}

fn classify_anchor(
    edit: &PartEdit<'_>,
    body: &BodyId,
    point: Point3,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<FaceCellOpenClassification> {
    let classification =
        classify_point_in_body_in_scope(&edit.as_part(), body, point, linear, scope)?;
    match classification.verdict() {
        PointBodyVerdict::Interior => Ok(FaceCellOpenClassification::Interior),
        PointBodyVerdict::Exterior => Ok(FaceCellOpenClassification::Exterior),
        PointBodyVerdict::Boundary { .. } => {
            refused(CurvedBooleanPipelineRefusal::ClassificationBoundaryContact)
        }
        PointBodyVerdict::Indeterminate { reason } => {
            refused(CurvedBooleanPipelineRefusal::ClassificationIndeterminate { reason })
        }
    }
}

fn adapt_cell_classification(
    classification: FaceCellOpenClassification,
) -> BoundaryFragmentClassification {
    match classification {
        FaceCellOpenClassification::Interior => BoundaryFragmentClassification::Interior,
        FaceCellOpenClassification::Exterior => BoundaryFragmentClassification::Exterior,
    }
}

fn operand_side(operand: u8) -> OperandSide {
    if operand == 0 {
        OperandSide::Left
    } else {
        OperandSide::Right
    }
}

fn adapt_operation(operation: PlanarBooleanOperation) -> RegularizedBooleanOperation {
    match operation {
        PlanarBooleanOperation::Unite => RegularizedBooleanOperation::Unite,
        PlanarBooleanOperation::Intersect => RegularizedBooleanOperation::Intersect,
        PlanarBooleanOperation::Subtract => RegularizedBooleanOperation::Subtract,
    }
}

fn axial_parameter(source: &CertifiedCylinderSource, point: Point3) -> Option<f64> {
    let cylinder = source.cylinder();
    let frame = cylinder.frame();
    let parameter = (point - frame.origin()).dot(frame.z());
    parameter.is_finite().then_some(parameter)
}

fn charge_source_scan(scope: &mut OperationScope<'_, '_>, amount: usize) -> Result<()> {
    let amount = u64::try_from(amount).map_err(|_| Error::Core {
        source: kcore::error::Error::InvalidGeometry {
            reason: "curved Boolean source dispatch exceeds u64 accounting",
        },
    })?;
    scope
        .ledger_mut()
        .charge(super::extract::PLANAR_SOURCE_EXTRACTION_WORK, amount)
        .map_err(Error::from)
}

fn partition_failure(error: FacePartitionError) -> PipelineFailure {
    refused_error(CurvedBooleanPipelineRefusal::Partition(error))
}

fn cell_classification_failure(error: FaceCellClassificationError) -> PipelineFailure {
    refused_error(CurvedBooleanPipelineRefusal::CellClassification(error))
}

fn fragment_contract(operand: u8) -> PipelineFailure {
    refused_error(CurvedBooleanPipelineRefusal::PlanarSourceUncertified {
        operand,
        failure: PlanarSourceProofFailure::FragmentContract,
    })
}

fn partition_contract() -> PipelineFailure {
    refused_error(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported)
}

fn cell_classification_contract() -> PipelineFailure {
    refused_error(CurvedBooleanPipelineRefusal::CellClassification(
        FaceCellClassificationError::DisconnectedDualGraph,
    ))
}

fn section_contract() -> PipelineFailure {
    refused_error(CurvedBooleanPipelineRefusal::SectionIncomplete)
}

fn result_topology_contract() -> PipelineFailure {
    refused_error(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported)
}

fn work_overflow() -> PipelineFailure {
    refused_error(CurvedBooleanPipelineRefusal::WorkCountOverflow)
}

fn refused_error(refusal: CurvedBooleanPipelineRefusal) -> PipelineFailure {
    PipelineFailure::Refused(refusal)
}

fn refused<T>(refusal: CurvedBooleanPipelineRefusal) -> StageResult<T> {
    Err(refused_error(refusal))
}

#[cfg(test)]
mod tests {
    use kgeom::frame::Frame;
    use kgeom::vec::{Point3, Vec3};
    use ktopo::check::CheckOutcome;
    use ktopo::entity::RegionKind;

    use super::*;
    use crate::{BlockRequest, CylinderRequest, Kernel};

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
    fn capped_features_ignore_operand_and_face_storage_order() {
        for (operation, cylinder_first) in [
            (PlanarBooleanOperation::Unite, false),
            (PlanarBooleanOperation::Unite, true),
            (PlanarBooleanOperation::Subtract, false),
        ] {
            let mut session = Kernel::new().create_session();
            let part = session.create_part();
            let (block, cylinder) = {
                let mut edit = session.edit_part(part.clone()).unwrap();
                let block = edit
                    .create_block(BlockRequest::new(Frame::world(), [4.0, 4.0, 2.0]))
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
                &mut session.edit_part(part).unwrap(),
                operation,
                left,
                right,
                crate::OperationSettings::new(),
            )
            .unwrap()
            .into_result()
            .unwrap();
            let super::super::dispatch::BooleanPipelineOutcome::Curved(
                CurvedBooleanPipelineOutcome::Committed(committed),
            ) = outcome
            else {
                panic!("expected committed capped feature, got {outcome:?}")
            };
            assert_eq!(committed.bodies.len(), 1);
            assert!(
                committed
                    .full_checks
                    .iter()
                    .all(|check| check.report().outcome() == CheckOutcome::Valid)
            );
        }
    }

    #[test]
    fn axial_band_results_ignore_both_operand_face_storage_orders() {
        let mut session = Kernel::new().create_session();
        let part = session.create_part();
        let base = Point3::new(3.0, -2.0, 1.25);
        let cylinder_frame =
            Frame::new(base, Vec3::new(0.0, 0.6, 0.8), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let block_frame = cylinder_frame.with_origin(base + cylinder_frame.z());
        let (block, cylinder) = {
            let mut edit = session.edit_part(part.clone()).unwrap();
            let block = edit
                .create_block(BlockRequest::new(block_frame, [4.0, 4.0, 1.0]))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let cylinder = edit
                .create_cylinder(CylinderRequest::new(cylinder_frame, 0.75, 2.0))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            reverse_body_face_storage(&mut edit, &block);
            reverse_body_face_storage(&mut edit, &cylinder);
            (block, cylinder)
        };

        for (operation, left, right, expected_bodies) in [
            (
                PlanarBooleanOperation::Intersect,
                block.clone(),
                cylinder.clone(),
                1,
            ),
            (PlanarBooleanOperation::Subtract, cylinder, block, 2),
        ] {
            let outcome = super::super::dispatch::execute_boolean(
                &mut session.edit_part(part.clone()).unwrap(),
                operation,
                left,
                right,
                crate::OperationSettings::new(),
            )
            .unwrap()
            .into_result()
            .unwrap();
            let super::super::dispatch::BooleanPipelineOutcome::Curved(
                CurvedBooleanPipelineOutcome::Committed(committed),
            ) = outcome
            else {
                panic!("expected committed curved result, got {outcome:?}")
            };
            assert_eq!(committed.bodies.len(), expected_bodies);
        }
    }

    #[test]
    fn zero_cut_contained_cylinder_is_one_complete_source_copy() {
        let mut session = Kernel::new().create_session();
        let part = session.create_part();
        let (block, cylinder) = {
            let mut edit = session.edit_part(part.clone()).unwrap();
            let block = edit
                .create_block(BlockRequest::new(Frame::world(), [6.0, 6.0, 6.0]))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let cylinder = edit
                .create_cylinder(CylinderRequest::new(
                    Frame::world().with_origin(Point3::new(0.0, 0.0, -1.0)),
                    0.75,
                    2.0,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (block, cylinder)
        };
        let outcome = super::super::dispatch::execute_boolean(
            &mut session.edit_part(part).unwrap(),
            PlanarBooleanOperation::Intersect,
            block,
            cylinder,
            crate::OperationSettings::new(),
        )
        .unwrap()
        .into_result()
        .unwrap();
        assert!(
            matches!(
                outcome,
                super::super::dispatch::BooleanPipelineOutcome::Curved(
                    CurvedBooleanPipelineOutcome::Committed(_)
                )
            ),
            "outcome: {outcome:?}"
        );
    }

    #[test]
    fn zero_cut_disjoint_cylinder_is_proven_empty() {
        let mut session = Kernel::new().create_session();
        let part = session.create_part();
        let (block, cylinder) = {
            let mut edit = session.edit_part(part.clone()).unwrap();
            let block = edit
                .create_block(BlockRequest::new(
                    Frame::world().with_origin(Point3::new(8.0, 0.0, 0.0)),
                    [2.0, 2.0, 2.0],
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let cylinder = edit
                .create_cylinder(CylinderRequest::new(
                    Frame::world().with_origin(Point3::new(-8.0, 0.0, -1.0)),
                    0.75,
                    2.0,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (block, cylinder)
        };
        let outcome = super::super::dispatch::execute_boolean(
            &mut session.edit_part(part).unwrap(),
            PlanarBooleanOperation::Intersect,
            block,
            cylinder,
            crate::OperationSettings::new(),
        )
        .unwrap()
        .into_result()
        .unwrap();
        assert!(
            matches!(
                outcome,
                super::super::dispatch::BooleanPipelineOutcome::Curved(
                    CurvedBooleanPipelineOutcome::ProvenEmpty
                )
            ),
            "outcome: {outcome:?}"
        );
    }
}
