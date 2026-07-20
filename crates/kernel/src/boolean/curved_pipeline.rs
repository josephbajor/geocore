//! Proof-bearing convex-planar/finite-cylinder Boolean pipeline.
//!
//! The first curved realization slice is not a primitive case switch. It
//! consumes complete Plane/Cylinder section rings, partitions every affected
//! source face into two-dimensional cells, classifies one anchor per exact
//! dual component, and propagates occupancy across transverse cuts. Generic
//! boundary truth selection then decides which cells survive. The current
//! topology adapter commits selected axial cylinder bands or proof-matched
//! complete source boundaries. Partial, mixed, and reversed whole-boundary
//! classes remain explicit typed refusals.

use std::collections::{BTreeMap, BTreeSet};

use kcore::operation::{AccountingMode, OperationScope, ResourceKind};
use kcore::predicates::{Orientation, affine_dot3};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::vec::Point3;
use ktopo::cylindrical_band::CylindricalBandSolidInput;
use ktopo::entity::{BodyId as RawBodyId, FaceId as RawFaceId};
use ktopo::geom::SurfaceGeom;
use ktopo::transaction::{FullBodyCheck, FullCommitRequirement, Journal};

use super::boundary_select::{
    BoundaryFragmentClassification, BoundarySelectionError, ClassifiedBoundaryFragment,
    OperandSide, RegularizedBooleanOperation, SelectedOrientation, select_boundary_fragments,
};
use super::curved_source::{
    CertifiedCylinderSource, CylinderSourceGap, CylinderSourceOutcome, extract_cylinder_source,
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
use super::pipeline::{
    PLANAR_BOOLEAN_BSP_FRAGMENTS, PLANAR_BOOLEAN_BSP_WORK, PLANAR_BOOLEAN_REALIZATION_WORK,
    PLANAR_BOOLEAN_REALIZED_VERTICES,
};
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
enum PipelineFailure {
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

type StageResult<T> = core::result::Result<T, PipelineFailure>;

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
struct CertifiedRingCut {
    key: usize,
    planar_face: RawFaceId,
    center: Point3,
    planar_representative: PlanarCircleRepresentative,
    axial_parameter: f64,
    exact_order: usize,
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
enum CurvedFragmentKey {
    Planar(FaceCellKey<SourcePlaneRef, usize>),
    CylinderSide(FaceCellKey<u8, usize>),
    CylinderCap { boundary: usize },
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum CurvedFragment {
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

    let graph = section_bodies_in_scope(&edit.as_part(), &bodies[0], &bodies[1], linear, scope)?;
    let cuts = certify_section_rings(&graph, planar_operand, cylinder_operand, &cylinder_source)?;
    precharge_curved_partition(planar_source.faces().len(), cuts.len(), scope)?;
    let classified = build_classified_fragments(
        edit,
        &bodies,
        planar_operand,
        cylinder_operand,
        &planar_source,
        &cylinder_source,
        &cuts,
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
    let proposals = prepare_result_proposals(
        &bodies,
        &source_boundary_keys,
        &cylinder_source,
        &cuts,
        selected,
    )?;
    if proposals.is_empty() {
        return Ok(CurvedBooleanPipelineOutcome::ProvenEmpty);
    }
    commit_proposals(edit, proposals, scope)
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
        let face_cuts = cuts
            .iter()
            .filter(|cut| cut.planar_face == raw_face)
            .map(|cut| CertifiedPlanarCircleCut::new(cut.key, cut.planar_representative))
            .collect::<Vec<_>>();
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
            let classification = *classes
                .get(cell.key())
                .ok_or_else(cell_classification_contract)?;
            output.push(ClassifiedBoundaryFragment::new(
                CurvedFragmentKey::Planar(cell.key().clone()),
                operand_side(operand),
                CurvedFragment::Planar {
                    face: raw_face,
                    region: cell.key().region().clone(),
                },
                adapt_cell_classification(classification),
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
        let classification = classify_anchor(edit, planar_body, evidence.center(), linear, scope)?;
        output.push(ClassifiedBoundaryFragment::new(
            CurvedFragmentKey::CylinderCap { boundary },
            operand_side(operand),
            CurvedFragment::CylinderCap {
                face: evidence.cap_face(),
                boundary,
            },
            adapt_cell_classification(classification),
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

#[derive(Debug, Clone)]
enum PreparedCurvedResult {
    CylindricalBands(Vec<CylindricalBandSolidInput>),
    WholeSources(Vec<RawBodyId>),
}

impl PreparedCurvedResult {
    fn is_empty(&self) -> bool {
        match self {
            Self::CylindricalBands(bands) => bands.is_empty(),
            Self::WholeSources(sources) => sources.is_empty(),
        }
    }
}

fn prepare_result_proposals(
    bodies: &[BodyId; 2],
    source_boundary_keys: &BTreeMap<OperandSide, BTreeSet<CurvedFragmentKey>>,
    cylinder: &CertifiedCylinderSource,
    cuts: &[CertifiedRingCut],
    selected: Vec<
        super::boundary_select::SelectedBoundaryFragment<CurvedFragmentKey, CurvedFragment>,
    >,
) -> StageResult<PreparedCurvedResult> {
    if let Some(source_copies) =
        prepare_whole_source_copies(bodies, source_boundary_keys, &selected)
    {
        return Ok(PreparedCurvedResult::WholeSources(source_copies));
    }
    prepare_band_proposals(cylinder, cuts, selected).map(PreparedCurvedResult::CylindricalBands)
}

/// Accept a copy only when truth selection retained every canonical cell of
/// each represented source boundary and retained no partial source boundary.
/// Reversed whole boundaries describe cavities, so they remain unsupported.
fn prepare_whole_source_copies(
    bodies: &[BodyId; 2],
    source_boundary_keys: &BTreeMap<OperandSide, BTreeSet<CurvedFragmentKey>>,
    selected: &[super::boundary_select::SelectedBoundaryFragment<
        CurvedFragmentKey,
        CurvedFragment,
    >],
) -> Option<Vec<RawBodyId>> {
    let mut selected_keys = BTreeMap::<OperandSide, BTreeSet<CurvedFragmentKey>>::new();
    for fragment in selected {
        if fragment.orientation() != SelectedOrientation::Preserved {
            return None;
        }
        selected_keys
            .entry(fragment.operand())
            .or_default()
            .insert(fragment.key().clone());
    }
    if selected_keys.is_empty() {
        return Some(Vec::new());
    }

    let mut proposals = Vec::with_capacity(selected_keys.len());
    for (operand, keys) in selected_keys {
        if source_boundary_keys.get(&operand) != Some(&keys) {
            return None;
        }
        let body = match operand {
            OperandSide::Left => bodies[0].raw(),
            OperandSide::Right => bodies[1].raw(),
        };
        proposals.push(body);
    }
    Some(proposals)
}

fn prepare_band_proposals(
    source: &CertifiedCylinderSource,
    cuts: &[CertifiedRingCut],
    selected: Vec<
        super::boundary_select::SelectedBoundaryFragment<CurvedFragmentKey, CurvedFragment>,
    >,
) -> StageResult<Vec<CylindricalBandSolidInput>> {
    let mut planar_disks = BTreeMap::<usize, RawFaceId>::new();
    let mut source_caps = [None; 2];
    let mut side_bands = Vec::<(AxialBoundary<usize>, AxialBoundary<usize>)>::new();
    for selected in selected {
        let (_, _, fragment, orientation) = selected.into_parts();
        match (fragment, orientation) {
            (
                CurvedFragment::Planar {
                    face,
                    region: FaceRegionKey::PlanarDisk(cut),
                },
                // Band assembly regenerates result-oriented cap geometry;
                // this selected face supplies lineage only, so reversal is
                // admissible here but not for side or source-cap fragments.
                SelectedOrientation::Preserved | SelectedOrientation::Reversed,
            ) => {
                if planar_disks.insert(cut, face).is_some() {
                    return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
                }
            }
            (
                CurvedFragment::CylinderSide {
                    region: FaceRegionKey::AxialBand { lower, upper },
                },
                SelectedOrientation::Preserved,
            ) => side_bands.push((lower, upper)),
            (CurvedFragment::CylinderCap { face, boundary }, SelectedOrientation::Preserved)
                if boundary < 2 =>
            {
                if source_caps[boundary].replace(face).is_some() {
                    return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
                }
            }
            _ => return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported),
        }
    }

    let cut_data = cuts
        .iter()
        .map(|cut| (cut.key, (cut.exact_order, cut.axial_parameter)))
        .collect::<BTreeMap<_, _>>();
    side_bands.sort_by_key(|(lower, _)| boundary_rank(lower, &cut_data));
    let bounds = source.boundaries();
    let source_parameters = [
        axial_parameter(source, bounds[0].center()).ok_or_else(section_contract)?,
        axial_parameter(source, bounds[1].center()).ok_or_else(section_contract)?,
    ];
    let mut proposals = Vec::with_capacity(side_bands.len());
    for (lower, upper) in side_bands {
        let Some(lower_rank) = boundary_rank(&lower, &cut_data) else {
            return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
        };
        let Some(upper_rank) = boundary_rank(&upper, &cut_data) else {
            return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
        };
        if lower_rank.checked_add(1) != Some(upper_rank) {
            return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
        }
        let low_parameter = boundary_parameter(&lower, source_parameters, &cut_data)
            .ok_or_else(section_contract)?;
        let high_parameter = boundary_parameter(&upper, source_parameters, &cut_data)
            .ok_or_else(section_contract)?;
        let low_face = consume_cap_source(&lower, &mut planar_disks, &mut source_caps)
            .ok_or_else(result_topology_contract)?;
        let high_face = consume_cap_source(&upper, &mut planar_disks, &mut source_caps)
            .ok_or_else(result_topology_contract)?;
        let cylinder = source.cylinder();
        proposals.push(
            CylindricalBandSolidInput::new(
                *cylinder.frame(),
                cylinder.radius(),
                ParamRange::new(low_parameter, high_parameter),
            )
            .with_side_source(source.side_face())
            .with_cap_sources([Some(low_face), Some(high_face)]),
        );
    }
    if !planar_disks.is_empty() || source_caps.iter().any(Option::is_some) {
        return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
    }
    Ok(proposals)
}

fn boundary_rank(
    boundary: &AxialBoundary<usize>,
    cuts: &BTreeMap<usize, (usize, f64)>,
) -> Option<usize> {
    match boundary {
        AxialBoundary::LowerSource => Some(0),
        AxialBoundary::Cut(cut) => cuts.get(cut).and_then(|(order, _)| order.checked_add(1)),
        AxialBoundary::UpperSource => cuts.len().checked_add(1),
    }
}

fn boundary_parameter(
    boundary: &AxialBoundary<usize>,
    source: [f64; 2],
    cuts: &BTreeMap<usize, (usize, f64)>,
) -> Option<f64> {
    match boundary {
        AxialBoundary::LowerSource => Some(source[0]),
        AxialBoundary::Cut(cut) => cuts.get(cut).map(|(_, parameter)| *parameter),
        AxialBoundary::UpperSource => Some(source[1]),
    }
}

fn consume_cap_source(
    boundary: &AxialBoundary<usize>,
    planar_disks: &mut BTreeMap<usize, RawFaceId>,
    source_caps: &mut [Option<RawFaceId>; 2],
) -> Option<RawFaceId> {
    match boundary {
        AxialBoundary::LowerSource => source_caps[0].take(),
        AxialBoundary::Cut(cut) => planar_disks.remove(cut),
        AxialBoundary::UpperSource => source_caps[1].take(),
    }
}

fn commit_proposals(
    edit: &mut PartEdit<'_>,
    proposals: PreparedCurvedResult,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    if let PreparedCurvedResult::WholeSources(sources) = &proposals {
        precharge_whole_source_copies(edit, sources, scope)?;
    }
    let mut transaction = edit.state.store.transaction().map_err(Error::from)?;
    let mut raw_bodies = match &proposals {
        PreparedCurvedResult::CylindricalBands(bands) => Vec::with_capacity(bands.len()),
        PreparedCurvedResult::WholeSources(sources) => Vec::with_capacity(sources.len()),
    };
    match proposals {
        PreparedCurvedResult::CylindricalBands(proposals) => {
            for proposal in proposals {
                let body = match transaction.assemble_cylindrical_band_solid(&proposal) {
                    Ok(output) => output.body(),
                    Err(kcore::error::Error::InvalidGeometry { reason }) => {
                        return refused(CurvedBooleanPipelineRefusal::AssemblyContract(reason));
                    }
                    Err(source) => return Err(source.into()),
                };
                raw_bodies.push(body);
            }
        }
        PreparedCurvedResult::WholeSources(sources) => {
            for source in sources {
                let body = transaction
                    .copy_body_rigid_with_source(source, Frame::world())
                    .map_err(Error::from_body_copy)?;
                raw_bodies.push(body);
            }
        }
    }
    let decision = match transaction.commit_full_in_scope(
        &raw_bodies,
        FullCommitRequirement::RequireValid,
        scope,
        0,
    ) {
        Ok(decision) => decision,
        Err(kcore::error::Error::TopologyCheckFailed { fault_count }) => {
            return refused(CurvedBooleanPipelineRefusal::FullTopologyFault { fault_count });
        }
        Err(source) => return Err(source.into()),
    };
    let (journal, full_checks) = decision.into_parts();
    let Some(journal) = journal else {
        return Ok(CurvedBooleanPipelineOutcome::Refused(
            CurvedBooleanPipelineRefusal::FullProofRejected(full_checks),
        ));
    };
    let bodies = raw_bodies
        .into_iter()
        .map(|body| BodyId::new(edit.id.clone(), body))
        .collect();
    Ok(CurvedBooleanPipelineOutcome::Committed(
        CommittedCurvedBoolean {
            bodies,
            journal,
            full_checks,
        },
    ))
}

/// Charge a conservative visit/allocation bound for identity rigid copies
/// before opening the transaction. Primitive source geometry is leaf-valued;
/// counting geometry per owning use deliberately overcharges shared handles.
fn precharge_whole_source_copies(
    edit: &PartEdit<'_>,
    sources: &[RawBodyId],
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<()> {
    let store = &edit.state.store;
    let mut work = 0_u64;
    let mut vertices = 0_u64;
    for source in sources {
        add_copy_work(&mut work, 1)?;
        for region in store.get(*source)?.regions() {
            add_copy_work(&mut work, 1)?;
            for shell in store.get(*region)?.shells() {
                add_copy_work(&mut work, 1)?;
                for face in store.get(*shell)?.faces() {
                    // One topology identity and one supporting surface use.
                    add_copy_work(&mut work, 2)?;
                    for loop_id in store.get(*face)?.loops() {
                        add_copy_work(&mut work, 1)?;
                        for fin in store.get(*loop_id)?.fins() {
                            // One fin plus at most one pcurve geometry use.
                            add_copy_work(
                                &mut work,
                                if store.get(*fin)?.pcurve().is_some() {
                                    2
                                } else {
                                    1
                                },
                            )?;
                        }
                    }
                }
            }
        }
        for edge in store.edges_of_body(*source)? {
            // One edge plus at most one curve geometry use.
            add_copy_work(
                &mut work,
                if store.get(edge)?.curve().is_some() {
                    2
                } else {
                    1
                },
            )?;
        }
        let source_vertices = store.vertices_of_body(*source)?;
        let vertex_count = u64::try_from(source_vertices.len()).map_err(|_| work_overflow())?;
        vertices = vertices
            .checked_add(vertex_count)
            .ok_or_else(work_overflow)?;
        // One vertex and its point geometry.
        work = work
            .checked_add(vertex_count.checked_mul(2).ok_or_else(work_overflow)?)
            .ok_or_else(work_overflow)?;
    }
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_REALIZATION_WORK, work)
        .map_err(Error::from)?;
    scope
        .ledger_mut()
        .observe(
            PLANAR_BOOLEAN_REALIZED_VERTICES,
            ResourceKind::Items,
            vertices,
        )
        .map_err(Error::from)?;
    Ok(())
}

fn add_copy_work(work: &mut u64, amount: usize) -> StageResult<()> {
    let amount = u64::try_from(amount).map_err(|_| work_overflow())?;
    *work = work.checked_add(amount).ok_or_else(work_overflow)?;
    Ok(())
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
