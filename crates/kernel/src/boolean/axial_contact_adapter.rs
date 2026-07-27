//! Operation-local arrangement adapters for exact parallel-cylinder relations.

#![allow(clippy::result_large_err)]

use kcore::expansion::two_sum;
use kcore::interval::Interval;
use kcore::operation::OperationScope;
use kcore::predicates::{Orientation, affine_dot3, orient3d, squared_distance_difference3};
use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::curve::Circle;
use kgeom::vec::{Point3, Vec3};
use ktopo::geom::CurveGeom;
use ktopo::store::Store;

use super::boundary_select::{
    BoundaryFragmentClassification, ClassifiedBoundaryFragment, select_boundary_fragments,
};
use super::curved_pipeline::{
    CurvedBooleanPipelineOutcome, CurvedBooleanPipelineRefusal, PipelineFailure, StageResult,
    adapt_operation, mixed_plan_failure, realize_mixed_shell,
};
use super::curved_realize::realize_certified_cylinder_source_copies;
use super::curved_source::CertifiedCylinderSource;
use super::disk_face_arrangement::{
    ArrangedDiskFace, DiskCellClassification, arrange_section_disk_face_from_fragment_subset,
};
use super::mixed_boundary::{
    MixedBoundaryError, classify_disk_face_from_source_offset, operand_side,
};
use super::mixed_cap_boundary::{
    MixedCylinderCapRing, bind_cylinder_cap_ring_from_embedding, classified_exterior_cap,
};
use super::mixed_periodic_arrangement::{
    MixedPeriodicFaceArrangement, arrange_mixed_periodic_face_from_embedding,
};
use super::mixed_shell_plan::{
    MixedArrangementBinding, MixedShellCellKey, arrange_coincident_cylinder_sides_mixed_shell,
    arrange_common_support_spans_mixed_shell, arrange_projected_ring_hole_with_source_lineage,
    arrange_source_arc_overlays_mixed_shell, complete_mixed_shell_plan,
    plan_internal_tangency_bands_mixed_shell, plan_internal_tangency_union_mixed_shell,
    source_face_key,
};
use super::parallel_cylinder_relation::{
    CertifiedParallelCylinderAxialContact, CertifiedParallelCylinderCommonSupport,
    CertifiedParallelCylinderInternalRadialTangency, interval_axis_distance_squared,
};
use super::pipeline::PLANAR_BOOLEAN_BSP_WORK;
use super::select::PlanarBooleanOperation;
use crate::error::Error;
use crate::section::{
    GAP_CLOSED_CONIC_COINCIDENT_BOUNDARY, GAP_CLOSED_CONIC_NONSECTANT_BOUNDARY,
    GAP_CLOSED_CONIC_TANGENTIAL_CONTACT, GAP_COINCIDENT_FACE_PAIR, GAP_PAIR_UNRESOLVED,
    GAP_TANGENT_CONTACT, certify_periodic_face_fragment_subset, periodic_face_fragment_subset_work,
};
use crate::session::PartEdit;
use crate::{
    BodyId, BodySectionGraph, FaceId, Part, SectionCompletion, SectionPeriodicEmbeddingGap,
    SectionPeriodicFaceEmbeddingEvidence,
};

#[derive(Clone, Copy)]
struct ContactSource<'a> {
    source: &'a CertifiedCylinderSource,
    contact_boundary: usize,
    far_boundary: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ContactRadialRelation {
    StrictSecant,
    StrictInternal { outer: usize },
    Coincident,
    ExactExternalTangent,
    BoundaryContact,
}

struct PreparedPeriodicFace {
    face: FaceId,
    operand: usize,
    arrangement: MixedPeriodicFaceArrangement,
    embedding: crate::CertifiedSectionPeriodicFaceEmbedding,
}

struct PreparedDiskFace {
    face: FaceId,
    operand: usize,
    arrangement: ArrangedDiskFace,
}

struct PreparedStrictSecantBoundary {
    periodic: [PreparedPeriodicFace; 2],
    disks: [PreparedDiskFace; 2],
    caps: [MixedCylinderCapRing; 2],
    classified: Vec<ClassifiedBoundaryFragment<MixedShellCellKey, ()>>,
}

struct PreparedSourceOnlyBoundary {
    periodic: [PreparedPeriodicFace; 2],
    rings: [[MixedCylinderCapRing; 2]; 2],
    classified: Vec<ClassifiedBoundaryFragment<MixedShellCellKey, ()>>,
}

impl PreparedSourceOnlyBoundary {
    fn bindings(&self) -> Vec<MixedArrangementBinding<'_>> {
        self.periodic
            .iter()
            .map(|face| MixedArrangementBinding::Periodic {
                face: face.face.clone(),
                operand: face.operand,
                arrangement: &face.arrangement,
                embedding: Some(&face.embedding),
            })
            .chain(
                self.rings
                    .iter()
                    .flatten()
                    .map(|ring| MixedArrangementBinding::CylinderCap { ring }),
            )
            .collect()
    }
}

impl PreparedStrictSecantBoundary {
    fn bindings(&self) -> Vec<MixedArrangementBinding<'_>> {
        self.periodic
            .iter()
            .map(|face| MixedArrangementBinding::Periodic {
                face: face.face.clone(),
                operand: face.operand,
                arrangement: &face.arrangement,
                embedding: Some(&face.embedding),
            })
            .chain(self.disks.iter().map(|face| MixedArrangementBinding::Disk {
                face: face.face.clone(),
                operand: face.operand,
                arranged: &face.arrangement,
            }))
            .chain(
                self.caps
                    .iter()
                    .map(|ring| MixedArrangementBinding::CylinderCap { ring }),
            )
            .collect()
    }
}
#[allow(clippy::too_many_arguments)]
pub(super) fn execute_axial_contact_unite(
    edit: &mut PartEdit<'_>,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    graph: &BodySectionGraph,
    contact: &CertifiedParallelCylinderAxialContact,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let sources = bind_contact_sources(cylinders, contact).map_err(contact_boundary_failure)?;
    let radial =
        classify_radial_contact(&edit.state.store, &sources).map_err(contact_boundary_failure)?;
    match radial {
        ContactRadialRelation::StrictSecant => {
            execute_strict_secant_contact(edit, bodies, cylinders, graph, contact, linear, scope)
        }
        ContactRadialRelation::StrictInternal { outer } => execute_internal_contact(
            edit, bodies, cylinders, graph, sources, outer, linear, scope,
        ),
        ContactRadialRelation::Coincident => {
            execute_coincident_contact(edit, bodies, cylinders, graph, sources, linear, scope)
        }
        ContactRadialRelation::ExactExternalTangent => realize_certified_cylinder_source_copies(
            edit,
            &[
                (bodies[0].clone(), cylinders[0]),
                (bodies[1].clone(), cylinders[1]),
            ],
            scope,
        ),
        ContactRadialRelation::BoundaryContact => Err(PipelineFailure::Refused(
            CurvedBooleanPipelineRefusal::ClassificationBoundaryContact,
        )),
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn execute_common_support_boolean(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    graph: &BodySectionGraph,
    relation: &CertifiedParallelCylinderCommonSupport,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let interval = super::axial_interval_sweep::plan_axial_interval_sweep(
        adapt_operation(operation),
        relation.preorder(),
    );
    if interval.spans().is_empty() {
        return Ok(CurvedBooleanPipelineOutcome::ProvenEmpty);
    }
    validate_common_support_graph(graph, cylinders, relation).map_err(contact_boundary_failure)?;
    let mut prepared =
        prepare_uncut_cylinder_boundary(&edit.as_part(), bodies, cylinders, graph, linear, scope)
            .map_err(contact_boundary_failure)?;
    for ring in prepared.rings.iter().flatten() {
        prepared.classified.push(classified_exterior_cap(
            MixedShellCellKey::cylinder_cap(ring.cap_source(), ring.boundary()),
            ring.operand(),
        ));
    }
    let selected = select_boundary_fragments(
        super::boundary_select::RegularizedBooleanOperation::Unite,
        prepared.classified.clone(),
    )
    .map_err(|error| PipelineFailure::Refused(CurvedBooleanPipelineRefusal::Selection(error)))?;
    let arrangement = arrange_common_support_spans_mixed_shell(
        &edit.state.store,
        graph,
        prepared.bindings(),
        selected,
        &interval,
        relation.preorder(),
        linear,
    )
    .map_err(mixed_plan_failure)?;
    let plan = complete_mixed_shell_plan(&edit.state.store, graph, arrangement)
        .map_err(mixed_plan_failure)?;
    realize_mixed_shell(edit, &plan, linear, scope)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn execute_internal_tangency_boolean(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    graph: &BodySectionGraph,
    relation: &CertifiedParallelCylinderInternalRadialTangency,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    use super::axial_interval_sweep::{
        AuthoredAxialEndpoint, AxialEndpointContributor, AxialIntervalOperand,
        plan_axial_interval_difference, plan_axial_interval_sweep,
    };

    let contained = relation.contained_operand();
    let containing = relation.containing_operand();
    let intersection = plan_axial_interval_sweep(
        adapt_operation(PlanarBooleanOperation::Intersect),
        relation.preorder(),
    );
    let [overlap] = intersection.spans() else {
        return Err(PipelineFailure::Refused(
            CurvedBooleanPipelineRefusal::ResultTopologyUnsupported,
        ));
    };
    let operand = |index| {
        if index == 0 {
            AxialIntervalOperand::Left
        } else {
            AxialIntervalOperand::Right
        }
    };
    let whole = |span: &super::axial_interval_sweep::PlannedAxialSpan, index| {
        let operand = operand(index);
        let start = AxialEndpointContributor::new(operand, AuthoredAxialEndpoint::Start);
        let end = AxialEndpointContributor::new(operand, AuthoredAxialEndpoint::End);
        (span.low().contains(start) && span.high().contains(end))
            || (span.low().contains(end) && span.high().contains(start))
    };
    let request = match operation {
        PlanarBooleanOperation::Intersect if whole(overlap, contained) => {
            return realize_certified_cylinder_source_copies(
                edit,
                &[(bodies[contained].clone(), cylinders[contained])],
                scope,
            );
        }
        PlanarBooleanOperation::Intersect => InternalTangencyRequest::Bands(intersection),
        PlanarBooleanOperation::Unite => {
            let tails = plan_axial_interval_difference(relation.preorder(), operand(contained));
            if tails.spans().is_empty() {
                return realize_certified_cylinder_source_copies(
                    edit,
                    &[(bodies[containing].clone(), cylinders[containing])],
                    scope,
                );
            }
            InternalTangencyRequest::Union(tails.spans().to_vec())
        }
        PlanarBooleanOperation::Subtract if contained == 0 => {
            let difference =
                plan_axial_interval_sweep(adapt_operation(operation), relation.preorder());
            if difference.spans().is_empty() {
                return Ok(CurvedBooleanPipelineOutcome::ProvenEmpty);
            }
            InternalTangencyRequest::Bands(difference)
        }
        PlanarBooleanOperation::Subtract => {
            return Err(PipelineFailure::Refused(
                CurvedBooleanPipelineRefusal::ResultTopologyUnsupported,
            ));
        }
    };

    validate_internal_tangency_graph(graph, cylinders, relation)
        .map_err(contact_boundary_failure)?;
    let mut prepared =
        prepare_uncut_cylinder_boundary(&edit.as_part(), bodies, cylinders, graph, linear, scope)
            .map_err(contact_boundary_failure)?;
    for ring in prepared.rings.iter().flatten() {
        prepared.classified.push(classified_exterior_cap(
            MixedShellCellKey::cylinder_cap(ring.cap_source(), ring.boundary()),
            ring.operand(),
        ));
    }
    let selected = select_boundary_fragments(
        super::boundary_select::RegularizedBooleanOperation::Unite,
        prepared.classified.clone(),
    )
    .map_err(|error| PipelineFailure::Refused(CurvedBooleanPipelineRefusal::Selection(error)))?;
    let plan = match &request {
        InternalTangencyRequest::Bands(interval) => plan_internal_tangency_bands_mixed_shell(
            &edit.state.store,
            graph,
            relation,
            cylinders,
            interval,
            prepared.bindings(),
            selected,
        ),
        InternalTangencyRequest::Union(tails) => plan_internal_tangency_union_mixed_shell(
            &edit.state.store,
            graph,
            relation,
            cylinders,
            tails,
            prepared.bindings(),
            selected,
        ),
    }
    .map_err(mixed_plan_failure)?;
    realize_mixed_shell(edit, &plan, linear, scope)
}

enum InternalTangencyRequest {
    Bands(super::axial_interval_sweep::AxialIntervalPlan),
    Union(Vec<super::axial_interval_sweep::PlannedAxialSpan>),
}

#[allow(clippy::too_many_arguments)]
fn execute_strict_secant_contact(
    edit: &mut PartEdit<'_>,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    graph: &BodySectionGraph,
    contact: &CertifiedParallelCylinderAxialContact,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let prepared = prepare_strict_secant_boundary(
        &edit.as_part(),
        bodies,
        cylinders,
        graph,
        contact,
        linear,
        scope,
    )
    .map_err(contact_boundary_failure)?;
    let selected = select_boundary_fragments(
        super::boundary_select::RegularizedBooleanOperation::Unite,
        prepared.classified.clone(),
    )
    .map_err(|error| PipelineFailure::Refused(CurvedBooleanPipelineRefusal::Selection(error)))?;
    let arrangement = arrange_source_arc_overlays_mixed_shell(
        &edit.state.store,
        graph,
        prepared.bindings(),
        selected,
        linear,
    )
    .map_err(mixed_plan_failure)?;
    let plan = complete_mixed_shell_plan(&edit.state.store, graph, arrangement)
        .map_err(mixed_plan_failure)?;
    realize_mixed_shell(edit, &plan, linear, scope)
}

#[allow(clippy::too_many_arguments)]
fn prepare_strict_secant_boundary(
    part: &Part<'_>,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    graph: &BodySectionGraph,
    contact: &CertifiedParallelCylinderAxialContact,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<PreparedStrictSecantBoundary, MixedBoundaryError> {
    if graph.curve_fragments().is_empty() {
        return Err(MixedBoundaryError::SourceTopology);
    }
    let sources = bind_contact_sources(cylinders, contact)?;
    let fragment_subsets = validate_strict_secant_graph(graph, &sources)?;
    let projection_work = fragment_subsets.iter().try_fold(0_u64, |total, subset| {
        total.checked_add(periodic_face_fragment_subset_work(subset.len())?)
    });
    let collection_work = u64::try_from(graph.curve_fragments().len())
        .ok()
        .and_then(|value| value.checked_mul(4))
        .and_then(|value| {
            value.checked_add(
                u64::try_from(graph.curve_endpoints().len())
                    .ok()?
                    .checked_mul(2)?,
            )
        })
        .and_then(|value| value.checked_add(6))
        .and_then(|value| value.checked_add(projection_work?))
        .ok_or(MixedBoundaryError::SourceTopology)?;
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_BSP_WORK, collection_work)
        .map_err(Error::from)?;

    let store = &part.state.store;
    let mut periodic = Vec::with_capacity(2);
    let mut disks = Vec::with_capacity(2);
    let mut caps = Vec::with_capacity(2);
    let mut classified = Vec::new();
    for operand in 0..2 {
        let side_face = FaceId::new(
            bodies[operand].part().clone(),
            cylinders[operand].side_face(),
        );
        let embedding = certify_periodic_face_fragment_subset(
            store,
            side_face.part(),
            graph,
            operand,
            side_face.clone(),
            &fragment_subsets[operand],
            linear,
        )
        .map_err(|gap| {
            MixedBoundaryError::PeriodicArrangement(
                super::mixed_periodic_arrangement::MixedPeriodicArrangementError::EmbeddingIndeterminate(
                    gap,
                ),
            )
        })?;
        let arrangement = arrange_mixed_periodic_face_from_embedding(graph, &embedding)
            .map_err(MixedBoundaryError::PeriodicArrangement)?;
        let side_source = source_face_key(store, graph, &side_face, operand)
            .map_err(|_| MixedBoundaryError::SourceTopology)?;
        if arrangement.cells().len() != 2
            || arrangement
                .cells()
                .iter()
                .filter(|cell| {
                    matches!(
                        cell.key(),
                        super::mixed_periodic_arrangement::PeriodicArrangementCellKey::AnnularRemainder
                    )
                })
                .count()
                != 1
        {
            return Err(MixedBoundaryError::SourceTopology);
        }
        classified.extend(arrangement.cells().iter().map(|cell| {
            let class = if matches!(
                cell.key(),
                super::mixed_periodic_arrangement::PeriodicArrangementCellKey::AnnularRemainder
            ) {
                BoundaryFragmentClassification::Exterior
            } else {
                // The relation's coincident-boundary gap proves this trace
                // cell is the zero-area overlay between the Section arc and
                // the matching source-ring span.
                BoundaryFragmentClassification::Interior
            };
            ClassifiedBoundaryFragment::new(
                MixedShellCellKey::periodic(side_source, *cell.key()),
                operand_side(operand),
                (),
                class,
            )
        }));

        let far = bind_cylinder_cap_ring_from_embedding(
            store,
            graph,
            cylinders[operand],
            operand,
            sources[operand].far_boundary,
            &side_face,
            &arrangement,
            &embedding,
        )
        .map_err(|_| MixedBoundaryError::SourceTopology)?;
        classified.push(classified_exterior_cap(
            MixedShellCellKey::cylinder_cap(far.cap_source(), far.boundary()),
            operand,
        ));

        let contact_boundary = cylinders[operand].boundaries()[sources[operand].contact_boundary];
        let cap_face = FaceId::new(bodies[operand].part().clone(), contact_boundary.cap_face());
        let cap_fragments = fragment_subsets[1 - operand].as_slice();
        let disk = arrange_section_disk_face_from_fragment_subset(
            store,
            graph,
            &cap_face,
            operand,
            cap_fragments,
        )
        .map_err(|_| MixedBoundaryError::SourceTopology)?;
        let peer = &sources[1 - operand];
        let peer_contact = peer.source.boundaries()[peer.contact_boundary].center();
        let peer_far = peer.source.boundaries()[peer.far_boundary].center();
        let classes = classify_disk_face_from_source_offset(
            part,
            &bodies[1 - operand],
            &disk,
            (peer_far - peer_contact) * 0.5,
            linear,
            scope,
        )?;
        let cap_source = source_face_key(store, graph, &cap_face, operand)
            .map_err(|_| MixedBoundaryError::SourceTopology)?;
        classified.extend(disk.arrangement().cells().iter().map(|cell| {
            let class = match classes[&cell.key()] {
                DiskCellClassification::Interior => BoundaryFragmentClassification::Interior,
                DiskCellClassification::Exterior => BoundaryFragmentClassification::Exterior,
            };
            ClassifiedBoundaryFragment::new(
                MixedShellCellKey::disk(cap_source, cell.key()),
                operand_side(operand),
                (),
                class,
            )
        }));
        periodic.push(PreparedPeriodicFace {
            face: side_face,
            operand,
            arrangement,
            embedding,
        });
        disks.push(PreparedDiskFace {
            face: cap_face,
            operand,
            arrangement: disk,
        });
        caps.push(far);
    }
    let periodic = periodic
        .try_into()
        .map_err(|_| MixedBoundaryError::SourceTopology)?;
    let disks = disks
        .try_into()
        .map_err(|_| MixedBoundaryError::SourceTopology)?;
    let caps = caps
        .try_into()
        .map_err(|_| MixedBoundaryError::SourceTopology)?;
    Ok(PreparedStrictSecantBoundary {
        periodic,
        disks,
        caps,
        classified,
    })
}

#[allow(clippy::too_many_arguments)]
fn execute_internal_contact(
    edit: &mut PartEdit<'_>,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    graph: &BodySectionGraph,
    sources: [ContactSource<'_>; 2],
    outer: usize,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let inner = 1 - outer;
    let mut prepared = prepare_source_only_boundary(
        &edit.as_part(),
        bodies,
        cylinders,
        graph,
        &sources,
        GAP_CLOSED_CONIC_NONSECTANT_BOUNDARY,
        linear,
        scope,
    )
    .map_err(contact_boundary_failure)?;
    for (operand, source) in sources.iter().enumerate() {
        let far = &prepared.rings[operand][source.far_boundary];
        prepared.classified.push(classified_exterior_cap(
            MixedShellCellKey::cylinder_cap(far.cap_source(), far.boundary()),
            operand,
        ));
    }
    let outer_contact = &prepared.rings[outer][sources[outer].contact_boundary];
    prepared.classified.push(classified_exterior_cap(
        MixedShellCellKey::cylinder_cap(outer_contact.cap_source(), outer_contact.boundary()),
        outer,
    ));
    let selected = select_boundary_fragments(
        super::boundary_select::RegularizedBooleanOperation::Unite,
        prepared.classified.clone(),
    )
    .map_err(|error| PipelineFailure::Refused(CurvedBooleanPipelineRefusal::Selection(error)))?;
    let inner_contact = &prepared.rings[inner][sources[inner].contact_boundary];
    let arrangement = arrange_projected_ring_hole_with_source_lineage(
        &edit.state.store,
        graph,
        prepared.bindings(),
        selected,
        outer_contact.cap_face().raw(),
        inner_contact,
        linear,
    )
    .map_err(mixed_plan_failure)?;
    let plan = complete_mixed_shell_plan(&edit.state.store, graph, arrangement)
        .map_err(mixed_plan_failure)?;
    realize_mixed_shell(edit, &plan, linear, scope)
}

#[allow(clippy::too_many_arguments)]
fn execute_coincident_contact(
    edit: &mut PartEdit<'_>,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    graph: &BodySectionGraph,
    sources: [ContactSource<'_>; 2],
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let mut prepared = prepare_source_only_boundary(
        &edit.as_part(),
        bodies,
        cylinders,
        graph,
        &sources,
        GAP_CLOSED_CONIC_COINCIDENT_BOUNDARY,
        linear,
        scope,
    )
    .map_err(contact_boundary_failure)?;
    for (operand, source) in sources.iter().enumerate() {
        let far = &prepared.rings[operand][source.far_boundary];
        prepared.classified.push(classified_exterior_cap(
            MixedShellCellKey::cylinder_cap(far.cap_source(), far.boundary()),
            operand,
        ));
    }
    let selected = select_boundary_fragments(
        super::boundary_select::RegularizedBooleanOperation::Unite,
        prepared.classified.clone(),
    )
    .map_err(|error| PipelineFailure::Refused(CurvedBooleanPipelineRefusal::Selection(error)))?;
    let far_rings = [
        &prepared.rings[0][sources[0].far_boundary],
        &prepared.rings[1][sources[1].far_boundary],
    ];
    let arrangement = arrange_coincident_cylinder_sides_mixed_shell(
        &edit.state.store,
        graph,
        prepared.bindings(),
        selected,
        far_rings,
        linear,
    )
    .map_err(mixed_plan_failure)?;
    let plan = complete_mixed_shell_plan(&edit.state.store, graph, arrangement)
        .map_err(mixed_plan_failure)?;
    realize_mixed_shell(edit, &plan, linear, scope)
}

#[allow(clippy::too_many_arguments)]
fn prepare_source_only_boundary(
    part: &Part<'_>,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    graph: &BodySectionGraph,
    sources: &[ContactSource<'_>; 2],
    side_cap_reason: &'static str,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<PreparedSourceOnlyBoundary, MixedBoundaryError> {
    validate_source_only_contact_graph(graph, sources, side_cap_reason)?;
    prepare_uncut_cylinder_boundary(part, bodies, cylinders, graph, linear, scope)
}

fn prepare_uncut_cylinder_boundary(
    part: &Part<'_>,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    graph: &BodySectionGraph,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> Result<PreparedSourceOnlyBoundary, MixedBoundaryError> {
    let projection_work =
        periodic_face_fragment_subset_work(0).ok_or(MixedBoundaryError::SourceTopology)?;
    let work = projection_work
        .checked_mul(2)
        .and_then(|value| value.checked_add(6))
        .ok_or(MixedBoundaryError::SourceTopology)?;
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_BSP_WORK, work)
        .map_err(Error::from)?;

    let store = &part.state.store;
    let mut periodic = Vec::with_capacity(2);
    let mut rings = Vec::with_capacity(2);
    let mut classified = Vec::with_capacity(2);
    for operand in 0..2 {
        let side_face = FaceId::new(
            bodies[operand].part().clone(),
            cylinders[operand].side_face(),
        );
        let embedding = certify_periodic_face_fragment_subset(
            store,
            side_face.part(),
            graph,
            operand,
            side_face.clone(),
            &[],
            linear,
        )
        .map_err(|gap| {
            MixedBoundaryError::PeriodicArrangement(
                super::mixed_periodic_arrangement::MixedPeriodicArrangementError::EmbeddingIndeterminate(
                    gap,
                ),
            )
        })?;
        let arrangement = arrange_mixed_periodic_face_from_embedding(graph, &embedding)
            .map_err(MixedBoundaryError::PeriodicArrangement)?;
        let [cell] = arrangement.cells() else {
            return Err(MixedBoundaryError::SourceTopology);
        };
        if cell.key()
            != &super::mixed_periodic_arrangement::PeriodicArrangementCellKey::AnnularRemainder
        {
            return Err(MixedBoundaryError::SourceTopology);
        }
        let side_source = source_face_key(store, graph, &side_face, operand)
            .map_err(|_| MixedBoundaryError::SourceTopology)?;
        classified.push(ClassifiedBoundaryFragment::new(
            MixedShellCellKey::periodic(side_source, *cell.key()),
            operand_side(operand),
            (),
            BoundaryFragmentClassification::Exterior,
        ));
        let operand_rings = [
            bind_cylinder_cap_ring_from_embedding(
                store,
                graph,
                cylinders[operand],
                operand,
                0,
                &side_face,
                &arrangement,
                &embedding,
            )
            .map_err(|_| MixedBoundaryError::SourceTopology)?,
            bind_cylinder_cap_ring_from_embedding(
                store,
                graph,
                cylinders[operand],
                operand,
                1,
                &side_face,
                &arrangement,
                &embedding,
            )
            .map_err(|_| MixedBoundaryError::SourceTopology)?,
        ];
        periodic.push(PreparedPeriodicFace {
            face: side_face,
            operand,
            arrangement,
            embedding,
        });
        rings.push(operand_rings);
    }
    Ok(PreparedSourceOnlyBoundary {
        periodic: periodic
            .try_into()
            .map_err(|_| MixedBoundaryError::SourceTopology)?,
        rings: rings
            .try_into()
            .map_err(|_| MixedBoundaryError::SourceTopology)?,
        classified,
    })
}

fn validate_source_only_contact_graph(
    graph: &BodySectionGraph,
    sources: &[ContactSource<'_>; 2],
    side_cap_reason: &'static str,
) -> Result<(), MixedBoundaryError> {
    if graph.completion() != SectionCompletion::Indeterminate
        || !graph.vertices().is_empty()
        || !graph.edges().is_empty()
        || !graph.loops().is_empty()
        || !graph.rings().is_empty()
        || graph.branches().len() != 2
        || !graph.curve_endpoints().is_empty()
        || !graph.curve_fragments().is_empty()
        || !graph.curve_components().is_empty()
        || !graph
            .cylinder_cylinder_exterior_radial_separations()
            .is_empty()
        || !graph.periodic_face_embeddings().is_empty()
    {
        return Err(MixedBoundaryError::SourceTopology);
    }
    validate_source_only_branches(graph, sources)?;
    validate_contact_gaps(graph, sources, side_cap_reason)?;
    Ok(())
}

fn validate_common_support_graph(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    relation: &CertifiedParallelCylinderCommonSupport,
) -> Result<(), MixedBoundaryError> {
    if graph.completion() != SectionCompletion::Indeterminate
        || !graph.vertices().is_empty()
        || !graph.edges().is_empty()
        || !graph.loops().is_empty()
        || !graph.rings().is_empty()
        || !(2..=4).contains(&graph.branches().len())
        || !graph.curve_endpoints().is_empty()
        || !graph.curve_fragments().is_empty()
        || !graph.curve_components().is_empty()
        || !graph.periodic_face_embeddings().is_empty()
        || graph.gaps().is_empty()
    {
        return Err(MixedBoundaryError::SourceTopology);
    }
    let mut boundaries = [[false; 2]; 2];
    for witness in relation.boundaries() {
        let source = cylinders
            .get(witness.operand())
            .and_then(|source| source.boundaries().get(witness.boundary()))
            .ok_or(MixedBoundaryError::SourceTopology)?;
        if boundaries[witness.operand()][witness.boundary()]
            || source.cap_face() != witness.cap_face()
            || source.edge() != witness.edge()
        {
            return Err(MixedBoundaryError::SourceTopology);
        }
        boundaries[witness.operand()][witness.boundary()] = true;
    }
    if boundaries != [[true; 2]; 2] {
        return Err(MixedBoundaryError::SourceTopology);
    }
    let side_cap = |faces: &[FaceId]| {
        faces.len() == 2
            && (0..2).any(|operand| {
                let peer = 1 - operand;
                faces
                    .iter()
                    .any(|face| face.raw() == cylinders[operand].side_face())
                    && faces.iter().any(|face| {
                        cylinders[peer]
                            .boundaries()
                            .iter()
                            .any(|boundary| face.raw() == boundary.cap_face())
                    })
            })
    };
    let cap_pair = |faces: &[FaceId]| {
        faces.len() == 2
            && (0..2).all(|operand| {
                faces.iter().any(|face| {
                    cylinders[operand]
                        .boundaries()
                        .iter()
                        .any(|boundary| face.raw() == boundary.cap_face())
                })
            })
    };
    if graph
        .branches()
        .iter()
        .any(|branch| !side_cap(branch.faces()))
    {
        return Err(MixedBoundaryError::SourceTopology);
    }
    let side_pair = |faces: &[FaceId]| {
        faces.len() == 2
            && (0..2).all(|operand| {
                faces
                    .iter()
                    .any(|face| face.raw() == cylinders[operand].side_face())
            })
    };
    let mut counts = [0_usize; 3];
    for gap in graph.gaps() {
        let class = if gap.reason() == GAP_PAIR_UNRESOLVED && side_pair(gap.faces()) {
            0
        } else if gap.reason() == GAP_CLOSED_CONIC_COINCIDENT_BOUNDARY && side_cap(gap.faces()) {
            1
        } else if gap.reason() == GAP_COINCIDENT_FACE_PAIR && cap_pair(gap.faces()) {
            2
        } else {
            return Err(MixedBoundaryError::SourceTopology);
        };
        counts[class] += 1;
    }
    (counts[0] == 1 && counts[1] == graph.branches().len() && counts[2] <= 2)
        .then_some(())
        .ok_or(MixedBoundaryError::SourceTopology)
}

fn validate_internal_tangency_graph(
    graph: &BodySectionGraph,
    cylinders: [&CertifiedCylinderSource; 2],
    relation: &CertifiedParallelCylinderInternalRadialTangency,
) -> Result<(), MixedBoundaryError> {
    if graph.completion() != SectionCompletion::Indeterminate
        || !graph.vertices().is_empty()
        || !graph.edges().is_empty()
        || !graph.loops().is_empty()
        || !graph.rings().is_empty()
        || !graph.curve_endpoints().is_empty()
        || !graph.curve_fragments().is_empty()
        || !graph.curve_components().is_empty()
        || !graph.periodic_face_embeddings().is_empty()
        || graph.gaps().is_empty()
    {
        return Err(MixedBoundaryError::SourceTopology);
    }
    let mut boundaries = [[false; 2]; 2];
    for witness in relation.boundaries() {
        let source = cylinders
            .get(witness.operand())
            .and_then(|source| source.boundaries().get(witness.boundary()))
            .ok_or(MixedBoundaryError::SourceTopology)?;
        if boundaries[witness.operand()][witness.boundary()]
            || source.cap_face() != witness.cap_face()
            || source.edge() != witness.edge()
        {
            return Err(MixedBoundaryError::SourceTopology);
        }
        boundaries[witness.operand()][witness.boundary()] = true;
    }
    if boundaries != [[true; 2]; 2] {
        return Err(MixedBoundaryError::SourceTopology);
    }
    let side_pair = |faces: &[FaceId]| {
        faces.len() == 2
            && (0..2).all(|operand| {
                faces
                    .iter()
                    .any(|face| face.raw() == cylinders[operand].side_face())
            })
    };
    let side_cap = |faces: &[FaceId]| {
        faces.len() == 2
            && (0..2).any(|side| {
                faces
                    .iter()
                    .any(|face| face.raw() == cylinders[side].side_face())
                    && faces.iter().any(|face| {
                        cylinders[1 - side]
                            .boundaries()
                            .iter()
                            .any(|boundary| face.raw() == boundary.cap_face())
                    })
            })
    };
    let cap_pair = |faces: &[FaceId]| {
        faces.len() == 2
            && (0..2).all(|operand| {
                faces.iter().any(|face| {
                    cylinders[operand]
                        .boundaries()
                        .iter()
                        .any(|boundary| face.raw() == boundary.cap_face())
                })
            })
    };
    if graph
        .branches()
        .iter()
        .any(|branch| !side_cap(branch.faces()))
    {
        return Err(MixedBoundaryError::SourceTopology);
    }
    let mut side_pair_count = 0;
    for gap in graph.gaps() {
        let valid = if side_pair(gap.faces()) {
            side_pair_count += 1;
            gap.reason() == GAP_TANGENT_CONTACT || gap.reason() == GAP_PAIR_UNRESOLVED
        } else if side_cap(gap.faces()) {
            gap.reason() == GAP_CLOSED_CONIC_NONSECTANT_BOUNDARY
                || gap.reason() == GAP_CLOSED_CONIC_TANGENTIAL_CONTACT
                || gap.reason() == GAP_TANGENT_CONTACT
                || gap.reason() == GAP_PAIR_UNRESOLVED
        } else if cap_pair(gap.faces()) {
            gap.reason() == GAP_COINCIDENT_FACE_PAIR
        } else {
            false
        };
        if !valid {
            return Err(MixedBoundaryError::SourceTopology);
        }
    }
    (side_pair_count == 1)
        .then_some(())
        .ok_or(MixedBoundaryError::SourceTopology)
}

fn validate_source_only_branches(
    graph: &BodySectionGraph,
    sources: &[ContactSource<'_>; 2],
) -> Result<(), MixedBoundaryError> {
    let mut seen = [false; 2];
    for branch in graph.branches() {
        let owner = (0..2).find(|&operand| {
            let peer = 1 - operand;
            branch.faces()[operand].raw() == sources[operand].source.side_face()
                && branch.faces()[peer].raw()
                    == sources[peer].source.boundaries()[sources[peer].contact_boundary].cap_face()
        });
        let Some(owner) = owner else {
            return Err(MixedBoundaryError::SourceTopology);
        };
        if seen[owner] {
            return Err(MixedBoundaryError::SourceTopology);
        }
        seen[owner] = true;
    }
    seen.into_iter()
        .all(|value| value)
        .then_some(())
        .ok_or(MixedBoundaryError::SourceTopology)
}

fn bind_contact_sources<'a>(
    cylinders: [&'a CertifiedCylinderSource; 2],
    relation: &CertifiedParallelCylinderAxialContact,
) -> Result<[ContactSource<'a>; 2], MixedBoundaryError> {
    let mut boundaries = [None; 2];
    for witness in relation.contact_boundaries() {
        let operand = witness.operand();
        let source = cylinders
            .get(operand)
            .ok_or(MixedBoundaryError::SourceTopology)?;
        let boundary = source
            .boundaries()
            .get(witness.boundary())
            .ok_or(MixedBoundaryError::SourceTopology)?;
        if boundary.cap_face() != witness.cap_face()
            || boundary.edge() != witness.edge()
            || boundaries[operand].replace(witness.boundary()).is_some()
        {
            return Err(MixedBoundaryError::SourceTopology);
        }
    }
    let [Some(first), Some(second)] = boundaries else {
        return Err(MixedBoundaryError::SourceTopology);
    };
    Ok([
        contact_source(cylinders[0], first)?,
        contact_source(cylinders[1], second)?,
    ])
}

fn contact_source(
    source: &CertifiedCylinderSource,
    contact_boundary: usize,
) -> Result<ContactSource<'_>, MixedBoundaryError> {
    let far_boundary = 1_usize
        .checked_sub(contact_boundary)
        .ok_or(MixedBoundaryError::SourceTopology)?;
    Ok(ContactSource {
        source,
        contact_boundary,
        far_boundary,
    })
}

fn classify_radial_contact(
    store: &Store,
    sources: &[ContactSource<'_>; 2],
) -> Result<ContactRadialRelation, MixedBoundaryError> {
    let circles = [
        source_circle(
            store,
            sources[0].source.boundaries()[sources[0].contact_boundary],
        )?,
        source_circle(
            store,
            sources[1].source.boundaries()[sources[1].contact_boundary],
        )?,
    ];
    let centers = circles.map(|circle| circle.frame().origin());
    let radii = circles.map(|circle| circle.radius());
    if radii
        .into_iter()
        .any(|radius| !radius.is_finite() || radius <= 0.0)
    {
        return Err(MixedBoundaryError::SourceTopology);
    }
    if centers[0] == centers[1]
        && radii[0].to_bits() == radii[1].to_bits()
        && coincident_supports_are_coalescible(sources)
    {
        return Ok(ContactRadialRelation::Coincident);
    }
    let distance_squared = interval_distance_squared(centers[1], centers[0]);
    let first = Interval::point(radii[0]);
    let second = Interval::point(radii[1]);
    let difference_squared = (first - second).square();
    let sum_squared = (first + second).square();
    let outer = usize::from(radii[1] > radii[0]);
    let internal_clearance = Interval::point(radii[outer])
        - Interval::point(radii[1 - outer])
        - Interval::point(2.0 * LINEAR_RESOLUTION);
    if !finite_interval(distance_squared)
        || !finite_interval(difference_squared)
        || !finite_interval(sum_squared)
    {
        return Err(MixedBoundaryError::SourceTopology);
    }
    if distance_squared.lo() > difference_squared.hi() && distance_squared.hi() < sum_squared.lo() {
        Ok(ContactRadialRelation::StrictSecant)
    } else if finite_interval(internal_clearance)
        && internal_clearance.lo() > 0.0
        && distance_squared.hi() < internal_clearance.square().lo()
    {
        if strictly_contains_cylinder_support(sources, outer) {
            Ok(ContactRadialRelation::StrictInternal { outer })
        } else {
            Ok(ContactRadialRelation::BoundaryContact)
        }
    } else if exact_external_tangency(centers, radii) {
        Ok(ContactRadialRelation::ExactExternalTangent)
    } else {
        Ok(ContactRadialRelation::BoundaryContact)
    }
}

fn exact_external_tangency(centers: [Point3; 2], radii: [f64; 2]) -> bool {
    let Some(radius_sum) = exactly_representable_sum(radii[0], radii[1]) else {
        return false;
    };
    squared_distance_difference3(
        centers[1].to_array(),
        centers[0].to_array(),
        0.0,
        radius_sum,
    )
    .is_some_and(|difference| difference.sign() == Orientation::Zero)
}

fn exactly_representable_sum(first: f64, second: f64) -> Option<f64> {
    let (sum, residual) = two_sum(first, second);
    if !first.is_finite() || !second.is_finite() || !sum.is_finite() {
        return None;
    }
    (residual == 0.0).then_some(sum)
}

fn coincident_supports_are_coalescible(sources: &[ContactSource<'_>; 2]) -> bool {
    let first = sources[0].source.cylinder();
    let second = sources[1].source.cylinder();
    cylinders_have_exact_common_support(first, second)
        && sources.iter().all(|source| {
            source.source.boundaries().iter().all(|boundary| {
                points_are_exactly_axis_aligned(
                    boundary.center(),
                    first.frame().origin(),
                    first.frame().z(),
                )
            })
        })
}

fn cylinders_have_exact_common_support(
    first: kgeom::surface::Cylinder,
    second: kgeom::surface::Cylinder,
) -> bool {
    first.radius().to_bits() == second.radius().to_bits()
        && vectors_are_exactly_parallel(first.frame().z(), second.frame().z())
        && points_are_exactly_axis_aligned(
            second.frame().origin(),
            first.frame().origin(),
            first.frame().z(),
        )
}

fn vectors_are_exactly_parallel(first: Vec3, second: Vec3) -> bool {
    if first == second || first == -second {
        return true;
    }
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        .into_iter()
        .all(|basis| {
            orient3d(first.to_array(), second.to_array(), basis, [0.0; 3]) == Orientation::Zero
        })
}

fn points_are_exactly_axis_aligned(point: Point3, origin: Point3, axis: Vec3) -> bool {
    let cross_normals = [
        Vec3::new(0.0, axis.z, -axis.y),
        Vec3::new(-axis.z, 0.0, axis.x),
        Vec3::new(axis.y, -axis.x, 0.0),
    ];
    cross_normals.into_iter().all(|normal| {
        affine_dot3(normal.to_array(), point.to_array(), origin.to_array(), 0.0)
            .is_some_and(|value| value.sign() == Orientation::Zero)
    })
}

fn strictly_contains_cylinder_support(sources: &[ContactSource<'_>; 2], outer: usize) -> bool {
    let inner = 1 - outer;
    let outer_cylinder = sources[outer].source.cylinder();
    let inner_cylinder = sources[inner].source.cylinder();
    if outer_cylinder.radius() <= inner_cylinder.radius()
        || !vectors_are_exactly_parallel(outer_cylinder.frame().z(), inner_cylinder.frame().z())
    {
        return false;
    }
    let Some(distance) = interval_axis_distance_squared(
        inner_cylinder.frame().origin(),
        outer_cylinder.frame().origin(),
        outer_cylinder.frame().z(),
    ) else {
        return false;
    };
    let clearance = Interval::point(outer_cylinder.radius())
        - Interval::point(inner_cylinder.radius())
        - Interval::point(2.0 * LINEAR_RESOLUTION);
    finite_interval(distance)
        && finite_interval(clearance)
        && clearance.lo() > 0.0
        && distance.hi() < clearance.square().lo()
}

fn interval_distance_squared(point: Point3, origin: Point3) -> Interval {
    point.to_array().into_iter().zip(origin.to_array()).fold(
        Interval::point(0.0),
        |sum, (point, origin)| {
            let point = Interval::point(point);
            let origin = Interval::point(origin);
            sum + point.square() - Interval::point(2.0) * point * origin + origin.square()
        },
    )
}

fn finite_interval(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite()
}

fn source_circle(
    store: &Store,
    boundary: super::curved_source::CertifiedCylinderBoundary,
) -> Result<Circle, MixedBoundaryError> {
    let edge = store
        .get(boundary.edge())
        .map_err(|_| MixedBoundaryError::SourceTopology)?;
    let curve = edge.curve().ok_or(MixedBoundaryError::SourceTopology)?;
    match store
        .curve(curve)
        .map_err(|_| MixedBoundaryError::SourceTopology)?
    {
        CurveGeom::Circle(circle) => Ok(*circle),
        _ => Err(MixedBoundaryError::SourceTopology),
    }
}

fn validate_strict_secant_graph(
    graph: &BodySectionGraph,
    sources: &[ContactSource<'_>; 2],
) -> Result<[Vec<usize>; 2], MixedBoundaryError> {
    if graph.completion() != SectionCompletion::Indeterminate
        || !graph.vertices().is_empty()
        || !graph.edges().is_empty()
        || !graph.loops().is_empty()
        || !graph.rings().is_empty()
        || !graph
            .cylinder_cylinder_exterior_radial_separations()
            .is_empty()
        || graph.branches().len() != 2
        || graph.curve_endpoints().len() != 2
        || graph.curve_fragments().len() != 2
    {
        return Err(MixedBoundaryError::SourceTopology);
    }
    let [component] = graph.curve_components() else {
        return Err(MixedBoundaryError::SourceTopology);
    };
    let mut component_fragments = component.fragments().to_vec();
    component_fragments.sort_unstable();
    if !component.closed() || component_fragments != [0, 1] {
        return Err(MixedBoundaryError::SourceTopology);
    }
    validate_contact_gaps(graph, sources, GAP_CLOSED_CONIC_COINCIDENT_BOUNDARY)?;
    let mut subsets = [Vec::new(), Vec::new()];
    for (fragment, value) in graph.curve_fragments().iter().enumerate() {
        let branch = graph
            .branches()
            .get(value.branch())
            .ok_or(MixedBoundaryError::SourceTopology)?;
        let mut owner = None;
        for operand in 0..2 {
            let peer = 1 - operand;
            let peer_contact = sources[peer].source.boundaries()[sources[peer].contact_boundary];
            if branch.faces()[operand].raw() == sources[operand].source.side_face()
                && branch.faces()[peer].raw() == peer_contact.cap_face()
                && owner.replace(operand).is_some()
            {
                return Err(MixedBoundaryError::SourceTopology);
            }
        }
        let owner = owner.ok_or(MixedBoundaryError::SourceTopology)?;
        subsets[owner].push(fragment);
    }
    if subsets.iter().any(|subset| subset.len() != 1) {
        return Err(MixedBoundaryError::SourceTopology);
    }
    validate_periodic_gaps(graph, sources, [subsets[0][0], subsets[1][0]])?;
    Ok(subsets)
}

fn validate_contact_gaps(
    graph: &BodySectionGraph,
    sources: &[ContactSource<'_>; 2],
    side_cap_reason: &'static str,
) -> Result<(), MixedBoundaryError> {
    let contact = sources.map(|source| source.source.boundaries()[source.contact_boundary]);
    let expected = [
        (
            side_cap_reason,
            [sources[0].source.side_face(), contact[1].cap_face()],
        ),
        (
            side_cap_reason,
            [contact[0].cap_face(), sources[1].source.side_face()],
        ),
        (
            GAP_COINCIDENT_FACE_PAIR,
            [contact[0].cap_face(), contact[1].cap_face()],
        ),
        (
            GAP_PAIR_UNRESOLVED,
            [sources[0].source.side_face(), sources[1].source.side_face()],
        ),
    ];
    if graph.gaps().len() != expected.len() {
        return Err(MixedBoundaryError::SourceTopology);
    }
    let mut consumed = [false; 4];
    for gap in graph.gaps() {
        let actual = gap.faces();
        let Some(index) = expected.iter().enumerate().position(|(index, candidate)| {
            !consumed[index]
                && gap.reason() == candidate.0
                && actual.len() == 2
                && ((actual[0].raw() == candidate.1[0] && actual[1].raw() == candidate.1[1])
                    || (actual[0].raw() == candidate.1[1] && actual[1].raw() == candidate.1[0]))
        }) else {
            return Err(MixedBoundaryError::SourceTopology);
        };
        consumed[index] = true;
    }
    consumed
        .into_iter()
        .all(|value| value)
        .then_some(())
        .ok_or(MixedBoundaryError::SourceTopology)
}

fn validate_periodic_gaps(
    graph: &BodySectionGraph,
    sources: &[ContactSource<'_>; 2],
    fragments: [usize; 2],
) -> Result<(), MixedBoundaryError> {
    if graph.periodic_face_embeddings().len() != 2 {
        return Err(MixedBoundaryError::SourceTopology);
    }
    let mut seen = [false; 2];
    for evidence in graph.periodic_face_embeddings() {
        let operand = evidence.operand();
        if operand >= 2
            || seen[operand]
            || evidence.face().raw() != sources[operand].source.side_face()
        {
            return Err(MixedBoundaryError::SourceTopology);
        }
        match evidence {
            SectionPeriodicFaceEmbeddingEvidence::Certified(certified) => {
                let [trace] = certified.boundary_traces() else {
                    return Err(MixedBoundaryError::SourceTopology);
                };
                if !certified.components().is_empty()
                    || trace.fragments().len() != 1
                    || trace.fragments()[0].fragment() != fragments[operand]
                {
                    return Err(MixedBoundaryError::SourceTopology);
                }
            }
            SectionPeriodicFaceEmbeddingEvidence::Indeterminate {
                gap:
                    SectionPeriodicEmbeddingGap::BoundaryTerminalUnavailable {
                        component,
                        fragment,
                        end,
                    },
                ..
            } if *fragment == fragments[operand] && *component == 0 && *end <= 1 => {}
            SectionPeriodicFaceEmbeddingEvidence::Indeterminate { .. } => {
                return Err(MixedBoundaryError::SourceTopology);
            }
        }
        seen[operand] = true;
    }
    seen.into_iter()
        .all(|value| value)
        .then_some(())
        .ok_or(MixedBoundaryError::SourceTopology)
}

fn contact_boundary_failure(error: MixedBoundaryError) -> PipelineFailure {
    match error {
        MixedBoundaryError::Execution(error) => PipelineFailure::Execution(error),
        _ => PipelineFailure::Refused(CurvedBooleanPipelineRefusal::AssemblyContract(
            "axial-contact arrangement adapter contract failed",
        )),
    }
}
