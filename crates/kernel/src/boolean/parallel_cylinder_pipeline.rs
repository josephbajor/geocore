//! Full-checked Boolean consumer of parallel-cylinder Section evidence.

use kcore::operation::OperationScope;
use ktopo::entity::Body as TopologyBody;

use super::boundary_select::select_boundary_fragments;
use super::curved_pipeline::{
    CurvedBooleanPipelineOutcome, CurvedBooleanPipelineRefusal, PipelineFailure, StageResult,
    adapt_operation, extract_cylinder_operand, mixed_boundary_failure, mixed_plan_failure,
    realize_mixed_shell, refused,
};
use super::curved_realize::realize_certified_cylinder_source_copies;
use super::curved_source::CertifiedCylinderSource;
use super::mixed_shell_plan::{
    MixedBoundedSourceRoot, MixedShellProofPlan, plan_mixed_shell,
    plan_parallel_cylinder_coincident_boolean,
};
use super::parallel_cylinder_boundary::{
    prepare_parallel_cylinder_boundary, prepare_parallel_cylinder_coincident_boundary,
};
use super::parallel_cylinder_relation::{
    CertifiedParallelCylinderCoincidentCapRelation, CertifiedParallelCylinderLensRelation,
    ParallelCylinderRelationOutcome, ParallelCylinderSourceRootWitness,
    certify_parallel_cylinder_relation,
};
use super::select::PlanarBooleanOperation;
use crate::section::section_bodies_in_scope;
use crate::session::PartEdit;
use crate::{BodyId, BodySectionGraph};

/// Consume proof-complete parallel-cylinder relations through Full-checked
/// realization paths.
///
/// Strictly disjoint sources retain whole boundaries; positive overlaps use
/// shared arrangement, truth selection, and shell planning. Commutative
/// operations receive a canonical source order; Subtract preserves caller
/// order.
pub(super) fn execute_parallel_cylinder_boolean(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    bodies: [BodyId; 2],
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let bodies = match operation {
        PlanarBooleanOperation::Intersect | PlanarBooleanOperation::Unite => {
            canonical_commutative_order(edit, bodies)?
        }
        PlanarBooleanOperation::Subtract => bodies,
    };
    let first = extract_cylinder_operand(edit, bodies[0].clone(), 0, scope)?;
    let second = extract_cylinder_operand(edit, bodies[1].clone(), 1, scope)?;
    let graph = section_bodies_in_scope(&edit.as_part(), &bodies[0], &bodies[1], linear, scope)?;
    let relation =
        certify_parallel_cylinder_relation(&edit.state.store, &graph, [&first, &second], scope)?;
    match relation {
        ParallelCylinderRelationOutcome::CertifiedExteriorRadialSeparation
        | ParallelCylinderRelationOutcome::CertifiedAxialSeparation(_) => {
            execute_disjoint_source_boolean(edit, operation, &bodies, [&first, &second], scope)
        }
        ParallelCylinderRelationOutcome::CertifiedAxialContact(_) => {
            refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported)
        }
        ParallelCylinderRelationOutcome::Certified(relation) => execute_complete_relation(
            edit,
            operation,
            &bodies,
            [&first, &second],
            &graph,
            &relation,
            linear,
            scope,
        ),
        ParallelCylinderRelationOutcome::CertifiedCoincidentCaps(relation) => {
            execute_coincident_cap_boolean(
                edit,
                operation,
                &bodies,
                [&first, &second],
                &graph,
                &relation,
                linear,
                scope,
            )
        }
        ParallelCylinderRelationOutcome::Indeterminate(_) => {
            refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported)
        }
    }
}

fn execute_disjoint_source_boolean(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    match operation {
        PlanarBooleanOperation::Intersect => Ok(CurvedBooleanPipelineOutcome::ProvenEmpty),
        PlanarBooleanOperation::Unite => realize_certified_cylinder_source_copies(
            edit,
            &[
                (bodies[0].clone(), cylinders[0]),
                (bodies[1].clone(), cylinders[1]),
            ],
            scope,
        ),
        PlanarBooleanOperation::Subtract => realize_certified_cylinder_source_copies(
            edit,
            &[(bodies[0].clone(), cylinders[0])],
            scope,
        ),
    }
}

#[allow(clippy::too_many_arguments)]
fn execute_complete_relation(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    graph: &BodySectionGraph,
    relation: &CertifiedParallelCylinderLensRelation,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let prepared = prepare_parallel_cylinder_boundary(
        &edit.as_part(),
        graph,
        bodies,
        cylinders,
        relation,
        linear,
        scope,
    )
    .map_err(mixed_boundary_failure)?;
    let selected = select_boundary_fragments(adapt_operation(operation), prepared.classified())
        .map_err(|error| {
            PipelineFailure::Refused(CurvedBooleanPipelineRefusal::Selection(error))
        })?;
    if selected.is_empty() {
        return refused(CurvedBooleanPipelineRefusal::AssemblyContract(
            "certified positive-volume parallel-cylinder Boolean selected no boundary",
        ));
    }
    let plan = plan_mixed_shell(&edit.state.store, graph, prepared.bindings(), selected)
        .map_err(mixed_plan_failure)?;
    if !plan_matches_relation(&plan, relation) {
        return refused(CurvedBooleanPipelineRefusal::AssemblyContract(
            "parallel-cylinder shell omitted certified section evidence",
        ));
    }
    realize_mixed_shell(edit, &plan, linear, scope)
}

#[allow(clippy::too_many_arguments)]
fn execute_coincident_cap_boolean(
    edit: &mut PartEdit<'_>,
    operation: PlanarBooleanOperation,
    bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    graph: &BodySectionGraph,
    relation: &CertifiedParallelCylinderCoincidentCapRelation,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let prepared = prepare_parallel_cylinder_coincident_boundary(
        &edit.as_part(),
        graph,
        bodies,
        cylinders,
        relation,
        linear,
        scope,
    )
    .map_err(mixed_boundary_failure)?;
    let selected = prepared
        .select(adapt_operation(operation))
        .map_err(|error| {
            PipelineFailure::Refused(CurvedBooleanPipelineRefusal::Selection(error))
        })?;
    if selected.arranged().is_empty() || selected.caps().is_empty() {
        return refused(CurvedBooleanPipelineRefusal::AssemblyContract(
            "certified coincident-cap Boolean selected no boundary",
        ));
    }
    let plan = plan_parallel_cylinder_coincident_boolean(
        &edit.state.store,
        graph,
        prepared.bindings(),
        selected,
        relation,
        linear,
    )
    .map_err(mixed_plan_failure)?;
    if !coincident_cap_plan_matches_relation(&plan, relation) {
        return refused(CurvedBooleanPipelineRefusal::AssemblyContract(
            "coincident-cap shell omitted certified boundary evidence",
        ));
    }
    realize_mixed_shell(edit, &plan, linear, scope)
}

/// Give a commutative operation one caller-order-independent source order.
/// Store iteration is deterministic slot order and carries no geometric case
/// decision; operand meaning is recovered later by the certified relation.
fn canonical_commutative_order(
    edit: &PartEdit<'_>,
    bodies: [BodyId; 2],
) -> StageResult<[BodyId; 2]> {
    for (candidate, _) in edit.state.store.iter::<TopologyBody>() {
        if candidate == bodies[0].raw() {
            return Ok(bodies);
        }
        if candidate == bodies[1].raw() {
            return Ok([bodies[1].clone(), bodies[0].clone()]);
        }
    }
    Err(kcore::error::Error::InvalidGeometry {
        reason: "prevalidated parallel-cylinder operands left the part store",
    }
    .into())
}

fn plan_matches_relation(
    plan: &MixedShellProofPlan,
    relation: &CertifiedParallelCylinderLensRelation,
) -> bool {
    if plan.section_edges().len() != 4 {
        return false;
    }
    let rulings_match = relation.rulings().iter().all(|witness| {
        let mut matches = plan
            .section_edges()
            .iter()
            .filter(|edge| edge.fragment_index() == witness.fragment());
        let Some(edge) = matches.next() else {
            return false;
        };
        if matches.next().is_some() {
            return false;
        }
        let actual_endpoints = edge.endpoints();
        let expected_endpoints = witness.endpoints();
        edge.fragment().branch() == witness.branch()
            && (actual_endpoints == expected_endpoints
                || actual_endpoints == [expected_endpoints[1], expected_endpoints[0]])
    });
    let caps_match = relation.overlap_ends().iter().all(|witness| {
        let mut matches = plan
            .section_edges()
            .iter()
            .filter(|edge| edge.fragment_index() == witness.fragment());
        let Some(edge) = matches.next() else {
            return false;
        };
        matches.next().is_none() && edge.fragment().branch() == witness.branch()
    });
    rulings_match && caps_match
}

fn coincident_cap_plan_matches_relation(
    plan: &MixedShellProofPlan,
    relation: &CertifiedParallelCylinderCoincidentCapRelation,
) -> bool {
    let expected_source_span_count = relation
        .overlap_ends()
        .iter()
        .flat_map(|end| end.sources().iter().flatten())
        .count();
    if plan.faces().len() != 4 + plan.cap_rings().len()
        || plan.section_edges().len() != 2 + relation.unique_end_count()
        || plan.bounded_source_spans().len() != expected_source_span_count
    {
        return false;
    }
    let rulings_match = relation.rulings().iter().all(|witness| {
        unique_section_edge(
            plan,
            witness.fragment(),
            witness.branch(),
            Some(witness.endpoints()),
        )
    });
    let cap_arcs_match = relation
        .overlap_ends()
        .iter()
        .filter_map(|end| end.cap_arc())
        .all(|witness| {
            unique_section_edge(
                plan,
                witness.fragment(),
                witness.branch(),
                Some(witness.endpoints()),
            )
        });
    let source_arcs_match = relation
        .overlap_ends()
        .iter()
        .flat_map(|end| end.sources().iter().flatten())
        .all(|witness| {
            let mut matches = plan.bounded_source_spans().iter().filter(|span| {
                span.source().operand() == witness.operand()
                    && span.edge() == witness.edge()
                    && source_roots_match(span.roots(), &witness.roots())
            });
            matches.next().is_some() && matches.next().is_none()
        });
    rulings_match && cap_arcs_match && source_arcs_match
}

fn unique_section_edge(
    plan: &MixedShellProofPlan,
    fragment: usize,
    branch: usize,
    endpoints: Option<[usize; 2]>,
) -> bool {
    let mut matches = plan
        .section_edges()
        .iter()
        .filter(|edge| edge.fragment_index() == fragment);
    let Some(edge) = matches.next() else {
        return false;
    };
    matches.next().is_none()
        && edge.fragment().branch() == branch
        && endpoints.is_none_or(|expected| {
            edge.endpoints() == expected || edge.endpoints() == [expected[1], expected[0]]
        })
}

fn source_roots_match(
    actual: &[MixedBoundedSourceRoot; 2],
    expected: &[ParallelCylinderSourceRootWitness; 2],
) -> bool {
    let root_matches = |actual: MixedBoundedSourceRoot,
                        expected: ParallelCylinderSourceRootWitness| {
        actual.endpoint() == expected.endpoint()
            && actual.root_ordinal() == expected.root_ordinal()
            && actual.parameter().to_bits() == expected.parameter().to_bits()
            && actual.enclosure().map(f64::to_bits) == expected.enclosure().map(f64::to_bits)
    };
    root_matches(actual[0], expected[0]) && root_matches(actual[1], expected[1])
        || root_matches(actual[0], expected[1]) && root_matches(actual[1], expected[0])
}
