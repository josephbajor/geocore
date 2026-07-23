//! Read-only complete-plan proof for one transverse finite-cylinder pair.
//!
//! The generic mixed-shell planner intentionally permits relation-specific
//! Section subsets.  A prepared transverse cylinder pair has the stronger
//! contract that every admitted Section fragment must bound the selected
//! result.  This adapter proves that stronger invariant after exact physical
//! edge coalescing, precharges persistent composite certification, and stops
//! before certificate construction or topology allocation.

use std::collections::{BTreeMap, BTreeSet};

use kcore::operation::OperationScope;
use ktopo::store::Store;

use super::super::boundary_select::{
    BoundarySelectionError, RegularizedBooleanOperation, select_boundary_fragments,
};
use super::super::cylinder_pair_boundary::PreparedCylinderPairBoundary;
use super::super::pipeline::PLANAR_BOOLEAN_REALIZATION_WORK;
use super::materialize::{
    MixedShellMaterializationBlueprint, MixedShellMaterializationError, PhysicalCarrier,
    PhysicalVertex, prepare_mixed_shell_materialization,
};
use super::{
    MixedPcurveLineage, MixedSectionEdgePlan, MixedShellEdgeKey, MixedShellPlanError,
    MixedShellProofPlan, MixedSourceFaceKey, plan_mixed_shell,
};
use crate::BodySectionGraph;
use crate::error::Error;

/// Allocation-free complete cylinder-pair plan, including coalesced physical incidence.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct CertifiedCylinderPairPlan {
    plan: MixedShellProofPlan,
    blueprint: MixedShellMaterializationBlueprint,
    work: u64,
}

impl CertifiedCylinderPairPlan {
    pub(crate) const fn plan(&self) -> &MixedShellProofPlan {
        &self.plan
    }

    pub(crate) const fn blueprint(&self) -> &MixedShellMaterializationBlueprint {
        &self.blueprint
    }

    /// Exact charged work for coalescing, composites, and complete validation.
    pub(crate) const fn work(&self) -> u64 {
        self.work
    }

    #[cfg(test)]
    pub(crate) fn plan_mut_for_test(&mut self) -> &mut MixedShellProofPlan {
        &mut self.plan
    }
}

/// Typed refusal before any topology transaction is opened.
#[derive(Debug)]
pub(crate) enum CylinderPairPlanError {
    Selection(BoundarySelectionError),
    Plan(MixedShellPlanError),
    PhysicalIncidence(MixedShellMaterializationError),
    Execution(Error),
    WorkCountOverflow,
    UnknownPlannedSectionFragment {
        fragment: usize,
    },
    SectionEdgePayloadMismatch {
        fragment: usize,
    },
    SectionFragmentCoverage {
        fragment: usize,
        planned: usize,
        physical: usize,
    },
    SectionCarrierFacesMismatch {
        fragment: usize,
    },
    SectionUseLineageMismatch {
        fragment: usize,
        face: MixedSourceFaceKey,
    },
}

/// Select, plan, coalesce, precharge, and certify complete physical incidence
/// without constructing persistent certificates or allocating topology.
pub(crate) fn plan_cylinder_pair_boundary(
    store: &Store,
    graph: &BodySectionGraph,
    prepared: &PreparedCylinderPairBoundary,
    operation: RegularizedBooleanOperation,
    scope: &mut OperationScope<'_, '_>,
) -> Result<CertifiedCylinderPairPlan, CylinderPairPlanError> {
    let selected = select_boundary_fragments(operation, prepared.classified())
        .map_err(CylinderPairPlanError::Selection)?;
    let plan = plan_mixed_shell(store, graph, prepared.bindings(), selected)
        .map_err(CylinderPairPlanError::Plan)?;
    let blueprint = prepare_mixed_shell_materialization(&plan, store)
        .map_err(CylinderPairPlanError::PhysicalIncidence)?;
    let validation_work = cylinder_pair_validation_work(
        graph.curve_fragments().len(),
        plan.section_edges().len(),
        blueprint.edges().len(),
    )
    .ok_or(CylinderPairPlanError::WorkCountOverflow)?;
    let work = blueprint
        .work()
        .checked_add(validation_work)
        .ok_or(CylinderPairPlanError::WorkCountOverflow)?;
    scope
        .ledger_mut()
        .charge(PLANAR_BOOLEAN_REALIZATION_WORK, work)
        .map_err(Error::from)
        .map_err(CylinderPairPlanError::Execution)?;
    validate_complete_section_incidence(graph, &plan, &blueprint)?;
    Ok(CertifiedCylinderPairPlan {
        plan,
        blueprint,
        work,
    })
}

/// `G + S + 3E`: one admitted-fragment scan, one planned-edge scan, and one
/// physical-edge scan plus its already-proven two uses.
fn cylinder_pair_validation_work(
    graph_fragments: usize,
    section_edges: usize,
    physical_edges: usize,
) -> Option<u64> {
    u64::try_from(graph_fragments)
        .ok()?
        .checked_add(u64::try_from(section_edges).ok()?)?
        .checked_add(u64::try_from(physical_edges).ok()?.checked_mul(3)?)
}

fn validate_complete_section_incidence(
    graph: &BodySectionGraph,
    plan: &MixedShellProofPlan,
    blueprint: &MixedShellMaterializationBlueprint,
) -> Result<(), CylinderPairPlanError> {
    let fragment_count = graph.curve_fragments().len();
    let mut planned_counts = vec![0_usize; fragment_count];
    let mut planned = BTreeMap::<usize, &MixedSectionEdgePlan>::new();
    for edge in plan.section_edges() {
        let fragment = edge.fragment_index();
        let Some(count) = planned_counts.get_mut(fragment) else {
            return Err(CylinderPairPlanError::UnknownPlannedSectionFragment { fragment });
        };
        *count = count
            .checked_add(1)
            .ok_or(CylinderPairPlanError::WorkCountOverflow)?;
        planned.entry(fragment).or_insert(edge);
        let graph_fragment = &graph.curve_fragments()[fragment];
        let graph_branch = graph
            .branches()
            .get(graph_fragment.branch())
            .ok_or(CylinderPairPlanError::SectionEdgePayloadMismatch { fragment })?;
        if edge.fragment() != graph_fragment || edge.branch() != graph_branch {
            return Err(CylinderPairPlanError::SectionEdgePayloadMismatch { fragment });
        }
    }

    let mut physical_counts = vec![0_usize; fragment_count];
    for physical in blueprint.edges() {
        let PhysicalCarrier::Section(fragment) = physical.carrier() else {
            continue;
        };
        let Some(count) = physical_counts.get_mut(fragment) else {
            return Err(CylinderPairPlanError::UnknownPlannedSectionFragment { fragment });
        };
        *count = count
            .checked_add(1)
            .ok_or(CylinderPairPlanError::WorkCountOverflow)?;
        let Some(section) = planned.get(&fragment).copied() else {
            continue;
        };
        if physical.endpoints() != Some(section.endpoints().map(PhysicalVertex::Section)) {
            return Err(CylinderPairPlanError::SectionEdgePayloadMismatch { fragment });
        }
        validate_section_uses(fragment, section, physical.uses(), plan)?;
    }

    validate_fragment_coverage(&planned_counts, &physical_counts)
}

fn validate_fragment_coverage(
    planned_counts: &[usize],
    physical_counts: &[usize],
) -> Result<(), CylinderPairPlanError> {
    for (fragment, (&planned, &physical)) in planned_counts.iter().zip(physical_counts).enumerate()
    {
        if planned != 1 || physical != 1 {
            return Err(CylinderPairPlanError::SectionFragmentCoverage {
                fragment,
                planned,
                physical,
            });
        }
    }
    Ok(())
}

fn validate_section_uses(
    fragment: usize,
    section: &MixedSectionEdgePlan,
    uses: &[super::materialize::PhysicalUse],
    plan: &MixedShellProofPlan,
) -> Result<(), CylinderPairPlanError> {
    let expected_faces = section.carrier_faces().into_iter().collect::<BTreeSet<_>>();
    let mut actual_faces = BTreeSet::new();
    for physical_use in uses {
        let Some(face) = plan.faces().get(physical_use.face()) else {
            return Err(CylinderPairPlanError::SectionCarrierFacesMismatch { fragment });
        };
        let source = face.source();
        let Some(use_) = face
            .loops()
            .get(physical_use.loop_index())
            .and_then(|loop_| loop_.uses().get(physical_use.use_index()))
        else {
            return Err(CylinderPairPlanError::SectionUseLineageMismatch {
                fragment,
                face: source,
            });
        };
        let lineage_matches = matches!(
            (use_.edge(), use_.pcurve()),
            (
                MixedShellEdgeKey::SectionFragment(candidate),
                MixedPcurveLineage::Section {
                    branch,
                    operand,
                    ..
                }
            ) if *candidate == fragment
                && *branch == section.fragment().branch()
                && *operand == source.operand()
        );
        if !lineage_matches || !actual_faces.insert(source) {
            return Err(CylinderPairPlanError::SectionUseLineageMismatch {
                fragment,
                face: source,
            });
        }
    }
    if actual_faces != expected_faces {
        return Err(CylinderPairPlanError::SectionCarrierFacesMismatch { fragment });
    }
    Ok(())
}

#[cfg(test)]
mod unit_tests {
    use super::{CylinderPairPlanError, cylinder_pair_validation_work, validate_fragment_coverage};

    #[test]
    fn complete_validation_work_is_checked_and_exact() {
        assert_eq!(cylinder_pair_validation_work(8, 8, 12), Some(52));
        assert_eq!(cylinder_pair_validation_work(8, 8, 14), Some(58));
        assert_eq!(cylinder_pair_validation_work(0, 0, 0), Some(0));
    }

    #[test]
    fn complete_fragment_coverage_rejects_missing_and_duplicate_entries() {
        assert!(matches!(
            validate_fragment_coverage(&[1, 0], &[1, 1]),
            Err(CylinderPairPlanError::SectionFragmentCoverage {
                fragment: 1,
                planned: 0,
                physical: 1,
            })
        ));
        assert!(matches!(
            validate_fragment_coverage(&[1], &[2]),
            Err(CylinderPairPlanError::SectionFragmentCoverage {
                fragment: 0,
                planned: 1,
                physical: 2,
            })
        ));
    }
}
